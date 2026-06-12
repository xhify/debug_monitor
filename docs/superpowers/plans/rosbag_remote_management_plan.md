# 车端 rosbag 管理功能第一版实现计划

## 1. 背景与目标

debug_monitor 是 Windows 端 PySide6 上位机，当前已经具备 ROSbridge/roslibpy 通信能力，并且可以通过 `/launch_manager/command` 向小车 ROS 端发送 `std_msgs/String` 命令。

本计划只实现 debug_monitor Windows 端功能，不实现车端 `launch_manager` 内部逻辑。车端按本文协议提供 rosbag 录制、状态、列表、详情、删除等能力。

核心目标：

1. Windows 端不通过 ROSbridge 接收高频原始数据写 bag。
2. 原始 rosbag 必须在小车本地录制。
3. ROSbridge 只负责命令和状态。
4. 大文件同步使用 `rsync`，失败后自动降级 `scp`。
5. debug_monitor 提供车端 rosbag 的录制、查看、同步、删除管理能力。
6. 第一版必须包含永久删除功能，但必须二次确认。
7. 默认不把 `.bag` 大文件打入 zip，只记录 manifest。

最终用户体验：

- 在 Windows 软件中看到当前小车是否正在录制 rosbag。
- 看到录制时长、大小、topic、磁盘剩余。
- 刷新小车上的 rosbag session 列表。
- 选中某个 rosbag 同步到 Windows。
- 可以移动到车端回收站。
- 可以永久删除，但必须输入完整 session_id 确认。
- 汇总记录可以联动车端 rosbag start/stop。

---

## 2. 现有代码基础

重点阅读并保持兼容：

- `src/main_window.py`
- `src/ros_bridge_worker.py`
- `src/summary_package.py`
- `src/ros_topic_recorders.py`
- `src/app_config.py`
- `requirements.txt`

当前已有能力：

- `ros_bridge_worker.py` 中已有 `RosBridgeSession` 和 `RosBridgeWorker`。
- 已支持向 `/cmd_vel`、`/line_follow_control`、`/launch_manager/command` 发布消息。
- `main_window.py` 已有汇总页、数据源检查、开始/停止同步记录、session.json、manifest.json、zip 打包。
- 当前 ROS CSV 记录是通过 ROSbridge 收消息后写 CSV，只适合轻量数据，不适合点云和高频原始数据。

设计约束：

- 不要破坏现有 CSV 汇总记录、雷达记录、串口记录、ROS CSV 记录。
- 不要进行与 rosbag 管理无关的大规模重构。
- UI 线程不能被同步、刷新、删除、等待状态等操作阻塞。
- 尽量新增模块，小范围接入 `main_window.py`。

---

## 3. launch_manager 协议约定

继续使用现有命令 topic：

- `/launch_manager/command`
- 类型：`std_msgs/String`
- payload：JSON 字符串

debug_monitor 新增订阅状态 topic：

- `/launch_manager/status`
- 类型：`std_msgs/String`
- payload：JSON 字符串

### 3.1 开始 rosbag

```json
{
  "action": "start_rosbag",
  "session_id": "session_20260612_153000",
  "bag_dir": "/home/wheeltec/bags",
  "prefix": "fastlio",
  "topics": [
    "/point_cloud_raw",
    "/imu",
    "/Odometry",
    "/path",
    "/cmd_vel",
    "/tf",
    "/tf_static"
  ],
  "split_size_mb": 2048,
  "compression": "lz4"
}
```

### 3.2 停止 rosbag

```json
{
  "action": "stop_rosbag",
  "session_id": "session_20260612_153000"
}
```

### 3.3 查询状态

```json
{
  "action": "query_status"
}
```

### 3.4 列出车端 rosbag

```json
{
  "action": "list_rosbags",
  "bag_dir": "/home/wheeltec/bags"
}
```

### 3.5 查看详情

```json
{
  "action": "inspect_rosbag",
  "session_id": "session_20260612_153000"
}
```

### 3.6 移到车端回收站

```json
{
  "action": "trash_rosbag",
  "session_id": "session_20260612_153000"
}
```

### 3.7 永久删除

```json
{
  "action": "delete_rosbag",
  "session_id": "session_20260612_153000",
  "confirm": "session_20260612_153000"
}
```

永久删除必须在 UI 中要求用户输入完整 `session_id`。只有输入完全一致，才允许发送 `delete_rosbag` 命令。

### 3.8 状态返回格式

`/launch_manager/status` 建议兼容以下字段：

```json
{
  "rosbag": {
    "active": true,
    "state": "recording",
    "session_id": "session_20260612_153000",
    "duration_s": 52.4,
    "remote_dir": "/home/wheeltec/bags/session_20260612_153000",
    "current_size_bytes": 123456789,
    "bag_files": ["fastlio_0.bag.active"],
    "topics": ["/point_cloud_raw", "/imu", "/Odometry"],
    "disk_free_gb": 36.2,
    "last_error": ""
  },
  "rosbag_library": {
    "bag_dir": "/home/wheeltec/bags",
    "disk_free_gb": 36.2,
    "sessions": [
      {
        "session_id": "session_20260612_153000",
        "status": "stopped",
        "remote_dir": "/home/wheeltec/bags/session_20260612_153000",
        "size_bytes": 2469606195,
        "duration_s": 272.5,
        "file_count": 2,
        "topic_count": 7,
        "created_at": "2026-06-12T15:30:00",
        "bag_files": ["fastlio_0.bag", "fastlio_1.bag"],
        "downloaded": false
      }
    ]
  }
}
```

解析时必须容错。缺字段使用安全默认值，非法 JSON 不应导致程序崩溃。

---

## 4. 修改 `src/ros_bridge_worker.py`

### 4.1 新增状态订阅

在 `RosBridgeSession.SUBSCRIPTIONS` 中新增：

```python
("/launch_manager/status", "std_msgs/String")
```

在 `_callback_for()` 中新增分支：

```python
if name == "/launch_manager/status":
    return lambda message, topic_name=name: self._on_launch_manager_status(topic_name, message)
```

### 4.2 新增状态解析回调

`RosBridgeSession.__init__` 增加可选参数：

```python
on_launch_manager_status: Callable[[dict], None] | None = None
```

新增方法：

```python
def _on_launch_manager_status(self, topic_name: str, message: dict) -> None:
    ...
```

要求：

- 从 `message.get("data", "")` 解析 JSON。
- 成功解析后调用 `self._on_launch_manager_status(payload)`。
- 同时可调用 `_publish_message()` 保持事件流一致。
- JSON 解析失败时不崩溃，可通过错误回调或 `_publish_message()` 传递错误事件。

### 4.3 新增 Worker 信号

`RosBridgeWorker` 增加：

```python
launch_manager_status_received = Signal(object)
```

创建 `RosBridgeSession` 时传入：

```python
on_launch_manager_status=self.launch_manager_status_received.emit
```

### 4.4 封装 rosbag 命令

在 `RosBridgeSession` 或 `RosBridgeWorker` 中封装：

```python
def request_rosbag_start(self, config: dict) -> None

def request_rosbag_stop(self, session_id: str) -> None

def request_rosbag_list(self, bag_dir: str = "/home/wheeltec/bags") -> None

def request_rosbag_inspect(self, session_id: str) -> None

def request_rosbag_trash(self, session_id: str) -> None

def request_rosbag_delete(self, session_id: str, confirm: str) -> None

def request_launch_manager_status(self) -> None
```

底层继续使用 `publish_launch_manager_command()`，统一 `json.dumps(..., ensure_ascii=False)`。

---

## 5. 新增 `src/rosbag_models.py`

新增数据模型和解析函数，避免 UI 中到处操作裸 dict。

建议包含：

```python
from dataclasses import dataclass, field

@dataclass
class RosbagRecordingStatus:
    active: bool = False
    state: str = "idle"
    session_id: str = ""
    duration_s: float = 0.0
    remote_dir: str = ""
    current_size_bytes: int = 0
    disk_free_gb: float = 0.0
    topics: list[str] = field(default_factory=list)
    bag_files: list[str] = field(default_factory=list)
    last_error: str = ""

@dataclass
class RemoteRosbagSession:
    session_id: str = ""
    status: str = "unknown"
    remote_dir: str = ""
    size_bytes: int = 0
    duration_s: float = 0.0
    file_count: int = 0
    topic_count: int = 0
    created_at: str = ""
    bag_files: list[str] = field(default_factory=list)
    downloaded: bool = False
    local_dir: str = ""

@dataclass
class RosbagLibraryState:
    bag_dir: str = ""
    disk_free_gb: float = 0.0
    sessions: list[RemoteRosbagSession] = field(default_factory=list)
```

提供函数：

```python
parse_rosbag_recording_status(payload: dict) -> RosbagRecordingStatus
parse_rosbag_library_state(payload: dict) -> RosbagLibraryState
format_bytes(size_bytes: int) -> str
format_duration(duration_s: float) -> str
```

要求：

- 对缺失字段容错。
- 对类型错误容错。
- 不抛出 UI 难以处理的异常。

---

## 6. 修改 `src/app_config.py`

新增：

```python
DEFAULT_ROSBAG_REMOTE_DIR = "/home/wheeltec/bags"
DEFAULT_ROSBAG_SPLIT_SIZE_MB = 2048
DEFAULT_ROSBAG_COMPRESSION = "lz4"

ROSBAG_TOPIC_PRESETS = {
    "control": [
        "/imu",
        "/Odometry",
        "/cmd_vel",
        "/line_follow_control",
        "/tf",
        "/tf_static",
    ],
    "fastlio": [
        "/point_cloud_raw",
        "/imu",
        "/Odometry",
        "/path",
        "/cmd_vel",
        "/tf",
        "/tf_static",
    ],
    "full": [
        "/point_cloud_raw",
        "/imu",
        "/active_imu",
        "/odom",
        "/Odometry",
        "/path",
        "/cmd_vel",
        "/line_follow_control",
        "/PowerVoltage",
        "/wheeltec/akm_state",
        "/wheeltec/control_debug",
        "/wheeltec/chassis_diagnostics",
        "/tf",
        "/tf_static",
    ],
}
```

默认模式使用 `fastlio`。

---

## 7. 新增 `src/widgets/rosbag_panel.py`

新增 ROSBag 管理页面。

### 7.1 当前录制状态区域

显示：

- 状态：空闲 / 录制中 / 停止中 / 错误
- session_id
- 远程目录
- 录制时长
- 当前大小
- 磁盘剩余
- 当前文件数
- topic 数量
- last_error

### 7.2 录制配置区域

控件：

- 模式 combo：`control` / `fastlio` / `full` / `custom`
- 车端目录 `QLineEdit`，默认 `/home/wheeltec/bags`
- prefix `QLineEdit`，默认 `fastlio`
- split size `QSpinBox`，默认 `2048 MB`
- compression combo：`lz4` / `none`，默认 `lz4`
- topics `QPlainTextEdit`，多行 topic

切换模式时自动填充 topics，`custom` 模式不自动覆盖用户输入。

### 7.3 操作按钮

- 开始车端 rosbag
- 停止车端 rosbag
- 刷新列表
- 查询状态

### 7.4 信号

```python
start_requested = Signal(object)      # config: dict
stop_requested = Signal(str)          # session_id
list_requested = Signal(str)          # bag_dir
inspect_requested = Signal(str)       # session_id
sync_requested = Signal(object)       # RemoteRosbagSession
trash_requested = Signal(str)         # session_id
delete_requested = Signal(str, str)   # session_id, confirm
query_status_requested = Signal()
```

### 7.5 车端 rosbag 列表

使用 `QTableWidget` 或 `QTableView`。第一版可以用 `QTableWidget`。

列：

- session_id
- 状态
- 时长
- 大小
- 文件数
- topic数
- 创建时间
- 已同步
- 远程目录

按钮：

- 查看详情
- 同步到 Windows
- 移到车端回收站
- 永久删除

### 7.6 删除保护

永久删除要求：

1. 弹出确认对话框。
2. 明确提示“永久删除不可恢复”。
3. 提示“未同步的数据会丢失”。
4. 要求用户输入完整 `session_id`。
5. 输入完全一致后才 emit `delete_requested(session_id, confirm)`。
6. 正在录制的 session 不允许删除。

移到回收站也要提示，但不需要输入 session_id。

---

## 8. 新增 `src/rosbag_sync_worker.py`

实现 Windows 端后台同步车端 rosbag 到本地。

建议类：

```python
class RosbagSyncWorker(QObject):
    progress = Signal(str)
    finished = Signal(object)
    error = Signal(str)
```

输入参数：

- `host: str`
- `username: str = "wheeltec"`
- `remote_dir: str`
- `local_dir: Path`

同步策略：

1. 优先尝试 rsync：

```bash
rsync -avP wheeltec@HOST:REMOTE_DIR/ LOCAL_DIR/
```

2. 如果 rsync 命令不存在、启动失败、返回非零，则自动降级 scp：

```bash
scp -r wheeltec@HOST:REMOTE_DIR LOCAL_PARENT_DIR
```

要求：

- 同步过程不能阻塞 UI。
- 捕获 stdout/stderr，输出简短进度到 `progress`。
- rsync 失败时通过 progress 提示“rsync 失败，正在降级为 scp”。
- scp 成功后 `finished` 返回：

```python
{
    "method": "scp",
    "local_dir": "...",
    "remote_dir": "...",
    "returncode": 0,
}
```

- rsync 成功后返回 method 为 `rsync`。
- scp 也失败时 emit `error`。
- 不要把密码写入代码。
- 第一版假设用户已经配置 SSH 可登录小车。

---

## 9. 修改 `src/main_window.py`

### 9.1 初始化与 UI 接入

导入：

- `RosbagPanel`
- `RosbagSyncWorker`
- `rosbag_models`
- app_config 中 rosbag 默认配置

连接：

```python
self._ros_worker.launch_manager_status_received.connect(self._on_launch_manager_status)
```

新增模块按钮：

- `ROSBag`

创建：

```python
self._rosbag_panel = RosbagPanel()
```

加入 `module_stack`。

### 9.2 连接 RosbagPanel 信号

```python
self._rosbag_panel.start_requested.connect(self._ros_worker.request_rosbag_start)
self._rosbag_panel.stop_requested.connect(self._ros_worker.request_rosbag_stop)
self._rosbag_panel.list_requested.connect(self._ros_worker.request_rosbag_list)
self._rosbag_panel.inspect_requested.connect(self._ros_worker.request_rosbag_inspect)
self._rosbag_panel.trash_requested.connect(self._ros_worker.request_rosbag_trash)
self._rosbag_panel.delete_requested.connect(self._ros_worker.request_rosbag_delete)
self._rosbag_panel.query_status_requested.connect(self._ros_worker.request_launch_manager_status)
self._rosbag_panel.sync_requested.connect(self._start_rosbag_sync)
```

### 9.3 新增 `_on_launch_manager_status(payload)`

职责：

- 解析 `rosbag` 状态。
- 更新 `RosbagPanel` 当前录制状态。
- 如果包含 `rosbag_library`，更新 rosbag 列表。
- 如果当前正在汇总记录，并启用了 `rosbag_raw`，缓存 latest rosbag 状态用于写 `session.json`。
- 如果状态中包含错误，显示到状态栏或面板中。

### 9.4 新增 `_start_rosbag_sync(session)`

逻辑：

1. 确定本地目录：
   - 如果当前有 summary session_dir，默认放到：
     `session_dir / "raw" / "rosbag" / session.session_id`
   - 如果没有当前 session，弹出目录选择框，默认：
     `DEFAULT_RECORDINGS_DIR / "rosbags" / session.session_id`
2. 后台启动 `RosbagSyncWorker`。
3. progress 更新状态栏或 RosbagPanel 日志。
4. finished 后更新本地 metadata：
   - `downloaded=true`
   - `local_dir`
   - `downloaded_at`
   - `sync_method`
5. 同步完成后自动刷新车端 rosbag 列表。
6. 同步失败只显示错误，不删除远程文件。

### 9.5 汇总页新增数据源

在 `_build_summary_source_group()` 的 `source_items` 中新增：

```python
("rosbag_raw", "车端 rosbag 原始数据")
```

### 9.6 汇总开始记录接入 rosbag

在 `_start_summary_recording()` 中：

1. 生成 timestamp/session_id 后，如果 `rosbag_raw` 勾选：
   - 从 `RosbagPanel` 读取当前 config。
   - 设置 `config["session_id"] = f"session_{timestamp}"`。
   - 发送 `start_rosbag`。
   - 记录 `start_command_sent=true`。
2. 尽量等待最多 3 秒状态确认。
3. 如果超时，不阻断整个记录，但写入 warning。
4. 然后继续现有 CSV / 串口 / 雷达记录流程。

### 9.7 汇总停止记录接入 rosbag

在 `_stop_summary_recording_async()` 或相关停止流程中：

1. 如果本次 session 启用了 `rosbag_raw`：
   - 先发送 `stop_rosbag`。
   - 等待最多 10 秒状态变为 `stopped` 或 `active=false`。
   - 超时写 warning，但继续本地 finalize。
2. 更新 `session.json` 的 rosbag 字段：
   - enabled
   - session_id
   - remote_dir
   - topics
   - compression
   - split_size_mb
   - start_command_sent
   - stop_command_sent
   - latest_status
   - remote_files
   - duration_s
   - size_bytes
   - downloaded
   - local_dir

注意：不要长时间卡 UI。等待逻辑应在后台线程或异步流程中处理。

### 9.8 `_summary_files_metadata()` 增加

如果 `rosbag_raw` 启用：

```python
files["rosbag_manifest"] = "raw/rosbag_manifest.json"
```

---

## 10. 修改 `src/summary_package.py`

目标：支持 rosbag manifest，但不打包 `.bag` 大文件。

要求：

1. `build_summary_package()` 中，如果 `session.json` 存在 `rosbag` 字段，则生成：

```text
raw/rosbag_manifest.json
```

2. `rosbag_manifest.json` 内容来自 `session.json["rosbag"]`，并补充本地文件信息。

3. zip 中包含：

- `session.json`
- `manifest.json`
- `raw/rosbag_manifest.json`

4. 默认不要把 `.bag` 文件加入 zip。

5. 如果 `raw/rosbag/` 中已经有下载好的 `.bag`，也不要自动打包 `.bag`；只在 manifest 中记录文件路径、大小、数量。

6. `manifest.json` 中新增 `rosbag` 字段。

---

## 11. 永久删除功能要求

第一版必须上线永久删除。

实现要求：

1. RosbagPanel 中必须有“永久删除”按钮。
2. 点击后弹窗，说明：
   - 将永久删除小车端该 rosbag session；
   - 删除后不可恢复；
   - 未同步的数据会丢失；
   - 正在录制的 session 不允许删除。
3. 要求用户输入完整 `session_id`。
4. 只有输入完全一致才发送 `delete_rosbag` 命令。
5. 发送命令后状态栏显示“已发送永久删除请求”。
6. 收到 `/launch_manager/status` 中列表刷新后，UI 更新。
7. 永久删除失败时显示 `last_error` 或错误状态。
8. 不要在 Windows 端通过 SSH 执行 `rm -rf`。
9. 删除必须通过 `/launch_manager/command` 的 `delete_rosbag` action 完成。

---

## 12. 同步功能要求

第一版必须上线同步功能。

实现要求：

1. 选中 rosbag session，点击“同步到 Windows”。
2. 同步优先使用 rsync：

```bash
rsync -avP wheeltec@HOST:REMOTE_DIR/ LOCAL_DIR/
```

3. rsync 失败后自动降级 scp：

```bash
scp -r wheeltec@HOST:REMOTE_DIR LOCAL_PARENT_DIR
```

4. 同步过程通过后台线程运行，UI 不阻塞。
5. 显示当前使用的方法：rsync 或 scp。
6. 显示简短进度日志。
7. 同步成功后更新本地 metadata：
   - downloaded=true
   - local_dir
   - downloaded_at
   - sync_method
8. 同步失败后显示错误，不删除远程文件。
9. 同步成功后不要自动删除车端文件，删除必须由用户手动点击。

---

## 13. 测试要求

新增或修改测试，尽量使用 mock，不依赖真实 ROS、小车、rsync、scp。

必须覆盖：

1. rosbag command JSON 生成：
   - start_rosbag
   - stop_rosbag
   - list_rosbags
   - inspect_rosbag
   - trash_rosbag
   - delete_rosbag
2. launch_manager/status JSON 解析：
   - 正常 rosbag 状态
   - 正常 rosbag_library 列表
   - 缺字段容错
   - 非法 JSON 不崩溃
3. `RosbagSyncWorker`：
   - rsync 成功时不调用 scp
   - rsync 失败时调用 scp
   - rsync 不存在时调用 scp
   - scp 失败时报 error
4. 永久删除确认逻辑：
   - confirm 不匹配不发送命令
   - 正在录制 session 不允许删除
5. `summary_package.py`：
   - session.json 有 rosbag 字段时生成 raw/rosbag_manifest.json
   - zip 包含 rosbag_manifest.json
   - zip 默认不包含 .bag 文件

---

## 14. 验收标准

第一版完成后应满足：

1. Windows 软件中出现“ROSBag”页面。
2. 连接 ROSbridge 后，可以点击“查询状态”和“刷新列表”。
3. 可以看到车端返回的 rosbag 当前录制状态。
4. 可以看到车端 rosbag session 列表。
5. 可以从 UI 发出 `start_rosbag` / `stop_rosbag`。
6. 汇总记录页面可以勾选“车端 rosbag 原始数据”。
7. 汇总记录开始/停止时会联动发送 `start_rosbag` / `stop_rosbag`。
8. 可以选中一个远程 rosbag session，同步到 Windows。
9. 同步优先 rsync，失败后自动降级 scp。
10. 可以将 session 移到车端回收站。
11. 可以永久删除 session，但必须输入完整 session_id 二次确认。
12. `session.json`、`manifest.json`、`raw/rosbag_manifest.json` 能记录远程 rosbag 信息。
13. 默认 zip 不包含 `.bag` 大文件。
14. 现有 CSV 记录、雷达记录、串口记录、ROS CSV 记录功能不被破坏。

---

## 15. 建议实施顺序

1. 先实现 `rosbag_models.py`。
2. 修改 `ros_bridge_worker.py`，接入 `/launch_manager/status` 和命令封装。
3. 新增 `widgets/rosbag_panel.py`，先做 UI 和本地状态刷新。
4. 在 `main_window.py` 中接入 ROSBag 页面和状态回调。
5. 新增 `rosbag_sync_worker.py`，实现 rsync -> scp fallback。
6. 汇总页新增 `rosbag_raw` 数据源。
7. 汇总开始/停止流程联动车端 rosbag。
8. 修改 `summary_package.py`，生成 `raw/rosbag_manifest.json`。
9. 加永久删除确认逻辑。
10. 补 mock 测试。
11. 运行现有测试和新增测试。
12. 做一次手动 UI 检查：打开软件、切换 ROSBag 页面、触发 mock 状态、确认表格与按钮正常。

---

## 16. 非目标

第一版不做：

1. 不实现车端 `launch_manager` 内部 rosbag 逻辑。
2. 不通过 ROSbridge 传输 `.bag` 文件。
3. 不自动删除已同步的远程 bag。
4. 不默认把 `.bag` 打入 zip。
5. 不做复杂的 rosbag info 可视化分析。
6. 不做 MATLAB 自动分析，只保留后续扩展空间。

后续版本可以考虑：

- 车端 `rosbag info --yaml` 解析展示 topic 频率。
- bag 完整性检查与 reindex。
- MATLAB README 自动生成。
- 本地/远程文件大小一致性校验。
- 自动清理策略：保留最近 N 次或磁盘低于阈值提醒清理。
