# ROSbridge 重启、状态检测与定位页雷达直线校准设计

日期：2026-06-18

## 目标

在不重启底盘、IMU 等整套 ROS launch 的前提下，为 Debug Monitor 增加以下能力：

1. ROS 页面一键通过 SSH 重启 `/rosbridge_websocket`，并自动恢复上位机连接与 topic 订阅。
2. 更完整地实时显示 ROSbridge 节点健康状态，而不只显示本地 WebSocket 是否曾经连接成功。
3. 定位精度页面顶部增加 FAST-LIO 里程计 topic 输入框，并与 ROS 页面双向同步。
4. 将定位精度页面原有的“控制反馈接口预留”区域替换为雷达直线校准节点启动与运动控制。

## 已确认的运行环境

- ROS 主机默认地址为当前 ROS 页面填写的主机，常用地址为 `192.168.0.14`。
- SSH 用户默认为 `wheeltec`。
- `wheeltec@192.168.0.14` 已验证可使用密钥免密登录。
- ROSbridge 不是独立 systemd 服务。
- `/rosbridge_websocket` 当前由
  `/home/wheeltec/wheeltec_robot/src/turn_on_wheeltec_robot/launch/turn_on_wheeltec_robot.launch`
  中包含的 `rosbridge_websocket.launch` 启动。
- 远端 `sudo` 需要密码，因此一键重启不能依赖无交互 `sudo systemctl restart`。
- 现有 ROSbridge 地址为 `0.0.0.0:9090`，并提供 `rosapi`。

## 总体方案

采用统一 ROSbridge 控制器方案。ROS 页面和定位精度页面共享现有
`RosBridgeWorker` 的连接、订阅、节点状态和命令发布能力。定位精度页面不再为
FAST-LIO 里程计额外建立独立 WebSocket 连接。

`MainWindow` 负责协调：

- ROSbridge 主机、端口和连接状态；
- ROS 页面选中的普通数据 topic；
- 定位精度页面填写的 FAST-LIO 里程计 topic；
- 两个页面之间的 topic 双向同步；
- ROSbridge 重启后的断开、等待、重连和订阅恢复；
- `/launch_manager/status` 向 ROS 页面和定位精度页面的分发。

## ROSbridge 一键重启

### 页面交互

ROS 页面连接栏新增：

- `重启 ROSbridge` 按钮；
- ROSbridge 节点状态标签。

按钮仅在以下条件下可用：

- 主机输入有效；
- 当前没有另一轮重启正在执行。

重启期间按钮禁用，状态依次显示“正在通过 SSH 停止”“正在启动”“等待端口”
“正在恢复连接”等阶段。成功后显示在线和恢复结果；失败时显示明确错误，允许再次点击。

### 远端操作

新增独立后台 worker，使用参数数组调用本机 `ssh`，避免阻塞 Qt GUI 和 shell 拼接注入。
默认连接目标为：

```text
wheeltec@<当前 ROS 主机>
```

默认操作分两阶段执行：

1. source ROS Noetic 环境后执行
   `rosnode kill /rosbridge_websocket`；节点已不存在不视为致命错误。
2. 使用 `nohup` 后台启动
   `roslaunch rosbridge_server rosbridge_websocket.launch port:=<当前端口> address:=0.0.0.0`，
   日志写入远端 `/tmp/debug_monitor_rosbridge.log`。

SSH 用户、可执行文件和重启命令允许通过环境变量覆盖，但 GUI 不提供命令输入框，
保持一键操作。

### 本地恢复状态机

1. 保存当前主机、端口、ROS 页面勾选 topic 和 FAST-LIO 里程计 topic。
2. 主动关闭现有 `RosBridgeWorker`。
3. 后台执行 SSH 重启。
4. 轮询 TCP 端口，使用条件等待，不依赖固定睡眠时间。
5. 端口开放后重新启动统一 `RosBridgeWorker`。
6. 等待 WebSocket 连接和 `/rosapi/get_time` 健康探测成功。
7. 恢复普通数据 topic 与 FAST-LIO 里程计 topic 的订阅。
8. 请求一次 `/launch_manager/status`，刷新 FAST-LIO、雷达和校准节点状态。

整个流程设置总超时。SSH 不可用、远端启动失败、端口超时或 WebSocket 健康检查失败时，
保留清晰错误信息，不无限重试。

## ROSbridge 实时健康检测

不能只用“线程仍运行”或“9090 端口开放”判断 ROSbridge 正常。健康状态综合以下信号：

- WebSocket 会话连接状态；
- 每秒调用 `/rosapi/get_time` 的成功或失败；
- 网络往返延迟；
- 最近一次收到任意订阅消息或 `/launch_manager/status` 的时间；
- 连续健康探测失败次数；
- worker 累计错误数。

状态分为：

- `未连接`：没有活动会话；
- `连接中`：已开始连接但尚未完成健康探测；
- `在线`：WebSocket 已连接且最近健康探测成功；
- `数据静默`：健康探测成功，但已订阅数据 topic 在阈值时间内没有消息；
- `异常`：连续多次健康探测失败；
- `重启中`：SSH 重启和恢复状态机正在执行。

单次探测失败不立即判离线，避免网络抖动造成误报。连续失败达到阈值后才显示异常。
状态刷新保持在约 1 Hz，不增加高频 UI 更新。

## FAST-LIO 里程计 topic 同步

### 页面布局

定位精度页面顶部保留 ROSbridge 主机和端口显示，并增加清晰标注的
`FAST-LIO 里程计 topic` 输入框，默认值来自
`DEFAULT_FASTLIO_ODOM_TOPIC`，通常为 `/Odometry`。

ROS 页面数据 topic 区增加对应的可编辑 FAST-LIO 里程计 topic 控件。两个输入框均表示
同一个配置值。

### 同步规则

- 任一页面完成编辑后，另一页面立即显示相同值。
- 输入为空时回退到 `DEFAULT_FASTLIO_ODOM_TOPIC`。
- topic 必须规范化为以 `/` 开头的非空字符串。
- 使用信号阻断防止双向同步递归。
- ROSbridge 在线时，修改后立即更新统一 worker 的订阅集合。
- ROS 页面普通勾选 topic 与 FAST-LIO topic 合并去重。
- FAST-LIO topic 始终作为定位页面工作所需订阅，不依赖 ROS 页复选框是否勾选。
- 连接断开或重启后，恢复最后一次确认的 topic。

统一 worker 收到该 topic 的 `nav_msgs/Odometry` 后：

- 继续发出通用 ROS message 事件；
- 解析为 `LocalizationSample`；
- 直接更新定位精度页面的 buffer、在线状态和图表。

原 `RosOdometryWorker` 暂时保留为独立可测试客户端，但主窗口运行路径不再使用它建立第二条连接。

## 定位精度页雷达直线校准区域

删除原“控制反馈接口预留”组，替换为“雷达直线校准”组。

区域包含：

- `启动校准节点`；
- `停止校准节点`；
- 目标速度设置，默认 `0.2 m/s`，范围与 ROS 页 PID 直行控制一致；
- `前进`；
- `后退`；
- `停止`；
- 节点实时状态标签。

节点启动命令沿用已经存在的配置：

```text
restart pid_control simple_follower pid_control_lidar_assisted.launch imu_topic:=/active_imu lidar_odom_topic:=<FAST-LIO topic>
```

停止命令：

```text
stop pid_control
```

运动控制复用 `/line_follow_control`：

- 前进：`linear_x=<目标速度>, forward=true, backward=false`
- 后退：`linear_x=<目标速度>, forward=false, backward=true`
- 停止：`linear_x=0, forward=false, backward=false`

启动命令中的 `lidar_odom_topic` 使用当前同步后的 FAST-LIO topic，而不是固定写死
`/Odometry`。

### 安全与按钮状态

- ROSbridge 未在线时，启动、停止和运动按钮全部禁用。
- 只有 `/launch_manager/status` 确认校准 variant 正在运行时，前进和后退按钮才可用。
- 节点停止、ROSbridge 断开或页面关闭时，先发送一次停止运动命令。
- PID 普通节点和雷达直线校准节点共用 `pid_control` 标识，保持互斥。
- 启停命令发送后进入 pending 状态，等待真实状态回报，不凭按钮点击直接认定成功。

## 节点状态同步

统一订阅 `/launch_manager/status`，并将结果同时发送给 ROS 页面和定位精度页面。

识别规则：

- `running` 包含 `pid_control` 且 detail 中 launch 为
  `pid_control_lidar_assisted.launch`：校准节点运行；
- `running` 包含 `pid_control` 且为其他 launch：普通 PID 节点运行；
- `running` 包含 `fastlio`：FAST-LIO 节点运行；
- `running` 包含 `lidar`：雷达驱动节点运行。

状态请求时机：

- ROSbridge 健康连接建立后；
- 任一节点启停命令发送后；
- ROSbridge 重启恢复后；
- 正常在线期间按低频定时查询。

## 代码边界

预计涉及：

- `src/app_config.py`
  - SSH 用户、重启超时和可覆盖默认配置。
- 新增 `src/rosbridge_restart_worker.py`
  - SSH 重启、TCP 条件等待和结果报告。
- `src/ros_bridge_worker.py`
  - 动态 FAST-LIO topic、健康状态事件、里程计解析与订阅恢复。
- `src/widgets/ros_panel.py`
  - 重启按钮、健康状态、FAST-LIO topic 控件。
- `src/widgets/localization_panel.py`
  - 使用共享连接、topic 同步信号和雷达直线校准控制区。
- `src/main_window.py`
  - 统一连接、重启状态机、topic 协调、状态分发。
- 对应 `tests/` 单元测试。
- `README.md`
  - 一键重启依赖、环境变量和交互说明。

不会修改远端 ROS 文件，不会创建 systemd 服务，也不会重启整套
`turn_on_wheeltec_robot.launch`。

## 错误处理

- 本机找不到 `ssh`：显示安装或 PATH 提示。
- SSH 免密失败：显示目标用户和主机，不弹出密码输入框。
- `rosnode kill` 找不到节点：继续启动流程。
- 远端 `roslaunch` 启动后立即退出：读取并显示远端日志末尾摘要。
- TCP 已开放但 ROS API 探测失败：判为恢复失败，不误报成功。
- topic 无消息但 ROS API 正常：显示“数据静默”，不把 ROSbridge 判离线。
- 重启流程中再次点击：忽略重复请求。

## 测试策略

按 TDD 增加以下测试：

1. SSH 重启 worker 生成安全、确定的参数列表和远端命令。
2. SSH 失败、启动失败、端口超时和成功路径。
3. ROSbridge 健康状态从连接中、在线、数据静默到异常的转换。
4. 单次探测失败不会立即判离线。
5. 重启成功后自动重连并恢复原订阅。
6. ROS 页面与定位页面 FAST-LIO topic 双向同步，且不会递归。
7. 在线修改 topic 会更新订阅并取消旧的动态订阅。
8. FAST-LIO Odometry 经统一 worker 更新定位 buffer。
9. 定位页校准启停命令使用当前 FAST-LIO topic。
10. 前进、后退和停止发布正确的 `/line_follow_control` 数据。
11. 校准节点未确认运行时禁止运动。
12. `/launch_manager/status` 同步更新两个页面的按钮和标签。
13. 现有 ROS、定位、rosbag 和汇总记录测试保持通过。

完成实现后运行全部单元测试和 `git diff --check`，并进行 Qt offscreen 页面构造检查。
