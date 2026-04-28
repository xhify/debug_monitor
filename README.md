# WHEELTEC C50X 调试监视器

WHEELTEC C50X 多底盘机器人调试上位机。程序用于串口电机调试、双 IMU 采集、ROS 数据监视、ROS IMU 可视化，以及编码器/IMU/ROS/雷达的同步实验记录。

主要能力：

- 编码器调试串口实时接收、绘图、统计分析、CSV 记录与回放
- PID、速度限制、USART1 目标速度、目标 PWM 在线下发
- 双 YESENSE IMU 独立串口采集、曲线显示和会话记录
- rosbridge 连接 `/odom`、`/imu`、`/active_imu`、`/PowerVoltage`
- 发布 `/cmd_vel` 和 `/line_follow_control`
- 汇总模块统一记录编码器、IMU、ROS 和可选雷达数据

## 1. 环境准备

### 1.1 Python

建议使用 Python 3.11 或更高版本。

```bash
python --version
```

### 1.2 uv

项目使用 uv 管理虚拟环境和依赖。

Windows PowerShell：

```powershell
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
```

也可以通过 pip 安装：

```bash
pip install uv
```

验证：

```bash
uv --version
```

### 1.3 创建环境

```bash
cd debug_monitor
uv venv .venv

# Windows PowerShell
.venv\Scripts\Activate.ps1

# Windows CMD
.venv\Scripts\activate.bat

# Git Bash / MSYS2
source .venv/Scripts/activate

uv pip install -r requirements.txt
```

## 2. 启动

```bash
python src/main.py
```

程序启动后顶部可以切换模块：

- **汇总**：统一配置编码器、IMU A、IMU B 来源并同步记录
- **编码器**：调试串口实时绘图、命令下发、CSV 记录/回放
- **IMU**：两路 YESENSE IMU 串口采集和记录
- **ROS**：rosbridge 数据监视、速度命令和 ROS CSV 记录
- **ROS IMU**：`/imu` 与 `/active_imu` 双 IMU 曲线监视

## 3. 硬件与外部服务

### 3.1 编码器调试串口

使用 USB 转 TTL 模块连接机器人 UART1：

| 机器人 | USB 转 TTL |
|--------|------------|
| TX PA9 | RX |
| RX PA10 | TX |
| GND | GND |

默认参数：`115200 bps, 8N1`。

### 3.2 YESENSE IMU 串口

IMU 模块支持 IMU A / IMU B 两路独立串口。常用波特率列表包含：

```text
460800, 230400, 115200, 57600, 38400, 19200, 9600, 921600
```

### 3.3 ROS / rosbridge

ROS 功能通过 rosbridge WebSocket 连接，默认地址：

```text
host: 192.168.0.14
port: 9090
```

订阅主题：

| 主题 | 类型 | 用途 |
|------|------|------|
| `/odom` | `nav_msgs/Odometry` | 轮速、角速度、位姿 |
| `/imu` | `sensor_msgs/Imu` | ROS IMU A |
| `/active_imu` | `sensor_msgs/Imu` | ROS 活动 IMU |
| `/PowerVoltage` | `std_msgs/Float32` | 电压 |

发布主题：

| 主题 | 类型 | 用途 |
|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 速度控制 |
| `/line_follow_control` | `simple_follower/LineFollowControl` | PID 直行控制 |

### 3.4 雷达同步

汇总模块可以通过本机 TCP SCPI 控制雷达软件录制：

```text
host: 127.0.0.1
port: 5026
identify: *IDN?
start: MEMMory:RECord:STARt <filename>
stop: MEMM:REC:STOP
```

只有点击 **测试雷达连接** 并收到 `PHASELOCK...` 识别响应后，才会启用 **同步雷达录制**。

## 4. 编码器模块

编码器模块负责调试串口数据流：

- 解析 48 字节数据帧，约 100Hz
- 实时显示 T 法、M 法、融合速度、目标速度、PWM 和 AFC 输出
- 上方速度图显示 8 条速度曲线
- 下方 PWM/AFC 图显示电机 A/B 输出
- 支持暂停绘图、清空数据、CSV 记录和 CSV 回放
- 支持统计分析最近 10 秒实时窗口或当前回放窗口

命令区支持：

- 两轮同步或单轮独立设置 PID
- 查询固件参数
- 设置遥控速度限制、最大速度、平滑步进
- 下发 USART1 目标速度，协议 `0x23`
- 下发 USART1 目标 PWM，协议 `0x24`

CSV 记录字段包含：

```text
frame_index, time_s,
t_raw_a, t_raw_b,
m_raw_a, m_raw_b,
final_a, final_b,
target_a, target_b,
output_a, output_b,
afc_output_a, afc_output_b
```

回放流程：

1. 点击 **加载 CSV**
2. 模式切换为 **回放**
3. 使用播放/暂停、进度条和 `0.5x / 1x / 2x` 浏览

回放不会停止后台串口接收，也不会影响继续录制实时数据。

## 5. IMU 模块

IMU 模块用于两路 YESENSE IMU 独立采集：

- IMU A / IMU B 各自选择端口、波特率、连接状态
- 曲线显示加速度、角速度、欧拉角
- 支持暂停绘图和清空数据
- 一键开始/停止双 IMU 会话记录

独立 IMU 记录默认写入 `recordings/imu_session_<timestamp>/`：

| 文件 | 说明 |
|------|------|
| `imu_A.csv` | IMU A 原始样本 |
| `imu_B.csv` | IMU B 原始样本 |
| `merged_aligned.csv` | A/B 时间对齐后的合并数据 |
| `session.json` | 设备配置、行数、备注和对齐参数 |

## 6. ROS 模块

ROS 模块用于 rosbridge 数据监视和控制：

- 显示 `/odom` 中的左右轮速度、角速度和位姿
- 显示 `/imu` 基础 IMU 字段和 `/PowerVoltage` 电压
- 绘制实际左右轮速度与目标左右轮速度
- 可发布 `/cmd_vel`
- 可发布 `/line_follow_control` 的 PID 前进、后退、停止控制
- 可单独将 ROS 快照记录为 CSV

## 7. ROS IMU 模块

ROS IMU 模块专门对比 `/imu` 和 `/active_imu`：

- 同屏显示 Acc X/Y/Z、Gyro X/Y/Z、Roll/Pitch/Yaw 九个子图
- `/imu` 使用实线，`/active_imu` 使用虚线
- 支持暂停绘图、清空数据、显示原始数据
- 支持 50 ms、100 ms、200 ms、500 ms 平滑窗口
- 右侧显示两路 IMU 当前数值和帧数

## 8. 汇总模块

汇总模块用于一次实验中统一记录多源数据。可配置三类设备：

| 设备 | 可选来源 |
|------|----------|
| 编码器 | 串口、ROS `/odom` |
| IMU A | 串口、ROS `/imu`、ROS `/active_imu` |
| IMU B | 串口、ROS `/imu`、ROS `/active_imu` |

使用流程：

1. 为编码器、IMU A、IMU B 选择来源
2. 串口来源需要选择端口和波特率并连接
3. ROS 来源会使用 ROS 模块中的 rosbridge 地址
4. 可填写实验备注
5. 如需雷达同步，先测试雷达连接，再勾选 **同步雷达录制**
6. 点击 **全部开始记录**
7. 点击 **全部停止记录** 保存会话

汇总记录默认写入：

```text
recordings/session_<timestamp>/
```

可能生成的文件：

| 文件 | 条件 | 说明 |
|------|------|------|
| `session.json` | 总是生成 | 汇总元数据、设备来源、文件清单、雷达信息 |
| `encoder.csv` | 编码器来源为串口 | 编码器调试串口数据 |
| `imu_A.csv` | 任一 IMU 来源为串口 | IMU A 串口样本 |
| `imu_B.csv` | 任一 IMU 来源为串口 | IMU B 串口样本 |
| `imu_session.json` | 任一 IMU 来源为串口 | 串口 IMU 会话元数据 |
| `imu_merged_aligned.csv` | 任一 IMU 来源为串口 | 串口 IMU A/B 对齐数据 |
| `ros_odom.csv` | 编码器来源为 ROS `/odom` | ROS 里程计数据 |
| `ros_imu.csv` | 任一 IMU 来源为 ROS IMU | ROS `/imu` 原始数据 |
| `ros_active_imu.csv` | 任一 IMU 来源为 ROS IMU | ROS `/active_imu` 原始数据 |
| `ros_imu_merged_aligned.csv` | 任一 IMU 来源为 ROS IMU | ROS 双 IMU 对齐数据 |

如果启用雷达同步，雷达软件会使用同一时间戳生成 `.bin` 记录文件，文件名写入 `session.json` 的 `devices.radar.filename`。

## 9. 项目结构

```text
debug_monitor/
├── README.md
├── AGENTS.md
├── CLAUDE.md
├── requirements.txt
├── docs/
│   └── Debug_UART_Protocol.md
├── src/
│   ├── main.py
│   ├── main_window.py
│   ├── protocol.py
│   ├── serial_worker.py
│   ├── data_buffer.py
│   ├── recording_session.py
│   ├── replay_data.py
│   ├── analytics.py
│   ├── imu_protocol.py
│   ├── imu_serial_worker.py
│   ├── imu_buffer.py
│   ├── imu_recording.py
│   ├── ros_bridge_worker.py
│   ├── ros_data.py
│   ├── radar_scpi.py
│   └── widgets/
│       ├── serial_panel.py
│       ├── plot_panel.py
│       ├── data_panel.py
│       ├── analysis_panel.py
│       ├── command_panel.py
│       ├── param_panel.py
│       ├── imu_panel.py
│       ├── ros_panel.py
│       └── ros_imu_panel.py
└── tests/
```

## 10. 技术规格

| 项目 | 规格 |
|------|------|
| 编码器串口 | UART 115200 bps, 8N1 |
| 编码器数据帧 | 48 字节，约 100Hz，帧头 `0xAA 0x55` |
| 参数帧 | 40 字节，按需触发 |
| 校验 | 逐字节 XOR |
| 字节序 | 小端序 |
| 绘图刷新 | 约 30 fps |
| 编码器实时缓冲 | 3000 点，约 30 秒 |
| IMU 实时缓冲 | 3000 点 |
| ROS 连接 | rosbridge WebSocket |
| 雷达控制 | TCP SCPI，默认 `127.0.0.1:5026` |

串口协议详情见 [docs/Debug_UART_Protocol.md](docs/Debug_UART_Protocol.md)。

## 11. 常见问题

**端口列表中找不到串口怎么办？**

确认 USB 转 TTL 或 IMU 串口模块已插入并安装驱动，然后点击 **刷新**。

**编码器连接后没有数据怎么办？**

检查 TX/RX 是否交叉连接、GND 是否共地、波特率是否为 `115200`、机器人是否已上电运行。

**ROS 模块连接失败怎么办？**

确认机器人端 rosbridge 已启动，电脑能访问对应 IP 和端口，并且防火墙没有阻断 WebSocket 连接。

**雷达同步无法勾选怎么办？**

必须先点击 **测试雷达连接**，并让雷达控制软件在 `127.0.0.1:5026` 响应 `PHASELOCK...` 识别信息。

**T 法或 M 法速度一直为 0 正常吗？**

非 AKM 底盘可能不会回传这些原始测速字段，显示 0 属于协议兼容行为。
