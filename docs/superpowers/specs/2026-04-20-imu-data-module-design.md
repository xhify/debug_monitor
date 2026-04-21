# IMU Data Module Design

## Goal

在现有 WHEELTEC C50X 调试上位机中新增一个独立 IMU 数据接收采集模块。IMU 模块与当前编码器调试模块处于同一层级，通过窗口顶部模块切换按钮在两者之间切换，二者不混在同一个操作界面中。

## Scope

首版实现双 IMU 同时接收采集：

- YESENSE `0x59 0x53` UART 输出帧解析。
- IMU A / IMU B 两路串口独立连接、断开、实时接收。
- 同时显示两路加速度、角速度、欧拉角、磁力计、四元数、温度、序号和时间戳。
- 实时曲线显示加速度、角速度、欧拉角。
- 一个“开始记录”按钮同时记录 A/B 两路数据。
- 停止记录后保存会话目录：`imu_A.csv`、`imu_B.csv`、`session.json`、`merged_aligned.csv`。
- 顶部模块切换：`编码器` 和 `IMU`。

首版不发送 IMU 配置命令，不嵌入 `D:/radar/car/IMU-data` 的 Tkinter UI。

## Architecture

现有编码器调试界面保留其 Qt/PySide6 实现。主窗口改为顶层模块容器：顶部放模块切换按钮，下面使用 `QStackedWidget` 承载编码器页面和 IMU 页面。编码器页面复用现有布局和逻辑，IMU 页面是独立 QWidget。

IMU 数据链路拆成无 Qt 协议层、串口线程、缓冲与录制、Qt 页面四层：

- `imu_protocol.py`：纯数据模块，定义 `ImuSample`、YESENSE 校验、增量解析器和 CSV 行转换。
- `imu_buffer.py`：线程安全环形缓冲，供串口线程写入、UI 定时读取。
- `imu_recording.py`：单 IMU CSV 流式记录，以及双 IMU 会话目录记录和停止后对齐导出。
- `imu_serial_worker.py`：QThread 串口采集，解析样本并写入缓冲。
- `widgets/imu_panel.py`：独立 IMU 模块页面，负责连接控件、曲线、数值面板和记录按钮。
- `main_window.py`：只负责模块容器和生命周期协调。

## Protocol Decisions

按集成说明中的官方协议语义实现：

- 帧头为 `0x59 0x53`。
- 总帧长为 `payload_len + 7`。
- CK1/CK2 校验范围为 `packet[2:-2]`。
- `0x01` 温度按 signed int16 解析并乘 `0.01`，字段名为 `temperature_c`。
- `0x10` 加速度按 signed int32、`1e-6` 缩放，单位 `m/s^2`。
- `0x20` 角速度按 signed int32、`1e-6` 缩放，单位 `deg/s`。
- `0x31` 磁力计按 signed int32、`1e-3` 缩放，单位 `mGauss`。
- `0x40` 欧拉角按手册顺序 `pitch, roll, yaw` 解析，并在样本字段中明确命名。
- `0x41` 四元数按 `q0, q1, q2, q3` 解析为 `quat_w/x/y/z`。
- `0x51` 和 `0x52` 作为 unsigned int32 微秒时间戳保存。
- 未识别 block 忽略，坏校验和坏长度帧丢弃。

## UI Behavior

窗口顶部提供两个互斥按钮：

- `编码器`：显示现有编码器调试功能。
- `IMU`：显示 IMU 接收采集功能。

IMU 页面包含：

- IMU A / IMU B 两个串口连接区：端口、刷新、波特率、连接、断开、状态。
- 曲线区：加速度、角速度、欧拉角三组曲线，每组同时显示 A/B 两路。
- 控制行：清空数据、开始/停止记录、暂停绘图。
- 数值区：A/B 两列显示序号、温度、device/sync 时间戳、加速度、角速度、欧拉角和帧数。

切换模块不自动断开已连接串口。关闭主窗口时同时关闭编码器和 IMU 串口线程，停止未保存录制。

## Testing

测试覆盖：

- YESENSE 校验计算。
- 解析单帧、多帧、半帧、噪声前缀。
- 坏 CK1/CK2 丢弃。
- 温度、角速度缩放、欧拉角字段顺序。
- IMU 缓冲快照、清空、记录绑定。
- 双 IMU 会话目录写入、取消、A/B 对齐导出。
- 主窗口存在模块切换控件，且能切换到 IMU 页面。
