# Debug Monitor 上位机

## 项目概述

WHEELTEC C50X 多底盘机器人的调试上位机软件。通过串口实时接收电机调试数据，进行可视化绘图和数据分析，同时支持下发命令调整 PID 参数和控制参数。

**配套固件项目：** `WHEELTEC_C50X_2025.08.07`（STM32F407VE，调试串口模块通过 UART1 发送数据）

## 技术栈

- **语言：** Python 3.11+
- **GUI 框架：** PySide6（Qt6 官方 Python 绑定）
- **串口通信：** pyserial
- **实时绘图：** pyqtgraph（高性能，适合 100Hz 数据流）
- **数据处理：** numpy, pandas
- **数据导出：** CSV（pandas 内置）

## 串口协议

详见 [docs/Debug_UART_Protocol.md](docs/Debug_UART_Protocol.md)

**关键参数：**
- 接口：UART，115200 bps，8N1
- TX 数据帧：32 字节，100Hz（电机速度、目标、PWM 输出）
- TX 参数帧：40 字节，按需（PID 增益、速度限制等）
- RX 命令帧：变长（设置 PID、速度限制、查询参数）
- 校验：逐字节 XOR
- 字节序：小端序（ARM Cortex-M4 native）
- 帧头：0xAA 0x55

## 功能需求

### 核心功能
1. **串口连接** — 选择端口、波特率，连接/断开控制
2. **实时数据显示** — 解析 32 字节数据帧，表格或数值面板展示当前值
3. **实时绘图** — 多通道曲线（原始速度、滤波速度、目标速度、PWM 输出），支持缩放、暂停
4. **数据记录** — 将接收到的数据保存为 CSV 文件，支持开始/停止记录
5. **命令发送** — 界面控件发送命令帧：
   - 设置 PID 参数（两轮同步 / 单轮独立）
   - 设置遥控速度限制、最大速度、平滑步进
   - 查询当前参数
6. **参数显示** — 解析 40 字节参数帧，展示当前固件参数

### 扩展功能（可选）
- 数据回放（加载 CSV 重新绘图）
- 简单统计分析（均值、标准差、响应时间）

## 环境管理

使用 **uv** 管理 Python 虚拟环境和依赖：

```bash
# 创建虚拟环境
uv venv .venv

# 激活虚拟环境
source .venv/Scripts/activate    # Windows (Git Bash / MSYS2)
# .venv\Scripts\activate.bat    # Windows (CMD)
# .venv\Scripts\Activate.ps1    # Windows (PowerShell)

# 安装依赖
uv pip install -r requirements.txt

# 运行程序
python src/main.py
```

## 项目结构

```
debug_monitor/
├── AGENTS.md
├── requirements.txt
├── .venv/                       # uv 虚拟环境（不提交到 git）
├── docs/
│   └── Debug_UART_Protocol.md
└── src/
    ├── main.py                  # 入口点
    ├── protocol.py              # 协议解析/组装（纯数据，无 Qt 依赖）
    ├── serial_worker.py         # QThread 串口通信线程
    ├── data_buffer.py           # 线程安全 numpy 环形缓冲区
    ├── main_window.py           # 主窗口布局与信号编排
    └── widgets/
        ├── __init__.py
        ├── serial_panel.py      # 串口连接面板
        ├── plot_panel.py        # pyqtgraph 实时绘图
        ├── data_panel.py        # 实时数值显示
        ├── command_panel.py     # 命令发送面板
        └── param_panel.py       # 固件参数显示
```

## 编码约定

- 文件编码：UTF-8
- 注释语言：中文
- 代码风格：PEP 8
- 类型注解：推荐使用（Python type hints）
