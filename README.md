# WHEELTEC C50X 调试监视器

实时电机调试上位机软件，通过串口接收 100Hz 数据流，可视化绘图并支持 PID 参数在线调整。

---

## 1. 环境准备

### 1.1 安装 Python

需要 Python 3.10 或更高版本。下载地址：https://www.python.org/downloads/

安装时勾选 **"Add Python to PATH"**。

验证安装：

```bash
python --version
```

### 1.2 安装 uv

uv 是一个高性能的 Python 包管理器，用于创建虚拟环境和安装依赖。

**Windows（PowerShell）：**

```powershell
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
```

**Windows（pip 方式）：**

```bash
pip install uv
```

**Linux / macOS：**

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

验证安装：

```bash
uv --version
```

### 1.3 创建虚拟环境并安装依赖

```bash
cd debug_monitor

# 创建虚拟环境
uv venv .venv

# 激活虚拟环境
# Windows (Git Bash / MSYS2):
source .venv/Scripts/activate
# Windows (CMD):
.venv\Scripts\activate.bat
# Windows (PowerShell):
.venv\Scripts\Activate.ps1
# Linux / macOS:
source .venv/bin/activate

# 安装所有依赖
uv pip install -r requirements.txt
```

## 2. 启动程序

确保虚拟环境已激活，然后运行：

```bash
python src/main.py
```

## 3. 硬件连接

1. 使用 USB 转 TTL 模块连接机器人调试串口（UART1）
   - TX（PA9）→ 转换模块 RX
   - RX（PA10）→ 转换模块 TX
   - GND → GND
2. 将 USB 转 TTL 模块插入电脑 USB 口
3. 记下设备管理器中显示的 COM 端口号（如 COM3）

## 4. 使用说明

### 4.1 界面布局

```
+—— 顶部：串口连接 ——————————————————————————————————+
|                              |  实时数值              |
|  速度曲线图（6条曲线）        |  （原始/滤波/目标速度） |
|  PWM 输出图（2条曲线）        |  固件参数              |
|  [暂停] [开始记录]           |  （PID/速度限制等）     |
+——————————————————————————————+————————————————————————+
|  PID 设置                    |  速度参数（当前不可用） |
|  (两轮/A/B) Kp Ki Kd [发送]  |  [查询参数]           |
+——————————————————————————————+————————————————————————+
|  状态栏：帧数 | 错误数 | 记录状态                      |
+———————————————————————————————————————————————————————+
```

### 4.2 连接串口

1. 在顶部面板的 **端口** 下拉框选择对应 COM 口（如 `COM3 - USB Serial Port`）
2. **波特率** 保持默认 `115200`
3. 点击 **连接**

连接成功后状态栏显示"已连接"，绘图区开始实时滚动。

如果端口列表为空，点击 **刷新** 重新扫描。

### 4.3 实时绘图

连接后自动开始绘制，包含两个子图：

**速度图（上）— 6 条曲线：**
| 曲线 | 颜色 | 线型 |
|------|------|------|
| 原始速度 A | 红色 | 虚线 |
| 原始速度 B | 深红 | 点划线 |
| 滤波速度 A | 绿色 | 实线（粗）|
| 滤波速度 B | 深绿 | 虚线（粗）|
| 目标速度 A | 蓝色 | 实线 |
| 目标速度 B | 深蓝 | 虚线 |

**PWM 输出图（下）— 2 条曲线：**
| 曲线 | 颜色 |
|------|------|
| PWM A | 橙色 |
| PWM B | 紫色 |

**交互操作：**
- **鼠标滚轮**：缩放
- **鼠标左键拖拽**：平移
- **鼠标右键**：弹出菜单，可重置视图（Auto Range）
- **暂停绘图** 勾选框：冻结曲线显示，但数据仍在后台接收

### 4.4 查看实时数据

右侧 **实时数据** 面板以数值形式显示当前最新一帧：
- 原始速度 A/B（m/s）
- 滤波速度 A/B（m/s）
- 目标速度 A/B（m/s）
- PWM 输出 A/B
- 帧率（Hz）

### 4.5 调整 PID 参数

1. 选择目标：**两轮同步** / **电机 A** / **电机 B**
2. 填写 **Kp**、**Ki**、**Kd** 值
3. 点击 **设置 PID** 发送

> 建议先点击 **查询参数** 获取当前固件参数值，程序会自动回填到输入框中，在此基础上微调。

> 注意：PID 参数修改仅运行时生效，机器人断电后恢复为默认值。

### 4.6 查询固件参数

点击 **查询参数** 按钮，右侧 **固件参数** 面板将显示：
- 电机 A/B 的 Kp、Ki、Kd
- 遥控速度限制
- 最大速度限制
- 电机加速平滑步进

### 4.7 数据记录（CSV 导出）

1. 点击 **开始记录**（按钮变红），数据开始在内存中缓存
2. 需要时点击 **停止记录**，弹出保存对话框
3. 选择保存路径和文件名，数据一次性写入 CSV 文件
4. 如果取消保存对话框，记录的数据将被丢弃

CSV 文件包含以下列：

| 列名 | 说明 |
|------|------|
| frame_index | 帧序号 |
| time_s | 时间（秒） |
| raw_speed_a / raw_speed_b | 原始编码器速度 |
| filtered_a / filtered_b | 滤波后速度 |
| target_a / target_b | 目标速度 |
| output_a / output_b | PWM 输出 |

### 4.8 清空数据

点击 **清空数据** 按钮可清除缓冲区中的所有数据，图表和数值面板会立即恢复为空白状态。如果正在记录数据，会先自动停止记录（数据丢弃）再清空。

### 4.9 断开连接

点击 **断开** 按钮。如果正在记录数据，程序会自动停止记录并保存文件。

关闭窗口时也会自动清理连接和记录。

## 5. 常见问题

**Q: 端口列表中找不到我的串口？**
A: 确认 USB 转 TTL 模块已插入并安装驱动（CH340/CP2102/FT232R），然后点击"刷新"。

**Q: 连接后没有数据？**
A: 检查接线（TX/RX 是否交叉连接）、波特率是否为 115200、机器人是否已上电运行。

**Q: 绘图卡顿？**
A: 程序已内置降采样和视口裁剪优化，正常情况下不会卡顿。如仍有问题，尝试缩小窗口或减少显示的曲线数量（在图例中点击曲线名可隐藏）。

**Q: 速度参数设置按钮为什么是灰色的？**
A: 当前固件版本不支持通过调试串口下发速度参数命令，此功能已禁用。PID 参数设置和参数查询功能正常可用。

## 6. 项目结构

```
debug_monitor/
├── README.md                    # 本文件
├── CLAUDE.md                    # 项目开发说明
├── requirements.txt             # Python 依赖
├── docs/
│   └── Debug_UART_Protocol.md   # 串口通信协议文档
└── src/
    ├── main.py                  # 程序入口
    ├── protocol.py              # 协议解析与命令组装
    ├── serial_worker.py         # 串口通信线程
    ├── data_buffer.py           # 数据缓冲区与 CSV 记录
    ├── main_window.py           # 主窗口
    └── widgets/                 # 界面组件
        ├── serial_panel.py      # 串口连接面板
        ├── plot_panel.py        # 实时绘图
        ├── data_panel.py        # 数值显示
        ├── command_panel.py     # 命令发送
        └── param_panel.py       # 参数显示
```

## 7. 技术规格

| 项目 | 规格 |
|------|------|
| 串口协议 | UART 115200 bps, 8N1 |
| 数据帧 | 32 字节, 100Hz, 帧头 0xAA 0x55 |
| 参数帧 | 40 字节, 按需触发 |
| 校验方式 | 逐字节 XOR |
| 字节序 | 小端序 (Little-Endian) |
| 绘图刷新率 | ~30 fps |
| 数据缓冲 | 3000 点（约 30 秒） |

详细协议说明见 [docs/Debug_UART_Protocol.md](docs/Debug_UART_Protocol.md)。
