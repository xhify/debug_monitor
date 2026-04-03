# WHEELTEC C50X 调试串口通信协议

## 目录
- [1. 概述](#1-概述)
- [2. 硬件配置](#2-硬件配置)
- [3. 通用帧格式](#3-通用帧格式)
- [4. TX 数据帧（0x01）](#4-tx-数据帧0x01)
- [5. TX 参数帧（0x02）](#5-tx-参数帧0x02)
- [6. RX 命令帧](#6-rx-命令帧)
- [7. 命令列表](#7-命令列表)
- [8. 校验算法](#8-校验算法)
- [9. 注意事项](#9-注意事项)
- [10. 源码位置索引](#10-源码位置索引)

---

## 1. 概述

调试串口模块通过 UART1 + DMA 实现实时数据流输出和命令接收，用于电机调试和参数调优。

**通信方向：**
- **TX（Robot → PC）：** 100Hz 实时数据帧（T法原始速度、M法原始速度、融合后速度、当前目标、当前输出）+ 按需参数回复帧
- **RX（PC → Robot）：** 命令帧（设置 PID、速度限制、USART1 目标速度、USART1 目标 PWM，查询当前参数）

**关键特性：**
- DMA 非阻塞发送，不影响 Balance_task 实时性
- DMA 忙时丢帧（不缓冲），保证最新数据优先
- 参数修改仅运行时生效，断电不保存
- USART1 的目标速度模式与目标 PWM 模式互斥，后下发的模式会覆盖前一种模式

---

## 2. 硬件配置

| 项目 | 配置 |
|------|------|
| 串口 | USART1 |
| 波特率 | 115200 bps |
| 数据位 | 8 bit |
| 停止位 | 1 bit |
| 校验位 | 无 |
| TX 引脚 | PA9 |
| RX 引脚 | PA10 |
| DMA 通道 | DMA2_Stream7_Channel4（Memory → Peripheral） |
| DMA 模式 | Normal（单次） |
| DMA 优先级 | Medium |
| DMA 完成中断 | DMA2_Stream7_IRQn（抢占优先级 5） |
| UART 接收中断 | USART1_IRQn（抢占优先级 4） |

---

## 3. 通用帧格式

### 3.1 字节序

所有多字节数据均为**小端序**（ARM Cortex-M4 native），float 为 IEEE-754 单精度（32 位）。

### 3.2 TX 帧通用结构

```
[0]     0xAA          帧头1
[1]     0x55          帧头2
[2]     FrameID       帧类型标识 (0x01=数据, 0x02=参数)
[3..N-2] Payload      数据域
[N-1]   Checksum      XOR校验 (bytes 0 ~ N-2)
```

### 3.3 RX 帧通用结构

```
[0]     0xAA          帧头1
[1]     0x55          帧头2
[2]     CmdID         命令ID
[3]     Length         Payload长度 (字节数)
[4..3+Length] Payload  数据域
[4+Length]   Checksum  XOR校验 (bytes 0 ~ 3+Length)
```

---

## 4. TX 数据帧（0x01）

**总长度：** 48 字节
**发送频率：** 100 Hz（由 Balance_task 调用）
**FrameID：** `0x01`

### 字段定义

| 偏移 | 长度 | 类型 | 字段 | 说明 |
|------|------|------|------|------|
| 0 | 1 | uint8 | header1 | 固定 `0xAA` |
| 1 | 1 | uint8 | header2 | 固定 `0x55` |
| 2 | 1 | uint8 | frame_id | 固定 `0x01` |
| 3-6 | 4 | float | t_raw_A | 电机A T法原始速度 (m/s) |
| 7-10 | 4 | float | t_raw_B | 电机B T法原始速度 (m/s) |
| 11-14 | 4 | float | m_raw_A | 电机A M法原始速度 (m/s) |
| 15-18 | 4 | float | m_raw_B | 电机B M法原始速度 (m/s) |
| 19-22 | 4 | float | final_A | 电机A 融合并滤波后的闭环反馈速度 (m/s) |
| 23-26 | 4 | float | final_B | 电机B 融合并滤波后的闭环反馈速度 (m/s) |
| 27-30 | 4 | float | target_A | 电机A 当前目标值 |
| 31-34 | 4 | float | target_B | 电机B 当前目标值 |
| 35-36 | 2 | int16 | output_A | 电机A 当前输出 PWM |
| 37-38 | 2 | int16 | output_B | 电机B 当前输出 PWM |
| 39-42 | 4 | float | afc_output_A | 电机A AFC 增量输出 PWM |
| 43-46 | 4 | float | afc_output_B | 电机B AFC 增量输出 PWM |
| 47 | 1 | uint8 | checksum | XOR(bytes 0..46) |

> **注意：**
> - `t_raw_A/B`、`m_raw_A/B` 仅在 AKM（阿克曼）底盘构建时有效，其他底盘类型固定为 `0.0f`。
> - `final_A/B` 为当前闭环实际使用的速度反馈，已包含 T/M 融合与一阶 Kalman 平滑。
> - `afc_output_A/B` 为 AFC 单独输出的增量 PWM，用于观察学习结果，不包含 PI 本体输出。
> - 在 `SPEED` 模式下，`target_A/B` 单位为 m/s，`output_A/B` 为 PI 与 AFC 叠加后的最终 PWM 输出。
> - 在 `PWM` 模式下，`target_A/B` 直接表示当前动作映射后的目标 PWM 值，`output_A/B` 为实际下发的 PWM 输出；`afc_output_A/B` 会复位为 `0.0f`。

---

## 5. TX 参数帧（0x02）

**总长度：** 40 字节
**发送触发：** 收到 `0x30` 查询命令后，在下一个 100Hz 周期发送
**FrameID：** `0x02`

### 字段定义

| 偏移 | 长度 | 类型 | 字段 | 说明 |
|------|------|------|------|------|
| 0 | 1 | uint8 | header1 | 固定 `0xAA` |
| 1 | 1 | uint8 | header2 | 固定 `0x55` |
| 2 | 1 | uint8 | frame_id | 固定 `0x02` |
| 3-6 | 4 | float | A_kp | 电机A 比例增益 Kp |
| 7-10 | 4 | float | A_ki | 电机A 积分增益 Ki |
| 11-14 | 4 | float | A_kd | 电机A 微分增益 Kd |
| 15-18 | 4 | float | B_kp | 电机B 比例增益 Kp |
| 19-22 | 4 | float | B_ki | 电机B 积分增益 Ki |
| 23-26 | 4 | float | B_kd | 电机B 微分增益 Kd |
| 27-30 | 4 | float | rc_speed | 遥控速度限制 |
| 31-34 | 4 | float | limt_max_speed | 最大速度限制 (m/s) |
| 35-38 | 4 | float | smooth_MotorStep | 电机加速平滑步进 |
| 39 | 1 | uint8 | checksum | XOR(bytes 0..38) |

---

## 6. RX 命令帧

### 帧结构

```
+--------+--------+-------+--------+------------------+----------+
| 0xAA   | 0x55   | CmdID | Length | Payload (0~12B)  | Checksum |
+--------+--------+-------+--------+------------------+----------+
  1 byte   1 byte  1 byte  1 byte    Length bytes        1 byte
```

### 接收状态机

```
WAIT_HEADER1 → (0xAA) → WAIT_HEADER2
    → (0x55): WAIT_CMD → WAIT_LEN
        → Length=0: WAIT_CHECKSUM
        → Length>24: 丢弃,回到WAIT_HEADER1
        → 其他: WAIT_PAYLOAD → 收齐Length字节 → WAIT_CHECKSUM
            → 校验通过: 执行命令
            → 校验失败: 静默丢弃
    → (0xAA): 留在WAIT_HEADER2 (帧头重评估,避免丢失紧跟的有效帧)
    → 其他: 回到WAIT_HEADER1

超时: 约50ms (5个Balance_task周期) 无新字节时,自动复位到WAIT_HEADER1
```

**最大 Payload 长度：** 24 字节（`DEBUG_RX_BUF_LEN`，当前最长命令为 12 字节）

### USART1 目标控制模式

- `0x23` 进入 `SPEED` 模式，设置 AKM A/B 两轮目标速度
- `0x24` 进入 `PWM` 模式，设置 AKM A/B 两轮目标 PWM
- 两种模式互斥，后收到的新模式会清理前一种模式的运行状态
- 这两个命令只负责“设定目标值”，实际动作方向仍由蓝牙 `UART4` 的 APP 命令触发

---

## 7. 命令列表

### 7.1 设置两轮 PID（0x10）

同时设置电机 A 和电机 B 的 PID 参数为相同值。

| 字段 | 值 |
|------|-----|
| CmdID | `0x10` |
| Length | `12` |
| Payload | `Kp(float)` + `Ki(float)` + `Kd(float)` |

**效果：** 同步更新 `PI_MotorA`、`PI_MotorB` 以及全局 `robot.V_KP`、`robot.V_KI`。

### 7.2 设置电机 A PID（0x11）

仅设置电机 A 的 PID 参数。

| 字段 | 值 |
|------|-----|
| CmdID | `0x11` |
| Length | `12` |
| Payload | `Kp(float)` + `Ki(float)` + `Kd(float)` |

### 7.3 设置电机 B PID（0x12）

仅设置电机 B 的 PID 参数。

| 字段 | 值 |
|------|-----|
| CmdID | `0x12` |
| Length | `12` |
| Payload | `Kp(float)` + `Ki(float)` + `Kd(float)` |

### 7.4 设置遥控速度限制（0x20）

| 字段 | 值 |
|------|-----|
| CmdID | `0x20` |
| Length | `4` |
| Payload | `Value(float)` |
| 有效范围 | `0 < Value ≤ 10000.0` |

超出范围的值会被固件静默拒绝。

### 7.5 设置最大速度限制（0x21）

| 字段 | 值 |
|------|-----|
| CmdID | `0x21` |
| Length | `4` |
| Payload | `Value(float)` |
| 有效范围 | `0 < Value ≤ 10.0` (m/s) |

### 7.6 设置电机加速平滑步进（0x22）

| 字段 | 值 |
|------|-----|
| CmdID | `0x22` |
| Length | `4` |
| Payload | `Value(float)` |
| 有效范围 | `0 < Value ≤ 1.0` |

### 7.7 查询当前参数（0x30）

| 字段 | 值 |
|------|-----|
| CmdID | `0x30` |
| Length | `0` |
| Payload | 无 |

**效果：** 固件在下一个 100Hz 周期回复一帧参数帧（0x02）。

### 7.8 设置 A/B 两轮目标速度（0x23）

该命令仅用于 AKM 底盘，设置 USART1 的左右轮目标速度。进入该模式后，蓝牙 APP 继续负责动作方向，固件按 APP 方向组合这两个目标值，再走闭环控制。

| 字段 | 值 |
|------|-----|
| CmdID | `0x23` |
| Length | `8` |
| Payload | `speed_a(float)` + `speed_b(float)` |
| 有效范围 | `0 ≤ Value ≤ 10.0` (m/s) |

**说明：**
- `SPEED` 模式与 `PWM` 模式互斥。
- `A/E` 使用 `speed_a/speed_b` 的正反向组合作为左右轮目标。
- `B/H/D/F` 通过交换或翻转 A/B 目标实现左右转方向组合。
- `C/G` 在 `SPEED` 模式下使用左右轮反向目标进行原地调试转向。

### 7.9 设置 A/B 两轮目标 PWM（0x24）

该命令仅用于 AKM 底盘，设置 USART1 的左右轮目标 PWM。进入该模式后，蓝牙 APP 继续负责动作方向，固件按 APP 方向组合这两个目标值，再直接输出 PWM，不经过 PI。

| 字段 | 值 |
|------|-----|
| CmdID | `0x24` |
| Length | `4` |
| Payload | `pwm_a(int16)` + `pwm_b(int16)` |
| 有效范围 | `0 ≤ Value ≤ 16800` |

**说明：**
- `PWM` 模式与 `SPEED` 模式互斥。
- `A/E` 使用 `pwm_a/pwm_b` 的正反向组合作为左右轮输出。
- `B/H/D/F` 通过交换或翻转 A/B 输出实现左右转方向组合。
- `C/G` 在 `PWM` 模式下等同停止，收到后清零 A/B 电机输出。

---

## 8. 校验算法

TX 和 RX 均使用**逐字节 XOR** 校验。

```
checksum = 0
for byte in frame[0 .. N-2]:
    checksum ^= byte
frame[N-1] = checksum
```

- **TX 数据帧：** XOR(bytes 0..46)，存入 byte[47]
- **TX 参数帧：** XOR(bytes 0..38)，存入 byte[39]
- **RX 命令帧：** XOR(bytes 0..3+Length)，与接收的 checksum 字节比较

---

## 9. 注意事项

1. **非阻塞发送：** DMA 忙时当前帧被丢弃，不排队缓冲，保证发送的始终是最新数据
2. **参数不持久化：** 通过命令修改的参数仅运行时有效，断电后恢复为初始值
3. **PID 参数校验：** 固件对 Kp/Ki/Kd 做范围校验（`0 ≤ val ≤ 50000`），NaN/Inf 及超出范围的值会被静默拒绝，整条命令丢弃
4. **参数帧优先：** 有参数回复请求时，该周期会发送参数帧替代数据帧（不会同时发送两帧）
5. **底盘差异：** 数据帧中 `t_raw_A/B`、`m_raw_A/B` 字段仅 AKM 底盘有效，其他底盘类型为 0
6. **AKM 混合测速：** AKM 车型的 `final_A/B` 为低速 T 法、中高速 M 法和过渡区融合后的最终反馈，不再等同于单一 T 法输出
7. **RX Payload 上限：** 最大 24 字节（`DEBUG_RX_BUF_LEN`），超出长度的帧会被静默丢弃
8. **RX 超时恢复：** 状态机在约 50ms（5 个 Balance_task 周期）内未收到新字节时自动复位，防止卡在半帧状态
9. **无逐帧 ACK：** USART1 不为每条 RX 命令发送独立确认帧；上位机应通过周期性数据帧或查询参数帧观察执行结果
10. **协议变更：** `0x01` 数据帧已扩展到 48 字节，上位机解析偏移与校验范围需要同步更新
11. **带宽估算：** 115200bps 下有效吞吐约 `11520B/s`；现有 `48B * 100Hz = 4800B/s`，新增目标控制命令按 `13B * 100Hz ≈ 1300B/s` 估算，总占用约 `6100B/s`，约为链路的 `53%`

---

## 10. 源码位置索引

| 文件 | 说明 |
|------|------|
| `HARDWARE/debug_uart.c` | 模块实现（DMA 初始化、帧组装、RX 状态机、命令执行） |
| `HARDWARE/Inc/debug_uart.h` | 接口声明与协议常量定义 |
| `BALANCE/balance_task.c` | TX 调用点（Balance_task 中 100Hz 调用 `Debug_SendDataFrame()`） |
| `HARDWARE/uartx_callback.c` | RX 调用点（USART1 中断中调用 `Debug_ProcessRxByte()`） |
| `USER/system.c` | 初始化调用（`Debug_UART_DMA_Init()`） |
