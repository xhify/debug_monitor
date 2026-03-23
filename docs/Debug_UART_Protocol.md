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
- **TX（Robot → PC）：** 100Hz 实时数据帧（编码器速度、目标速度、PID 输出）+ 按需参数回复帧
- **RX（PC → Robot）：** 命令帧（设置 PID、速度限制等参数，查询当前参数）

**关键特性：**
- DMA 非阻塞发送，不影响 Balance_task 实时性
- DMA 忙时丢帧（不缓冲），保证最新数据优先
- 参数修改仅运行时生效，断电不保存

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

**总长度：** 32 字节
**发送频率：** 100 Hz（由 Balance_task 调用）
**FrameID：** `0x01`

### 字段定义

| 偏移 | 长度 | 类型 | 字段 | 说明 |
|------|------|------|------|------|
| 0 | 1 | uint8 | header1 | 固定 `0xAA` |
| 1 | 1 | uint8 | header2 | 固定 `0x55` |
| 2 | 1 | uint8 | frame_id | 固定 `0x01` |
| 3-6 | 4 | float | raw_speed_A | 电机A T法原始编码器速度 (m/s) |
| 7-10 | 4 | float | raw_speed_B | 电机B T法原始编码器速度 (m/s) |
| 11-14 | 4 | float | filtered_A | 电机A Kalman滤波后速度 (m/s) |
| 15-18 | 4 | float | filtered_B | 电机B Kalman滤波后速度 (m/s) |
| 19-22 | 4 | float | target_A | 电机A 目标速度 (m/s) |
| 23-26 | 4 | float | target_B | 电机B 目标速度 (m/s) |
| 27-28 | 2 | int16 | output_A | 电机A PID输出 (PWM值) |
| 29-30 | 2 | int16 | output_B | 电机B PID输出 (PWM值) |
| 31 | 1 | uint8 | checksum | XOR(bytes 0..30) |

> **注意：** `raw_speed_A/B` 仅在 AKM（阿克曼）底盘构建时有效，其他底盘类型固定为 `0.0f`。

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
WAIT_HEADER1 → (0xAA) → WAIT_HEADER2 → (0x55) → WAIT_CMD → WAIT_LEN
    → Length=0: WAIT_CHECKSUM
    → Length>12: 丢弃,回到WAIT_HEADER1
    → 其他: WAIT_PAYLOAD → 收齐Length字节 → WAIT_CHECKSUM
        → 校验通过: 执行命令
        → 校验失败: 静默丢弃
```

**最大 Payload 长度：** 12 字节（3 个 float）

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

---

## 8. 校验算法

TX 和 RX 均使用**逐字节 XOR** 校验。

```
checksum = 0
for byte in frame[0 .. N-2]:
    checksum ^= byte
frame[N-1] = checksum
```

- **TX 数据帧：** XOR(bytes 0..30)，存入 byte[31]
- **TX 参数帧：** XOR(bytes 0..38)，存入 byte[39]
- **RX 命令帧：** XOR(bytes 0..3+Length)，与接收的 checksum 字节比较

---

## 9. 注意事项

1. **非阻塞发送：** DMA 忙时当前帧被丢弃，不排队缓冲，保证发送的始终是最新数据
2. **参数不持久化：** 通过命令修改的参数仅运行时有效，断电后恢复为初始值
3. **PID 命令无范围限制：** 固件不对 Kp/Ki/Kd 做范围校验，上位机需自行确保合理值
4. **参数帧优先：** 有参数回复请求时，该周期会发送参数帧替代数据帧（不会同时发送两帧）
5. **底盘差异：** 数据帧中 `raw_speed_A/B` 字段仅 AKM 底盘有效，其他底盘类型为 0
6. **RX Payload 上限：** 最大 12 字节，超出长度的帧会被静默丢弃

---

## 10. 源码位置索引

| 文件 | 说明 |
|------|------|
| `HARDWARE/debug_uart.c` | 模块实现（DMA 初始化、帧组装、RX 状态机、命令执行） |
| `HARDWARE/Inc/debug_uart.h` | 接口声明与协议常量定义 |
| `BALANCE/balance_task.c` | TX 调用点（Balance_task 中 100Hz 调用 `Debug_SendDataFrame()`） |
| `HARDWARE/uartx_callback.c` | RX 调用点（USART1 中断中调用 `Debug_ProcessRxByte()`） |
| `USER/system.c` | 初始化调用（`Debug_UART_DMA_Init()`） |
