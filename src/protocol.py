"""
串口协议解析与组装模块

定义 WHEELTEC C50X 调试串口的帧格式常量、数据类、解析函数和命令构建函数。
纯数据模块，无 Qt 依赖。
"""

import struct
from dataclasses import dataclass

# ─── 帧头 ───────────────────────────────────────────────
HEADER1 = 0xAA
HEADER2 = 0x55

# ─── TX 帧类型 ID ───────────────────────────────────────
FRAME_ID_DATA = 0x01   # 数据帧（32字节，100Hz）
FRAME_ID_PARAM = 0x02  # 参数帧（40字节，按需）

# ─── TX 帧长度 ──────────────────────────────────────────
DATA_FRAME_LEN = 32
PARAM_FRAME_LEN = 40

# ─── RX 命令 ID ─────────────────────────────────────────
CMD_SET_PID_BOTH = 0x10    # 两轮同步设置 PID
CMD_SET_PID_A = 0x11       # 仅设置电机 A PID
CMD_SET_PID_B = 0x12       # 仅设置电机 B PID
CMD_SET_RC_SPEED = 0x20    # 设置遥控速度限制
CMD_SET_MAX_SPEED = 0x21   # 设置最大速度限制
CMD_SET_SMOOTH_STEP = 0x22 # 设置电机加速平滑步进
CMD_QUERY_PARAMS = 0x30    # 查询当前参数

# ─── struct 格式串（小端序）──────────────────────────────
# 数据帧 payload：6 个 float + 2 个 int16，从偏移 3 开始
_DATA_FMT = '<6f2h'
# 参数帧 payload：9 个 float，从偏移 3 开始
_PARAM_FMT = '<9f'


@dataclass(slots=True)
class DataFrame:
    """TX 数据帧 (0x01) — 32 字节"""
    raw_speed_A: float    # 电机A T法原始编码器速度 (m/s)
    raw_speed_B: float    # 电机B T法原始编码器速度 (m/s)
    filtered_A: float     # 电机A Kalman滤波后速度 (m/s)
    filtered_B: float     # 电机B Kalman滤波后速度 (m/s)
    target_A: float       # 电机A 目标速度 (m/s)
    target_B: float       # 电机B 目标速度 (m/s)
    output_A: int         # 电机A PID输出 (PWM值, int16)
    output_B: int         # 电机B PID输出 (PWM值, int16)


@dataclass(slots=True)
class ParamFrame:
    """TX 参数帧 (0x02) — 40 字节"""
    A_kp: float           # 电机A 比例增益
    A_ki: float           # 电机A 积分增益
    A_kd: float           # 电机A 微分增益
    B_kp: float           # 电机B 比例增益
    B_ki: float           # 电机B 积分增益
    B_kd: float           # 电机B 微分增益
    rc_speed: float       # 遥控速度限制
    limt_max_speed: float # 最大速度限制 (m/s)
    smooth_MotorStep: float  # 电机加速平滑步进


def compute_xor_checksum(data: bytes | bytearray) -> int:
    """计算逐字节 XOR 校验和"""
    result = 0
    for b in data:
        result ^= b
    return result


def parse_data_frame(raw: bytes | bytearray) -> DataFrame | None:
    """
    解析 32 字节 TX 数据帧。
    校验通过返回 DataFrame，否则返回 None。
    """
    if len(raw) != DATA_FRAME_LEN:
        return None
    if raw[0] != HEADER1 or raw[1] != HEADER2 or raw[2] != FRAME_ID_DATA:
        return None
    if compute_xor_checksum(raw[:31]) != raw[31]:
        return None
    values = struct.unpack_from(_DATA_FMT, raw, 3)
    return DataFrame(*values)


def parse_param_frame(raw: bytes | bytearray) -> ParamFrame | None:
    """
    解析 40 字节 TX 参数帧。
    校验通过返回 ParamFrame，否则返回 None。
    """
    if len(raw) != PARAM_FRAME_LEN:
        return None
    if raw[0] != HEADER1 or raw[1] != HEADER2 or raw[2] != FRAME_ID_PARAM:
        return None
    if compute_xor_checksum(raw[:39]) != raw[39]:
        return None
    values = struct.unpack_from(_PARAM_FMT, raw, 3)
    return ParamFrame(*values)


def build_command(cmd_id: int, payload: bytes = b'') -> bytes:
    """
    构建 RX 命令帧。
    格式：[0xAA, 0x55, CmdID, Length, Payload..., Checksum]
    """
    frame = bytes([HEADER1, HEADER2, cmd_id, len(payload)]) + payload
    checksum = compute_xor_checksum(frame)
    return frame + bytes([checksum])


def build_pid_command(cmd_id: int, kp: float, ki: float, kd: float) -> bytes:
    """构建设置 PID 命令（cmd_id: 0x10/0x11/0x12）"""
    payload = struct.pack('<3f', kp, ki, kd)
    return build_command(cmd_id, payload)


def build_float_command(cmd_id: int, value: float) -> bytes:
    """构建设置单 float 值命令（cmd_id: 0x20/0x21/0x22）"""
    payload = struct.pack('<f', value)
    return build_command(cmd_id, payload)


def build_query_command() -> bytes:
    """构建查询参数命令 (0x30)"""
    return build_command(CMD_QUERY_PARAMS)
