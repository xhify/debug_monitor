"""
串口协议解析与组装模块

定义 WHEELTEC C50X 调试串口的帧格式常量、数据类、解析函数和命令构建函数。
纯数据模块，无 Qt 依赖。
"""

import struct
from dataclasses import dataclass

HEADER1 = 0xAA
HEADER2 = 0x55

FRAME_ID_DATA = 0x01
FRAME_ID_PARAM = 0x02

DATA_FRAME_LEN = 48
PARAM_FRAME_LEN = 40

CMD_SET_PID_BOTH = 0x10
CMD_SET_PID_A = 0x11
CMD_SET_PID_B = 0x12
CMD_SET_RC_SPEED = 0x20
CMD_SET_MAX_SPEED = 0x21
CMD_SET_SMOOTH_STEP = 0x22
CMD_SET_TARGET_SPEED_AB = 0x23
CMD_SET_TARGET_PWM_AB = 0x24
CMD_QUERY_PARAMS = 0x30

_DATA_FMT = '<8f2h2f'
_PARAM_FMT = '<9f'


@dataclass(slots=True)
class DataFrame:
    """TX 数据帧 (0x01) — 48 字节"""

    t_raw_A: float
    t_raw_B: float
    m_raw_A: float
    m_raw_B: float
    final_A: float
    final_B: float
    target_A: float
    target_B: float
    output_A: int
    output_B: int
    afc_output_A: float
    afc_output_B: float


@dataclass(slots=True)
class ParamFrame:
    """TX 参数帧 (0x02) — 40 字节"""

    A_kp: float
    A_ki: float
    A_kd: float
    B_kp: float
    B_ki: float
    B_kd: float
    rc_speed: float
    limt_max_speed: float
    smooth_MotorStep: float


def compute_xor_checksum(data: bytes | bytearray) -> int:
    """计算逐字节 XOR 校验和"""
    result = 0
    for b in data:
        result ^= b
    return result


def parse_data_frame(raw: bytes | bytearray) -> DataFrame | None:
    """解析 48 字节 TX 数据帧。"""
    if len(raw) != DATA_FRAME_LEN:
        return None
    if raw[0] != HEADER1 or raw[1] != HEADER2 or raw[2] != FRAME_ID_DATA:
        return None
    if compute_xor_checksum(raw[:47]) != raw[47]:
        return None
    return DataFrame(*struct.unpack_from(_DATA_FMT, raw, 3))


def parse_param_frame(raw: bytes | bytearray) -> ParamFrame | None:
    """解析 40 字节 TX 参数帧。"""
    if len(raw) != PARAM_FRAME_LEN:
        return None
    if raw[0] != HEADER1 or raw[1] != HEADER2 or raw[2] != FRAME_ID_PARAM:
        return None
    if compute_xor_checksum(raw[:39]) != raw[39]:
        return None
    return ParamFrame(*struct.unpack_from(_PARAM_FMT, raw, 3))


def build_command(cmd_id: int, payload: bytes = b'') -> bytes:
    """构建 RX 命令帧。"""
    frame = bytes([HEADER1, HEADER2, cmd_id, len(payload)]) + payload
    checksum = compute_xor_checksum(frame)
    return frame + bytes([checksum])


def build_pid_command(cmd_id: int, kp: float, ki: float, kd: float) -> bytes:
    """构建设置 PID 命令（cmd_id: 0x10/0x11/0x12）"""
    return build_command(cmd_id, struct.pack('<3f', kp, ki, kd))


def build_float_command(cmd_id: int, value: float) -> bytes:
    """构建设置单 float 值命令（cmd_id: 0x20/0x21/0x22）"""
    return build_command(cmd_id, struct.pack('<f', value))


def build_dual_float_command(cmd_id: int, value_a: float, value_b: float) -> bytes:
    """构建双 float 命令（cmd_id: 0x23）"""
    return build_command(cmd_id, struct.pack('<2f', value_a, value_b))


def build_dual_int16_command(cmd_id: int, value_a: int, value_b: int) -> bytes:
    """构建双 int16 命令（cmd_id: 0x24）"""
    return build_command(cmd_id, struct.pack('<2h', value_a, value_b))


def build_query_command() -> bytes:
    """构建查询参数命令 (0x30)"""
    return build_command(CMD_QUERY_PARAMS)
