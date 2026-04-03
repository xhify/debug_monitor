"""
串口模拟测试脚本 — 模拟 WHEELTEC C50X 小车发送调试数据帧

用法（TCP 模式，无需额外软件）：
    python tests/fake_serial.py
    上位机端口栏输入: socket://localhost:9999

用法（虚拟串口模式，需 HHD Free Virtual Serial Ports 或 VSPE）：
    python tests/fake_serial.py --serial COM10
    上位机连接对端串口（如 COM11）

数据模式：
    - T法原始速度：低噪声正弦波
    - M法原始速度：高噪声正弦波
    - 融合反馈速度：T/M 加权结果
    - 目标速度：方波（周期 ~5 秒）
    - PWM 输出：与目标速度成比例
    - AFC 输出：仅在 SPEED 模式下模拟增量 PWM
"""

import argparse
import math
import os
import random
import socket
import struct
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

import serial
from protocol import (
    HEADER1, HEADER2,
    FRAME_ID_DATA, FRAME_ID_PARAM,
    CMD_SET_PID_BOTH, CMD_SET_PID_A, CMD_SET_PID_B,
    CMD_SET_RC_SPEED, CMD_SET_MAX_SPEED, CMD_SET_SMOOTH_STEP,
    CMD_SET_TARGET_SPEED_AB, CMD_SET_TARGET_PWM_AB,
    CMD_QUERY_PARAMS,
    compute_xor_checksum,
)

CMD_NAMES = {
    CMD_SET_PID_BOTH: "设置两轮PID",
    CMD_SET_PID_A: "设置电机A PID",
    CMD_SET_PID_B: "设置电机B PID",
    CMD_SET_RC_SPEED: "设置遥控速度",
    CMD_SET_MAX_SPEED: "设置最大速度",
    CMD_SET_SMOOTH_STEP: "设置平滑步进",
    CMD_SET_TARGET_SPEED_AB: "设置A/B目标速度",
    CMD_SET_TARGET_PWM_AB: "设置A/B目标PWM",
    CMD_QUERY_PARAMS: "查询参数",
}

FAKE_PARAMS = {
    'A_kp': 80.0, 'A_ki': 0.6, 'A_kd': 20.0,
    'B_kp': 80.0, 'B_ki': 0.6, 'B_kd': 20.0,
    'rc_speed': 100.0, 'limt_max_speed': 0.8, 'smooth_MotorStep': 0.02,
}

CONTROL_STATE = {
    'mode': 'SPEED',
    'target_a': 0.6,
    'target_b': 0.6,
    'pwm_a': 300,
    'pwm_b': 300,
}


def build_data_frame(
    t_raw_a: float,
    t_raw_b: float,
    m_raw_a: float,
    m_raw_b: float,
    final_a: float,
    final_b: float,
    tgt_a: float,
    tgt_b: float,
    out_a: int,
    out_b: int,
    afc_a: float,
    afc_b: float,
) -> bytes:
    """构建 48 字节数据帧。"""
    payload = struct.pack(
        '<8f2h2f',
        t_raw_a, t_raw_b,
        m_raw_a, m_raw_b,
        final_a, final_b,
        tgt_a, tgt_b,
        out_a, out_b,
        afc_a, afc_b,
    )
    frame = bytes([HEADER1, HEADER2, FRAME_ID_DATA]) + payload
    return frame + bytes([compute_xor_checksum(frame)])


def build_param_frame() -> bytes:
    """构建 40 字节参数帧。"""
    payload = struct.pack(
        '<9f',
        FAKE_PARAMS['A_kp'], FAKE_PARAMS['A_ki'], FAKE_PARAMS['A_kd'],
        FAKE_PARAMS['B_kp'], FAKE_PARAMS['B_ki'], FAKE_PARAMS['B_kd'],
        FAKE_PARAMS['rc_speed'], FAKE_PARAMS['limt_max_speed'],
        FAKE_PARAMS['smooth_MotorStep'],
    )
    frame = bytes([HEADER1, HEADER2, FRAME_ID_PARAM]) + payload
    return frame + bytes([compute_xor_checksum(frame)])


def parse_rx_commands(data: bytes) -> bool:
    """解析上位机发来的命令帧字节流。"""
    has_query = False
    pos = 0
    while pos < len(data) - 4:
        idx = data.find(bytes([HEADER1, HEADER2]), pos)
        if idx < 0 or idx + 4 > len(data):
            break

        cmd_id = data[idx + 2]
        length = data[idx + 3]
        frame_len = 4 + length + 1
        if idx + frame_len > len(data):
            break

        frame_bytes = data[idx:idx + frame_len]
        if compute_xor_checksum(frame_bytes[:-1]) == frame_bytes[-1]:
            name = CMD_NAMES.get(cmd_id, f"未知(0x{cmd_id:02X})")
            payload = frame_bytes[4:4 + length]
            if cmd_id == CMD_QUERY_PARAMS:
                has_query = True
                print(f"  ← 收到命令: {name}")
            elif length == 12:
                kp, ki, kd = struct.unpack('<3f', payload)
                if cmd_id in (CMD_SET_PID_BOTH, CMD_SET_PID_A):
                    FAKE_PARAMS['A_kp'] = kp
                    FAKE_PARAMS['A_ki'] = ki
                    FAKE_PARAMS['A_kd'] = kd
                if cmd_id in (CMD_SET_PID_BOTH, CMD_SET_PID_B):
                    FAKE_PARAMS['B_kp'] = kp
                    FAKE_PARAMS['B_ki'] = ki
                    FAKE_PARAMS['B_kd'] = kd
                print(f"  ← 收到命令: {name} (Kp={kp:.4f}, Ki={ki:.4f}, Kd={kd:.4f})")
            elif length == 4:
                if cmd_id == CMD_SET_TARGET_PWM_AB:
                    pwm_a, pwm_b = struct.unpack('<2h', payload)
                    CONTROL_STATE['mode'] = 'PWM'
                    CONTROL_STATE['pwm_a'] = pwm_a
                    CONTROL_STATE['pwm_b'] = pwm_b
                    print(f"  ← 收到命令: {name} (A={pwm_a}, B={pwm_b})")
                else:
                    value, = struct.unpack('<f', payload)
                    if cmd_id == CMD_SET_RC_SPEED:
                        FAKE_PARAMS['rc_speed'] = value
                    elif cmd_id == CMD_SET_MAX_SPEED:
                        FAKE_PARAMS['limt_max_speed'] = value
                    elif cmd_id == CMD_SET_SMOOTH_STEP:
                        FAKE_PARAMS['smooth_MotorStep'] = value
                    print(f"  ← 收到命令: {name} (值={value:.4f})")
            elif length == 8 and cmd_id == CMD_SET_TARGET_SPEED_AB:
                speed_a, speed_b = struct.unpack('<2f', payload)
                CONTROL_STATE['mode'] = 'SPEED'
                CONTROL_STATE['target_a'] = speed_a
                CONTROL_STATE['target_b'] = speed_b
                print(f"  ← 收到命令: {name} (A={speed_a:.4f}, B={speed_b:.4f})")
            else:
                print(f"  ← 收到命令: {name}")
        else:
            print(f"  ← 校验失败: CmdID=0x{cmd_id:02X}")

        pos = idx + frame_len

    return has_query


def generate_data(t: float) -> tuple:
    """根据时间 t（秒）生成一组模拟数据。"""
    freq = 0.5
    amplitude = 0.3
    offset = 0.5

    final_a = offset + amplitude * math.sin(2 * math.pi * freq * t)
    final_b = offset + amplitude * math.sin(2 * math.pi * freq * t + 0.5)

    t_raw_a = final_a + random.gauss(0, 0.03)
    t_raw_b = final_b + random.gauss(0, 0.03)
    m_raw_a = final_a + random.gauss(0, 0.06)
    m_raw_b = final_b + random.gauss(0, 0.06)

    square_period = 5.0
    if CONTROL_STATE['mode'] == 'SPEED':
        tgt_a = CONTROL_STATE['target_a']
        tgt_b = CONTROL_STATE['target_b']
        if (t % square_period) >= (square_period / 2):
            tgt_a *= 0.5
            tgt_b *= 0.5
        afc_a = 80.0 * math.sin(2 * math.pi * 0.3 * t)
        afc_b = 80.0 * math.sin(2 * math.pi * 0.3 * t + 0.4)
        out_a = int(tgt_a * 500 + (final_a - tgt_a) * 200 + afc_a)
        out_b = int(tgt_b * 500 + (final_b - tgt_b) * 200 + afc_b)
    else:
        tgt_a = float(CONTROL_STATE['pwm_a'])
        tgt_b = float(CONTROL_STATE['pwm_b'])
        if (t % square_period) >= (square_period / 2):
            tgt_a = 0.0
            tgt_b = 0.0
        afc_a = 0.0
        afc_b = 0.0
        out_a = int(tgt_a)
        out_b = int(tgt_b)

    if CONTROL_STATE['mode'] == 'PWM':
        out_a = max(-16800, min(16800, out_a))
        out_b = max(-16800, min(16800, out_b))
    else:
        out_a = max(-1000, min(1000, out_a))
        out_b = max(-1000, min(1000, out_b))

    return (
        t_raw_a, t_raw_b,
        m_raw_a, m_raw_b,
        final_a, final_b,
        tgt_a, tgt_b,
        out_a, out_b,
        afc_a, afc_b,
    )


def run_tcp_server(host: str, port: int) -> None:
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    print(f"TCP 服务器已启动，监听 {host}:{port}")
    print(f"上位机端口栏输入: socket://localhost:{port}")
    print("等待上位机连接...\n")

    while True:
        conn, addr = srv.accept()
        print(f"上位机已连接: {addr}")
        conn.setblocking(False)

        try:
            _stream_loop(
                send_fn=lambda data: conn.sendall(data),
                recv_fn=lambda: _tcp_recv(conn),
            )
        except (ConnectionResetError, ConnectionAbortedError, BrokenPipeError):
            print(f"\n上位机断开连接: {addr}")
        except KeyboardInterrupt:
            print("\n用户中断。")
            conn.close()
            srv.close()
            return

        conn.close()
        print("等待上位机重新连接...\n")


def _tcp_recv(conn: socket.socket) -> bytes:
    try:
        data = conn.recv(4096)
        if not data:
            raise ConnectionResetError("连接已关闭")
        return data
    except BlockingIOError:
        return b''


def run_serial_mode(port: str, baudrate: int) -> None:
    print(f"打开串口 {port} @ {baudrate} bps ...")
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0,
        )
    except Exception as e:
        print(f"无法打开串口: {e}")
        sys.exit(1)

    print("串口已打开，上位机请连接对端串口。\n")

    try:
        _stream_loop(
            send_fn=lambda data: ser.write(data),
            recv_fn=lambda: ser.read_all() if ser.in_waiting > 0 else b'',
        )
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print("串口已关闭。")


def _stream_loop(send_fn, recv_fn) -> None:
    frame_count = 0
    send_param_next = False
    start_time = time.perf_counter()
    next_tick = start_time

    print("开始以 100Hz 发送数据帧。按 Ctrl+C 停止。\n")

    while True:
        t = time.perf_counter() - start_time

        rx_data = recv_fn()
        if rx_data and parse_rx_commands(rx_data):
            send_param_next = True

        if send_param_next:
            send_fn(build_param_frame())
            send_param_next = False
            print("  → 已回复参数帧 (40 字节)")
        else:
            send_fn(build_data_frame(*generate_data(t)))

        frame_count += 1
        if frame_count % 500 == 0:
            elapsed = time.perf_counter() - start_time
            actual_hz = frame_count / elapsed if elapsed > 0 else 0
            print(f"[{elapsed:.1f}s] 已发送 {frame_count} 帧, 实际帧率: {actual_hz:.1f} Hz")

        next_tick += 0.01
        now = time.perf_counter()
        if next_tick > now:
            sleep_time = next_tick - now - 0.002
            if sleep_time > 0:
                time.sleep(sleep_time)
            while time.perf_counter() < next_tick:
                pass
        else:
            next_tick = time.perf_counter()


def main() -> None:
    parser = argparse.ArgumentParser(
        description='模拟 WHEELTEC C50X 调试串口数据发送',
        epilog='示例:\n'
               '  TCP 模式:    python tests/fake_serial.py\n'
               '  串口模式:   python tests/fake_serial.py --serial COM10',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('--serial', metavar='PORT',
                        help='使用虚拟串口模式（需 HHD Free Virtual Serial Ports 或 VSPE）')
    parser.add_argument('--tcp-port', type=int, default=9999,
                        help='TCP 模式监听端口（默认 9999）')
    parser.add_argument('--baudrate', type=int, default=115200,
                        help='串口波特率（默认 115200，仅串口模式有效）')
    args = parser.parse_args()

    if args.serial:
        run_serial_mode(args.serial, args.baudrate)
    else:
        run_tcp_server('0.0.0.0', args.tcp_port)


if __name__ == '__main__':
    main()
