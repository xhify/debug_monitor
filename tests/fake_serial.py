"""
串口模拟测试脚本 — 模拟 WHEELTEC C50X 小车发送调试数据帧

用法（TCP 模式，无需额外软件）：
    python tests/fake_serial.py
    上位机端口栏输入: socket://localhost:9999

用法（虚拟串口模式，需 HHD Free Virtual Serial Ports 或 VSPE）：
    python tests/fake_serial.py --serial COM10
    上位机连接对端串口（如 COM11）

数据模式：
    - 原始速度：正弦波 + 随机噪声
    - 滤波速度：纯正弦波
    - 目标速度：方波（周期 ~5 秒）
    - PWM 输出：与目标速度成比例
"""

import sys
import os
import math
import time
import struct
import socket
import random
import argparse
import threading

# 将 src/ 加入 sys.path，以便导入 protocol 模块
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

import serial
from protocol import (
    HEADER1, HEADER2,
    FRAME_ID_DATA, FRAME_ID_PARAM,
    CMD_SET_PID_BOTH, CMD_SET_PID_A, CMD_SET_PID_B,
    CMD_SET_RC_SPEED, CMD_SET_MAX_SPEED, CMD_SET_SMOOTH_STEP,
    CMD_QUERY_PARAMS,
    compute_xor_checksum,
)

# ─── 命令 ID → 名称映射（用于日志打印）─────────────────────
CMD_NAMES = {
    CMD_SET_PID_BOTH:   "设置两轮PID",
    CMD_SET_PID_A:      "设置电机A PID",
    CMD_SET_PID_B:      "设置电机B PID",
    CMD_SET_RC_SPEED:   "设置遥控速度",
    CMD_SET_MAX_SPEED:  "设置最大速度",
    CMD_SET_SMOOTH_STEP: "设置平滑步进",
    CMD_QUERY_PARAMS:   "查询参数",
}

# ─── 模拟参数帧中的固件参数值 ────────────────────────────────
FAKE_PARAMS = {
    'A_kp': 80.0, 'A_ki': 0.6, 'A_kd': 20.0,
    'B_kp': 80.0, 'B_ki': 0.6, 'B_kd': 20.0,
    'rc_speed': 100.0, 'limt_max_speed': 0.8, 'smooth_MotorStep': 0.02,
}


def build_data_frame(raw_a: float, raw_b: float,
                     filt_a: float, filt_b: float,
                     tgt_a: float, tgt_b: float,
                     out_a: int, out_b: int) -> bytes:
    """构建 32 字节数据帧。"""
    payload = struct.pack('<6f2h', raw_a, raw_b, filt_a, filt_b,
                          tgt_a, tgt_b, out_a, out_b)
    frame = bytes([HEADER1, HEADER2, FRAME_ID_DATA]) + payload
    checksum = compute_xor_checksum(frame)
    return frame + bytes([checksum])


def build_param_frame() -> bytes:
    """构建 40 字节参数帧。"""
    payload = struct.pack('<9f',
                          FAKE_PARAMS['A_kp'], FAKE_PARAMS['A_ki'], FAKE_PARAMS['A_kd'],
                          FAKE_PARAMS['B_kp'], FAKE_PARAMS['B_ki'], FAKE_PARAMS['B_kd'],
                          FAKE_PARAMS['rc_speed'], FAKE_PARAMS['limt_max_speed'],
                          FAKE_PARAMS['smooth_MotorStep'])
    frame = bytes([HEADER1, HEADER2, FRAME_ID_PARAM]) + payload
    checksum = compute_xor_checksum(frame)
    return frame + bytes([checksum])


def parse_rx_commands(data: bytes) -> bool:
    """
    解析上位机发来的命令帧字节流。
    返回 True 如果包含查询参数命令（0x30），否则 False。
    """
    has_query = False
    pos = 0
    while pos < len(data) - 4:  # 最小命令帧 5 字节
        # 搜索帧头
        idx = data.find(bytes([HEADER1, HEADER2]), pos)
        if idx < 0 or idx + 4 > len(data):
            break

        cmd_id = data[idx + 2]
        length = data[idx + 3]

        # 检查帧完整性：header(2) + cmd(1) + len(1) + payload(length) + checksum(1)
        frame_len = 4 + length + 1
        if idx + frame_len > len(data):
            break  # 不完整帧，忽略

        # 校验
        frame_bytes = data[idx:idx + frame_len]
        expected_checksum = compute_xor_checksum(frame_bytes[:-1])
        if expected_checksum == frame_bytes[-1]:
            name = CMD_NAMES.get(cmd_id, f"未知(0x{cmd_id:02X})")
            if cmd_id == CMD_QUERY_PARAMS:
                has_query = True
                print(f"  ← 收到命令: {name}")
            else:
                # 解析 payload 中的 float 值用于日志
                payload = frame_bytes[4:4 + length]
                if length == 12:
                    kp, ki, kd = struct.unpack('<3f', payload)
                    print(f"  ← 收到命令: {name} (Kp={kp:.4f}, Ki={ki:.4f}, Kd={kd:.4f})")
                elif length == 4:
                    val, = struct.unpack('<f', payload)
                    print(f"  ← 收到命令: {name} (值={val:.4f})")
                else:
                    print(f"  ← 收到命令: {name}")
        else:
            print(f"  ← 校验失败: CmdID=0x{cmd_id:02X}")

        pos = idx + frame_len

    return has_query


def generate_data(t: float) -> tuple:
    """
    根据时间 t（秒）生成一组模拟数据。
    返回 (raw_a, raw_b, filt_a, filt_b, tgt_a, tgt_b, out_a, out_b)
    """
    # 正弦波参数
    freq = 0.5       # 0.5Hz，2秒一个周期
    amplitude = 0.3  # 速度幅值 m/s
    offset = 0.5     # 基准速度 m/s

    # 滤波速度：纯正弦波
    filt_a = offset + amplitude * math.sin(2 * math.pi * freq * t)
    filt_b = offset + amplitude * math.sin(2 * math.pi * freq * t + 0.5)  # B相位偏移

    # 原始速度：滤波值 + 随机噪声
    noise_scale = 0.05
    raw_a = filt_a + random.gauss(0, noise_scale)
    raw_b = filt_b + random.gauss(0, noise_scale)

    # 目标速度：方波，周期 ~5 秒
    square_period = 5.0
    if (t % square_period) < (square_period / 2):
        tgt_a = 0.6
        tgt_b = 0.6
    else:
        tgt_a = 0.3
        tgt_b = 0.3

    # PWM 输出：与目标速度成比例（模拟 PID 输出）
    out_a = int(tgt_a * 500 + (filt_a - tgt_a) * 200)
    out_b = int(tgt_b * 500 + (filt_b - tgt_b) * 200)
    out_a = max(-1000, min(1000, out_a))
    out_b = max(-1000, min(1000, out_b))

    return raw_a, raw_b, filt_a, filt_b, tgt_a, tgt_b, out_a, out_b


# ─── TCP 服务器模式 ──────────────────────────────────────────

def run_tcp_server(host: str, port: int) -> None:
    """以 TCP 服务器模式运行，等待上位机通过 socket://host:port 连接。"""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    print(f"TCP 服务器已启动，监听 {host}:{port}")
    print(f"上位机端口栏输入: socket://localhost:{port}")
    print(f"等待上位机连接...\n")

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
        print(f"等待上位机重新连接...\n")


def _tcp_recv(conn: socket.socket) -> bytes:
    """非阻塞从 TCP 连接读取所有可用数据。"""
    try:
        data = conn.recv(4096)
        if not data:
            raise ConnectionResetError("连接已关闭")
        return data
    except BlockingIOError:
        return b''


# ─── 虚拟串口模式 ────────────────────────────────────────────

def run_serial_mode(port: str, baudrate: int) -> None:
    """通过虚拟串口发送数据（需要 HHD Free Virtual Serial Ports 或 VSPE）。"""
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

    print(f"串口已打开，上位机请连接对端串口。\n")

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


# ─── 通用数据流循环 ──────────────────────────────────────────

def _stream_loop(send_fn, recv_fn) -> None:
    """
    100Hz 数据发送主循环，通过回调函数抽象传输层。
    send_fn(data: bytes) — 发送数据
    recv_fn() -> bytes   — 非阻塞接收数据
    """
    frame_count = 0
    send_param_next = False
    start_time = time.perf_counter()
    next_tick = start_time

    print(f"开始以 100Hz 发送数据帧。按 Ctrl+C 停止。\n")

    while True:
        t = time.perf_counter() - start_time

        # ── 检查上位机命令 ────────────────────────────────
        rx_data = recv_fn()
        if rx_data:
            has_query = parse_rx_commands(rx_data)
            if has_query:
                send_param_next = True

        # ── 发送帧 ───────────────────────────────────────
        if send_param_next:
            param_frame = build_param_frame()
            send_fn(param_frame)
            send_param_next = False
            print(f"  → 已回复参数帧 (40 字节)")
        else:
            raw_a, raw_b, filt_a, filt_b, tgt_a, tgt_b, out_a, out_b = generate_data(t)
            data_frame = build_data_frame(raw_a, raw_b, filt_a, filt_b,
                                          tgt_a, tgt_b, out_a, out_b)
            send_fn(data_frame)

        frame_count += 1

        # 每 500 帧（~5秒）打印统计
        if frame_count % 500 == 0:
            elapsed = time.perf_counter() - start_time
            actual_hz = frame_count / elapsed if elapsed > 0 else 0
            print(f"[{elapsed:.1f}s] 已发送 {frame_count} 帧, 实际帧率: {actual_hz:.1f} Hz")

        # ── 精确 100Hz 节拍控制 ──────────────────────────
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
