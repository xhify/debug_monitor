"""YESENSE IMU protocol parsing helpers."""

from __future__ import annotations

from dataclasses import dataclass


IMU_HEADER = b"\x59\x53"


@dataclass(slots=True)
class ImuSample:
    host_time: float
    sequence: int
    accel: tuple[float, float, float] | None = None
    gyro: tuple[float, float, float] | None = None
    mag: tuple[float, float, float] | None = None
    euler: tuple[float, float, float] | None = None
    quat: tuple[float, float, float, float] | None = None
    temperature_c: float | None = None
    device_time: int | None = None
    sync_time: int | None = None


def yesense_checksum(buffer: bytes | bytearray) -> tuple[int, int]:
    ck1 = 0
    ck2 = 0
    for value in buffer:
        ck1 = (ck1 + value) & 0xFF
        ck2 = (ck2 + ck1) & 0xFF
    return ck1, ck2


class YesenseParser:
    """Incrementally parse YESENSE UART output frames."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes | bytearray) -> list[ImuSample]:
        if data:
            self._buffer.extend(data)

        samples: list[ImuSample] = []
        while True:
            header_pos = self._buffer.find(IMU_HEADER)
            if header_pos < 0:
                self._keep_possible_header_prefix()
                break

            if header_pos > 0:
                del self._buffer[:header_pos]

            if len(self._buffer) < 5:
                break

            payload_len = self._buffer[4]
            total_len = payload_len + 7
            if len(self._buffer) < total_len:
                break

            packet = bytes(self._buffer[:total_len])
            del self._buffer[:total_len]
            sample = _parse_packet(packet)
            if sample is not None:
                samples.append(sample)

        return samples

    def _keep_possible_header_prefix(self) -> None:
        if len(self._buffer) > 1:
            if self._buffer[-1] == IMU_HEADER[0]:
                del self._buffer[:-1]
            else:
                self._buffer.clear()


def _parse_packet(packet: bytes) -> ImuSample | None:
    if len(packet) < 7 or packet[:2] != IMU_HEADER:
        return None

    expected_ck1, expected_ck2 = yesense_checksum(packet[2:-2])
    if packet[-2:] != bytes([expected_ck1, expected_ck2]):
        return None

    sequence = int.from_bytes(packet[2:4], "little", signed=False)
    payload_len = packet[4]
    payload = packet[5:-2]
    if len(payload) != payload_len:
        return None

    sample = ImuSample(host_time=0.0, sequence=sequence)
    pos = 0
    while pos < len(payload):
        if pos + 2 > len(payload):
            return None
        block_id = payload[pos]
        block_len = payload[pos + 1]
        pos += 2
        if pos + block_len > len(payload):
            return None
        block = payload[pos:pos + block_len]
        pos += block_len
        _apply_block(sample, block_id, block)

    return sample


def _apply_block(sample: ImuSample, block_id: int, block: bytes) -> None:
    if block_id == 0x01 and len(block) == 2:
        raw = int.from_bytes(block, "little", signed=True)
        sample.temperature_c = raw * 0.01
    elif block_id == 0x10 and len(block) == 12:
        sample.accel = _read_vec3(block, 1e-6)
    elif block_id == 0x20 and len(block) == 12:
        sample.gyro = _read_vec3(block, 1e-6)
    elif block_id == 0x31 and len(block) == 12:
        sample.mag = _read_vec3(block, 1e-3)
    elif block_id == 0x40 and len(block) == 12:
        sample.euler = _read_vec3(block, 1e-6)
    elif block_id == 0x41 and len(block) == 16:
        sample.quat = _read_vec4(block, 1e-6)
    elif block_id == 0x51 and len(block) == 4:
        sample.device_time = int.from_bytes(block, "little", signed=False)
    elif block_id == 0x52 and len(block) == 4:
        sample.sync_time = int.from_bytes(block, "little", signed=False)


def _read_vec3(block: bytes, scale: float) -> tuple[float, float, float]:
    return tuple(
        int.from_bytes(block[index:index + 4], "little", signed=True) * scale
        for index in range(0, 12, 4)
    )


def _read_vec4(block: bytes, scale: float) -> tuple[float, float, float, float]:
    return tuple(
        int.from_bytes(block[index:index + 4], "little", signed=True) * scale
        for index in range(0, 16, 4)
    )


def assign_batch_host_times(samples: list[ImuSample], start_time: float, end_time: float) -> None:
    if not samples:
        return
    if len(samples) == 1:
        samples[0].host_time = float(end_time)
        return

    span = float(end_time) - float(start_time)
    last_index = len(samples) - 1
    for index, sample in enumerate(samples):
        sample.host_time = float(start_time) + span * index / last_index


CSV_FIELDS = [
    "host_time",
    "sequence",
    "device_time",
    "sync_time",
    "temperature_c",
    "acc_x",
    "acc_y",
    "acc_z",
    "gyro_x",
    "gyro_y",
    "gyro_z",
    "pitch",
    "roll",
    "yaw",
    "mag_x",
    "mag_y",
    "mag_z",
    "quat_w",
    "quat_x",
    "quat_y",
    "quat_z",
]


def sample_to_csv_row(sample: ImuSample) -> dict[str, str]:
    row = dict.fromkeys(CSV_FIELDS, "")
    row["host_time"] = f"{sample.host_time:.6f}"
    row["sequence"] = str(sample.sequence)
    row["device_time"] = "" if sample.device_time is None else str(sample.device_time)
    row["sync_time"] = "" if sample.sync_time is None else str(sample.sync_time)
    row["temperature_c"] = _format_optional(sample.temperature_c)
    _write_vec(row, ("acc_x", "acc_y", "acc_z"), sample.accel)
    _write_vec(row, ("gyro_x", "gyro_y", "gyro_z"), sample.gyro)
    _write_vec(row, ("pitch", "roll", "yaw"), sample.euler)
    _write_vec(row, ("mag_x", "mag_y", "mag_z"), sample.mag)
    _write_vec(row, ("quat_w", "quat_x", "quat_y", "quat_z"), sample.quat)
    return row


def _format_optional(value: float | None) -> str:
    return "" if value is None else f"{value:.6f}"


def _write_vec(
    row: dict[str, str],
    keys: tuple[str, ...],
    values: tuple[float, ...] | None,
) -> None:
    if values is None:
        return
    for key, value in zip(keys, values):
        row[key] = f"{value:.6f}"

