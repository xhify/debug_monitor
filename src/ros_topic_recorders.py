"""ROS 话题检查与 CSV 记录工具。"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from recording_clock import RecordingClock


ODOMETRY_FIELDNAMES = [
    "ros_time",
    "recv_time_epoch_s",
    "session_elapsed_s",
    "frame_id",
    "child_frame_id",
    "position_x",
    "position_y",
    "position_z",
    "orientation_x",
    "orientation_y",
    "orientation_z",
    "orientation_w",
    "linear_x",
    "linear_y",
    "linear_z",
    "angular_x",
    "angular_y",
    "angular_z",
    "session_id",
]

ROS_IMU_FIELDNAMES = [
    "accel_x",
    "accel_y",
    "accel_z",
    "gyro_x",
    "gyro_y",
    "gyro_z",
    "orientation_x",
    "orientation_y",
    "orientation_z",
    "orientation_w",
    "ros_time",
    "recv_time_epoch_s",
    "session_elapsed_s",
    "frame_id",
    "session_id",
]

POWER_VOLTAGE_FIELDNAMES = [
    "time_s",
    "voltage",
    "recv_time_epoch_s",
    "session_elapsed_s",
    "session_id",
]


def get_nested(data: dict, path: str, default=""):
    current = data
    for part in path.split("."):
        if not isinstance(current, dict):
            return default
        current = current.get(part, default)
    return current


def extract_header_stamp(message: dict) -> float:
    header = message.get("header", {}) if isinstance(message, dict) else {}
    stamp = header.get("stamp", {}) if isinstance(header, dict) else {}
    if not isinstance(stamp, dict):
        return 0.0
    secs = stamp.get("secs", stamp.get("sec", 0))
    nsecs = stamp.get("nsecs", stamp.get("nanosec", 0))
    try:
        return float(secs) + float(nsecs) / 1_000_000_000.0
    except (TypeError, ValueError):
        return 0.0


def extract_frame_id(message: dict) -> str:
    header = message.get("header", {}) if isinstance(message, dict) else {}
    if not isinstance(header, dict):
        return ""
    return str(header.get("frame_id", ""))


@dataclass(slots=True)
class RosTopicCheckResult:
    topic: str
    expected_type: str
    status: str
    estimated_hz: float
    has_header_stamp: bool
    messages_received: int
    notes: str


class RosTopicMonitor:
    """记录检查阶段采样结果。"""

    def __init__(
        self,
        topic: str,
        expected_type: str,
        required_fields: tuple[str, ...] = (),
        warning_hz_below: float | None = None,
    ) -> None:
        self.topic = topic
        self.expected_type = expected_type
        self.required_fields = tuple(required_fields)
        self.warning_hz_below = warning_hz_below
        self._samples: list[tuple[float, dict, str]] = []

    def observe(self, message: dict, message_type: str, recv_time_epoch_s: float) -> None:
        self._samples.append((float(recv_time_epoch_s), dict(message), str(message_type)))

    def result(self) -> RosTopicCheckResult:
        messages_received = len(self._samples)
        if messages_received == 0:
            return RosTopicCheckResult(
                topic=self.topic,
                expected_type=self.expected_type,
                status="offline",
                estimated_hz=0.0,
                has_header_stamp=False,
                messages_received=0,
                notes="未收到消息",
            )

        times = [sample[0] for sample in self._samples]
        span = times[-1] - times[0]
        estimated_hz = float((messages_received - 1) / span) if span > 0 and messages_received > 1 else 0.0
        sample_message = self._samples[-1][1]
        message_type = self._samples[-1][2]
        has_header_stamp = extract_header_stamp(sample_message) > 0.0

        notes: list[str] = []
        status = "ok"
        if message_type != self.expected_type:
            status = "error"
            notes.append(f"message type 不匹配: {message_type}")
        for field_path in self.required_fields:
            if get_nested(sample_message, field_path, "") in ("", None):
                status = "error"
                notes.append(f"缺少字段 {field_path}")
        if self.warning_hz_below is not None and estimated_hz < self.warning_hz_below:
            if status == "ok":
                status = "warning"
            notes.append(f"频率偏低: {estimated_hz:.1f} Hz")

        return RosTopicCheckResult(
            topic=self.topic,
            expected_type=self.expected_type,
            status=status,
            estimated_hz=estimated_hz,
            has_header_stamp=has_header_stamp,
            messages_received=messages_received,
            notes="; ".join(notes),
        )


class RosTopicCsvRecorder:
    """通用 ROS CSV 记录器。"""

    def __init__(
        self,
        path: Path,
        fieldnames: list[str],
        clock: RecordingClock,
        row_builder: Callable[[dict, RecordingClock, float | None], dict[str, object]],
    ) -> None:
        self.path = Path(path)
        self.fieldnames = list(fieldnames)
        self._clock = clock
        self._row_builder = row_builder
        self._handle = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._handle, fieldnames=self.fieldnames)
        self._writer.writeheader()
        self.rows_written = 0

    def write_message(self, message: dict, recv_time_epoch_s: float | None = None) -> None:
        row = self._row_builder(message, self._clock, recv_time_epoch_s)
        self._writer.writerow(row)
        self._handle.flush()
        self.rows_written += 1

    def close(self) -> None:
        self._handle.close()


def make_odometry_recorder(path: Path, clock: RecordingClock) -> RosTopicCsvRecorder:
    return RosTopicCsvRecorder(path, ODOMETRY_FIELDNAMES, clock, _build_odometry_row)


def make_ros_imu_recorder(path: Path, clock: RecordingClock) -> RosTopicCsvRecorder:
    return RosTopicCsvRecorder(path, ROS_IMU_FIELDNAMES, clock, _build_imu_row)


def make_power_voltage_recorder(path: Path, clock: RecordingClock) -> RosTopicCsvRecorder:
    return RosTopicCsvRecorder(path, POWER_VOLTAGE_FIELDNAMES, clock, _build_power_voltage_row)


def _record_fields(clock: RecordingClock, recv_time_epoch_s: float | None) -> dict[str, object]:
    fields = clock.now_record_fields()
    if recv_time_epoch_s is not None:
        fields["recv_time_epoch_s"] = float(recv_time_epoch_s)
    return fields


def _build_odometry_row(message: dict, clock: RecordingClock, recv_time_epoch_s: float | None) -> dict[str, object]:
    record = _record_fields(clock, recv_time_epoch_s)
    pose = get_nested(message, "pose.pose", {}) or {}
    position = pose.get("position", {}) if isinstance(pose, dict) else {}
    orientation = pose.get("orientation", {}) if isinstance(pose, dict) else {}
    twist = get_nested(message, "twist.twist", {}) or {}
    linear = twist.get("linear", {}) if isinstance(twist, dict) else {}
    angular = twist.get("angular", {}) if isinstance(twist, dict) else {}
    return {
        "ros_time": extract_header_stamp(message),
        "recv_time_epoch_s": record["recv_time_epoch_s"],
        "session_elapsed_s": record["session_elapsed_s"],
        "frame_id": extract_frame_id(message),
        "child_frame_id": message.get("child_frame_id", ""),
        "position_x": position.get("x", ""),
        "position_y": position.get("y", ""),
        "position_z": position.get("z", ""),
        "orientation_x": orientation.get("x", ""),
        "orientation_y": orientation.get("y", ""),
        "orientation_z": orientation.get("z", ""),
        "orientation_w": orientation.get("w", ""),
        "linear_x": linear.get("x", ""),
        "linear_y": linear.get("y", ""),
        "linear_z": linear.get("z", ""),
        "angular_x": angular.get("x", ""),
        "angular_y": angular.get("y", ""),
        "angular_z": angular.get("z", ""),
        "session_id": record["session_id"],
    }


def _build_imu_row(message: dict, clock: RecordingClock, recv_time_epoch_s: float | None) -> dict[str, object]:
    record = _record_fields(clock, recv_time_epoch_s)
    accel = message.get("linear_acceleration", {}) if isinstance(message, dict) else {}
    gyro = message.get("angular_velocity", {}) if isinstance(message, dict) else {}
    orientation = message.get("orientation", {}) if isinstance(message, dict) else {}
    return {
        "accel_x": accel.get("x", ""),
        "accel_y": accel.get("y", ""),
        "accel_z": accel.get("z", ""),
        "gyro_x": gyro.get("x", ""),
        "gyro_y": gyro.get("y", ""),
        "gyro_z": gyro.get("z", ""),
        "orientation_x": orientation.get("x", ""),
        "orientation_y": orientation.get("y", ""),
        "orientation_z": orientation.get("z", ""),
        "orientation_w": orientation.get("w", ""),
        "ros_time": extract_header_stamp(message),
        "recv_time_epoch_s": record["recv_time_epoch_s"],
        "session_elapsed_s": record["session_elapsed_s"],
        "frame_id": extract_frame_id(message),
        "session_id": record["session_id"],
    }


def _build_power_voltage_row(message: dict, clock: RecordingClock, recv_time_epoch_s: float | None) -> dict[str, object]:
    record = _record_fields(clock, recv_time_epoch_s)
    return {
        "time_s": record["session_elapsed_s"],
        "voltage": message.get("data", ""),
        "recv_time_epoch_s": record["recv_time_epoch_s"],
        "session_elapsed_s": record["session_elapsed_s"],
        "session_id": record["session_id"],
    }
