"""ROS 话题检查与 CSV 记录工具。"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from math import atan2, copysign, degrees, pi, sqrt
from pathlib import Path
import time
from typing import Callable

from recording_clock import RecordingClock
from ros_data import ROS_IMU_RAW_HEADER, ROS_SUMMARY_ODOM_HEADER


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


def sample_ros_topic(
    host: str,
    port: int,
    topic: str,
    expected_type: str,
    *,
    sample_seconds: float = 2.0,
    required_fields: tuple[str, ...] = (),
    warning_hz_below: float | None = None,
    ros_factory=None,
    topic_factory=None,
    time_fn: Callable[[], float] = time.time,
    sleep_fn: Callable[[float], None] = time.sleep,
    connection_timeout: float = 3.0,
) -> RosTopicCheckResult:
    """临时订阅一个 ROS topic，并用真实消息估计在线状态和频率。"""

    if ros_factory is None or topic_factory is None:
        import roslibpy

        ros_factory = ros_factory or roslibpy.Ros
        topic_factory = topic_factory or roslibpy.Topic

    monitor = RosTopicMonitor(
        topic=topic,
        expected_type=expected_type,
        required_fields=required_fields,
        warning_hz_below=warning_hz_below,
    )
    ros = ros_factory(host, int(port))
    _run_ros_with_timeout(ros, connection_timeout)
    topic_obj = topic_factory(ros, topic, expected_type)

    def _on_message(message: dict) -> None:
        monitor.observe(message, expected_type, time_fn())

    try:
        topic_obj.subscribe(_on_message)
        if sample_seconds > 0:
            sleep_fn(float(sample_seconds))
    finally:
        try:
            topic_obj.unsubscribe()
        finally:
            ros.close()
    return monitor.result()


def sample_ros_topics(
    host: str,
    port: int,
    topic_specs: list[dict[str, object]],
    *,
    sample_seconds: float = 2.0,
    ros_factory=None,
    topic_factory=None,
    time_fn: Callable[[], float] = time.time,
    sleep_fn: Callable[[float], None] = time.sleep,
    connection_timeout: float = 3.0,
) -> dict[str, RosTopicCheckResult]:
    """临时连接一次 ROSbridge，同时采样多个 topic。"""

    if not topic_specs:
        return {}
    if ros_factory is None or topic_factory is None:
        import roslibpy

        ros_factory = ros_factory or roslibpy.Ros
        topic_factory = topic_factory or roslibpy.Topic

    monitors: dict[str, RosTopicMonitor] = {}
    topics = []
    ros = ros_factory(host, int(port))
    _run_ros_with_timeout(ros, connection_timeout)
    try:
        for spec in topic_specs:
            source_id = str(spec["source_id"])
            topic = str(spec["topic"])
            expected_type = str(spec["expected_type"])
            monitor = RosTopicMonitor(
                topic=topic,
                expected_type=expected_type,
                required_fields=tuple(spec.get("required_fields", ())),
                warning_hz_below=spec.get("warning_hz_below"),
            )
            monitors[source_id] = monitor
            topic_obj = topic_factory(ros, topic, expected_type)
            topics.append(topic_obj)

            def _on_message(
                message: dict,
                _monitor=monitor,
                _expected_type=expected_type,
            ) -> None:
                _monitor.observe(message, _expected_type, time_fn())

            topic_obj.subscribe(_on_message)
        if sample_seconds > 0:
            sleep_fn(float(sample_seconds))
    finally:
        for topic_obj in topics:
            try:
                topic_obj.unsubscribe()
            except Exception:
                pass
        ros.close()
    return {source_id: monitor.result() for source_id, monitor in monitors.items()}


def _run_ros_with_timeout(ros, connection_timeout: float) -> None:
    try:
        ros.run(timeout=float(connection_timeout))
    except TypeError:
        ros.run()


class RosTopicCsvRecorder:
    """通用 ROS CSV 记录器。"""

    def __init__(
        self,
        path: Path,
        fieldnames: list[str],
        clock: RecordingClock,
        row_builder: Callable[[dict, RecordingClock, float | None], dict[str, object]],
        flush_every_rows: int = 100,
    ) -> None:
        self.path = Path(path)
        self.fieldnames = list(fieldnames)
        self._clock = clock
        self._row_builder = row_builder
        self._flush_every_rows = max(1, int(flush_every_rows))
        self._frame_count = 0
        self._handle = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._handle, fieldnames=self.fieldnames)
        self._writer.writeheader()
        self._handle.flush()
        self.rows_written = 0

    def write_message(self, message: dict, recv_time_epoch_s: float | None = None) -> None:
        self._frame_count += 1
        row = self._row_builder(message, self._clock, recv_time_epoch_s)
        if "frame_count" in self.fieldnames:
            row["frame_count"] = self._frame_count
        self._writer.writerow(row)
        self.rows_written += 1
        if self.rows_written % self._flush_every_rows == 0:
            self._handle.flush()

    def close(self) -> None:
        self._handle.flush()
        self._handle.close()


def make_odometry_recorder(path: Path, clock: RecordingClock) -> RosTopicCsvRecorder:
    return RosTopicCsvRecorder(path, ODOMETRY_FIELDNAMES, clock, _build_odometry_row)


def make_ros_imu_recorder(path: Path, clock: RecordingClock) -> RosTopicCsvRecorder:
    return RosTopicCsvRecorder(path, ROS_IMU_FIELDNAMES, clock, _build_imu_row)


def make_power_voltage_recorder(path: Path, clock: RecordingClock) -> RosTopicCsvRecorder:
    return RosTopicCsvRecorder(path, POWER_VOLTAGE_FIELDNAMES, clock, _build_power_voltage_row)


def make_ros_odom_compat_recorder(path: Path, clock: RecordingClock) -> RosTopicCsvRecorder:
    return RosTopicCsvRecorder(path, ROS_SUMMARY_ODOM_HEADER, clock, _build_ros_odom_compat_row)


def make_ros_imu_compat_recorder(path: Path, clock: RecordingClock) -> RosTopicCsvRecorder:
    return RosTopicCsvRecorder(path, ROS_IMU_RAW_HEADER, clock, _build_ros_imu_compat_row)


def _record_fields(clock: RecordingClock, recv_time_epoch_s: float | None) -> dict[str, object]:
    fields = clock.now_record_fields()
    if recv_time_epoch_s is not None:
        fields["recv_time_epoch_s"] = float(recv_time_epoch_s)
        fields["session_elapsed_s"] = clock.elapsed_from_epoch(float(recv_time_epoch_s))
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


def _build_ros_odom_compat_row(message: dict, clock: RecordingClock, recv_time_epoch_s: float | None) -> dict[str, object]:
    odom = _build_odometry_row(message, clock, recv_time_epoch_s)
    record = _record_fields(clock, recv_time_epoch_s)
    return {
        "time_s": record["session_elapsed_s"],
        "frame_count": "",
        "motor_a_left_speed": odom["linear_x"],
        "motor_b_right_speed": odom["linear_y"],
        "angular_z": odom["angular_z"],
        "pose_x": odom["position_x"],
        "pose_y": odom["position_y"],
        "pose_z": odom["position_z"],
        "orientation_x": odom["orientation_x"],
        "orientation_y": odom["orientation_y"],
        "orientation_z": odom["orientation_z"],
        "orientation_w": odom["orientation_w"],
        "ros_time": odom["ros_time"],
        "recv_time": record["recv_time_epoch_s"],
        "frame_id": odom["frame_id"],
    }


def _build_ros_imu_compat_row(message: dict, clock: RecordingClock, recv_time_epoch_s: float | None) -> dict[str, object]:
    imu = _build_imu_row(message, clock, recv_time_epoch_s)
    record = _record_fields(clock, recv_time_epoch_s)
    roll_deg, pitch_deg, yaw_deg = _quaternion_to_euler_degrees(
        _float_or_zero(imu["orientation_x"]),
        _float_or_zero(imu["orientation_y"]),
        _float_or_zero(imu["orientation_z"]),
        _float_or_default(imu["orientation_w"], 1.0),
    )
    return {
        "time_s": record["session_elapsed_s"],
        "frame_count": "",
        "accel_x": imu["accel_x"],
        "accel_y": imu["accel_y"],
        "accel_z": imu["accel_z"],
        "gyro_x": imu["gyro_x"],
        "gyro_y": imu["gyro_y"],
        "gyro_z": imu["gyro_z"],
        "orientation_x": imu["orientation_x"],
        "orientation_y": imu["orientation_y"],
        "orientation_z": imu["orientation_z"],
        "orientation_w": imu["orientation_w"],
        "roll_deg": roll_deg,
        "pitch_deg": pitch_deg,
        "yaw_deg": yaw_deg,
        "ros_time": imu["ros_time"],
        "recv_time": record["recv_time_epoch_s"],
        "frame_id": imu["frame_id"],
    }


def _float_or_zero(value: object) -> float:
    return _float_or_default(value, 0.0)


def _float_or_default(value: object, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _quaternion_to_euler_degrees(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    norm = sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return 0.0, 0.0, 0.0
    x /= norm
    y /= norm
    z /= norm
    w /= norm

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = copysign(pi / 2.0, sinp)
    else:
        pitch = atan2(sinp, sqrt(max(0.0, 1.0 - sinp * sinp)))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)
    return degrees(roll), degrees(pitch), degrees(yaw)
