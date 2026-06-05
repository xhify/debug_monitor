"""Read-only rosbridge client for FAST-LIO2 nav_msgs/Odometry."""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Callable

from PySide6.QtCore import QThread, Signal

from app_config import DEFAULT_FASTLIO_ODOM_TOPIC, DEFAULT_ROSBRIDGE_HOST, DEFAULT_ROSBRIDGE_PORT
from localization_buffer import LocalizationSample


class RosOdometrySession:
    """Small testable wrapper around a single `/Odometry` subscription."""

    def __init__(
        self,
        host: str,
        port: int,
        *,
        topic: str = DEFAULT_FASTLIO_ODOM_TOPIC,
        ros_factory: Callable[[str, int], Any] | None = None,
        topic_factory: Callable[[Any, str, str], Any] | None = None,
        on_sample: Callable[[LocalizationSample], None] | None = None,
    ) -> None:
        self.host = host
        self.port = int(port)
        self.topic_name = topic
        self._ros_factory = ros_factory
        self._topic_factory = topic_factory
        self._on_sample = on_sample
        self.ros: Any | None = None
        self.topic: Any | None = None
        self.launch_command_topic: Any | None = None
        self.connected = False
        self._lock = threading.Lock()
        self._frame_count = 0
        self._last_sample: LocalizationSample | None = None

    @property
    def frame_count(self) -> int:
        with self._lock:
            return self._frame_count

    def last_sample(self) -> LocalizationSample | None:
        with self._lock:
            return self._last_sample

    def connect(self) -> None:
        if self.connected:
            return
        self._load_default_factories()
        self.ros = self._ros_factory(self.host, self.port)
        self.ros.run()
        self.topic = self._topic_factory(self.ros, self.topic_name, "nav_msgs/Odometry")
        self.topic.subscribe(self._on_message)
        self.launch_command_topic = self._topic_factory(self.ros, "/launch_manager/command", "std_msgs/String")
        self.connected = True

    def disconnect(self) -> None:
        if self.topic is not None:
            self.topic.unsubscribe()
        if self.ros is not None:
            self.ros.close()
        self.topic = None
        self.launch_command_topic = None
        self.ros = None
        self.connected = False

    def publish_launch_manager_command(self, command: str) -> None:
        if not self.connected or self.launch_command_topic is None:
            raise RuntimeError("rosbridge is not connected")
        self.launch_command_topic.publish({"data": str(command)})

    def _load_default_factories(self) -> None:
        if self._ros_factory is not None and self._topic_factory is not None:
            return
        try:
            import roslibpy
        except ImportError as exc:
            raise RuntimeError("缺少依赖 roslibpy，请先安装 requirements.txt") from exc
        if self._ros_factory is None:
            self._ros_factory = roslibpy.Ros
        if self._topic_factory is None:
            self._topic_factory = roslibpy.Topic

    def _on_message(self, message: dict) -> None:
        sample = parse_odometry_message(message, source=self.topic_name, recv_time=time.time())
        with self._lock:
            self._frame_count += 1
            self._last_sample = sample
        if self._on_sample is not None:
            self._on_sample(sample)


class RosOdometryWorker(QThread):
    """QThread wrapper so rosbridge connection attempts never block the GUI."""

    sample_received = Signal(object)
    error_occurred = Signal(str)
    connection_changed = Signal(bool)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._host = DEFAULT_ROSBRIDGE_HOST
        self._port = DEFAULT_ROSBRIDGE_PORT
        self._topic = DEFAULT_FASTLIO_ODOM_TOPIC
        self._running = False
        self._session: RosOdometrySession | None = None
        self._error_count = 0

    @property
    def error_count(self) -> int:
        return self._error_count

    @property
    def frame_count(self) -> int:
        if self._session is None:
            return 0
        return self._session.frame_count

    def open_bridge(
        self,
        host: str,
        port: int = DEFAULT_ROSBRIDGE_PORT,
        topic: str = DEFAULT_FASTLIO_ODOM_TOPIC,
    ) -> None:
        if self.isRunning():
            return
        self._host = host
        self._port = int(port)
        self._topic = topic
        self._running = True
        self.start()

    def close_bridge(self) -> None:
        self._running = False
        self.wait(2000)
        self.connection_changed.emit(False)

    def publish_launch_manager_command(self, command: str) -> None:
        try:
            if self._session is None:
                raise RuntimeError("rosbridge is not connected")
            self._session.publish_launch_manager_command(command)
        except Exception as exc:
            self._error_count += 1
            self.error_occurred.emit(f"launch_manager 命令发送失败: {exc}")

    def run(self) -> None:
        try:
            self._session = RosOdometrySession(
                self._host,
                self._port,
                topic=self._topic,
                on_sample=self.sample_received.emit,
            )
            self._session.connect()
            self.connection_changed.emit(True)
            while self._running:
                time.sleep(0.05)
        except Exception as exc:
            self._error_count += 1
            self.error_occurred.emit(f"FAST-LIO2 ROS 连接失败: {exc}")
        finally:
            if self._session is not None and self._session.connected:
                self._session.disconnect()
            self.connection_changed.emit(False)


def parse_odometry_message(message: dict, *, source: str, recv_time: float) -> LocalizationSample:
    header = message.get("header", {})
    pose = message.get("pose", {}).get("pose", {})
    position = pose.get("position", {})
    orientation = pose.get("orientation", {})
    qx = float(orientation.get("x", 0.0))
    qy = float(orientation.get("y", 0.0))
    qz = float(orientation.get("z", 0.0))
    qw = float(orientation.get("w", 1.0))
    roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)
    return LocalizationSample(
        ros_time=_stamp_to_seconds(header.get("stamp", {})),
        recv_time=float(recv_time),
        source=source,
        frame_id=str(header.get("frame_id", "")),
        child_frame_id=str(message.get("child_frame_id", "")),
        x=float(position.get("x", 0.0)),
        y=float(position.get("y", 0.0)),
        z=float(position.get("z", 0.0)),
        qx=qx,
        qy=qy,
        qz=qz,
        qw=qw,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
    )


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return 0.0, 0.0, 0.0
    x /= norm
    y /= norm
    z /= norm
    w /= norm

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _stamp_to_seconds(stamp: dict) -> float:
    return float(stamp.get("secs", 0.0)) + float(stamp.get("nsecs", 0.0)) * 1e-9
