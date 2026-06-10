"""rosbridge/roslibpy connection helpers and Qt worker."""

from __future__ import annotations

from dataclasses import dataclass, field, fields
from math import asin, atan2, copysign, degrees, pi, sqrt
import threading
import time
from typing import Any, Callable

from PySide6.QtCore import QThread, Signal

from app_config import DEFAULT_FASTLIO_ODOM_TOPIC, DEFAULT_ROSBRIDGE_HOST, DEFAULT_ROSBRIDGE_PORT


@dataclass(slots=True)
class RosImuReading:
    """Latest values for one ROS sensor_msgs/Imu topic."""

    frame_count: int = 0
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    orientation_x: float = 0.0
    orientation_y: float = 0.0
    orientation_z: float = 0.0
    orientation_w: float = 1.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    ros_time: float = 0.0
    frame_id: str = ""
    recv_time: float = 0.0

    def clone(self) -> "RosImuReading":
        return RosImuReading(**{field_.name: getattr(self, field_.name) for field_ in fields(RosImuReading)})


@dataclass(slots=True)
class RosSnapshot:
    """Latest values collected from ROS standard topics."""

    frame_count: int = 0
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0
    pose_x: float = 0.0
    pose_y: float = 0.0
    pose_z: float = 0.0
    orientation_x: float = 0.0
    orientation_y: float = 0.0
    orientation_z: float = 0.0
    orientation_w: float = 1.0
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    voltage: float = 0.0
    last_topic: str = ""
    odom_ros_time: float = 0.0
    odom_frame_id: str = ""
    odom_recv_time: float = 0.0
    imu_ros_time: float = 0.0
    imu_frame_id: str = ""
    imu_recv_time: float = 0.0
    active_imu_ros_time: float = 0.0
    active_imu_frame_id: str = ""
    active_imu_recv_time: float = 0.0
    imu: RosImuReading = field(default_factory=RosImuReading)
    active_imu: RosImuReading = field(default_factory=RosImuReading)

    def clone(self) -> "RosSnapshot":
        values = {field_.name: getattr(self, field_.name) for field_ in fields(RosSnapshot)}
        values["imu"] = self.imu.clone()
        values["active_imu"] = self.active_imu.clone()
        return RosSnapshot(**values)

    def to_display_dict(self) -> dict[str, float]:
        return {
            "linear_x": self.linear_x,
            "linear_y": self.linear_y,
            "angular_z": self.angular_z,
            "pose_x": self.pose_x,
            "pose_y": self.pose_y,
            "pose_z": self.pose_z,
            "accel_x": self.accel_x,
            "accel_y": self.accel_y,
            "accel_z": self.accel_z,
            "gyro_x": self.gyro_x,
            "gyro_y": self.gyro_y,
            "gyro_z": self.gyro_z,
            "voltage": self.voltage,
        }


class RosBridgeSession:
    """Small testable wrapper around roslibpy topic subscription and publish."""

    SUBSCRIPTIONS = (
        ("/odom", "nav_msgs/Odometry"),
        (DEFAULT_FASTLIO_ODOM_TOPIC, "nav_msgs/Odometry"),
        ("/imu", "sensor_msgs/Imu"),
        ("/active_imu", "sensor_msgs/Imu"),
        ("/PowerVoltage", "std_msgs/Float32"),
        ("/wheeltec/akm_state", "turn_on_wheeltec_robot/AkmState"),
        ("/wheeltec/control_debug", "turn_on_wheeltec_robot/ControlDebug"),
        ("/wheeltec/chassis_diagnostics", "turn_on_wheeltec_robot/ChassisDiagnostics"),
    )
    CMD_VEL_TOPIC = ("/cmd_vel", "geometry_msgs/Twist")
    LINE_FOLLOW_CONTROL_TOPIC = ("/line_follow_control", "simple_follower/LineFollowControl")
    LAUNCH_MANAGER_COMMAND_TOPIC = ("/launch_manager/command", "std_msgs/String")

    def __init__(
        self,
        host: str,
        port: int,
        *,
        ros_factory: Callable[[str, int], Any] | None = None,
        topic_factory: Callable[[Any, str, str], Any] | None = None,
        message_factory: Callable[[dict], Any] | None = None,
        service_factory: Callable[[Any, str, str], Any] | None = None,
        service_request_factory: Callable[[dict], Any] | None = None,
        monotonic_clock: Callable[[], float] | None = None,
        on_snapshot: Callable[[RosSnapshot], None] | None = None,
        on_message: Callable[[dict[str, Any]], None] | None = None,
    ) -> None:
        self.host = host
        self.port = int(port)
        self._ros_factory = ros_factory
        self._topic_factory = topic_factory
        self._message_factory = message_factory
        self._service_factory = service_factory
        self._service_request_factory = service_request_factory
        self._monotonic_clock = monotonic_clock or time.perf_counter
        self._on_snapshot = on_snapshot
        self._on_message = on_message
        self._lock = threading.Lock()
        self._snapshot = RosSnapshot()
        self._topics: dict[str, Any] = {}
        self._time_service: Any | None = None
        self.ros: Any | None = None
        self.connected = False

    def connect(self) -> None:
        if self.connected:
            return
        self._load_default_factories()
        self.ros = self._ros_factory(self.host, self.port)
        self.ros.run()

        for name, message_type in self.SUBSCRIPTIONS:
            topic = self._topic_factory(self.ros, name, message_type)
            topic.subscribe(self._callback_for(name))
            self._topics[name] = topic

        cmd_name, cmd_type = self.CMD_VEL_TOPIC
        self._topics[cmd_name] = self._topic_factory(self.ros, cmd_name, cmd_type)
        control_name, control_type = self.LINE_FOLLOW_CONTROL_TOPIC
        self._topics[control_name] = self._topic_factory(self.ros, control_name, control_type)
        launch_name, launch_type = self.LAUNCH_MANAGER_COMMAND_TOPIC
        self._topics[launch_name] = self._topic_factory(self.ros, launch_name, launch_type)
        if self._service_factory is not None:
            self._time_service = self._service_factory(self.ros, "/rosapi/get_time", "rosapi/GetTime")
        self.connected = True

    def disconnect(self) -> None:
        for name, _message_type in self.SUBSCRIPTIONS:
            topic = self._topics.get(name)
            if topic is not None:
                topic.unsubscribe()
        if self.ros is not None:
            self.ros.close()
        self.connected = False

    def publish_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        if not self.connected:
            raise RuntimeError("rosbridge is not connected")
        message = {
            "linear": {"x": float(linear_x), "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": float(angular_z)},
        }
        topic = self.topic("/cmd_vel")
        topic.publish(self._message_factory(message))

    def publish_line_follow_control(self, linear_x: float, forward: bool, backward: bool) -> None:
        if not self.connected:
            raise RuntimeError("rosbridge is not connected")
        active = bool(forward) or bool(backward)
        message = {
            "enable": active,
            "h_min": 18,
            "s_min": 60,
            "v_min": 60,
            "h_max": 34,
            "s_max": 255,
            "v_max": 255,
            "linear_x": float(linear_x) if active else 0.0,
            "angular_scale": 0.001,
            "forward": bool(forward),
            "backward": bool(backward),
        }
        topic = self.topic("/line_follow_control")
        topic.publish(self._message_factory(message))

    def publish_launch_manager_command(self, command: str) -> None:
        if not self.connected:
            raise RuntimeError("rosbridge is not connected")
        topic = self.topic("/launch_manager/command")
        topic.publish(self._message_factory({"data": str(command)}))

    def measure_network_latency_ms(self, timeout: float = 0.5) -> float:
        if not self.connected:
            raise RuntimeError("rosbridge is not connected")
        if self._time_service is None:
            raise RuntimeError("rosapi get_time service is unavailable")
        start = self._monotonic_clock()
        self._time_service.call(self._service_request_factory({}), timeout=timeout)
        elapsed_s = max(0.0, self._monotonic_clock() - start)
        return elapsed_s * 1000.0

    def snapshot(self) -> RosSnapshot:
        with self._lock:
            return self._snapshot.clone()

    def topic(self, name: str) -> Any:
        return self._topics[name]

    def _load_default_factories(self) -> None:
        if self._ros_factory and self._topic_factory:
            if self._message_factory is None:
                self._message_factory = lambda message: message
            if self._service_request_factory is None:
                self._service_request_factory = lambda request: request
            return
        try:
            import roslibpy
        except ImportError as exc:
            raise RuntimeError("缺少依赖 roslibpy，请先安装 requirements.txt") from exc
        if self._ros_factory is None:
            self._ros_factory = roslibpy.Ros
        if self._topic_factory is None:
            self._topic_factory = roslibpy.Topic
        if self._message_factory is None:
            self._message_factory = roslibpy.Message
        if self._service_factory is None:
            self._service_factory = roslibpy.Service
        if self._service_request_factory is None:
            self._service_request_factory = roslibpy.ServiceRequest

    def _callback_for(self, name: str) -> Callable[[dict], None]:
        if name == "/odom":
            return lambda message, topic_name=name: self._on_odom(topic_name, message)
        if name == DEFAULT_FASTLIO_ODOM_TOPIC:
            return lambda message, topic_name=name: self._on_passthrough(topic_name, message)
        if name in ("/imu", "/active_imu"):
            return lambda message, topic_name=name: self._on_imu(topic_name, message)
        if name == "/PowerVoltage":
            return lambda message, topic_name=name: self._on_voltage(topic_name, message)
        if name in (
            "/wheeltec/akm_state",
            "/wheeltec/control_debug",
            "/wheeltec/chassis_diagnostics",
        ):
            return lambda message, topic_name=name: self._on_passthrough(topic_name, message)
        raise ValueError(f"unsupported ROS topic: {name}")

    def _publish_snapshot(self) -> None:
        snapshot = self.snapshot()
        if self._on_snapshot is not None:
            self._on_snapshot(snapshot)

    def _on_odom(self, topic_name: str, message: dict) -> None:
        recv_time = time.time()
        header = message.get("header", {})
        twist = message.get("twist", {}).get("twist", {})
        linear = twist.get("linear", {})
        angular = twist.get("angular", {})
        pose = message.get("pose", {}).get("pose", {})
        position = pose.get("position", {})
        orientation = pose.get("orientation", {})
        with self._lock:
            self._snapshot.linear_x = float(linear.get("x", 0.0))
            self._snapshot.linear_y = float(linear.get("y", 0.0))
            self._snapshot.angular_z = float(angular.get("z", 0.0))
            self._snapshot.pose_x = float(position.get("x", 0.0))
            self._snapshot.pose_y = float(position.get("y", 0.0))
            self._snapshot.pose_z = float(position.get("z", 0.0))
            self._snapshot.orientation_x = float(orientation.get("x", 0.0))
            self._snapshot.orientation_y = float(orientation.get("y", 0.0))
            self._snapshot.orientation_z = float(orientation.get("z", 0.0))
            self._snapshot.orientation_w = float(orientation.get("w", 1.0))
            self._snapshot.odom_ros_time = _stamp_to_seconds(header)
            self._snapshot.odom_frame_id = str(header.get("frame_id", ""))
            self._snapshot.odom_recv_time = recv_time
            self._snapshot.last_topic = topic_name
            self._snapshot.frame_count += 1
        self._publish_snapshot()
        self._publish_message(topic_name, "nav_msgs/Odometry", message, recv_time)

    def _on_imu(self, topic_name: str, message: dict) -> None:
        recv_time = time.time()
        header = message.get("header", {})
        accel = message.get("linear_acceleration", {})
        gyro = message.get("angular_velocity", {})
        orientation = message.get("orientation", {})
        orientation_x = float(orientation.get("x", 0.0))
        orientation_y = float(orientation.get("y", 0.0))
        orientation_z = float(orientation.get("z", 0.0))
        orientation_w = float(orientation.get("w", 1.0))
        roll_deg, pitch_deg, yaw_deg = quaternion_to_euler_degrees(
            orientation_x,
            orientation_y,
            orientation_z,
            orientation_w,
        )
        with self._lock:
            reading = self._snapshot.active_imu if topic_name == "/active_imu" else self._snapshot.imu
            reading.accel_x = float(accel.get("x", 0.0))
            reading.accel_y = float(accel.get("y", 0.0))
            reading.accel_z = float(accel.get("z", 0.0))
            reading.gyro_x = float(gyro.get("x", 0.0))
            reading.gyro_y = float(gyro.get("y", 0.0))
            reading.gyro_z = float(gyro.get("z", 0.0))
            reading.orientation_x = orientation_x
            reading.orientation_y = orientation_y
            reading.orientation_z = orientation_z
            reading.orientation_w = orientation_w
            reading.roll_deg = roll_deg
            reading.pitch_deg = pitch_deg
            reading.yaw_deg = yaw_deg
            reading.ros_time = _stamp_to_seconds(header)
            reading.frame_id = str(header.get("frame_id", ""))
            reading.recv_time = recv_time
            reading.frame_count += 1
            if topic_name == "/imu":
                self._snapshot.accel_x = reading.accel_x
                self._snapshot.accel_y = reading.accel_y
                self._snapshot.accel_z = reading.accel_z
                self._snapshot.gyro_x = reading.gyro_x
                self._snapshot.gyro_y = reading.gyro_y
                self._snapshot.gyro_z = reading.gyro_z
                self._snapshot.imu_ros_time = reading.ros_time
                self._snapshot.imu_frame_id = reading.frame_id
                self._snapshot.imu_recv_time = reading.recv_time
            else:
                self._snapshot.active_imu_ros_time = reading.ros_time
                self._snapshot.active_imu_frame_id = reading.frame_id
                self._snapshot.active_imu_recv_time = reading.recv_time
            self._snapshot.last_topic = topic_name
            self._snapshot.frame_count += 1
        self._publish_snapshot()
        self._publish_message(topic_name, "sensor_msgs/Imu", message, recv_time)

    def _on_voltage(self, topic_name: str, message: dict) -> None:
        recv_time = time.time()
        with self._lock:
            self._snapshot.voltage = float(message.get("data", 0.0))
            self._snapshot.last_topic = topic_name
            self._snapshot.frame_count += 1
        self._publish_snapshot()
        self._publish_message(topic_name, "std_msgs/Float32", message, recv_time)

    def _on_passthrough(self, topic_name: str, message: dict) -> None:
        recv_time = time.time()
        with self._lock:
            self._snapshot.last_topic = topic_name
            self._snapshot.frame_count += 1
        self._publish_snapshot()
        self._publish_message(topic_name, self._topic_type(topic_name), message, recv_time)

    def _publish_message(self, topic_name: str, message_type: str, message: dict, recv_time: float) -> None:
        if self._on_message is None:
            return
        self._on_message(
            {
                "topic": topic_name,
                "message_type": message_type,
                "message": dict(message),
                "recv_time_epoch_s": float(recv_time),
            }
        )

    def _topic_type(self, topic_name: str) -> str:
        for name, message_type in self.SUBSCRIPTIONS:
            if name == topic_name:
                return message_type
        return ""


class RosBridgeWorker(QThread):
    """Qt thread wrapper for rosbridge websocket access."""

    snapshot_received = Signal(object)
    message_received = Signal(object)
    network_latency_measured = Signal(float)
    error_occurred = Signal(str)
    connection_changed = Signal(bool)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._session: RosBridgeSession | None = None
        self._running = False
        self._host = DEFAULT_ROSBRIDGE_HOST
        self._port = DEFAULT_ROSBRIDGE_PORT
        self._error_count = 0
        self._network_latency_ms: float | None = None

    @property
    def error_count(self) -> int:
        return self._error_count

    @property
    def frame_count(self) -> int:
        if self._session is None:
            return 0
        return self._session.snapshot().frame_count

    @property
    def network_latency_ms(self) -> float | None:
        return self._network_latency_ms

    def open_bridge(self, host: str, port: int = DEFAULT_ROSBRIDGE_PORT) -> None:
        if self.isRunning():
            return
        self._host = host
        self._port = int(port)
        self._running = True
        self.start()

    def close_bridge(self) -> None:
        self._running = False
        self.wait(2000)
        self.connection_changed.emit(False)

    def publish_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        try:
            if self._session is None:
                raise RuntimeError("rosbridge is not connected")
            self._session.publish_cmd_vel(linear_x, angular_z)
        except Exception as exc:
            self._error_count += 1
            self.error_occurred.emit(f"ROS 发送失败: {exc}")

    def publish_line_follow_control(self, linear_x: float, forward: bool, backward: bool) -> None:
        try:
            if self._session is None:
                raise RuntimeError("rosbridge is not connected")
            self._session.publish_line_follow_control(linear_x, forward, backward)
        except Exception as exc:
            self._error_count += 1
            self.error_occurred.emit(f"ROS PID 控制发送失败: {exc}")

    def publish_launch_manager_command(self, command: str) -> None:
        try:
            if self._session is None:
                raise RuntimeError("rosbridge is not connected")
            self._session.publish_launch_manager_command(command)
        except Exception as exc:
            self._error_count += 1
            self.error_occurred.emit(f"ROS launch 管理命令发送失败: {exc}")

    def latest_snapshot(self) -> RosSnapshot | None:
        if self._session is None:
            return None
        return self._session.snapshot()

    def run(self) -> None:
        try:
            self._session = RosBridgeSession(
                host=self._host,
                port=self._port,
                on_snapshot=self.snapshot_received.emit,
                on_message=self.message_received.emit,
            )
            self._session.connect()
            self.connection_changed.emit(True)
            next_latency_probe_s = 0.0
            while self._running:
                now = time.monotonic()
                if now >= next_latency_probe_s:
                    try:
                        self._network_latency_ms = self._session.measure_network_latency_ms(timeout=0.5)
                        self.network_latency_measured.emit(float(self._network_latency_ms))
                    except Exception:
                        self._network_latency_ms = None
                    next_latency_probe_s = now + 1.0
                time.sleep(0.05)
        except Exception as exc:
            self._error_count += 1
            self.error_occurred.emit(f"ROS 连接失败: {exc}")
        finally:
            if self._session is not None and self._session.connected:
                self._session.disconnect()
            self.connection_changed.emit(False)


def quaternion_to_euler_degrees(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
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
        pitch = asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)

    return degrees(roll), degrees(pitch), degrees(yaw)


def _stamp_to_seconds(header: dict) -> float:
    stamp = header.get("stamp", {}) if isinstance(header, dict) else {}
    if not isinstance(stamp, dict):
        return 0.0
    secs = stamp.get("secs", stamp.get("sec", 0))
    nsecs = stamp.get("nsecs", stamp.get("nanosec", 0))
    try:
        return float(secs) + float(nsecs) / 1_000_000_000.0
    except (TypeError, ValueError):
        return 0.0
