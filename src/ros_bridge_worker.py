"""rosbridge/roslibpy connection helpers and Qt worker."""

from __future__ import annotations

from dataclasses import dataclass, field, fields
import json
from math import asin, atan2, copysign, degrees, pi, sqrt
import threading
import time
from typing import Any, Callable, Iterable

from PySide6.QtCore import QThread, Signal

from app_config import (
    DEFAULT_FASTLIO_ODOM_TOPIC,
    DEFAULT_ROSBRIDGE_HOST,
    DEFAULT_ROSBRIDGE_PORT,
    ROSBRIDGE_DATA_SILENCE_S,
    ROSBRIDGE_DATA_TOPIC_OPTIONS,
    ROSBRIDGE_HEALTH_FAILURE_THRESHOLD,
)
from ros_odometry_client import parse_odometry_message


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


@dataclass(frozen=True, slots=True)
class RosbridgeHealth:
    state: str
    connected: bool
    latency_ms: float | None
    consecutive_failures: int
    last_message_age_s: float | None
    detail: str


def normalize_ros_topic(topic_name: str, fallback: str = DEFAULT_FASTLIO_ODOM_TOPIC) -> str:
    topic = str(topic_name or "").strip() or str(fallback).strip()
    return topic if topic.startswith("/") else f"/{topic}"


def evaluate_rosbridge_health(
    *,
    connected: bool,
    restarting: bool,
    probe_ok: bool,
    consecutive_failures: int,
    failure_threshold: int,
    selected_topic_count: int,
    last_message_age_s: float | None,
    data_silence_s: float,
    latency_ms: float | None,
) -> RosbridgeHealth:
    if restarting:
        state, detail = "restarting", "正在重启并恢复连接"
    elif not connected:
        state, detail = "disconnected", "没有活动 WebSocket 会话"
    elif not probe_ok:
        state = "abnormal" if consecutive_failures >= max(1, failure_threshold) else "connecting"
        detail = "ROS API 健康探测失败"
    elif (
        selected_topic_count > 0
        and last_message_age_s is not None
        and last_message_age_s >= max(0.0, data_silence_s)
    ):
        state, detail = "data_silent", "ROS API 正常，但订阅数据静默"
    else:
        state, detail = "online", "ROS API 健康探测正常"
    return RosbridgeHealth(
        state=state,
        connected=bool(connected),
        latency_ms=latency_ms,
        consecutive_failures=max(0, int(consecutive_failures)),
        last_message_age_s=last_message_age_s,
        detail=detail,
    )


class RosBridgeSession:
    """Small testable wrapper around roslibpy topic subscription and publish."""

    DATA_SUBSCRIPTIONS = tuple(
        (str(option["topic"]), str(option["type"])) for option in ROSBRIDGE_DATA_TOPIC_OPTIONS
    )
    CORE_SUBSCRIPTIONS = (
        ("/launch_manager/status", "std_msgs/String"),
    )
    SUBSCRIPTIONS = DATA_SUBSCRIPTIONS + CORE_SUBSCRIPTIONS
    CMD_VEL_TOPIC = ("/cmd_vel", "geometry_msgs/Twist")
    LINE_FOLLOW_CONTROL_TOPIC = ("/line_follow_control", "simple_follower/LineFollowControl")
    LAUNCH_MANAGER_COMMAND_TOPIC = ("/launch_manager/command", "std_msgs/String")
    PUBLISH_TOPICS = (CMD_VEL_TOPIC, LINE_FOLLOW_CONTROL_TOPIC, LAUNCH_MANAGER_COMMAND_TOPIC)

    def __init__(
        self,
        host: str,
        port: int,
        *,
        enabled_data_topics: Iterable[str] | None = None,
        fastlio_odometry_topic: str = "",
        fastlio_subscription_enabled: bool = False,
        ros_factory: Callable[[str, int], Any] | None = None,
        topic_factory: Callable[[Any, str, str], Any] | None = None,
        message_factory: Callable[[dict], Any] | None = None,
        service_factory: Callable[[Any, str, str], Any] | None = None,
        service_request_factory: Callable[[dict], Any] | None = None,
        monotonic_clock: Callable[[], float] | None = None,
        on_snapshot: Callable[[RosSnapshot], None] | None = None,
        on_message: Callable[[dict[str, Any]], None] | None = None,
        on_launch_manager_status: Callable[[dict], None] | None = None,
        on_localization_sample: Callable[[Any], None] | None = None,
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
        self._on_launch_manager_status_callback = on_launch_manager_status
        self._on_localization_sample = on_localization_sample
        self._lock = threading.Lock()
        self._snapshot = RosSnapshot()
        self._topics: dict[str, Any] = {}
        self._enabled_data_topics = self._validated_data_topics(enabled_data_topics or [])
        self._subscribed_data_topics: dict[str, Any] = {}
        self._fastlio_odometry_topic = (
            normalize_ros_topic(fastlio_odometry_topic) if fastlio_odometry_topic else ""
        )
        self._fastlio_subscription_enabled = bool(fastlio_subscription_enabled)
        self._fastlio_topic: Any | None = None
        self._core_topics: dict[str, Any] = {}
        self._publish_topics: dict[str, Any] = {}
        self._time_service: Any | None = None
        self.ros: Any | None = None
        self.connected = False

    def connect(self) -> None:
        if self.connected:
            return
        self._load_default_factories()
        self.ros = self._ros_factory(self.host, self.port)
        self.ros.run()

        for name, message_type in self.CORE_SUBSCRIPTIONS:
            self._subscribe_topic(name, message_type, self._core_topics)

        for name, message_type in self.DATA_SUBSCRIPTIONS:
            if name in self._enabled_data_topics:
                self._subscribe_topic(name, message_type, self._subscribed_data_topics)
        for name, message_type in self.PUBLISH_TOPICS:
            topic = self._topic_factory(self.ros, name, message_type)
            self._publish_topics[name] = topic
            self._topics[name] = topic
        if self._service_factory is not None:
            self._time_service = self._service_factory(self.ros, "/rosapi/get_time", "rosapi/GetTime")
        self.connected = True
        self._subscribe_fastlio_topic()

    def disconnect(self) -> None:
        for topic in list(self._subscribed_data_topics.values()):
            topic.unsubscribe()
        for topic in list(self._core_topics.values()):
            topic.unsubscribe()
        if self._fastlio_topic is not None:
            self._fastlio_topic.unsubscribe()
        if self.ros is not None:
            self.ros.close()
        self._topics.clear()
        self._subscribed_data_topics.clear()
        self._core_topics.clear()
        self._publish_topics.clear()
        self._fastlio_topic = None
        self._time_service = None
        self.connected = False

    def update_data_subscriptions(self, enabled_data_topics: Iterable[str]) -> None:
        enabled = self._validated_data_topics(enabled_data_topics)
        self._enabled_data_topics = enabled
        if not self.connected:
            return

        to_remove = set(self._subscribed_data_topics) - enabled
        to_add = enabled - set(self._subscribed_data_topics)
        if self._fastlio_odometry_topic in to_add and self._fastlio_topic is not None:
            self._fastlio_topic.unsubscribe()
            self._topics.pop(self._fastlio_odometry_topic, None)
            self._fastlio_topic = None
        for name in to_remove:
            topic = self._subscribed_data_topics.pop(name)
            topic.unsubscribe()
            self._topics.pop(name, None)

        data_types = dict(self.DATA_SUBSCRIPTIONS)
        for name in self._data_topic_order(to_add):
            self._subscribe_topic(name, data_types[name], self._subscribed_data_topics)
        self._subscribe_fastlio_topic()

    def update_fastlio_odometry_topic(self, topic_name: str) -> None:
        normalized = normalize_ros_topic(topic_name)
        if normalized == self._fastlio_odometry_topic:
            return
        old_name = self._fastlio_odometry_topic
        if self._fastlio_topic is not None:
            self._fastlio_topic.unsubscribe()
            self._topics.pop(old_name, None)
            self._fastlio_topic = None
        self._fastlio_odometry_topic = normalized
        self._subscribe_fastlio_topic()

    def update_fastlio_subscription_enabled(self, enabled: bool) -> None:
        enabled = bool(enabled)
        if enabled == self._fastlio_subscription_enabled:
            return
        self._fastlio_subscription_enabled = enabled
        if not enabled:
            if self._fastlio_topic is not None:
                self._fastlio_topic.unsubscribe()
                self._topics.pop(self._fastlio_odometry_topic, None)
                self._fastlio_topic = None
            return
        self._subscribe_fastlio_topic()

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

    def request_rosbag_start(self, config: dict) -> None:
        payload = dict(config)
        payload["action"] = "start_rosbag"
        self._publish_launch_manager_json(payload)

    def request_rosbag_stop(self, session_id: str) -> None:
        if not session_id:
            return
        self._publish_launch_manager_json({"action": "stop_rosbag", "session_id": session_id})

    def request_rosbag_list(self, bag_dir: str = "/home/wheeltec/bags") -> None:
        self._publish_launch_manager_json({"action": "list_rosbags", "bag_dir": bag_dir})

    def request_rosbag_inspect(self, session_id: str) -> None:
        self._publish_launch_manager_json({"action": "inspect_rosbag", "session_id": session_id})

    def request_rosbag_trash(self, session_id: str) -> None:
        self._publish_launch_manager_json({"action": "trash_rosbag", "session_id": session_id})

    def request_rosbag_delete(self, session_id: str, confirm: str) -> None:
        if confirm != session_id:
            return
        self._publish_launch_manager_json(
            {"action": "delete_rosbag", "session_id": session_id, "confirm": confirm}
        )

    def request_launch_manager_status(self) -> None:
        self._publish_launch_manager_json({"action": "query_status"})

    def _publish_launch_manager_json(self, payload: dict[str, Any]) -> None:
        self.publish_launch_manager_command(json.dumps(payload, ensure_ascii=False))

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

    def _subscribe_topic(self, name: str, message_type: str, destination: dict[str, Any]) -> None:
        topic = self._topic_factory(self.ros, name, message_type)
        topic.subscribe(self._callback_for(name))
        destination[name] = topic
        self._topics[name] = topic

    def _subscribe_fastlio_topic(self) -> None:
        if (
            not self.connected
            or self.ros is None
            or not self._fastlio_odometry_topic
            or not self._fastlio_subscription_enabled
            or self._fastlio_odometry_topic in self._subscribed_data_topics
            or self._fastlio_topic is not None
        ):
            return
        topic = self._topic_factory(
            self.ros,
            self._fastlio_odometry_topic,
            "nav_msgs/Odometry",
        )
        topic.subscribe(self._callback_for(self._fastlio_odometry_topic))
        self._fastlio_topic = topic
        self._topics[self._fastlio_odometry_topic] = topic

    @classmethod
    def _validated_data_topics(cls, enabled_data_topics: Iterable[str]) -> set[str]:
        allowed = {name for name, _message_type in cls.DATA_SUBSCRIPTIONS}
        selected = {str(topic) for topic in enabled_data_topics}
        unknown = selected - allowed
        if unknown:
            raise ValueError(f"unsupported ROS data topic: {', '.join(sorted(unknown))}")
        return selected

    @classmethod
    def _data_topic_order(cls, topics: Iterable[str]) -> list[str]:
        selected = set(topics)
        return [name for name, _message_type in cls.DATA_SUBSCRIPTIONS if name in selected]

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
        if name == self._fastlio_odometry_topic:
            return lambda message, topic_name=name: self._on_dynamic_odometry(topic_name, message)
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
        if name == "/launch_manager/status":
            return lambda message, topic_name=name: self._on_launch_manager_status(topic_name, message)
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
        self._publish_localization_sample_if_selected(topic_name, message, recv_time)
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
        self._publish_localization_sample_if_selected(topic_name, message, recv_time)
        self._publish_message(topic_name, self._topic_type(topic_name), message, recv_time)

    def _on_dynamic_odometry(self, topic_name: str, message: dict) -> None:
        recv_time = time.time()
        with self._lock:
            self._snapshot.last_topic = topic_name
            self._snapshot.frame_count += 1
        self._publish_snapshot()
        self._publish_localization_sample_if_selected(topic_name, message, recv_time)
        self._publish_message(topic_name, "nav_msgs/Odometry", message, recv_time)

    def _publish_localization_sample_if_selected(
        self,
        topic_name: str,
        message: dict,
        recv_time: float,
    ) -> None:
        if (
            not self._fastlio_subscription_enabled
            or topic_name != self._fastlio_odometry_topic
            or self._on_localization_sample is None
        ):
            return
        self._on_localization_sample(
            parse_odometry_message(
                message,
                source=topic_name,
                recv_time=recv_time,
            )
        )

    def _on_launch_manager_status(self, topic_name: str, message: dict) -> None:
        recv_time = time.time()
        raw_data = message.get("data", "")
        try:
            payload = json.loads(raw_data) if isinstance(raw_data, str) else {}
            if not isinstance(payload, dict):
                payload = {}
        except json.JSONDecodeError as exc:
            payload = {"error": "invalid_json", "raw": str(raw_data), "message": str(exc)}
        else:
            if self._on_launch_manager_status_callback is not None:
                self._on_launch_manager_status_callback(payload)
        self._publish_message(topic_name, "std_msgs/String", payload, recv_time)

    def _publish_message(self, topic_name: str, message_type: str, message: dict, recv_time: float) -> None:
        if self._on_message is None:
            return
        event = {
            "topic": topic_name,
            "message_type": message_type,
            "message": dict(message),
            "recv_time_epoch_s": float(recv_time),
        }
        if topic_name == self._fastlio_odometry_topic:
            event["localization_sample_emitted"] = True
        self._on_message(event)

    def _topic_type(self, topic_name: str) -> str:
        for name, message_type in self.SUBSCRIPTIONS:
            if name == topic_name:
                return message_type
        return ""


class RosBridgeWorker(QThread):
    """Qt thread wrapper for rosbridge websocket access."""

    snapshot_received = Signal(object)
    message_received = Signal(object)
    launch_manager_status_received = Signal(object)
    network_latency_measured = Signal(float)
    localization_sample_received = Signal(object)
    health_changed = Signal(object)
    error_occurred = Signal(str)
    connection_changed = Signal(bool)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._session: RosBridgeSession | None = None
        self._running = False
        self._host = DEFAULT_ROSBRIDGE_HOST
        self._port = DEFAULT_ROSBRIDGE_PORT
        self._enabled_data_topics: list[str] = []
        self._fastlio_odometry_topic = DEFAULT_FASTLIO_ODOM_TOPIC
        self._fastlio_subscription_enabled = False
        self._error_count = 0
        self._network_latency_ms: float | None = None
        self._last_message_monotonic: float | None = None
        self._consecutive_health_failures = 0
        self._probe_ok = False
        self._restarting = False

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

    def open_bridge(
        self,
        host: str,
        port: int = DEFAULT_ROSBRIDGE_PORT,
        enabled_data_topics: Iterable[str] | None = None,
    ) -> None:
        self._enabled_data_topics = list(enabled_data_topics or [])
        if self.isRunning():
            return
        self._host = host
        self._port = int(port)
        self._running = True
        self.start()

    def update_data_subscriptions(self, enabled_data_topics: Iterable[str]) -> None:
        self._enabled_data_topics = list(enabled_data_topics)
        try:
            if self._session is not None and self._session.connected:
                self._session.update_data_subscriptions(self._enabled_data_topics)
        except Exception as exc:
            self._error_count += 1
            self.error_occurred.emit(f"ROS 订阅更新失败: {exc}")

    def update_fastlio_odometry_topic(self, topic_name: str) -> None:
        self._fastlio_odometry_topic = normalize_ros_topic(topic_name)
        try:
            if self._session is not None and self._session.connected:
                self._session.update_fastlio_odometry_topic(self._fastlio_odometry_topic)
        except Exception as exc:
            self._error_count += 1
            self.error_occurred.emit(f"FAST-LIO topic 更新失败: {exc}")

    def update_fastlio_subscription_enabled(self, enabled: bool) -> None:
        self._fastlio_subscription_enabled = bool(enabled)
        try:
            if self._session is not None and self._session.connected:
                self._session.update_fastlio_subscription_enabled(
                    self._fastlio_subscription_enabled
                )
        except Exception as exc:
            self._error_count += 1
            self.error_occurred.emit(f"FAST-LIO 订阅更新失败: {exc}")

    def set_restarting(self, restarting: bool) -> None:
        self._restarting = bool(restarting)
        self._emit_health()

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

    def request_rosbag_start(self, config: dict) -> None:
        self._request_session_command("request_rosbag_start", config)

    def request_rosbag_stop(self, session_id: str) -> None:
        self._request_session_command("request_rosbag_stop", session_id)

    def request_rosbag_list(self, bag_dir: str = "/home/wheeltec/bags") -> None:
        self._request_session_command("request_rosbag_list", bag_dir)

    def request_rosbag_inspect(self, session_id: str) -> None:
        self._request_session_command("request_rosbag_inspect", session_id)

    def request_rosbag_trash(self, session_id: str) -> None:
        self._request_session_command("request_rosbag_trash", session_id)

    def request_rosbag_delete(self, session_id: str, confirm: str) -> None:
        self._request_session_command("request_rosbag_delete", session_id, confirm)

    def request_launch_manager_status(self) -> None:
        self._request_session_command("request_launch_manager_status")

    def _request_session_command(self, method_name: str, *args) -> None:
        try:
            if self._session is None:
                raise RuntimeError("rosbridge is not connected")
            getattr(self._session, method_name)(*args)
        except Exception as exc:
            self._error_count += 1
            self.error_occurred.emit(f"ROS rosbag 命令发送失败: {exc}")

    def latest_snapshot(self) -> RosSnapshot | None:
        if self._session is None:
            return None
        return self._session.snapshot()

    def run(self) -> None:
        try:
            self._session = RosBridgeSession(
                host=self._host,
                port=self._port,
                enabled_data_topics=self._enabled_data_topics,
                fastlio_odometry_topic=self._fastlio_odometry_topic,
                fastlio_subscription_enabled=self._fastlio_subscription_enabled,
                on_snapshot=self.snapshot_received.emit,
                on_message=self._on_session_message,
                on_launch_manager_status=self.launch_manager_status_received.emit,
                on_localization_sample=self.localization_sample_received.emit,
            )
            self._session.connect()
            self._last_message_monotonic = time.monotonic()
            self.connection_changed.emit(True)
            next_latency_probe_s = 0.0
            while self._running:
                now = time.monotonic()
                if now >= next_latency_probe_s:
                    try:
                        self._network_latency_ms = self._session.measure_network_latency_ms(timeout=0.5)
                        self._probe_ok = True
                        self._consecutive_health_failures = 0
                        self.network_latency_measured.emit(float(self._network_latency_ms))
                    except Exception:
                        self._network_latency_ms = None
                        self._probe_ok = False
                        self._consecutive_health_failures += 1
                    self._emit_health()
                    next_latency_probe_s = now + 1.0
                time.sleep(0.05)
        except Exception as exc:
            self._error_count += 1
            self.error_occurred.emit(f"ROS 连接失败: {exc}")
        finally:
            if self._session is not None and self._session.connected:
                self._session.disconnect()
            self._probe_ok = False
            self.connection_changed.emit(False)
            self._emit_health()

    def _on_session_message(self, event: dict[str, Any]) -> None:
        if event.get("topic") != "/launch_manager/status":
            self._last_message_monotonic = time.monotonic()
        self.message_received.emit(event)

    def _emit_health(self) -> None:
        now = time.monotonic()
        last_message_age_s = (
            None
            if self._last_message_monotonic is None
            else max(0.0, now - self._last_message_monotonic)
        )
        connected = self._session is not None and self._session.connected
        self.health_changed.emit(
            evaluate_rosbridge_health(
                connected=connected,
                restarting=self._restarting,
                probe_ok=self._probe_ok,
                consecutive_failures=self._consecutive_health_failures,
                failure_threshold=ROSBRIDGE_HEALTH_FAILURE_THRESHOLD,
                selected_topic_count=len(self._enabled_data_topics)
                + int(
                    self._fastlio_subscription_enabled
                    and bool(self._fastlio_odometry_topic)
                ),
                last_message_age_s=last_message_age_s,
                data_silence_s=ROSBRIDGE_DATA_SILENCE_S,
                latency_ms=self._network_latency_ms,
            )
        )


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
