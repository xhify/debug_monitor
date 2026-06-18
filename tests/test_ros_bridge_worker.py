import os
import sys
import unittest
import json
from math import cos, pi, sin

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from ros_bridge_worker import RosBridgeSession, RosBridgeWorker, evaluate_rosbridge_health


class FakeRos:
    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self.run_called = False
        self.terminated = False
        self.closed = False

    def run(self) -> None:
        self.run_called = True

    def close(self) -> None:
        self.closed = True

    def terminate(self) -> None:
        self.terminated = True


class FakeRosMissingThreadOnTerminate(FakeRos):
    def terminate(self) -> None:
        self.terminated = True
        raise AttributeError("'TwistedEventLoopManager' object has no attribute '_thread'")


class FakeTopic:
    created: list["FakeTopic"] = []

    def __init__(self, ros: FakeRos, name: str, message_type: str) -> None:
        self.ros = ros
        self.name = name
        self.message_type = message_type
        self.callback = None
        self.published: list[dict] = []
        self.unsubscribed = False
        FakeTopic.created.append(self)

    def subscribe(self, callback) -> None:
        self.callback = callback

    def publish(self, message) -> None:
        self.published.append(dict(message))

    def unsubscribe(self) -> None:
        self.unsubscribed = True


class FakeService:
    calls: list[tuple[str, str, dict, float | None]] = []

    def __init__(self, ros: FakeRos, name: str, service_type: str) -> None:
        self.ros = ros
        self.name = name
        self.service_type = service_type

    def call(self, request, timeout=None):
        FakeService.calls.append((self.name, self.service_type, dict(request), timeout))
        return {"time": {"secs": 1, "nsecs": 0}}


class FakeServiceRequest(dict):
    pass


class RosBridgeSessionTests(unittest.TestCase):
    def setUp(self) -> None:
        FakeTopic.created.clear()
        FakeService.calls.clear()

    def test_connect_subscribes_to_core_topic_and_publish_topics_by_default(self) -> None:
        session = RosBridgeSession(
            host="192.168.0.14",
            port=9090,
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
        )

        session.connect()

        self.assertTrue(session.connected)
        self.assertEqual(session.ros.host, "192.168.0.14")
        self.assertEqual(session.ros.port, 9090)
        self.assertTrue(session.ros.run_called)
        self.assertEqual(
            [(topic.name, topic.message_type) for topic in FakeTopic.created],
            [
                ("/launch_manager/status", "std_msgs/String"),
                ("/cmd_vel", "geometry_msgs/Twist"),
                ("/line_follow_control", "simple_follower/LineFollowControl"),
                ("/launch_manager/command", "std_msgs/String"),
            ],
        )
        subscribed_names = [topic.name for topic in FakeTopic.created if topic.callback is not None]
        self.assertEqual(subscribed_names, ["/launch_manager/status"])

    def test_connect_subscribes_only_selected_data_topics(self) -> None:
        session = RosBridgeSession(
            host="192.168.0.14",
            port=9090,
            enabled_data_topics=["/odom", "/imu"],
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
        )

        session.connect()

        subscribed_names = [topic.name for topic in FakeTopic.created if topic.callback is not None]
        self.assertEqual(subscribed_names, ["/launch_manager/status", "/odom", "/imu"])
        self.assertNotIn("/active_imu", subscribed_names)

    def test_dynamic_fastlio_topic_emits_localization_sample(self) -> None:
        samples = []
        session = RosBridgeSession(
            host="192.168.0.14",
            port=9090,
            fastlio_odometry_topic="/custom_odom",
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
            on_localization_sample=samples.append,
        )
        session.connect()

        session.topic("/custom_odom").callback(
            {
                "header": {"stamp": {"secs": 10, "nsecs": 0}, "frame_id": "camera_init"},
                "child_frame_id": "body",
                "pose": {
                    "pose": {
                        "position": {"x": 1.25, "y": -0.5, "z": 0.1},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    }
                },
            }
        )

        self.assertEqual(len(samples), 1)
        self.assertEqual(samples[0].source, "/custom_odom")
        self.assertAlmostEqual(samples[0].x, 1.25)
        self.assertAlmostEqual(samples[0].y, -0.5)

    def test_replacing_fastlio_topic_unsubscribes_old_dynamic_topic(self) -> None:
        session = RosBridgeSession(
            host="192.168.0.14",
            port=9090,
            fastlio_odometry_topic="/Odometry",
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
        )
        session.connect()
        old_topic = session.topic("/Odometry")

        session.update_fastlio_odometry_topic("/fastlio/odom")

        self.assertTrue(old_topic.unsubscribed)
        self.assertEqual(session.topic("/fastlio/odom").message_type, "nav_msgs/Odometry")

    def test_fastlio_topic_shared_with_data_subscription_is_not_duplicated(self) -> None:
        session = RosBridgeSession(
            host="192.168.0.14",
            port=9090,
            enabled_data_topics=["/Odometry"],
            fastlio_odometry_topic="/Odometry",
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
        )

        session.connect()

        odometry_topics = [topic for topic in FakeTopic.created if topic.name == "/Odometry"]
        self.assertEqual(len(odometry_topics), 1)

    def test_enabling_current_fastlio_as_data_subscription_reuses_one_topic(self) -> None:
        session = RosBridgeSession(
            host="192.168.0.14",
            port=9090,
            fastlio_odometry_topic="/Odometry",
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
        )
        session.connect()
        dynamic_topic = session.topic("/Odometry")

        session.update_data_subscriptions(["/Odometry"])

        self.assertTrue(dynamic_topic.unsubscribed)
        active_topics = [
            topic
            for topic in FakeTopic.created
            if topic.name == "/Odometry" and not topic.unsubscribed
        ]
        self.assertEqual(len(active_topics), 1)

    def test_update_data_subscriptions_subscribes_and_unsubscribes_without_touching_core(self) -> None:
        session = RosBridgeSession(
            host="192.168.0.14",
            port=9090,
            enabled_data_topics=["/odom", "/imu"],
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
        )
        session.connect()
        odom_topic = session.topic("/odom")
        imu_topic = session.topic("/imu")
        core_topic = session.topic("/launch_manager/status")

        session.update_data_subscriptions(["/imu", "/active_imu"])

        self.assertTrue(odom_topic.unsubscribed)
        self.assertFalse(imu_topic.unsubscribed)
        self.assertFalse(core_topic.unsubscribed)
        self.assertIn("/active_imu", session._topics)
        self.assertIsNotNone(session.topic("/active_imu").callback)

    def test_update_data_subscriptions_rejects_unknown_topic(self) -> None:
        session = RosBridgeSession(
            host="192.168.0.14",
            port=9090,
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
        )

        with self.assertRaises(ValueError):
            session.update_data_subscriptions(["/not_a_data_topic"])

    def test_measure_network_latency_uses_rosapi_get_time_round_trip(self) -> None:
        times = iter([10.0, 10.012])
        session = RosBridgeSession(
            host="192.168.0.14",
            port=9090,
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
            service_factory=FakeService,
            service_request_factory=FakeServiceRequest,
            monotonic_clock=lambda: next(times),
        )
        session.connect()

        latency_ms = session.measure_network_latency_ms(timeout=0.25)

        self.assertAlmostEqual(latency_ms, 12.0)
        self.assertEqual(
            FakeService.calls,
            [("/rosapi/get_time", "rosapi/GetTime", {}, 0.25)],
        )

    def test_callbacks_update_latest_ros_state(self) -> None:
        session = RosBridgeSession(
            host="192.168.0.14",
            port=9090,
            enabled_data_topics=["/odom", "/imu", "/active_imu", "/PowerVoltage"],
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
        )
        session.connect()

        session.topic("/odom").callback(
            {
                "twist": {
                    "twist": {
                        "linear": {"x": 0.42, "y": 0.01, "z": 0.0},
                        "angular": {"x": 0.0, "y": 0.0, "z": -0.13},
                    }
                },
                "pose": {
                    "pose": {
                        "position": {"x": 1.0, "y": 2.0, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.1, "w": 0.99},
                    }
                },
            }
        )
        session.topic("/imu").callback(
            {
                "linear_acceleration": {"x": 1.1, "y": 1.2, "z": 9.8},
                "angular_velocity": {"x": 0.01, "y": 0.02, "z": 0.03},
                "orientation": {"x": 0.0, "y": 0.0, "z": sin(pi / 4), "w": cos(pi / 4)},
            }
        )
        session.topic("/active_imu").callback(
            {
                "linear_acceleration": {"x": -1.1, "y": -1.2, "z": -9.8},
                "angular_velocity": {"x": -0.01, "y": -0.02, "z": -0.03},
                "orientation": {"x": 0.0, "y": sin(pi / 4), "z": 0.0, "w": cos(pi / 4)},
            }
        )
        session.topic("/PowerVoltage").callback({"data": 25.8})

        snapshot = session.snapshot()
        self.assertEqual(snapshot.frame_count, 4)
        self.assertAlmostEqual(snapshot.linear_x, 0.42)
        self.assertAlmostEqual(snapshot.angular_z, -0.13)
        self.assertAlmostEqual(snapshot.pose_x, 1.0)
        self.assertAlmostEqual(snapshot.accel_z, 9.8)
        self.assertAlmostEqual(snapshot.gyro_z, 0.03)
        self.assertAlmostEqual(snapshot.imu.yaw_deg, 90.0)
        self.assertEqual(snapshot.imu.frame_count, 1)
        self.assertAlmostEqual(snapshot.active_imu.accel_z, -9.8)
        self.assertAlmostEqual(snapshot.active_imu.gyro_z, -0.03)
        self.assertAlmostEqual(snapshot.active_imu.pitch_deg, 90.0)
        self.assertEqual(snapshot.active_imu.frame_count, 1)
        self.assertAlmostEqual(snapshot.voltage, 25.8)

    def test_odom_callback_preserves_ros_header_stamp_and_frame(self) -> None:
        session = RosBridgeSession(
            host="192.168.0.14",
            port=9090,
            enabled_data_topics=["/odom"],
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
        )
        session.connect()

        session.topic("/odom").callback(
            {
                "header": {
                    "stamp": {"secs": 100, "nsecs": 250000000},
                    "frame_id": "odom",
                },
                "twist": {"twist": {"linear": {"x": 0.1}, "angular": {"z": 0.2}}},
                "pose": {"pose": {"position": {"x": 1.0}, "orientation": {"w": 1.0}}},
            }
        )

        snapshot = session.snapshot()
        self.assertAlmostEqual(snapshot.odom_ros_time, 100.25)
        self.assertEqual(snapshot.odom_frame_id, "odom")
        self.assertGreater(snapshot.odom_recv_time, 0.0)

    def test_imu_callbacks_preserve_each_topic_ros_header_stamp_and_frame(self) -> None:
        session = RosBridgeSession(
            host="192.168.0.14",
            port=9090,
            enabled_data_topics=["/imu", "/active_imu"],
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
        )
        session.connect()

        session.topic("/imu").callback(
            {
                "header": {
                    "stamp": {"secs": 10, "nsecs": 500000000},
                    "frame_id": "imu_link",
                },
                "linear_acceleration": {"z": 9.8},
                "angular_velocity": {"z": 0.03},
                "orientation": {"w": 1.0},
            }
        )
        session.topic("/active_imu").callback(
            {
                "header": {
                    "stamp": {"sec": 11, "nanosec": 750000000},
                    "frame_id": "active_imu_link",
                },
                "linear_acceleration": {"z": -9.8},
                "angular_velocity": {"z": -0.03},
                "orientation": {"w": 1.0},
            }
        )

        snapshot = session.snapshot()
        self.assertAlmostEqual(snapshot.imu_ros_time, 10.5)
        self.assertEqual(snapshot.imu_frame_id, "imu_link")
        self.assertGreater(snapshot.imu_recv_time, 0.0)
        self.assertAlmostEqual(snapshot.imu.ros_time, 10.5)
        self.assertEqual(snapshot.imu.frame_id, "imu_link")
        self.assertGreater(snapshot.imu.recv_time, 0.0)
        self.assertAlmostEqual(snapshot.active_imu_ros_time, 11.75)
        self.assertEqual(snapshot.active_imu_frame_id, "active_imu_link")
        self.assertGreater(snapshot.active_imu_recv_time, 0.0)
        self.assertAlmostEqual(snapshot.active_imu.ros_time, 11.75)
        self.assertEqual(snapshot.active_imu.frame_id, "active_imu_link")
        self.assertGreater(snapshot.active_imu.recv_time, 0.0)

    def test_missing_header_stamp_defaults_ros_time_to_zero(self) -> None:
        session = RosBridgeSession(
            host="192.168.0.14",
            port=9090,
            enabled_data_topics=["/odom", "/imu"],
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
        )
        session.connect()

        session.topic("/odom").callback({})
        session.topic("/imu").callback({"orientation": {"w": 1.0}})

        snapshot = session.snapshot()
        self.assertEqual(snapshot.odom_ros_time, 0.0)
        self.assertEqual(snapshot.odom_frame_id, "")
        self.assertGreater(snapshot.odom_recv_time, 0.0)
        self.assertEqual(snapshot.imu_ros_time, 0.0)
        self.assertEqual(snapshot.imu_frame_id, "")
        self.assertGreater(snapshot.imu_recv_time, 0.0)

    def test_publish_cmd_vel_sends_twist_message(self) -> None:
        session = RosBridgeSession(
            host="192.168.0.14",
            port=9090,
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
        )
        session.connect()

        session.publish_cmd_vel(linear_x=0.2, angular_z=-0.4)

        self.assertEqual(
            session.topic("/cmd_vel").published[-1],
            {
                "linear": {"x": 0.2, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": -0.4},
            },
        )

    def test_publish_line_follow_control_sends_pid_control_message(self) -> None:
        session = RosBridgeSession(
            host="192.168.0.14",
            port=9090,
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
        )
        session.connect()

        session.publish_line_follow_control(linear_x=0.2, forward=True, backward=False)
        session.publish_line_follow_control(linear_x=0.0, forward=False, backward=False)

        self.assertEqual(
            session.topic("/line_follow_control").published[-2],
            {
                "enable": True,
                "h_min": 18,
                "s_min": 60,
                "v_min": 60,
                "h_max": 34,
                "s_max": 255,
                "v_max": 255,
                "linear_x": 0.2,
                "angular_scale": 0.001,
                "forward": True,
                "backward": False,
            },
        )
        self.assertEqual(
            session.topic("/line_follow_control").published[-1],
            {
                "enable": False,
                "h_min": 18,
                "s_min": 60,
                "v_min": 60,
                "h_max": 34,
                "s_max": 255,
                "v_max": 255,
                "linear_x": 0.0,
                "angular_scale": 0.001,
                "forward": False,
                "backward": False,
            },
        )

    def test_publish_launch_manager_command_sends_std_msgs_string(self) -> None:
        session = RosBridgeSession(
            host="192.168.0.14",
            port=9090,
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
        )
        session.connect()

        session.publish_launch_manager_command("start fastlio fast_lio mapping_c16.launch")

        self.assertEqual(
            session.topic("/launch_manager/command").published[-1],
            {"data": "start fastlio fast_lio mapping_c16.launch"},
        )

    def test_disconnect_unsubscribes_topics_and_closes_ros_without_terminating_reactor(self) -> None:
        session = RosBridgeSession(
            host="192.168.0.14",
            port=9090,
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
        )
        session.connect()

        session.disconnect()

        self.assertFalse(session.connected)
        self.assertTrue(session.ros.closed)
        self.assertFalse(session.ros.terminated)
        subscribed_topics = [topic for topic in FakeTopic.created if topic.callback is not None]
        publish_topics = [topic for topic in FakeTopic.created if topic.callback is None]
        self.assertTrue(all(topic.unsubscribed for topic in subscribed_topics))
        self.assertFalse(any(topic.unsubscribed for topic in publish_topics))

    def test_disconnect_tolerates_roslibpy_missing_thread_terminate_bug(self) -> None:
        session = RosBridgeSession(
            host="192.168.0.14",
            port=9090,
            ros_factory=FakeRosMissingThreadOnTerminate,
            topic_factory=FakeTopic,
        )
        session.connect()

        session.disconnect()

        self.assertFalse(session.connected)
        self.assertTrue(session.ros.closed)
        self.assertFalse(session.ros.terminated)
        subscribed_topics = [topic for topic in FakeTopic.created if topic.callback is not None]
        publish_topics = [topic for topic in FakeTopic.created if topic.callback is None]
        self.assertTrue(all(topic.unsubscribed for topic in subscribed_topics))
        self.assertFalse(any(topic.unsubscribed for topic in publish_topics))

    def test_session_can_connect_again_after_disconnect_without_restarting_reactor(self) -> None:
        session = RosBridgeSession(
            host="192.168.0.14",
            port=9090,
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
        )
        session.connect()
        first_ros = session.ros

        session.disconnect()
        session.connect()

        self.assertTrue(session.connected)
        self.assertTrue(first_ros.closed)
        self.assertFalse(first_ros.terminated)
        self.assertIsNot(session.ros, first_ros)
        self.assertTrue(session.ros.run_called)

    def test_launch_manager_status_parses_json_and_reports_invalid_json(self) -> None:
        statuses = []
        events = []
        session = RosBridgeSession(
            host="192.168.0.14",
            port=9090,
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
            on_launch_manager_status=statuses.append,
            on_message=events.append,
        )
        session.connect()

        payload = {"rosbag": {"active": True, "session_id": "session_1"}}
        session.topic("/launch_manager/status").callback({"data": json.dumps(payload)})
        session.topic("/launch_manager/status").callback({"data": "not-json"})

        self.assertEqual(statuses, [payload])
        self.assertEqual(events[0]["topic"], "/launch_manager/status")
        self.assertEqual(events[0]["message"], payload)
        self.assertEqual(events[1]["message"]["error"], "invalid_json")

    def test_launch_manager_status_forwards_complete_nested_payload(self) -> None:
        statuses = []
        session = RosBridgeSession(
            host="192.168.0.14",
            port=9090,
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
            on_launch_manager_status=statuses.append,
        )
        session.connect()

        payload = {
            "level": "state",
            "message": "ok",
            "data": {
                "running": ["rosbag"],
                "detail": {"rosbag": "recording"},
                "rosbag": {"active": True},
                "rosbag_library": {"sessions": []},
                "rosbag_detail": {"session_id": "session_1"},
                "last_command": {"ok": True},
            },
        }
        session.topic("/launch_manager/status").callback({"data": json.dumps(payload)})

        self.assertEqual(statuses, [payload])

    def test_rosbag_command_helpers_publish_expected_json(self) -> None:
        session = RosBridgeSession(
            host="192.168.0.14",
            port=9090,
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
        )
        session.connect()

        session.request_rosbag_start({"session_id": "session_1", "topics": ["/imu"]})
        session.request_rosbag_stop("session_1")
        session.request_rosbag_stop("")
        session.request_rosbag_list("/bags")
        session.request_rosbag_inspect("session_1")
        session.request_rosbag_trash("session_1")
        session.request_rosbag_delete("session_1", "session_1")
        session.request_rosbag_delete("session_2", "wrong")
        session.request_launch_manager_status()

        commands = [
            json.loads(message["data"])
            for message in session.topic("/launch_manager/command").published
        ]
        self.assertEqual(len(commands), 7)
        self.assertEqual(commands[0]["action"], "start_rosbag")
        self.assertEqual(commands[0]["topics"], ["/imu"])
        self.assertEqual(commands[1], {"action": "stop_rosbag", "session_id": "session_1"})
        self.assertFalse(any(command == {"action": "stop_rosbag", "session_id": ""} for command in commands))
        self.assertEqual(commands[2], {"action": "list_rosbags", "bag_dir": "/bags"})
        self.assertEqual(commands[3], {"action": "inspect_rosbag", "session_id": "session_1"})
        self.assertEqual(commands[4], {"action": "trash_rosbag", "session_id": "session_1"})
        self.assertEqual(
            commands[5],
            {"action": "delete_rosbag", "session_id": "session_1", "confirm": "session_1"},
        )
        self.assertEqual(commands[6], {"action": "query_status"})
        self.assertFalse(any(command.get("session_id") == "session_2" for command in commands))


class RosbridgeHealthTests(unittest.TestCase):
    def test_launch_manager_status_does_not_reset_data_silence_clock(self) -> None:
        worker = RosBridgeWorker()
        events = []
        worker.message_received.connect(events.append)

        worker._on_session_message({"topic": "/launch_manager/status"})

        self.assertIsNone(worker._last_message_monotonic)
        self.assertEqual(events, [{"topic": "/launch_manager/status"}])

        worker._on_session_message({"topic": "/imu"})

        self.assertIsNotNone(worker._last_message_monotonic)

    def test_single_probe_failure_keeps_bridge_connecting_before_threshold(self) -> None:
        health = evaluate_rosbridge_health(
            connected=True,
            restarting=False,
            probe_ok=False,
            consecutive_failures=1,
            failure_threshold=3,
            selected_topic_count=1,
            last_message_age_s=0.2,
            data_silence_s=3.0,
            latency_ms=None,
        )

        self.assertEqual(health.state, "connecting")

    def test_consecutive_probe_failures_mark_bridge_abnormal(self) -> None:
        health = evaluate_rosbridge_health(
            connected=True,
            restarting=False,
            probe_ok=False,
            consecutive_failures=3,
            failure_threshold=3,
            selected_topic_count=1,
            last_message_age_s=0.2,
            data_silence_s=3.0,
            latency_ms=None,
        )

        self.assertEqual(health.state, "abnormal")

    def test_healthy_bridge_without_recent_selected_topic_data_is_silent(self) -> None:
        health = evaluate_rosbridge_health(
            connected=True,
            restarting=False,
            probe_ok=True,
            consecutive_failures=0,
            failure_threshold=3,
            selected_topic_count=2,
            last_message_age_s=4.0,
            data_silence_s=3.0,
            latency_ms=12.5,
        )

        self.assertEqual(health.state, "data_silent")
        self.assertTrue(health.connected)


if __name__ == "__main__":
    unittest.main()
