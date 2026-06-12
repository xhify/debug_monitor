import os
import sys
import unittest
import json
from math import cos, pi, sin

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from ros_bridge_worker import RosBridgeSession


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

    def test_connect_subscribes_to_standard_robot_topics(self) -> None:
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
                ("/odom", "nav_msgs/Odometry"),
                ("/Odometry", "nav_msgs/Odometry"),
                ("/imu", "sensor_msgs/Imu"),
                ("/active_imu", "sensor_msgs/Imu"),
                ("/PowerVoltage", "std_msgs/Float32"),
                ("/wheeltec/akm_state", "turn_on_wheeltec_robot/AkmState"),
                ("/wheeltec/control_debug", "turn_on_wheeltec_robot/ControlDebug"),
                ("/wheeltec/chassis_diagnostics", "turn_on_wheeltec_robot/ChassisDiagnostics"),
                ("/launch_manager/status", "std_msgs/String"),
                ("/cmd_vel", "geometry_msgs/Twist"),
                ("/line_follow_control", "simple_follower/LineFollowControl"),
                ("/launch_manager/command", "std_msgs/String"),
            ],
        )

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
        self.assertTrue(all(topic.unsubscribed for topic in FakeTopic.created[:9]))

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
        self.assertTrue(all(topic.unsubscribed for topic in FakeTopic.created[:9]))

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
        session.request_rosbag_list("/bags")
        session.request_rosbag_inspect("session_1")
        session.request_rosbag_trash("session_1")
        session.request_rosbag_delete("session_1", "session_1")
        session.request_launch_manager_status()

        commands = [
            json.loads(message["data"])
            for message in session.topic("/launch_manager/command").published[-7:]
        ]
        self.assertEqual(commands[0]["action"], "start_rosbag")
        self.assertEqual(commands[0]["topics"], ["/imu"])
        self.assertEqual(commands[1], {"action": "stop_rosbag", "session_id": "session_1"})
        self.assertEqual(commands[2], {"action": "list_rosbags", "bag_dir": "/bags"})
        self.assertEqual(commands[3], {"action": "inspect_rosbag", "session_id": "session_1"})
        self.assertEqual(commands[4], {"action": "trash_rosbag", "session_id": "session_1"})
        self.assertEqual(
            commands[5],
            {"action": "delete_rosbag", "session_id": "session_1", "confirm": "session_1"},
        )
        self.assertEqual(commands[6], {"action": "query_status"})


if __name__ == "__main__":
    unittest.main()
