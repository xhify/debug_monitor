import os
import sys
import unittest
from math import cos, pi, sin

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from ros_bridge_worker import RosBridgeSession


class FakeRos:
    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self.run_called = False
        self.terminated = False

    def run(self) -> None:
        self.run_called = True

    def terminate(self) -> None:
        self.terminated = True


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


class RosBridgeSessionTests(unittest.TestCase):
    def setUp(self) -> None:
        FakeTopic.created.clear()

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
                ("/imu", "sensor_msgs/Imu"),
                ("/active_imu", "sensor_msgs/Imu"),
                ("/PowerVoltage", "std_msgs/Float32"),
                ("/cmd_vel", "geometry_msgs/Twist"),
                ("/line_follow_control", "simple_follower/LineFollowControl"),
            ],
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

    def test_disconnect_unsubscribes_topics_and_terminates_ros(self) -> None:
        session = RosBridgeSession(
            host="192.168.0.14",
            port=9090,
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
        )
        session.connect()

        session.disconnect()

        self.assertFalse(session.connected)
        self.assertTrue(session.ros.terminated)
        self.assertTrue(all(topic.unsubscribed for topic in FakeTopic.created[:4]))


if __name__ == "__main__":
    unittest.main()
