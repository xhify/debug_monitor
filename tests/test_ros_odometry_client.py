import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from ros_odometry_client import RosOdometrySession


class FakeRos:
    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self.run_called = False
        self.closed = False

    def run(self) -> None:
        self.run_called = True

    def close(self) -> None:
        self.closed = True


class FakeTopic:
    created = []

    def __init__(self, ros: FakeRos, name: str, message_type: str) -> None:
        self.ros = ros
        self.name = name
        self.message_type = message_type
        self.callback = None
        self.published = []
        self.unsubscribed = False
        FakeTopic.created.append(self)

    def subscribe(self, callback) -> None:
        self.callback = callback

    def unsubscribe(self) -> None:
        self.unsubscribed = True

    def publish(self, message) -> None:
        self.published.append(dict(message))


class RosOdometrySessionTests(unittest.TestCase):
    def setUp(self) -> None:
        FakeTopic.created.clear()

    def test_connect_subscribes_only_fast_lio_odometry_topic(self) -> None:
        session = RosOdometrySession(
            "192.168.0.14",
            9090,
            topic="/Odometry",
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
        )

        session.connect()

        self.assertTrue(session.connected)
        self.assertEqual(session.ros.host, "192.168.0.14")
        self.assertEqual([(topic.name, topic.message_type) for topic in FakeTopic.created], [
            ("/Odometry", "nav_msgs/Odometry"),
            ("/launch_manager/command", "std_msgs/String"),
        ])

    def test_publish_launch_manager_command_sends_std_msgs_string(self) -> None:
        session = RosOdometrySession(
            "localhost",
            9090,
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
        )
        session.connect()

        session.publish_launch_manager_command("start fastlio fast_lio mapping_c16.launch")

        self.assertEqual(
            session.launch_command_topic.published[-1],
            {"data": "start fastlio fast_lio mapping_c16.launch"},
        )

    def test_odometry_callback_extracts_pose_frames_and_yaw(self) -> None:
        received = []
        session = RosOdometrySession(
            "localhost",
            9090,
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
            on_sample=received.append,
        )
        session.connect()

        session.topic.callback({
            "header": {
                "stamp": {"secs": 12, "nsecs": 250_000_000},
                "frame_id": "camera_init",
            },
            "child_frame_id": "body",
            "pose": {
                "pose": {
                    "position": {"x": 1.2, "y": -0.3, "z": 0.4},
                    "orientation": {"x": 0.0, "y": 0.0, "z": math.sin(math.pi / 4), "w": math.cos(math.pi / 4)},
                }
            },
        })

        self.assertEqual(len(received), 1)
        sample = received[0]
        self.assertAlmostEqual(sample.ros_time, 12.25)
        self.assertEqual(sample.frame_id, "camera_init")
        self.assertEqual(sample.child_frame_id, "body")
        self.assertAlmostEqual(sample.x, 1.2)
        self.assertAlmostEqual(sample.yaw, math.pi / 2.0)

    def test_disconnect_unsubscribes_and_allows_reconnect(self) -> None:
        session = RosOdometrySession(
            "localhost",
            9090,
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
        )
        session.connect()
        first_ros = session.ros

        session.disconnect()
        session.connect()

        self.assertTrue(first_ros.closed)
        self.assertTrue(FakeTopic.created[0].unsubscribed)
        self.assertTrue(session.connected)
        self.assertIsNot(session.ros, first_ros)


if __name__ == "__main__":
    unittest.main()
