import importlib
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))


class AppConfigTests(unittest.TestCase):
    def test_env_host_changes_internal_worker_defaults(self) -> None:
        old_value = os.environ.get("DEBUG_MONITOR_ROSBRIDGE_HOST")
        os.environ["DEBUG_MONITOR_ROSBRIDGE_HOST"] = "robot-env.local"
        try:
            import app_config
            import ros_bridge_worker
            import ros_odometry_client

            importlib.reload(app_config)
            importlib.reload(ros_bridge_worker)
            importlib.reload(ros_odometry_client)

            bridge_worker = ros_bridge_worker.RosBridgeWorker()
            odom_worker = ros_odometry_client.RosOdometryWorker()

            self.assertEqual(bridge_worker._host, "robot-env.local")
            self.assertEqual(odom_worker._host, "robot-env.local")
        finally:
            if old_value is None:
                os.environ.pop("DEBUG_MONITOR_ROSBRIDGE_HOST", None)
            else:
                os.environ["DEBUG_MONITOR_ROSBRIDGE_HOST"] = old_value


if __name__ == "__main__":
    unittest.main()
