import os
import sys
import unittest
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from map_fetch_client import MapFetchClient, MapFetchConfig


class CompletedProcess:
    def __init__(self, returncode: int = 0, stdout: str = "", stderr: str = "") -> None:
        self.returncode = returncode
        self.stdout = stdout
        self.stderr = stderr


class MapFetchClientTests(unittest.TestCase):
    def test_fetch_remote_file_runs_one_snapshot_command_and_one_copy(self) -> None:
        calls = []

        def runner(args, capture_output, text, check, timeout, **kwargs):
            calls.append((args, timeout, kwargs.get("input")))
            return CompletedProcess(stdout="saved")

        client = MapFetchClient(
            MapFetchConfig(
                ssh_host="wheeltec14",
                remote_map_path="/tmp/frozen_map.ply",
                snapshot_command="rosservice call /fastlio/save_map",
                timeout=4.0,
            ),
            runner=runner,
        )

        result = client.fetch_once(Path("C:/tmp/local_cache"))

        self.assertEqual(result.local_path, Path("C:/tmp/local_cache/frozen_map.ply"))
        self.assertEqual(result.source, "wheeltec14:/tmp/frozen_map.ply")
        self.assertEqual(result.method, "remote_file")
        self.assertEqual(calls, [
            (["ssh", "wheeltec14", "rosservice call /fastlio/save_map"], 4.0, None),
            (["scp", "wheeltec14:/tmp/frozen_map.ply", "C:\\tmp\\local_cache\\frozen_map.ply"], 4.0, None),
        ])

    def test_fetch_ros_topic_snapshot_writes_pointcloud2_csv_once_then_copies(self) -> None:
        calls = []

        def runner(args, capture_output, text, check, timeout, **kwargs):
            calls.append((args, timeout, kwargs.get("input")))
            return CompletedProcess(stdout="snapshot saved")

        client = MapFetchClient(
            MapFetchConfig(
                ssh_host="wheeltec14",
                map_topic="/Laser_map",
                remote_snapshot_path="/tmp/frozen_laser_map.csv",
                timeout=5.0,
            ),
            runner=runner,
        )

        result = client.fetch_once(Path("C:/tmp/local_cache"))

        self.assertEqual(result.local_path, Path("C:/tmp/local_cache/frozen_laser_map.csv"))
        self.assertEqual(result.source, "/Laser_map")
        self.assertEqual(result.method, "ros_topic_snapshot")
        self.assertEqual(calls[0][0], [
            "ssh",
            "wheeltec14",
            "env PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages python3 -",
        ])
        self.assertEqual(calls[0][1], 5.0)
        self.assertIn('TOPIC = "/Laser_map"', calls[0][2])
        self.assertIn('OUTPUT = "/tmp/frozen_laser_map.csv"', calls[0][2])
        self.assertIn("sensor_msgs.point_cloud2", calls[0][2])
        self.assertEqual(calls[1], (
            ["scp", "wheeltec14:/tmp/frozen_laser_map.csv", "C:\\tmp\\local_cache\\frozen_laser_map.csv"],
            5.0,
            None,
        ))

    def test_local_file_fetch_does_not_start_remote_transfer_loop(self) -> None:
        calls = []

        def runner(args, capture_output, text, check, timeout, **kwargs):
            calls.append(args)
            return CompletedProcess(stdout="unused")

        client = MapFetchClient(
            MapFetchConfig(local_map_path="C:/tmp/frozen.csv"),
            runner=runner,
        )

        result = client.fetch_once(Path("C:/tmp/cache"))

        self.assertEqual(result.local_path, Path("C:/tmp/frozen.csv"))
        self.assertEqual(result.method, "local_file")
        self.assertEqual(calls, [])


if __name__ == "__main__":
    unittest.main()
