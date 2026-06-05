import base64
import csv
import os
import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from map_fetch_client import MapFetchClient, MapFetchConfig, pointcloud2_to_rows


class FakeRos:
    instances = []

    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self.closed = False
        FakeRos.instances.append(self)

    def run(self) -> None:
        pass

    def close(self) -> None:
        self.closed = True


class FakeTopic:
    instances = []
    message = None

    def __init__(self, ros: FakeRos, name: str, message_type: str) -> None:
        self.ros = ros
        self.name = name
        self.message_type = message_type
        self.unsubscribed = False
        FakeTopic.instances.append(self)

    def subscribe(self, callback) -> None:
        callback(FakeTopic.message)

    def unsubscribe(self) -> None:
        self.unsubscribed = True


class MapFetchClientTests(unittest.TestCase):
    def setUp(self) -> None:
        FakeRos.instances = []
        FakeTopic.instances = []
        FakeTopic.message = make_pointcloud2_message([(1.0, 2.0, 3.0, 4.0), (5.5, 6.5, 7.5, 8.5)])

    def test_map_topic_snapshot_uses_rosbridge_and_writes_pointcloud2_csv(self) -> None:
        def forbidden_runner(*args, **kwargs):
            raise AssertionError("ssh/scp runner must not be called")

        cache_dir = Path("C:/tmp/debug_monitor_map_fetch_test")
        client = MapFetchClient(
            MapFetchConfig(host="robot.local", port=19090, map_topic="/Laser_map", timeout=5.0),
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
            runner=forbidden_runner,
        )

        result = client.fetch_once(cache_dir)

        self.assertEqual(result.source, "/Laser_map")
        self.assertEqual(result.method, "rosbridge_topic_snapshot")
        self.assertEqual(FakeRos.instances[0].host, "robot.local")
        self.assertEqual(FakeRos.instances[0].port, 19090)
        self.assertEqual(FakeTopic.instances[0].message_type, "sensor_msgs/PointCloud2")
        self.assertTrue(FakeTopic.instances[0].unsubscribed)
        with result.local_path.open(newline="") as handle:
            rows = list(csv.DictReader(handle))
        self.assertEqual(rows[0], {"x": "1.0", "y": "2.0", "z": "3.0", "intensity": "4.0"})
        self.assertEqual(rows[1], {"x": "5.5", "y": "6.5", "z": "7.5", "intensity": "8.5"})

    def test_pointcloud2_parser_handles_big_endian_base64_data(self) -> None:
        message = make_pointcloud2_message([(1.25, -2.5, 3.75, 9.0)], big_endian=True, base64_data=True)

        rows = pointcloud2_to_rows(message)

        self.assertEqual(rows, [(1.25, -2.5, 3.75, 9.0)])

    def test_local_file_fetch_does_not_start_network_or_remote_commands(self) -> None:
        calls = []

        def forbidden_ros(*args, **kwargs):
            calls.append(args)
            raise AssertionError("network must not be used")

        cache_dir = Path("C:/tmp/debug_monitor_map_fetch_local")
        cache_dir.mkdir(parents=True, exist_ok=True)
        local_map = cache_dir / "frozen.csv"
        local_map.write_text("x,y,z,intensity\n1,2,3,4\n", encoding="utf-8")
        client = MapFetchClient(
            MapFetchConfig(local_map_path=str(local_map), map_topic="/Laser_map"),
            ros_factory=forbidden_ros,
        )

        result = client.fetch_once(cache_dir)

        self.assertEqual(result.local_path, local_map)
        self.assertEqual(result.method, "local_file")
        self.assertEqual(calls, [])


def make_pointcloud2_message(points, *, big_endian: bool = False, base64_data: bool = False) -> dict:
    endian = ">" if big_endian else "<"
    data = b"".join(struct.pack(f"{endian}ffff", *point) for point in points)
    payload = base64.b64encode(data).decode("ascii") if base64_data else list(data)
    return {
        "height": 1,
        "width": len(points),
        "fields": [
            {"name": "x", "offset": 0, "datatype": 7, "count": 1},
            {"name": "y", "offset": 4, "datatype": 7, "count": 1},
            {"name": "z", "offset": 8, "datatype": 7, "count": 1},
            {"name": "intensity", "offset": 12, "datatype": 7, "count": 1},
        ],
        "is_bigendian": big_endian,
        "point_step": 16,
        "row_step": 16 * len(points),
        "data": payload,
        "is_dense": True,
    }


if __name__ == "__main__":
    unittest.main()
