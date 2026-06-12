import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from rosbag_models import (
    format_bytes,
    format_duration,
    parse_rosbag_library_state,
    parse_rosbag_recording_status,
)


class RosbagModelsTests(unittest.TestCase):
    def test_parse_recording_status_handles_complete_payload(self) -> None:
        status = parse_rosbag_recording_status(
            {
                "rosbag": {
                    "active": True,
                    "state": "recording",
                    "session_id": "session_20260612_153000",
                    "duration_s": 52.4,
                    "remote_dir": "/home/wheeltec/bags/session_20260612_153000",
                    "current_size_bytes": 123456789,
                    "bag_files": ["fastlio_0.bag.active"],
                    "topics": ["/point_cloud_raw", "/imu"],
                    "disk_free_gb": 36.2,
                    "last_error": "",
                }
            }
        )

        self.assertTrue(status.active)
        self.assertEqual(status.state, "recording")
        self.assertEqual(status.session_id, "session_20260612_153000")
        self.assertEqual(status.bag_files, ["fastlio_0.bag.active"])
        self.assertEqual(status.topics, ["/point_cloud_raw", "/imu"])

    def test_parse_library_state_handles_sessions_and_bad_types(self) -> None:
        library = parse_rosbag_library_state(
            {
                "rosbag_library": {
                    "bag_dir": "/home/wheeltec/bags",
                    "disk_free_gb": "36.2",
                    "sessions": [
                        {
                            "session_id": "session_1",
                            "status": "stopped",
                            "remote_dir": "/bags/session_1",
                            "size_bytes": "2048",
                            "duration_s": "12.5",
                            "file_count": "2",
                            "topic_count": "7",
                            "created_at": "2026-06-12T15:30:00",
                            "bag_files": ["a.bag", 3],
                            "downloaded": True,
                            "local_dir": "D:/bags/session_1",
                        },
                        "bad",
                    ],
                }
            }
        )

        self.assertEqual(library.bag_dir, "/home/wheeltec/bags")
        self.assertAlmostEqual(library.disk_free_gb, 36.2)
        self.assertEqual(len(library.sessions), 1)
        self.assertEqual(library.sessions[0].size_bytes, 2048)
        self.assertEqual(library.sessions[0].bag_files, ["a.bag", "3"])

    def test_missing_and_invalid_fields_use_safe_defaults(self) -> None:
        status = parse_rosbag_recording_status({"rosbag": {"active": "yes", "duration_s": "bad"}})
        library = parse_rosbag_library_state({"rosbag_library": {"sessions": "bad"}})

        self.assertFalse(status.active)
        self.assertEqual(status.duration_s, 0.0)
        self.assertEqual(library.sessions, [])
        self.assertEqual(format_bytes(1536), "1.5 KB")
        self.assertEqual(format_duration(3661), "01:01:01")


if __name__ == "__main__":
    unittest.main()
