import csv
import json
import os
import shutil
import sys
import unittest
import uuid
import zipfile
from contextlib import contextmanager
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from summary_package import build_summary_package


TEST_TMP_ROOT = Path(__file__).resolve().parents[1] / ".test_tmp"


@contextmanager
def temp_dir():
    TEST_TMP_ROOT.mkdir(exist_ok=True)
    path = TEST_TMP_ROOT / f"summary_package_{uuid.uuid4().hex}"
    path.mkdir()
    try:
        yield path
    finally:
        shutil.rmtree(path, ignore_errors=True)


def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


class SummaryPackageTests(unittest.TestCase):
    def test_build_summary_package_creates_raw_aligned_manifest_and_zip(self) -> None:
        with temp_dir() as tmp:
            session_dir = tmp / "session_20260608_120000"
            session_dir.mkdir()
            write_csv(
                session_dir / "encoder.csv",
                ["frame_index", "time_s", "final_a"],
                [{"frame_index": 1, "time_s": 0.01, "final_a": 2.0}],
            )
            write_csv(
                session_dir / "fastlio_odometry.csv",
                ["ros_time", "session_elapsed_s", "position_x", "position_y", "position_z", "orientation_z", "orientation_w"],
                [{"ros_time": 1.0, "session_elapsed_s": 0.1, "position_x": 1.0, "position_y": 2.0, "position_z": 0.0, "orientation_z": 0.0, "orientation_w": 1.0}],
            )
            write_csv(
                session_dir / "akm_state.csv",
                ["ros_time", "session_elapsed_s", "seq_id", "control_tick_us", "dt_us", "left_wheel_speed", "right_wheel_speed", "steering_angle"],
                [{"ros_time": 1.01, "session_elapsed_s": 0.11, "seq_id": 1, "control_tick_us": 1000, "dt_us": 10000, "left_wheel_speed": 3.0, "right_wheel_speed": 3.1, "steering_angle": 0.2}],
            )
            write_csv(
                session_dir / "control_debug.csv",
                ["ros_time", "session_elapsed_s", "target_vx", "motor_left_pwm"],
                [{"ros_time": 1.015, "session_elapsed_s": 0.115, "target_vx": 0.5, "motor_left_pwm": 100}],
            )
            write_csv(
                session_dir / "chassis_diagnostics.csv",
                ["ros_time", "session_elapsed_s", "battery_voltage", "packet_drop_count", "checksum_error_count", "legacy_error_count", "command_timeout", "low_voltage", "steering_angle_valid"],
                [{"ros_time": 1.02, "session_elapsed_s": 0.12, "battery_voltage": 24.5, "packet_drop_count": 1, "checksum_error_count": 2, "legacy_error_count": 3, "command_timeout": 0, "low_voltage": 0, "steering_angle_valid": 1}],
            )
            session_json = {
                "session_id": "session_20260608_120000",
                "started_at": "20260608_120000",
                "files": {
                    "encoder": "encoder.csv",
                    "fastlio_odometry": "fastlio_odometry.csv",
                    "akm_state": "akm_state.csv",
                    "control_debug": "control_debug.csv",
                    "chassis_diagnostics": "chassis_diagnostics.csv",
                },
                "selected_sources": ["encoder", "/Odometry", "/wheeltec/akm_state"],
            }
            with (session_dir / "session.json").open("w", encoding="utf-8") as handle:
                json.dump(session_json, handle, ensure_ascii=False, indent=2)

            result = build_summary_package(session_dir)

            self.assertTrue((session_dir / "raw" / "serial_encoder.csv").exists())
            self.assertTrue((session_dir / "aligned" / "trajectory_aligned.csv").exists())
            self.assertTrue((session_dir / "manifest.json").exists())
            self.assertTrue((session_dir / "session_20260608_120000.zip").exists())
            with (session_dir / "manifest.json").open("r", encoding="utf-8") as handle:
                manifest = json.load(handle)
            self.assertEqual(manifest["session_id"], "session_20260608_120000")
            self.assertEqual(manifest["row_counts"]["serial_encoder.csv"], 1)
            self.assertIn("aligned/trajectory_aligned.csv", manifest["generated_files"])
            with zipfile.ZipFile(session_dir / "session_20260608_120000.zip") as archive:
                names = set(archive.namelist())
            self.assertIn("raw/serial_encoder.csv", names)
            self.assertIn("aligned/trajectory_aligned.csv", names)
            self.assertIn("session.json", names)
            self.assertIn("manifest.json", names)
            self.assertEqual(result["package_zip"], str(session_dir / "session_20260608_120000.zip"))


if __name__ == "__main__":
    unittest.main()
