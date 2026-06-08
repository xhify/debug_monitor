import csv
import os
import shutil
import sys
import unittest
import uuid
from contextlib import contextmanager
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from alignment import write_alignment_outputs


TEST_TMP_ROOT = Path(__file__).resolve().parents[1] / ".test_tmp"


@contextmanager
def temp_dir():
    TEST_TMP_ROOT.mkdir(exist_ok=True)
    path = TEST_TMP_ROOT / f"alignment_{uuid.uuid4().hex}"
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


class AlignmentTests(unittest.TestCase):
    def test_write_alignment_outputs_generates_trajectory_and_chassis_csv(self) -> None:
        with temp_dir() as tmp:
            raw_dir = tmp / "raw"
            aligned_dir = tmp / "aligned"
            raw_dir.mkdir()
            write_csv(
                raw_dir / "fastlio_odometry.csv",
                ["ros_time", "session_elapsed_s", "position_x", "position_y", "position_z", "orientation_z", "orientation_w"],
                [
                    {"ros_time": 1.0, "session_elapsed_s": 0.10, "position_x": 1.0, "position_y": 2.0, "position_z": 0.0, "orientation_z": 0.0, "orientation_w": 1.0},
                    {"ros_time": 2.0, "session_elapsed_s": 0.20, "position_x": 2.0, "position_y": 3.0, "position_z": 0.0, "orientation_z": 0.0, "orientation_w": 1.0},
                ],
            )
            write_csv(
                raw_dir / "akm_state.csv",
                ["ros_time", "session_elapsed_s", "seq_id", "control_tick_us", "dt_us", "left_wheel_speed", "right_wheel_speed", "steering_angle"],
                [
                    {"ros_time": 1.01, "session_elapsed_s": 0.11, "seq_id": 1, "control_tick_us": 1000, "dt_us": 10000, "left_wheel_speed": 3.0, "right_wheel_speed": 3.1, "steering_angle": 0.2},
                    {"ros_time": 2.01, "session_elapsed_s": 0.21, "seq_id": 2, "control_tick_us": 2000, "dt_us": 10000, "left_wheel_speed": 4.0, "right_wheel_speed": 4.1, "steering_angle": 0.3},
                ],
            )
            write_csv(
                raw_dir / "control_debug.csv",
                ["ros_time", "session_elapsed_s", "target_vx", "motor_left_pwm"],
                [
                    {"ros_time": 1.015, "session_elapsed_s": 0.115, "target_vx": 0.5, "motor_left_pwm": 100},
                    {"ros_time": 2.015, "session_elapsed_s": 0.215, "target_vx": 0.6, "motor_left_pwm": 110},
                ],
            )
            write_csv(
                raw_dir / "chassis_diagnostics.csv",
                ["ros_time", "session_elapsed_s", "battery_voltage", "packet_drop_count", "checksum_error_count", "legacy_error_count", "command_timeout", "low_voltage", "steering_angle_valid"],
                [
                    {"ros_time": 1.02, "session_elapsed_s": 0.12, "battery_voltage": 24.5, "packet_drop_count": 1, "checksum_error_count": 2, "legacy_error_count": 3, "command_timeout": 0, "low_voltage": 0, "steering_angle_valid": 1},
                    {"ros_time": 2.02, "session_elapsed_s": 0.22, "battery_voltage": 24.4, "packet_drop_count": 4, "checksum_error_count": 5, "legacy_error_count": 6, "command_timeout": 0, "low_voltage": 0, "steering_angle_valid": 1},
                ],
            )
            write_csv(
                raw_dir / "radar_sweeps.csv",
                ["radar_global_sweep_index", "radar_session_elapsed_s"],
                [
                    {"radar_global_sweep_index": 5, "radar_session_elapsed_s": 0.11},
                    {"radar_global_sweep_index": 6, "radar_session_elapsed_s": 0.21},
                ],
            )

            outputs = write_alignment_outputs(raw_dir=raw_dir, aligned_dir=aligned_dir)

            self.assertTrue((aligned_dir / "trajectory_aligned.csv").exists())
            self.assertTrue((aligned_dir / "chassis_100hz_aligned.csv").exists())
            with (aligned_dir / "trajectory_aligned.csv").open("r", encoding="utf-8", newline="") as handle:
                trajectory_rows = list(csv.DictReader(handle))
            with (aligned_dir / "chassis_100hz_aligned.csv").open("r", encoding="utf-8", newline="") as handle:
                chassis_rows = list(csv.DictReader(handle))
            self.assertEqual(trajectory_rows[0]["akm_left_wheel_speed"], "3.0")
            self.assertEqual(trajectory_rows[0]["radar_global_sweep_index"], "5")
            self.assertEqual(chassis_rows[0]["trajectory_x"], "1.0")
            self.assertEqual(chassis_rows[0]["packet_drop_count"], "1")
            self.assertEqual(outputs["trajectory_rows"], 2)
            self.assertEqual(outputs["chassis_rows"], 2)


if __name__ == "__main__":
    unittest.main()
