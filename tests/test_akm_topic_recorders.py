import csv
import os
import shutil
import sys
import unittest
import uuid
from contextlib import contextmanager
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from akm_topic_recorders import (
    AkmStateRecorder,
    CHASSIS_DIAGNOSTICS_FIELDNAMES,
    ControlDebugRecorder,
    ChassisDiagnosticsRecorder,
)
from recording_clock import RecordingClock


TEST_TMP_ROOT = Path(__file__).resolve().parents[1] / ".test_tmp"


@contextmanager
def temp_dir():
    TEST_TMP_ROOT.mkdir(exist_ok=True)
    path = TEST_TMP_ROOT / f"akm_recorders_{uuid.uuid4().hex}"
    path.mkdir()
    try:
        yield path
    finally:
        shutil.rmtree(path, ignore_errors=True)


class AkmRecorderTests(unittest.TestCase):
    def test_akm_recorder_elapsed_uses_recv_time_epoch_when_provided(self) -> None:
        with temp_dir() as tmp:
            path = tmp / "akm_state.csv"
            clock = RecordingClock(
                session_id="session_akm_time",
                start_epoch_s=1000.0,
                start_perf_s=1.0,
            )
            recorder = AkmStateRecorder(path, clock)
            recorder.write_message(
                {
                    "header": {"stamp": {"secs": 1, "nsecs": 0}, "frame_id": "base_link"},
                    "seq_id": 1,
                },
                recv_time_epoch_s=1002.5,
            )
            recorder.close()

            with path.open("r", encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))

            self.assertEqual(rows[0]["recv_time_epoch_s"], "1002.5")
            self.assertEqual(rows[0]["session_elapsed_s"], "2.5")

    def test_akm_state_recorder_writes_fixed_columns(self) -> None:
        with temp_dir() as tmp:
            path = tmp / "akm_state.csv"
            recorder = AkmStateRecorder(path, RecordingClock(session_id="session_akm"))
            recorder.write_message(
                {
                    "header": {"stamp": {"secs": 1, "nsecs": 500_000_000}, "frame_id": "base_link"},
                    "seq_id": 7,
                    "control_tick_us": 1000,
                    "dt_us": 10000,
                    "left_encoder_delta": 1,
                    "right_encoder_delta": 2,
                    "left_wheel_speed": 3.1,
                    "right_wheel_speed": 3.2,
                    "steering_feedback_raw": 4,
                    "steering_target_raw": 5,
                    "steering_angle": 6.5,
                    "steering_pwm": 7,
                    "status_flags": 8,
                    "control_mode": 9,
                    "robot_type": 10,
                },
                recv_time_epoch_s=20.0,
            )
            recorder.close()

            with path.open("r", encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))

            self.assertEqual(rows[0]["session_id"], "session_akm")
            self.assertEqual(rows[0]["ros_time"], "1.5")
            self.assertEqual(rows[0]["left_wheel_speed"], "3.1")
            self.assertEqual(rows[0]["robot_type"], "10")

    def test_control_debug_recorder_writes_expected_values(self) -> None:
        with temp_dir() as tmp:
            path = tmp / "control_debug.csv"
            recorder = ControlDebugRecorder(path, RecordingClock(session_id="session_ctrl"))
            recorder.write_message(
                {
                    "header": {"stamp": {"secs": 2, "nsecs": 0}, "frame_id": "base_link"},
                    "seq_id": 11,
                    "control_tick_us": 1200,
                    "target_vx": 0.1,
                    "target_vy": 0.2,
                    "target_vz": 0.3,
                    "legacy_vx": 0.4,
                    "legacy_vy": 0.5,
                    "legacy_vz": 0.6,
                    "motor_left_pwm": 10,
                    "motor_right_pwm": 11,
                    "steering_pwm": 12,
                    "status_flags": 13,
                    "control_mode": 14,
                },
                recv_time_epoch_s=21.0,
            )
            recorder.close()

            with path.open("r", encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))

            self.assertEqual(rows[0]["target_vx"], "0.1")
            self.assertEqual(rows[0]["motor_right_pwm"], "11")
            self.assertEqual(rows[0]["control_mode"], "14")

    def test_chassis_diagnostics_recorder_writes_expected_values(self) -> None:
        with temp_dir() as tmp:
            path = tmp / "chassis_diagnostics.csv"
            recorder = ChassisDiagnosticsRecorder(path, RecordingClock(session_id="session_diag"))
            recorder.write_message(
                {
                    "header": {"stamp": {"secs": 3, "nsecs": 250_000_000}, "frame_id": "base_link"},
                    "seq_id": 15,
                    "control_tick_us": 1300,
                    "battery_voltage": 24.6,
                    "flag_stop": 0,
                    "command_timeout": 1,
                    "low_voltage": 0,
                    "self_check_error": 0,
                    "steering_angle_valid": 1,
                    "status_flags": 2,
                    "packet_drop_count": 3,
                    "checksum_error_count": 4,
                    "legacy_error_count": 5,
                },
                recv_time_epoch_s=22.0,
            )
            recorder.close()

            with path.open("r", encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))

            self.assertEqual(rows[0]["battery_voltage"], "24.6")
            self.assertEqual(rows[0]["packet_drop_count"], "3")
            self.assertEqual(rows[0]["legacy_error_count"], "5")
            self.assertEqual(list(rows[0].keys()), CHASSIS_DIAGNOSTICS_FIELDNAMES)


if __name__ == "__main__":
    unittest.main()
