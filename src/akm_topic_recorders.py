"""AKM 三个固定格式话题记录器。"""

from __future__ import annotations

import csv
from pathlib import Path

from recording_clock import RecordingClock
from ros_topic_recorders import extract_frame_id, extract_header_stamp


AKM_STATE_FIELDNAMES = [
    "session_id",
    "ros_time",
    "recv_time_epoch_s",
    "session_elapsed_s",
    "frame_id",
    "seq_id",
    "control_tick_us",
    "dt_us",
    "left_encoder_delta",
    "right_encoder_delta",
    "left_wheel_speed",
    "right_wheel_speed",
    "steering_feedback_raw",
    "steering_target_raw",
    "steering_angle",
    "steering_pwm",
    "status_flags",
    "control_mode",
    "robot_type",
]

CONTROL_DEBUG_FIELDNAMES = [
    "session_id",
    "ros_time",
    "recv_time_epoch_s",
    "session_elapsed_s",
    "frame_id",
    "seq_id",
    "control_tick_us",
    "target_vx",
    "target_vy",
    "target_vz",
    "legacy_vx",
    "legacy_vy",
    "legacy_vz",
    "motor_left_pwm",
    "motor_right_pwm",
    "steering_pwm",
    "status_flags",
    "control_mode",
]

CHASSIS_DIAGNOSTICS_FIELDNAMES = [
    "session_id",
    "ros_time",
    "recv_time_epoch_s",
    "session_elapsed_s",
    "frame_id",
    "seq_id",
    "control_tick_us",
    "battery_voltage",
    "flag_stop",
    "command_timeout",
    "low_voltage",
    "self_check_error",
    "steering_angle_valid",
    "status_flags",
    "packet_drop_count",
    "checksum_error_count",
    "legacy_error_count",
]


class _BaseAkmRecorder:
    fieldnames: list[str] = []

    def __init__(self, path: Path, clock: RecordingClock) -> None:
        self.path = Path(path)
        self._clock = clock
        self._handle = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._handle, fieldnames=self.fieldnames)
        self._writer.writeheader()

    def write_message(self, message: dict, recv_time_epoch_s: float | None = None) -> None:
        self._writer.writerow(self._row(message, recv_time_epoch_s))
        self._handle.flush()

    def close(self) -> None:
        self._handle.close()

    def _base_row(self, message: dict, recv_time_epoch_s: float | None) -> dict[str, object]:
        fields = self._clock.now_record_fields()
        if recv_time_epoch_s is not None:
            fields["recv_time_epoch_s"] = float(recv_time_epoch_s)
        return {
            "session_id": fields["session_id"],
            "ros_time": extract_header_stamp(message),
            "recv_time_epoch_s": fields["recv_time_epoch_s"],
            "session_elapsed_s": fields["session_elapsed_s"],
            "frame_id": extract_frame_id(message),
        }

    def _row(self, message: dict, recv_time_epoch_s: float | None) -> dict[str, object]:
        raise NotImplementedError


class AkmStateRecorder(_BaseAkmRecorder):
    fieldnames = AKM_STATE_FIELDNAMES

    def _row(self, message: dict, recv_time_epoch_s: float | None) -> dict[str, object]:
        row = self._base_row(message, recv_time_epoch_s)
        for key in self.fieldnames[5:]:
            row[key] = message.get(key, "")
        return row


class ControlDebugRecorder(_BaseAkmRecorder):
    fieldnames = CONTROL_DEBUG_FIELDNAMES

    def _row(self, message: dict, recv_time_epoch_s: float | None) -> dict[str, object]:
        row = self._base_row(message, recv_time_epoch_s)
        for key in self.fieldnames[5:]:
            row[key] = message.get(key, "")
        return row


class ChassisDiagnosticsRecorder(_BaseAkmRecorder):
    fieldnames = CHASSIS_DIAGNOSTICS_FIELDNAMES

    def _row(self, message: dict, recv_time_epoch_s: float | None) -> dict[str, object]:
        row = self._base_row(message, recv_time_epoch_s)
        for key in self.fieldnames[5:]:
            row[key] = message.get(key, "")
        return row
