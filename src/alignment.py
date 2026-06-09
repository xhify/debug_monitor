"""对齐 raw 记录输出。"""

from __future__ import annotations

import csv
from bisect import bisect_left
from math import atan2
from pathlib import Path


def write_alignment_outputs(raw_dir: Path, aligned_dir: Path) -> dict[str, int]:
    raw_dir = Path(raw_dir)
    aligned_dir = Path(aligned_dir)
    aligned_dir.mkdir(parents=True, exist_ok=True)

    trajectory_rows = _read_rows(_trajectory_path(raw_dir))
    akm_rows = _read_rows(raw_dir / "akm_state.csv")
    control_rows = _read_rows(raw_dir / "control_debug.csv")
    diagnostic_rows = _read_rows(raw_dir / "chassis_diagnostics.csv")
    radar_rows = _read_rows(raw_dir / "radar" / "radar_sweeps.csv")
    ros_odom_rows = _read_rows(raw_dir / "ros_odom.csv")
    ros_imu_rows = _read_rows(raw_dir / "ros_imu.csv")
    ros_active_imu_rows = _read_rows(raw_dir / "ros_active_imu.csv")
    trajectory_ros_time_index = _NearestIndex(trajectory_rows, "ros_time")
    akm_ros_time_index = _NearestIndex(akm_rows, "ros_time")
    control_ros_time_index = _NearestIndex(control_rows, "ros_time")
    diagnostic_ros_time_index = _NearestIndex(diagnostic_rows, "ros_time")
    radar_session_index = _NearestIndex(radar_rows, "radar_session_elapsed_s")
    ros_odom_time_index = _NearestIndex(ros_odom_rows, "time_s")
    ros_imu_time_index = _NearestIndex(ros_imu_rows, "time_s")
    ros_active_imu_time_index = _NearestIndex(ros_active_imu_rows, "time_s")

    trajectory_aligned = _build_trajectory_aligned_rows(
        trajectory_rows,
        akm_ros_time_index,
        control_ros_time_index,
        diagnostic_ros_time_index,
        radar_session_index,
        ros_odom_time_index,
        ros_imu_time_index,
        ros_active_imu_time_index,
    )
    chassis_aligned = _build_chassis_aligned_rows(
        akm_rows,
        trajectory_ros_time_index,
        control_ros_time_index,
        diagnostic_ros_time_index,
        radar_session_index,
        ros_odom_time_index,
        ros_imu_time_index,
        ros_active_imu_time_index,
    )

    _write_rows(aligned_dir / "trajectory_aligned.csv", trajectory_aligned)
    _write_rows(aligned_dir / "chassis_100hz_aligned.csv", chassis_aligned)
    return {
        "trajectory_rows": len(trajectory_aligned),
        "chassis_rows": len(chassis_aligned),
    }


def _read_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def _trajectory_path(raw_dir: Path) -> Path:
    preferred = raw_dir / "trajectory_odometry.csv"
    if preferred.exists():
        return preferred
    return raw_dir / "fastlio_odometry.csv"


def _write_rows(path: Path, rows: list[dict[str, object]]) -> None:
    if rows:
        fieldnames = list(rows[0].keys())
    else:
        fieldnames = []
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        if fieldnames:
            writer.writeheader()
            writer.writerows(rows)


class _NearestIndex:
    def __init__(self, rows: list[dict[str, str]], field: str) -> None:
        pairs = []
        for row in rows:
            value = _float(row.get(field, ""))
            if value is not None:
                pairs.append((value, row))
        pairs.sort(key=lambda pair: pair[0])
        self._values = [value for value, _ in pairs]
        self._rows = [row for _, row in pairs]

    def nearest(self, target: float, max_delta: float) -> tuple[dict[str, str] | None, float]:
        if not self._values:
            return None, float("inf")
        position = bisect_left(self._values, target)
        best_row = None
        best_delta = float("inf")
        for candidate in (position - 1, position):
            if candidate < 0 or candidate >= len(self._values):
                continue
            delta = abs(self._values[candidate] - target)
            if delta < best_delta:
                best_delta = delta
                best_row = self._rows[candidate]
        if best_row is None or best_delta > max_delta:
            return None, best_delta
        return best_row, best_delta


def _nearest_row(rows: list[dict[str, str]], target: float, field: str, max_delta: float) -> tuple[dict[str, str] | None, float]:
    return _NearestIndex(rows, field).nearest(target, max_delta)


def _build_trajectory_aligned_rows(
    trajectory_rows: list[dict[str, str]],
    akm_index: _NearestIndex,
    control_index: _NearestIndex,
    diagnostic_index: _NearestIndex,
    radar_index: _NearestIndex,
    ros_odom_index: _NearestIndex,
    ros_imu_index: _NearestIndex,
    ros_active_imu_index: _NearestIndex,
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for trajectory in trajectory_rows:
        ros_time = _float(trajectory.get("ros_time", "")) or 0.0
        session_elapsed = _float(trajectory.get("session_elapsed_s", "")) or 0.0
        akm, akm_delta = akm_index.nearest(ros_time, 0.05)
        control, control_delta = control_index.nearest(ros_time, 0.05)
        diagnostic, diagnostic_delta = diagnostic_index.nearest(ros_time, 0.05)
        radar, radar_delta = radar_index.nearest(session_elapsed, 0.1)
        ros_odom, ros_odom_delta = ros_odom_index.nearest(session_elapsed, 0.05)
        ros_imu, ros_imu_delta = ros_imu_index.nearest(session_elapsed, 0.02)
        ros_active_imu, ros_active_imu_delta = ros_active_imu_index.nearest(session_elapsed, 0.02)
        rows.append(
            {
                "trajectory_ros_time": ros_time,
                "trajectory_session_elapsed_s": session_elapsed,
                "trajectory_x": trajectory.get("position_x", ""),
                "trajectory_y": trajectory.get("position_y", ""),
                "trajectory_z": trajectory.get("position_z", ""),
                "trajectory_yaw": _yaw_from_quaternion(trajectory),
                "akm_time_delta_ms": _delta_ms(akm_delta, akm),
                "control_debug_time_delta_ms": _delta_ms(control_delta, control),
                "diagnostics_time_delta_ms": _delta_ms(diagnostic_delta, diagnostic),
                "radar_time_delta_ms": _delta_ms(radar_delta, radar),
                "ros_odom_time_delta_ms": _delta_ms(ros_odom_delta, ros_odom),
                "ros_imu_time_delta_ms": _delta_ms(ros_imu_delta, ros_imu),
                "ros_active_imu_time_delta_ms": _delta_ms(ros_active_imu_delta, ros_active_imu),
                "akm_left_wheel_speed": _value(akm, "left_wheel_speed"),
                "akm_right_wheel_speed": _value(akm, "right_wheel_speed"),
                "akm_steering_angle": _value(akm, "steering_angle"),
                "control_target_vx": _value(control, "target_vx"),
                "control_motor_left_pwm": _value(control, "motor_left_pwm"),
                "diagnostics_battery_voltage": _value(diagnostic, "battery_voltage"),
                "diagnostics_packet_drop_count": _value(diagnostic, "packet_drop_count"),
                "radar_global_sweep_index": _value(radar, "radar_global_sweep_index"),
                "radar_relative_time_s": _value(radar, "radar_relative_time_s"),
                "radar_sample_count": _value(radar, "sample_count"),
                "ros_odom_x": _value_first(ros_odom, "pose_x", "position_x"),
                "ros_odom_y": _value_first(ros_odom, "pose_y", "position_y"),
                "ros_odom_linear_x": _value_first(ros_odom, "motor_a_left_speed", "linear_x"),
                "ros_odom_angular_z": _value(ros_odom, "angular_z"),
                "ros_imu_accel_x": _value_first(ros_imu, "accel_x", "linear_acceleration_x"),
                "ros_imu_gyro_z": _value_first(ros_imu, "gyro_z", "angular_velocity_z"),
                "ros_active_imu_accel_x": _value_first(ros_active_imu, "accel_x", "linear_acceleration_x"),
                "ros_active_imu_gyro_z": _value_first(ros_active_imu, "gyro_z", "angular_velocity_z"),
            }
        )
    return rows


def _build_chassis_aligned_rows(
    akm_rows: list[dict[str, str]],
    trajectory_index: _NearestIndex,
    control_index: _NearestIndex,
    diagnostic_index: _NearestIndex,
    radar_index: _NearestIndex,
    ros_odom_index: _NearestIndex,
    ros_imu_index: _NearestIndex,
    ros_active_imu_index: _NearestIndex,
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for akm in akm_rows:
        ros_time = _float(akm.get("ros_time", "")) or 0.0
        session_elapsed = _float(akm.get("session_elapsed_s", "")) or 0.0
        trajectory, trajectory_delta = trajectory_index.nearest(ros_time, 0.05)
        control, control_delta = control_index.nearest(ros_time, 0.05)
        diagnostic, diagnostic_delta = diagnostic_index.nearest(ros_time, 0.05)
        radar, radar_delta = radar_index.nearest(session_elapsed, 0.1)
        ros_odom, ros_odom_delta = ros_odom_index.nearest(session_elapsed, 0.05)
        ros_imu, ros_imu_delta = ros_imu_index.nearest(session_elapsed, 0.02)
        ros_active_imu, ros_active_imu_delta = ros_active_imu_index.nearest(session_elapsed, 0.02)
        rows.append(
            {
                "akm_ros_time": ros_time,
                "akm_session_elapsed_s": session_elapsed,
                "akm_seq_id": akm.get("seq_id", ""),
                "akm_control_tick_us": akm.get("control_tick_us", ""),
                "akm_dt_us": akm.get("dt_us", ""),
                "trajectory_time_delta_ms": _delta_ms(trajectory_delta, trajectory),
                "control_debug_time_delta_ms": _delta_ms(control_delta, control),
                "diagnostics_time_delta_ms": _delta_ms(diagnostic_delta, diagnostic),
                "radar_time_delta_ms": _delta_ms(radar_delta, radar),
                "ros_odom_time_delta_ms": _delta_ms(ros_odom_delta, ros_odom),
                "ros_imu_time_delta_ms": _delta_ms(ros_imu_delta, ros_imu),
                "ros_active_imu_time_delta_ms": _delta_ms(ros_active_imu_delta, ros_active_imu),
                "trajectory_x": _value(trajectory, "position_x"),
                "trajectory_y": _value(trajectory, "position_y"),
                "trajectory_z": _value(trajectory, "position_z"),
                "control_target_vx": _value(control, "target_vx"),
                "control_motor_left_pwm": _value(control, "motor_left_pwm"),
                "battery_voltage": _value(diagnostic, "battery_voltage"),
                "packet_drop_count": _value(diagnostic, "packet_drop_count"),
                "checksum_error_count": _value(diagnostic, "checksum_error_count"),
                "legacy_error_count": _value(diagnostic, "legacy_error_count"),
                "command_timeout": _value(diagnostic, "command_timeout"),
                "low_voltage": _value(diagnostic, "low_voltage"),
                "steering_angle_valid": _value(diagnostic, "steering_angle_valid"),
                "radar_global_sweep_index": _value(radar, "radar_global_sweep_index"),
                "radar_relative_time_s": _value(radar, "radar_relative_time_s"),
                "radar_sample_count": _value(radar, "sample_count"),
                "ros_odom_x": _value_first(ros_odom, "pose_x", "position_x"),
                "ros_odom_y": _value_first(ros_odom, "pose_y", "position_y"),
                "ros_odom_linear_x": _value_first(ros_odom, "motor_a_left_speed", "linear_x"),
                "ros_odom_angular_z": _value(ros_odom, "angular_z"),
                "ros_imu_accel_x": _value_first(ros_imu, "accel_x", "linear_acceleration_x"),
                "ros_imu_gyro_z": _value_first(ros_imu, "gyro_z", "angular_velocity_z"),
                "ros_active_imu_accel_x": _value_first(ros_active_imu, "accel_x", "linear_acceleration_x"),
                "ros_active_imu_gyro_z": _value_first(ros_active_imu, "gyro_z", "angular_velocity_z"),
            }
        )
    return rows


def _float(value: str | object) -> float | None:
    if value in ("", None):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _delta_ms(delta_s: float, row: dict[str, str] | None) -> str:
    if row is None or delta_s == float("inf"):
        return ""
    return f"{delta_s * 1000.0:.3f}"


def _value(row: dict[str, str] | None, key: str) -> str:
    if row is None:
        return ""
    return str(row.get(key, ""))


def _value_first(row: dict[str, str] | None, *keys: str) -> str:
    if row is None:
        return ""
    for key in keys:
        value = row.get(key, "")
        if value != "":
            return str(value)
    return ""


def _yaw_from_quaternion(row: dict[str, str]) -> float:
    z = _float(row.get("orientation_z", "")) or 0.0
    w = _float(row.get("orientation_w", "")) or 1.0
    return atan2(2.0 * w * z, 1.0 - 2.0 * z * z)
