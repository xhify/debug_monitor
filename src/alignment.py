"""对齐 raw 记录输出。"""

from __future__ import annotations

import csv
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

    trajectory_aligned = _build_trajectory_aligned_rows(
        trajectory_rows,
        akm_rows,
        control_rows,
        diagnostic_rows,
        radar_rows,
        ros_odom_rows,
        ros_imu_rows,
        ros_active_imu_rows,
    )
    chassis_aligned = _build_chassis_aligned_rows(
        akm_rows,
        trajectory_rows,
        control_rows,
        diagnostic_rows,
        radar_rows,
        ros_odom_rows,
        ros_imu_rows,
        ros_active_imu_rows,
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


def _nearest_row(rows: list[dict[str, str]], target: float, field: str, max_delta: float) -> tuple[dict[str, str] | None, float]:
    best_row = None
    best_delta = float("inf")
    for row in rows:
        value = _float(row.get(field, ""))
        if value is None:
            continue
        delta = abs(value - target)
        if delta < best_delta:
            best_delta = delta
            best_row = row
    if best_row is None or best_delta > max_delta:
        return None, best_delta
    return best_row, best_delta


def _build_trajectory_aligned_rows(
    trajectory_rows: list[dict[str, str]],
    akm_rows: list[dict[str, str]],
    control_rows: list[dict[str, str]],
    diagnostic_rows: list[dict[str, str]],
    radar_rows: list[dict[str, str]],
    ros_odom_rows: list[dict[str, str]],
    ros_imu_rows: list[dict[str, str]],
    ros_active_imu_rows: list[dict[str, str]],
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for trajectory in trajectory_rows:
        ros_time = _float(trajectory.get("ros_time", "")) or 0.0
        session_elapsed = _float(trajectory.get("session_elapsed_s", "")) or 0.0
        akm, akm_delta = _nearest_row(akm_rows, ros_time, "ros_time", 0.05)
        control, control_delta = _nearest_row(control_rows, ros_time, "ros_time", 0.05)
        diagnostic, diagnostic_delta = _nearest_row(diagnostic_rows, ros_time, "ros_time", 0.05)
        radar, radar_delta = _nearest_row(radar_rows, session_elapsed, "radar_session_elapsed_s", 0.1)
        ros_odom, ros_odom_delta = _nearest_row(ros_odom_rows, session_elapsed, "time_s", 0.05)
        ros_imu, ros_imu_delta = _nearest_row(ros_imu_rows, session_elapsed, "time_s", 0.02)
        ros_active_imu, ros_active_imu_delta = _nearest_row(ros_active_imu_rows, session_elapsed, "time_s", 0.02)
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
                "ros_odom_x": _value(ros_odom, "position_x"),
                "ros_odom_y": _value(ros_odom, "position_y"),
                "ros_odom_linear_x": _value(ros_odom, "linear_x"),
                "ros_imu_accel_x": _value(ros_imu, "linear_acceleration_x"),
                "ros_imu_gyro_z": _value(ros_imu, "angular_velocity_z"),
                "ros_active_imu_accel_x": _value(ros_active_imu, "linear_acceleration_x"),
                "ros_active_imu_gyro_z": _value(ros_active_imu, "angular_velocity_z"),
            }
        )
    return rows


def _build_chassis_aligned_rows(
    akm_rows: list[dict[str, str]],
    trajectory_rows: list[dict[str, str]],
    control_rows: list[dict[str, str]],
    diagnostic_rows: list[dict[str, str]],
    radar_rows: list[dict[str, str]],
    ros_odom_rows: list[dict[str, str]],
    ros_imu_rows: list[dict[str, str]],
    ros_active_imu_rows: list[dict[str, str]],
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for akm in akm_rows:
        ros_time = _float(akm.get("ros_time", "")) or 0.0
        session_elapsed = _float(akm.get("session_elapsed_s", "")) or 0.0
        trajectory, trajectory_delta = _nearest_row(trajectory_rows, ros_time, "ros_time", 0.05)
        control, control_delta = _nearest_row(control_rows, ros_time, "ros_time", 0.05)
        diagnostic, diagnostic_delta = _nearest_row(diagnostic_rows, ros_time, "ros_time", 0.05)
        radar, radar_delta = _nearest_row(radar_rows, session_elapsed, "radar_session_elapsed_s", 0.1)
        ros_odom, ros_odom_delta = _nearest_row(ros_odom_rows, session_elapsed, "time_s", 0.05)
        ros_imu, ros_imu_delta = _nearest_row(ros_imu_rows, session_elapsed, "time_s", 0.02)
        ros_active_imu, ros_active_imu_delta = _nearest_row(ros_active_imu_rows, session_elapsed, "time_s", 0.02)
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
                "ros_odom_x": _value(ros_odom, "position_x"),
                "ros_odom_y": _value(ros_odom, "position_y"),
                "ros_odom_linear_x": _value(ros_odom, "linear_x"),
                "ros_imu_accel_x": _value(ros_imu, "linear_acceleration_x"),
                "ros_imu_gyro_z": _value(ros_imu, "angular_velocity_z"),
                "ros_active_imu_accel_x": _value(ros_active_imu, "linear_acceleration_x"),
                "ros_active_imu_gyro_z": _value(ros_active_imu, "angular_velocity_z"),
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


def _yaw_from_quaternion(row: dict[str, str]) -> float:
    z = _float(row.get("orientation_z", "")) or 0.0
    w = _float(row.get("orientation_w", "")) or 1.0
    return atan2(2.0 * w * z, 1.0 - 2.0 * z * z)
