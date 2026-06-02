"""FAST-LIO2 odometry history, metrics, CSV export, and reports."""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from statistics import mean, pstdev
import threading
from typing import Iterable


LOCALIZATION_CSV_HEADER = [
    "ros_time",
    "recv_time",
    "elapsed_time",
    "source",
    "frame_id",
    "child_frame_id",
    "x",
    "y",
    "z",
    "qx",
    "qy",
    "qz",
    "qw",
    "roll",
    "pitch",
    "yaw",
    "yaw_deg",
    "x0_aligned",
    "y0_aligned",
    "yaw0_aligned",
    "lateral_error",
    "longitudinal_distance",
    "trajectory_length",
    "speed_estimated",
    "yaw_rate_estimated",
    "control_enabled",
    "control_mode",
    "target_speed",
    "target_yaw",
    "correction_vx",
    "correction_vz",
    "radar_quality",
    "radar_tracking_status",
    "safety_state",
    "notes",
]


@dataclass(slots=True)
class LocalizationSample:
    ros_time: float
    recv_time: float
    source: str
    frame_id: str
    child_frame_id: str
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    roll: float
    pitch: float
    yaw: float
    yaw_deg: float = 0.0
    x0_aligned: float = 0.0
    y0_aligned: float = 0.0
    yaw0_aligned: float = 0.0
    lateral_error: float = 0.0
    longitudinal_distance: float = 0.0
    trajectory_length: float = 0.0
    speed_estimated: float = 0.0
    yaw_rate_estimated: float = 0.0
    elapsed_time: float = 0.0
    control_enabled: bool = False
    control_mode: str = "monitor_only"
    target_speed: float = 0.0
    target_yaw: float = 0.0
    correction_vx: float = 0.0
    correction_vz: float = 0.0
    radar_quality: str = ""
    radar_tracking_status: str = ""
    safety_state: str = "monitor_only"
    notes: str = ""


@dataclass(slots=True)
class LocalizationStats:
    sample_count: int = 0
    duration_s: float = 0.0
    trajectory_length: float = 0.0
    endpoint_distance: float = 0.0
    speed_estimated: float = 0.0
    yaw_rate_estimated: float = 0.0
    lateral_error_current: float = 0.0
    lateral_error_max: float = 0.0
    lateral_error_rms: float = 0.0
    endpoint_lateral_error: float = 0.0
    yaw_mean: float = 0.0
    yaw_rms: float = 0.0
    estimated_speed_mean: float = 0.0
    estimated_speed_std: float = 0.0
    started_at: datetime | None = None
    ended_at: datetime | None = None


@dataclass(slots=True)
class _Origin:
    x: float
    y: float
    yaw: float


@dataclass(slots=True)
class _ControlState:
    enabled: bool = False
    mode: str = "monitor_only"
    target_speed: float = 0.0
    target_yaw: float = 0.0
    correction_vx: float = 0.0
    correction_vz: float = 0.0
    radar_quality: str = ""
    radar_tracking_status: str = ""
    safety_state: str = "monitor_only"
    notes: str = ""


class LocalizationBuffer:
    """Thread-safe FAST-LIO2 odometry buffer with local-origin metrics."""

    def __init__(self, max_points: int = 5000) -> None:
        self._lock = threading.Lock()
        self._rows: list[LocalizationSample] = []
        self._origin: _Origin | None = None
        self._first_recv_time: float | None = None
        self._trajectory_length = 0.0
        self._recording = False
        self._recorded_rows: list[LocalizationSample] = []
        self._record_started_at: datetime | None = None
        self._record_ended_at: datetime | None = None
        self._control_state = _ControlState()
        self._max_points = int(max_points)

    def append(self, sample: LocalizationSample) -> LocalizationSample:
        with self._lock:
            if self._origin is None:
                self._origin = _Origin(sample.x, sample.y, sample.yaw)
            if self._first_recv_time is None:
                self._first_recv_time = sample.recv_time

            enriched = self._enrich_sample(sample)
            if self._rows:
                previous = self._rows[-1]
                step = math.hypot(
                    enriched.x0_aligned - previous.x0_aligned,
                    enriched.y0_aligned - previous.y0_aligned,
                )
                self._trajectory_length += step
                dt = enriched.ros_time - previous.ros_time
                if dt <= 0.0:
                    dt = enriched.recv_time - previous.recv_time
                if dt > 0.0:
                    enriched.speed_estimated = step / dt
                    enriched.yaw_rate_estimated = _normalize_angle(enriched.yaw0_aligned - previous.yaw0_aligned) / dt
            enriched.trajectory_length = self._trajectory_length
            self._rows.append(enriched)
            if len(self._rows) > self._max_points:
                self._rows = self._rows[-self._max_points:]
            if self._recording:
                self._recorded_rows.append(enriched)
            return enriched

    def _enrich_sample(self, sample: LocalizationSample) -> LocalizationSample:
        assert self._origin is not None
        first_recv_time = self._first_recv_time if self._first_recv_time is not None else sample.recv_time
        dx = sample.x - self._origin.x
        dy = sample.y - self._origin.y
        cos_yaw = math.cos(self._origin.yaw)
        sin_yaw = math.sin(self._origin.yaw)
        x0_aligned = cos_yaw * dx + sin_yaw * dy
        y0_aligned = -sin_yaw * dx + cos_yaw * dy
        yaw0_aligned = _normalize_angle(sample.yaw - self._origin.yaw)
        control = self._control_state
        return LocalizationSample(
            ros_time=float(sample.ros_time),
            recv_time=float(sample.recv_time),
            elapsed_time=float(sample.recv_time - first_recv_time),
            source=sample.source,
            frame_id=sample.frame_id,
            child_frame_id=sample.child_frame_id,
            x=float(sample.x),
            y=float(sample.y),
            z=float(sample.z),
            qx=float(sample.qx),
            qy=float(sample.qy),
            qz=float(sample.qz),
            qw=float(sample.qw),
            roll=float(sample.roll),
            pitch=float(sample.pitch),
            yaw=float(sample.yaw),
            yaw_deg=math.degrees(sample.yaw),
            x0_aligned=x0_aligned,
            y0_aligned=y0_aligned,
            yaw0_aligned=yaw0_aligned,
            lateral_error=y0_aligned,
            longitudinal_distance=x0_aligned,
            control_enabled=control.enabled,
            control_mode=control.mode,
            target_speed=control.target_speed,
            target_yaw=control.target_yaw,
            correction_vx=control.correction_vx,
            correction_vz=control.correction_vz,
            radar_quality=control.radar_quality,
            radar_tracking_status=control.radar_tracking_status,
            safety_state=control.safety_state,
            notes=control.notes,
        )

    def set_current_pose_as_origin(self) -> None:
        with self._lock:
            latest = self._rows[-1] if self._rows else None
            if latest is None:
                return
            self._origin = _Origin(latest.x, latest.y, latest.yaw)
            existing = list(self._rows)
            recording = self._recording
            recorded_count = len(self._recorded_rows)
            self._rows = []
            self._trajectory_length = 0.0
            self._first_recv_time = existing[0].recv_time if existing else None
            self._recording = False
            for row in existing:
                enriched = self._enrich_sample(row)
                if self._rows:
                    previous = self._rows[-1]
                    step = math.hypot(
                        enriched.x0_aligned - previous.x0_aligned,
                        enriched.y0_aligned - previous.y0_aligned,
                    )
                    self._trajectory_length += step
                    dt = enriched.ros_time - previous.ros_time
                    if dt <= 0.0:
                        dt = enriched.recv_time - previous.recv_time
                    if dt > 0.0:
                        enriched.speed_estimated = step / dt
                        enriched.yaw_rate_estimated = _normalize_angle(
                            enriched.yaw0_aligned - previous.yaw0_aligned
                        ) / dt
                enriched.trajectory_length = self._trajectory_length
                self._rows.append(enriched)
            self._recording = recording
            if recording:
                self._recorded_rows = self._rows[-recorded_count:]

    def set_control_state(
        self,
        *,
        enabled: bool,
        mode: str,
        target_speed: float,
        target_yaw: float,
        correction_vx: float,
        correction_vz: float,
        radar_quality: str = "",
        radar_tracking_status: str = "",
        safety_state: str = "monitor_only",
        notes: str = "",
    ) -> None:
        with self._lock:
            self._control_state = _ControlState(
                enabled=bool(enabled),
                mode=str(mode),
                target_speed=float(target_speed),
                target_yaw=float(target_yaw),
                correction_vx=float(correction_vx),
                correction_vz=float(correction_vz),
                radar_quality=str(radar_quality),
                radar_tracking_status=str(radar_tracking_status),
                safety_state=str(safety_state),
                notes=str(notes),
            )

    def clear(self) -> None:
        with self._lock:
            self._rows.clear()
            self._origin = None
            self._first_recv_time = None
            self._trajectory_length = 0.0
            if self._recording:
                self._recorded_rows.clear()

    def start_recording(self) -> None:
        with self._lock:
            self._recording = True
            self._recorded_rows = []
            self._record_started_at = datetime.now()
            self._record_ended_at = None

    def stop_recording(self, path: Path) -> Path:
        with self._lock:
            self._recording = False
            self._record_ended_at = datetime.now()
            rows = list(self._recorded_rows)
        return self.write_csv(path, rows=rows)

    def cancel_recording(self) -> None:
        with self._lock:
            self._recording = False
            self._record_ended_at = datetime.now()
            self._recorded_rows = []

    def write_csv(self, path: Path, rows: Iterable[LocalizationSample] | None = None) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with self._lock:
            output_rows = list(self._rows if rows is None else rows)
        with path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=LOCALIZATION_CSV_HEADER)
            writer.writeheader()
            for row in output_rows:
                writer.writerow(_sample_to_csv_row(row))
        return path

    def write_report(self, path: Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        stats = self.stats()
        latest = self.latest()
        frame_text = "---"
        if latest is not None:
            frame_text = f"{latest.frame_id} -> {latest.child_frame_id}"
        started_at = stats.started_at or self._record_started_at
        ended_at = stats.ended_at or self._record_ended_at
        content = "\n".join([
            "# FAST-LIO2 定位稳定性测试报告",
            "",
            f"- 测试开始时间: {_format_dt(started_at)}",
            f"- 测试结束时间: {_format_dt(ended_at)}",
            "- 数据源 topic: /Odometry",
            f"- frame: {frame_text}",
            f"- 总采样数: {stats.sample_count}",
            f"- 测试时长: {stats.duration_s:.3f} s",
            f"- 轨迹长度: {stats.trajectory_length:.4f} m",
            f"- 起终点距离: {stats.endpoint_distance:.4f} m",
            "- 当前参考直线: 起点为原点，初始 yaw 方向为 x 轴，y0_aligned 为横向误差",
            f"- lateral_error_rms: {stats.lateral_error_rms:.4f} m",
            f"- lateral_error_max: {stats.lateral_error_max:.4f} m",
            f"- endpoint_lateral_error: {stats.endpoint_lateral_error:.4f} m",
            f"- yaw_mean: {stats.yaw_mean:.6f} rad",
            f"- yaw_rms: {stats.yaw_rms:.6f} rad",
            f"- estimated_speed_mean: {stats.estimated_speed_mean:.4f} m/s",
            f"- estimated_speed_std: {stats.estimated_speed_std:.4f} m/s",
            f"- 是否启用控制反馈: {str(bool(latest and latest.control_enabled)).lower()}",
            "",
            "备注: 当前无 ground truth，因此结果为轨迹稳定性和直线性评估，不是绝对定位精度。",
            "",
        ])
        path.write_text(content, encoding="utf-8")
        return path

    def latest(self) -> LocalizationSample | None:
        with self._lock:
            return self._rows[-1] if self._rows else None

    def stats(self) -> LocalizationStats:
        with self._lock:
            rows = list(self._rows)
            started_at = self._record_started_at
            ended_at = self._record_ended_at
        if not rows:
            return LocalizationStats(started_at=started_at, ended_at=ended_at)
        first = rows[0]
        latest = rows[-1]
        duration = max(0.0, latest.recv_time - first.recv_time)
        lateral_values = [row.lateral_error for row in rows]
        yaw_values = [row.yaw0_aligned for row in rows]
        speeds = [row.speed_estimated for row in rows[1:] if row.speed_estimated >= 0.0]
        yaw_rates = [row.yaw_rate_estimated for row in rows[1:]]
        return LocalizationStats(
            sample_count=len(rows),
            duration_s=duration,
            trajectory_length=latest.trajectory_length,
            endpoint_distance=math.hypot(latest.x0_aligned, latest.y0_aligned),
            speed_estimated=latest.speed_estimated,
            yaw_rate_estimated=latest.yaw_rate_estimated,
            lateral_error_current=latest.lateral_error,
            lateral_error_max=max((abs(value) for value in lateral_values), default=0.0),
            lateral_error_rms=_rms(lateral_values),
            endpoint_lateral_error=latest.lateral_error,
            yaw_mean=mean(yaw_values) if yaw_values else 0.0,
            yaw_rms=_rms(yaw_values),
            estimated_speed_mean=mean(speeds) if speeds else 0.0,
            estimated_speed_std=pstdev(speeds) if len(speeds) > 1 else 0.0,
            started_at=started_at,
            ended_at=ended_at,
        )

    def plot_xy(self) -> tuple[list[float], list[float]]:
        with self._lock:
            return (
                [row.x0_aligned for row in self._rows],
                [row.y0_aligned for row in self._rows],
            )

    @property
    def recording(self) -> bool:
        with self._lock:
            return self._recording


def _sample_to_csv_row(sample: LocalizationSample) -> dict[str, object]:
    row = {name: getattr(sample, name) for name in LOCALIZATION_CSV_HEADER if hasattr(sample, name)}
    row["control_enabled"] = "true" if sample.control_enabled else "false"
    return row


def _rms(values: list[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(value * value for value in values) / len(values))


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _format_dt(value: datetime | None) -> str:
    return "---" if value is None else value.strftime("%Y-%m-%d %H:%M:%S")
