"""SAR BP imaging utilities for XKBD radar bin files and ROS motion sessions."""

from __future__ import annotations

import json
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

import numpy as np
import pandas as pd


C_MPS = 299_792_458.0
HARDWARE_FRAME_BYTES = 130260
HARDWARE_FRAME_INT16 = HARDWARE_FRAME_BYTES // 2
PAYLOAD_INT16 = 65120
PAYLOAD_COMPLEX = PAYLOAD_INT16 // 2
PAYLOAD_START_INT16 = 6


@dataclass(frozen=True)
class SarBpRunConfig:
    bin_path: Path = Path(r"D:\radar\car\debug_monitor\recordings\radar.bin")
    sensor_dir: Path = Path(r"D:\radar\car\debug_monitor\recordings")
    output_path: Path = Path(r"D:\radar\car\debug_monitor\bp_image.npz")
    x_grid_m: tuple[float, float, int] = (-1.0, 1.0, 201)
    y_grid_m: tuple[float, float, int] = (1, 3.0, 281)
    z_m: float = 0.0
    radar_time_offset_s: float = 0.04
    radar_lever_arm_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    bp_method: str = "time_domain"
    pulse_start: int = 0
    max_pulses: int | None = 2048
    remove_slow_time_mean: bool = True
    range_fft_size: int = 2048
    use_gpu: bool = True
    gpu_chunk_pixels: int = 128
    show_plot: bool = True
    plot_dynamic_range_db: float = 60.0
    plot_colormap: str = "turbo"


# 修改这里的路径和成像范围，然后直接运行：python src\sar_bp.py
RUN_CONFIG = SarBpRunConfig(
    bin_path=Path(r"D:\radar\car\debug_monitor\recordings\2026_04_29_15_05_55.bin"),
    sensor_dir=Path(r"D:\radar\car\debug_monitor\recordings\session_20260429_150555"),
    output_path=Path(r"D:\radar\car\debug_monitor\bp_image.npz"),
    x_grid_m=(-1.0, 1.0, 201),
    y_grid_m=(10, 20, 281),
    z_m=0.0,
    radar_time_offset_s=0.0,
    radar_lever_arm_m=(0.0, 0.0, 0.0),
    bp_method="time_domain",
    pulse_start=0,
    max_pulses=2048,
    remove_slow_time_mean=True,
    range_fft_size=2048,
    use_gpu=True,
    gpu_chunk_pixels=128,
    show_plot=True,
    plot_dynamic_range_db=60.0,
    plot_colormap="turbo",
)


@dataclass(frozen=True)
class RadarParams:
    sweep_time_s: float = 408e-6
    adc_time_nominal_s: float = 176e-6
    adc_points_nominal: int = 220
    fs_hz: float = 1.25e6
    prf_hz: float = 1000.0
    range_bandwidth_hz: float = 600e6
    carrier_hz: float = 18.8e9
    ddc_hz: float = 270e3
    range_sample_offset: int = 0

    @property
    def wavelength_m(self) -> float:
        return C_MPS / self.carrier_hz

    @property
    def chirp_slope_hz_per_s(self) -> float:
        return self.range_bandwidth_hz / self.sweep_time_s

    def range_bin_spacing_m(self, range_fft_size: int) -> float:
        return C_MPS * self.fs_hz / (2.0 * self.chirp_slope_hz_per_s * range_fft_size)


@dataclass(frozen=True)
class MotionParams:
    vx_fallback_mps: float = 0.0
    imu_trend_window_s: float = 1.0
    imu_static_window_s: float = 1.0


@dataclass(frozen=True)
class BpGrid:
    x_m: np.ndarray
    y_m: np.ndarray
    z_m: float | np.ndarray = 0.0


@dataclass(frozen=True)
class RadarReadResult:
    complex_samples: np.ndarray
    offset_points: int
    row_count: int


@dataclass(frozen=True)
class MotionTrajectory:
    time_s: np.ndarray
    radar_xyz_sample: np.ndarray
    base_xyz_sample: np.ndarray
    roll_pitch_yaw_deg: np.ndarray
    diagnostics: dict[str, float] = field(default_factory=dict)


def calculate_one_sweep_points(
    sweep_time_s: float = 408e-6,
    adc_time_nominal_s: float = 176e-6,
    adc_points_nominal: int = 220,
) -> int:
    points = int(np.floor((sweep_time_s / adc_time_nominal_s) * adc_points_nominal))
    return points if points > 0 else int(adc_points_nominal)


def read_xkbd_bin(path: str | Path, one_sweep_points: int | None = None) -> RadarReadResult:
    path = Path(path)
    raw = np.fromfile(path, dtype=np.uint8)
    if raw.size % HARDWARE_FRAME_BYTES != 0:
        raise ValueError(
            f"{path} size {raw.size} is not divisible by {HARDWARE_FRAME_BYTES} bytes"
        )
    if raw.size == 0:
        raise ValueError(f"{path} is empty")
    if one_sweep_points is None:
        one_sweep_points = calculate_one_sweep_points()

    row_count = raw.size // HARDWARE_FRAME_BYTES
    raw_u16 = raw.reshape(-1, 2).astype(np.uint16)
    values = ((raw_u16[:, 0] << 8) | raw_u16[:, 1]).astype(np.int32)
    values[values >= 32768] -= 65536
    raw_i16 = values.astype(np.int16).reshape(row_count, HARDWARE_FRAME_INT16)
    payload = raw_i16[:, PAYLOAD_START_INT16:PAYLOAD_START_INT16 + PAYLOAD_INT16].reshape(-1)
    complex_samples = payload[0::2].astype(np.float32) + 1j * payload[1::2].astype(np.float32)

    header_offset = int(raw[8]) + int(raw[9]) * 256
    yushu = PAYLOAD_COMPLEX % int(one_sweep_points)
    if yushu < header_offset:
        temp = header_offset - yushu
    else:
        temp = header_offset + int(one_sweep_points) - yushu
    offset_points = int(one_sweep_points) - (temp - 2)

    return RadarReadResult(
        complex_samples=complex_samples.astype(np.complex64, copy=False),
        offset_points=offset_points,
        row_count=int(row_count),
    )


def reshape_sweeps(samples: np.ndarray, nr: int, offset_points: int = 0) -> np.ndarray:
    samples = np.asarray(samples, dtype=np.complex64)
    if nr <= 0:
        raise ValueError("nr must be positive")
    aligned = samples[int(offset_points):]
    total_pulses = aligned.size // int(nr)
    if total_pulses <= 0:
        raise ValueError("not enough samples for one sweep after offset")
    aligned = aligned[:total_pulses * int(nr)]
    return aligned.reshape((total_pulses, int(nr))).T


def load_motion_session(sensor_dir: str | Path) -> tuple[pd.DataFrame, pd.DataFrame]:
    odom_path, imu_path = find_motion_files(sensor_dir)
    return pd.read_csv(odom_path), pd.read_csv(imu_path)


def find_motion_files(sensor_dir: str | Path) -> tuple[Path, Path]:
    root = Path(sensor_dir)
    if not root.exists():
        raise FileNotFoundError(root)
    session_dir = root
    if not (session_dir / "session.json").exists() and not (session_dir / "ros_odom.csv").exists():
        candidates = sorted([p for p in root.glob("session_*") if p.is_dir()])
        if not candidates:
            raise FileNotFoundError(f"no session_* directory found under {root}")
        session_dir = candidates[-1]

    odom_path = session_dir / "ros_odom.csv"
    imu_path = session_dir / "ros_imu_merged_aligned.csv"
    session_json = session_dir / "session.json"
    if session_json.exists():
        odom_path, imu_path = _paths_from_session_json(session_json, odom_path, imu_path)
    if not odom_path.exists():
        raise FileNotFoundError(odom_path)
    if not imu_path.exists():
        raise FileNotFoundError(imu_path)
    return odom_path, imu_path


def _paths_from_session_json(session_json: Path, odom_default: Path, imu_default: Path) -> tuple[Path, Path]:
    data = json.loads(session_json.read_text(encoding="utf-8"))
    strings: list[str] = []

    def collect(value) -> None:
        if isinstance(value, str):
            strings.append(value)
        elif isinstance(value, dict):
            for item in value.values():
                collect(item)
        elif isinstance(value, list):
            for item in value:
                collect(item)

    collect(data)
    odom_path = odom_default
    imu_path = imu_default
    for text in strings:
        normalized = text.replace("\\", "/")
        candidate = Path(text)
        if not candidate.is_absolute():
            candidate = session_json.parent / candidate
        if normalized.endswith("ros_odom.csv"):
            odom_path = candidate
        elif normalized.endswith("ros_imu_merged_aligned.csv"):
            imu_path = candidate
    return odom_path, imu_path


def sync_motion_trajectory(
    odom_table: pd.DataFrame,
    imu_table: pd.DataFrame,
    *,
    params: MotionParams | None = None,
    radar_lever_arm_m: Iterable[float] = (0.0, 0.0, 0.0),
) -> MotionTrajectory:
    params = params or MotionParams()
    odom = odom_table.copy()
    imu = imu_table.copy()
    _require_columns(odom, ["time_s", "motor_a_left_speed", "motor_b_right_speed"], "ros_odom.csv")
    _require_columns(imu, ["pair_time"], "ros_imu_merged_aligned.csv")

    odom_time = _relative_time(odom["time_s"].to_numpy(dtype=np.float64))
    imu_time = _relative_time(imu["pair_time"].to_numpy(dtype=np.float64))
    odom_sync = _sync_odom_to_imu(odom, odom_time, imu_time)
    vx = 0.5 * (
        odom_sync["motor_a_left_speed"].to_numpy(dtype=np.float64)
        + odom_sync["motor_b_right_speed"].to_numpy(dtype=np.float64)
    )
    if np.isfinite(vx).any():
        vx = np.nan_to_num(vx, nan=float(np.nanmean(vx[np.isfinite(vx)])))
        x_encoder = _cumtrapz(vx, imu_time)
    else:
        x_encoder = params.vx_fallback_mps * imu_time
    x_main = x_encoder - np.mean([x_encoder[0], x_encoder[-1]])

    y_vib = _fused_displacement(imu, imu_time, "accel_y", params)
    z_vib = _fused_displacement(imu, imu_time, "accel_z", params)
    roll = _fused_angle(imu, imu_time, "roll_deg", params)
    pitch = _fused_angle(imu, imu_time, "pitch_deg", params)
    yaw = _fused_angle(imu, imu_time, "yaw_deg", params)

    base_xyz = np.column_stack([x_main, y_vib, z_vib])
    rpy = np.column_stack([roll, pitch, yaw])
    lever = np.asarray(tuple(radar_lever_arm_m), dtype=np.float64)
    if lever.shape != (3,):
        raise ValueError("radar_lever_arm_m must contain three values")
    radar_xyz = base_xyz.copy()
    if np.linalg.norm(lever) > 0:
        radar_xyz += _rotated_lever_arms(rpy, lever)

    diagnostics = {
        "duration_s": float(imu_time[-1] - imu_time[0]) if imu_time.size else 0.0,
        "encoder_mean_speed_mps": float(np.nanmean(vx)) if vx.size else 0.0,
    }
    return MotionTrajectory(
        time_s=imu_time,
        radar_xyz_sample=radar_xyz,
        base_xyz_sample=base_xyz,
        roll_pitch_yaw_deg=rpy,
        diagnostics=diagnostics,
    )


def radar_positions_for_pulses(
    trajectory: MotionTrajectory,
    total_pulses: int,
    prf_hz: float = 1000.0,
    radar_time_offset_s: float = 0.0,
) -> tuple[np.ndarray, np.ndarray]:
    pulse_time = (np.arange(int(total_pulses), dtype=np.float64) / float(prf_hz)) + float(radar_time_offset_s)
    xyz = np.column_stack([
        np.interp(pulse_time, trajectory.time_s, trajectory.radar_xyz_sample[:, axis])
        for axis in range(3)
    ])
    return pulse_time, xyz


def back_projection(
    data_matrix: np.ndarray,
    radar_xyz_pulse: np.ndarray,
    grid: BpGrid,
    *,
    params: RadarParams | None = None,
    range_fft_size: int | None = None,
    window: bool = True,
) -> np.ndarray:
    params = params or RadarParams()
    data = np.asarray(data_matrix, dtype=np.complex128)
    radar_xyz = np.asarray(radar_xyz_pulse, dtype=np.float64)
    if data.ndim != 2:
        raise ValueError("data_matrix must have shape Nr x pulses")
    nr, pulses = data.shape
    if radar_xyz.shape != (pulses, 3):
        raise ValueError("radar_xyz_pulse must have shape pulses x 3")
    if range_fft_size is None:
        range_fft_size = int(2 ** np.ceil(np.log2(max(nr, 1) * 2)))
    range_profiles = _range_profiles(data, params, int(range_fft_size), window)
    range_bin_spacing = params.range_bin_spacing_m(int(range_fft_size))
    x_grid = np.asarray(grid.x_m, dtype=np.float64)
    y_grid = np.asarray(grid.y_m, dtype=np.float64)
    z_grid = np.asarray(grid.z_m, dtype=np.float64)
    if z_grid.ndim == 0:
        z_grid = np.full((y_grid.size, x_grid.size), float(z_grid))
    image = np.zeros((y_grid.size, x_grid.size), dtype=np.complex128)
    phase_scale = 4.0 * np.pi / params.wavelength_m

    for iy, y_m in enumerate(y_grid):
        for ix, x_m in enumerate(x_grid):
            z_m = float(z_grid[iy, ix]) if z_grid.ndim == 2 else float(z_grid[ix])
            distances = np.sqrt(
                (x_m - radar_xyz[:, 0]) ** 2
                + (y_m - radar_xyz[:, 1]) ** 2
                + (z_m - radar_xyz[:, 2]) ** 2
            )
            bins = distances / range_bin_spacing
            samples = _interp_profiles(range_profiles, bins)
            image[iy, ix] = np.sum(samples * np.exp(1j * phase_scale * distances))
    return image


def back_projection_time_domain(
    data_matrix: np.ndarray,
    radar_xyz_pulse: np.ndarray,
    grid: BpGrid,
    *,
    params: RadarParams | None = None,
) -> np.ndarray:
    params = params or RadarParams()
    data = np.asarray(data_matrix, dtype=np.complex128)
    radar_xyz = np.asarray(radar_xyz_pulse, dtype=np.float64)
    if data.ndim != 2:
        raise ValueError("data_matrix must have shape Nr x pulses")
    nr, pulses = data.shape
    if radar_xyz.shape != (pulses, 3):
        raise ValueError("radar_xyz_pulse must have shape pulses x 3")
    x_grid = np.asarray(grid.x_m, dtype=np.float64)
    y_grid = np.asarray(grid.y_m, dtype=np.float64)
    z_grid = np.asarray(grid.z_m, dtype=np.float64)
    if z_grid.ndim == 0:
        z_grid = np.full((y_grid.size, x_grid.size), float(z_grid))
    tr = (np.arange(nr, dtype=np.float64) + int(params.range_sample_offset)) / params.fs_hz
    fast_time_freq = params.carrier_hz + params.chirp_slope_hz_per_s * tr
    phase_scale = -1j * 4.0 * np.pi / C_MPS
    image = np.zeros((y_grid.size, x_grid.size), dtype=np.complex128)

    for iy, y_m in enumerate(y_grid):
        for ix, x_m in enumerate(x_grid):
            z_m = float(z_grid[iy, ix]) if z_grid.ndim == 2 else float(z_grid[ix])
            value = 0.0j
            for pulse_index in range(pulses):
                distance = np.sqrt(
                    (x_m - radar_xyz[pulse_index, 0]) ** 2
                    + (y_m - radar_xyz[pulse_index, 1]) ** 2
                    + (z_m - radar_xyz[pulse_index, 2]) ** 2
                )
                phase_comp = np.exp(phase_scale * fast_time_freq * distance)
                value += np.sum(data[:, pulse_index] * phase_comp)
            image[iy, ix] = value
    return image


def back_projection_auto(
    data_matrix: np.ndarray,
    radar_xyz_pulse: np.ndarray,
    grid: BpGrid,
    *,
    params: RadarParams | None = None,
    range_fft_size: int | None = None,
    window: bool = True,
    use_gpu: bool = True,
    gpu_chunk_pixels: int = 4096,
    method: str = "range_fft",
) -> np.ndarray:
    method = method.lower()
    if use_gpu:
        try:
            if method == "time_domain":
                return back_projection_time_domain_gpu(
                    data_matrix,
                    radar_xyz_pulse,
                    grid,
                    params=params,
                    chunk_pixels=gpu_chunk_pixels,
                )
            return back_projection_gpu(
                    data_matrix,
                    radar_xyz_pulse,
                    grid,
                    params=params,
                    range_fft_size=range_fft_size,
                    window=window,
                    chunk_pixels=gpu_chunk_pixels,
                )
        except ImportError as exc:
            print(f"CuPy 导入失败，已自动回退到 CPU BP：{exc}")
        except Exception as exc:
            print(f"GPU BP 启动失败，已自动回退到 CPU BP：{exc}")
    if method == "time_domain":
        return back_projection_time_domain(
            data_matrix,
            radar_xyz_pulse,
            grid,
            params=params,
        )
    return back_projection(
        data_matrix,
        radar_xyz_pulse,
        grid,
        params=params,
        range_fft_size=range_fft_size,
        window=window,
    )


def back_projection_gpu(
    data_matrix: np.ndarray,
    radar_xyz_pulse: np.ndarray,
    grid: BpGrid,
    *,
    params: RadarParams | None = None,
    range_fft_size: int | None = None,
    window: bool = True,
    chunk_pixels: int = 4096,
) -> np.ndarray:
    _configure_cuda_runtime_paths()
    try:
        import cupy as cp
    except ImportError as exc:
        raise ImportError("CuPy is required for GPU BP") from exc

    params = params or RadarParams()
    data = np.asarray(data_matrix, dtype=np.complex64)
    radar_xyz_np = np.asarray(radar_xyz_pulse, dtype=np.float32)
    if data.ndim != 2:
        raise ValueError("data_matrix must have shape Nr x pulses")
    nr, pulses = data.shape
    if radar_xyz_np.shape != (pulses, 3):
        raise ValueError("radar_xyz_pulse must have shape pulses x 3")
    if range_fft_size is None:
        range_fft_size = int(2 ** np.ceil(np.log2(max(nr, 1) * 2)))

    data_gpu = cp.asarray(data, dtype=cp.complex64)
    tr = (cp.arange(nr, dtype=cp.float32) + int(params.range_sample_offset)) / np.float32(params.fs_hz)
    compensated = data_gpu * cp.exp(1j * np.float32(2.0 * np.pi * params.ddc_hz) * tr)[:, None]
    if window:
        compensated = compensated * cp.hanning(nr).astype(cp.float32)[:, None]
    range_profiles = cp.fft.fft(compensated, n=int(range_fft_size), axis=0).T

    x_grid = np.asarray(grid.x_m, dtype=np.float32)
    y_grid = np.asarray(grid.y_m, dtype=np.float32)
    z_grid = np.asarray(grid.z_m, dtype=np.float32)
    xx, yy = np.meshgrid(x_grid, y_grid)
    if z_grid.ndim == 0:
        zz = np.full_like(xx, float(z_grid), dtype=np.float32)
    else:
        zz = z_grid
    points = np.column_stack([xx.ravel(), yy.ravel(), zz.ravel()])

    radar_xyz = cp.asarray(radar_xyz_np, dtype=cp.float32)
    image_flat = np.zeros(points.shape[0], dtype=np.complex64)
    range_bin_spacing = np.float32(params.range_bin_spacing_m(int(range_fft_size)))
    phase_scale = np.float32(4.0 * np.pi / params.wavelength_m)
    profile_len = int(range_fft_size)
    chunk_pixels = max(1, int(chunk_pixels))

    for start in range(0, points.shape[0], chunk_pixels):
        stop = min(start + chunk_pixels, points.shape[0])
        chunk = cp.asarray(points[start:stop], dtype=cp.float32)
        distances = cp.sqrt(cp.sum((chunk[:, None, :] - radar_xyz[None, :, :]) ** 2, axis=2))
        bins = distances / range_bin_spacing
        lower = cp.floor(bins).astype(cp.int64)
        frac = bins - lower
        valid = (lower >= 0) & (lower + 1 < profile_len)
        lower_t = lower.T
        frac_t = frac.T
        valid_t = valid.T
        safe_lower = cp.clip(lower_t, 0, profile_len - 2)
        lo = cp.take_along_axis(range_profiles, safe_lower, axis=1)
        hi = cp.take_along_axis(range_profiles, safe_lower + 1, axis=1)
        samples = ((1.0 - frac_t) * lo + frac_t * hi) * valid_t
        phase = cp.exp(1j * phase_scale * distances.T)
        image_flat[start:stop] = cp.asnumpy(cp.sum(samples * phase, axis=0).astype(cp.complex64))
        del chunk, distances, bins, lower, frac, valid, lower_t, frac_t, valid_t
        del safe_lower, lo, hi, samples, phase

    return image_flat.reshape((y_grid.size, x_grid.size))


def back_projection_time_domain_gpu(
    data_matrix: np.ndarray,
    radar_xyz_pulse: np.ndarray,
    grid: BpGrid,
    *,
    params: RadarParams | None = None,
    chunk_pixels: int = 128,
) -> np.ndarray:
    _configure_cuda_runtime_paths()
    try:
        import cupy as cp
    except ImportError as exc:
        raise ImportError("CuPy is required for GPU BP") from exc

    params = params or RadarParams()
    data = np.asarray(data_matrix, dtype=np.complex64)
    radar_xyz_np = np.asarray(radar_xyz_pulse, dtype=np.float32)
    if data.ndim != 2:
        raise ValueError("data_matrix must have shape Nr x pulses")
    nr, pulses = data.shape
    if radar_xyz_np.shape != (pulses, 3):
        raise ValueError("radar_xyz_pulse must have shape pulses x 3")

    x_grid = np.asarray(grid.x_m, dtype=np.float32)
    y_grid = np.asarray(grid.y_m, dtype=np.float32)
    z_grid = np.asarray(grid.z_m, dtype=np.float32)
    xx, yy = np.meshgrid(x_grid, y_grid)
    if z_grid.ndim == 0:
        zz = np.full_like(xx, float(z_grid), dtype=np.float32)
    else:
        zz = z_grid.astype(np.float32)
    points = np.column_stack([xx.ravel(), yy.ravel(), zz.ravel()])

    data_gpu = cp.asarray(data, dtype=cp.complex64)
    radar_xyz = cp.asarray(radar_xyz_np, dtype=cp.float32)
    tr = (cp.arange(nr, dtype=cp.float32) + int(params.range_sample_offset)) / np.float32(params.fs_hz)
    fast_time_freq = np.float32(params.carrier_hz) + np.float32(params.chirp_slope_hz_per_s) * tr
    phase_scale = np.complex64(-1j * 4.0 * np.pi / C_MPS)
    image_flat = np.zeros(points.shape[0], dtype=np.complex64)
    chunk_pixels = max(1, int(chunk_pixels))

    for start in range(0, points.shape[0], chunk_pixels):
        stop = min(start + chunk_pixels, points.shape[0])
        chunk = cp.asarray(points[start:stop], dtype=cp.float32)
        chunk_image = cp.zeros(stop - start, dtype=cp.complex64)
        for pulse_index in range(pulses):
            distances = cp.sqrt(cp.sum((chunk - radar_xyz[pulse_index]) ** 2, axis=1))
            phase = cp.exp(phase_scale * fast_time_freq[:, None] * distances[None, :])
            chunk_image += cp.sum(data_gpu[:, pulse_index][:, None] * phase, axis=0)
        image_flat[start:stop] = cp.asnumpy(chunk_image)
        del chunk, chunk_image

    return image_flat.reshape((y_grid.size, x_grid.size))


def _configure_cuda_runtime_paths() -> None:
    if not os.environ.get("CUPY_CACHE_DIR"):
        cache_dir = Path.cwd() / ".cupy_cache"
        cache_dir.mkdir(parents=True, exist_ok=True)
        os.environ["CUPY_CACHE_DIR"] = str(cache_dir)
    temp_dir = Path.cwd() / ".cupy_tmp"
    temp_dir.mkdir(parents=True, exist_ok=True)
    for name in ("TMP", "TEMP", "TMPDIR"):
        os.environ[name] = str(temp_dir)
    for bin_dir in _nvidia_cuda_bin_dirs():
        root = bin_dir.parent
        if not bin_dir.exists():
            continue
        if not os.environ.get("CUDA_PATH"):
            os.environ["CUDA_PATH"] = str(root)
        path_parts = os.environ.get("PATH", "").split(os.pathsep)
        if str(bin_dir) not in path_parts:
            os.environ["PATH"] = str(bin_dir) + os.pathsep + os.environ.get("PATH", "")
        if hasattr(os, "add_dll_directory"):
            try:
                os.add_dll_directory(str(bin_dir))
            except OSError:
                pass


def _nvidia_cuda_bin_dirs() -> list[Path]:
    bin_dirs: list[Path] = []
    for entry in sys.path:
        if not entry:
            continue
        nvidia_dir = Path(entry) / "nvidia"
        if nvidia_dir.exists():
            bin_dirs.extend(path for path in nvidia_dir.glob("*\\bin") if path.exists())
    prefix_nvidia = Path(sys.prefix) / "Lib" / "site-packages" / "nvidia"
    if prefix_nvidia.exists():
        for path in prefix_nvidia.glob("*\\bin"):
            if path.exists() and path not in bin_dirs:
                bin_dirs.append(path)
    return bin_dirs


def save_image_npz(path: str | Path, image: np.ndarray, grid: BpGrid, pulse_time: np.ndarray | None = None) -> None:
    np.savez_compressed(
        path,
        image=image,
        magnitude=np.abs(image),
        x_m=np.asarray(grid.x_m),
        y_m=np.asarray(grid.y_m),
        pulse_time=np.asarray([] if pulse_time is None else pulse_time),
    )


def image_magnitude_db(image: np.ndarray, dynamic_range_db: float = 60.0) -> np.ndarray:
    magnitude = np.abs(np.asarray(image))
    peak = float(np.nanmax(magnitude)) if magnitude.size else 0.0
    if not np.isfinite(peak) or peak <= 0.0:
        return np.zeros(magnitude.shape, dtype=np.float64)
    db = 20.0 * np.log10(np.maximum(magnitude / peak, 10 ** (-float(dynamic_range_db) / 20.0)))
    return np.clip(db, -float(dynamic_range_db), 0.0)


def show_bp_image(
    image: np.ndarray,
    grid: BpGrid,
    *,
    dynamic_range_db: float = 60.0,
    colormap: str = "turbo",
) -> None:
    import matplotlib.pyplot as plt

    db = image_magnitude_db(image, dynamic_range_db=dynamic_range_db)
    extent = [
        float(np.min(grid.x_m)),
        float(np.max(grid.x_m)),
        float(np.min(grid.y_m)),
        float(np.max(grid.y_m)),
    ]
    fig, ax = plt.subplots(num="SAR BP Image")
    im = ax.imshow(db, extent=extent, origin="lower", aspect="auto", cmap=colormap)
    ax.set_title("SAR BP Magnitude")
    ax.set_xlabel("x / m")
    ax.set_ylabel("y / m")
    colorbar = fig.colorbar(im, ax=ax)
    colorbar.set_label("Magnitude / dB")
    fig.tight_layout()
    plt.show()


def _require_columns(table: pd.DataFrame, columns: list[str], label: str) -> None:
    missing = [name for name in columns if name not in table.columns]
    if missing:
        raise ValueError(f"{label} missing required columns: {', '.join(missing)}")


def _relative_time(values: np.ndarray) -> np.ndarray:
    if values.size == 0:
        raise ValueError("time series is empty")
    values = values.astype(np.float64)
    return values - values[0]


def _sync_odom_to_imu(odom: pd.DataFrame, odom_time: np.ndarray, imu_time: np.ndarray) -> pd.DataFrame:
    same_rows = len(odom) == len(imu_time)
    same_duration = abs(float(odom_time[-1] - imu_time[-1])) < 0.05 if same_rows else False
    if same_rows and same_duration:
        return odom.reset_index(drop=True)
    result: dict[str, object] = {}
    for name in odom.columns:
        series = odom[name]
        if pd.api.types.is_numeric_dtype(series):
            result[name] = np.interp(imu_time, odom_time, series.to_numpy(dtype=np.float64))
        else:
            result[name] = [series.iloc[0]] * len(imu_time)
    return pd.DataFrame(result)


def _cumtrapz(values: np.ndarray, time_s: np.ndarray) -> np.ndarray:
    if values.size != time_s.size:
        raise ValueError("values and time_s must have same length")
    out = np.zeros_like(values, dtype=np.float64)
    if values.size > 1:
        dt = np.diff(time_s)
        out[1:] = np.cumsum(0.5 * (values[:-1] + values[1:]) * dt)
    return out


def _fused_angle(imu: pd.DataFrame, time_s: np.ndarray, suffix: str, params: MotionParams) -> np.ndarray:
    a = _column_or_zeros(imu, f"imu_{suffix}")
    b = _column_or_zeros(imu, f"active_imu_{suffix}")
    static = _static_mask(time_s, params.imu_static_window_s)
    return _variance_weighted_fuse(a - np.nanmean(a[static]), b - np.nanmean(b[static]), static)


def _fused_displacement(imu: pd.DataFrame, time_s: np.ndarray, suffix: str, params: MotionParams) -> np.ndarray:
    a = _displacement_from_accel(_column_or_zeros(imu, f"imu_{suffix}"), time_s, params.imu_trend_window_s)
    b = _displacement_from_accel(_column_or_zeros(imu, f"active_imu_{suffix}"), time_s, params.imu_trend_window_s)
    return _variance_weighted_fuse(a, b, _static_mask(time_s, params.imu_static_window_s))


def _column_or_zeros(table: pd.DataFrame, name: str) -> np.ndarray:
    if name not in table.columns:
        return np.zeros(len(table), dtype=np.float64)
    return table[name].to_numpy(dtype=np.float64)


def _static_mask(time_s: np.ndarray, static_window_s: float) -> np.ndarray:
    mask = time_s <= float(static_window_s)
    if not np.any(mask):
        mask = np.ones_like(time_s, dtype=bool)
    return mask


def _variance_weighted_fuse(a: np.ndarray, b: np.ndarray, static: np.ndarray) -> np.ndarray:
    sigma_a = float(np.nanstd(a[static])) + 1e-9
    sigma_b = float(np.nanstd(b[static])) + 1e-9
    w_a = 1.0 / (sigma_a * sigma_a)
    w_b = 1.0 / (sigma_b * sigma_b)
    return (w_a * np.nan_to_num(a) + w_b * np.nan_to_num(b)) / (w_a + w_b)


def _displacement_from_accel(accel: np.ndarray, time_s: np.ndarray, trend_window_s: float) -> np.ndarray:
    accel = np.nan_to_num(accel.astype(np.float64))
    if accel.size < 2:
        return np.zeros_like(accel)
    median_dt = float(np.median(np.diff(time_s))) if time_s.size > 1 else 0.01
    window = max(1, int(round(float(trend_window_s) / max(median_dt, 1e-6))))
    trend = pd.Series(accel).rolling(window=window, center=True, min_periods=1).mean().to_numpy()
    acc_hp = accel - trend
    acc_hp -= np.mean(acc_hp)
    velocity = _linear_detrend(_cumtrapz(acc_hp, time_s))
    displacement = _linear_detrend(_cumtrapz(velocity, time_s))
    return displacement


def _linear_detrend(values: np.ndarray) -> np.ndarray:
    if values.size < 2:
        return values.copy()
    x = np.arange(values.size, dtype=np.float64)
    slope, intercept = np.polyfit(x, values, 1)
    return values - (slope * x + intercept)


def _rotated_lever_arms(rpy_deg: np.ndarray, lever: np.ndarray) -> np.ndarray:
    out = np.zeros_like(rpy_deg, dtype=np.float64)
    for index, (roll_deg, pitch_deg, yaw_deg) in enumerate(rpy_deg):
        roll, pitch, yaw = np.deg2rad([roll_deg, pitch_deg, yaw_deg])
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        out[index] = (rz @ ry @ rx @ lever.reshape(3, 1)).ravel()
    return out


def _range_profiles(
    data: np.ndarray,
    params: RadarParams,
    range_fft_size: int,
    window: bool,
) -> np.ndarray:
    nr = data.shape[0]
    tr = (np.arange(nr, dtype=np.float64) + int(params.range_sample_offset)) / params.fs_hz
    compensated = data * np.exp(1j * 2.0 * np.pi * params.ddc_hz * tr)[:, None]
    if window:
        compensated = compensated * np.hanning(nr)[:, None]
    return np.fft.fft(compensated, n=range_fft_size, axis=0)


def _interp_profiles(range_profiles: np.ndarray, bins: np.ndarray) -> np.ndarray:
    profile_len, pulses = range_profiles.shape
    lower = np.floor(bins).astype(np.int64)
    frac = bins - lower
    valid = (lower >= 0) & (lower + 1 < profile_len)
    out = np.zeros(pulses, dtype=np.complex128)
    pulse_indices = np.arange(pulses)
    lo = lower[valid]
    pi = pulse_indices[valid]
    out[valid] = (1.0 - frac[valid]) * range_profiles[lo, pi] + frac[valid] * range_profiles[lo + 1, pi]
    return out


def _grid_from_range(spec: tuple[float, float, int]) -> np.ndarray:
    start, stop, count = spec
    return np.linspace(float(start), float(stop), int(count))


def run_bp_imaging(config: SarBpRunConfig = RUN_CONFIG) -> np.ndarray:
    params = RadarParams()
    nr = calculate_one_sweep_points(
        params.sweep_time_s,
        params.adc_time_nominal_s,
        params.adc_points_nominal,
    )
    radar = read_xkbd_bin(config.bin_path, one_sweep_points=nr)
    data_matrix = reshape_sweeps(radar.complex_samples, nr=nr, offset_points=radar.offset_points)
    if config.remove_slow_time_mean:
        data_matrix = data_matrix - np.mean(data_matrix, axis=1, keepdims=True)
    pulse_start = max(0, int(config.pulse_start))
    pulse_stop = data_matrix.shape[1]
    if config.max_pulses is not None:
        pulse_stop = min(pulse_stop, pulse_start + int(config.max_pulses))
    data_matrix = data_matrix[:, pulse_start:pulse_stop]
    if data_matrix.shape[1] <= 0:
        raise ValueError("selected pulse range is empty")
    odom, imu = load_motion_session(config.sensor_dir)
    trajectory = sync_motion_trajectory(odom, imu, radar_lever_arm_m=config.radar_lever_arm_m)
    pulse_time, radar_xyz = radar_positions_for_pulses(
        trajectory,
        pulse_stop,
        prf_hz=params.prf_hz,
        radar_time_offset_s=config.radar_time_offset_s,
    )
    pulse_time = pulse_time[pulse_start:pulse_stop]
    radar_xyz = radar_xyz[pulse_start:pulse_stop]
    grid = BpGrid(
        x_m=_grid_from_range(config.x_grid_m),
        y_m=_grid_from_range(config.y_grid_m),
        z_m=config.z_m,
    )
    image = back_projection_auto(
        data_matrix,
        radar_xyz,
        grid,
        params=params,
        range_fft_size=config.range_fft_size,
        use_gpu=config.use_gpu,
        gpu_chunk_pixels=config.gpu_chunk_pixels,
        method=config.bp_method,
    )
    save_image_npz(config.output_path, image, grid, pulse_time)
    if config.show_plot:
        show_bp_image(
            image,
            grid,
            dynamic_range_db=config.plot_dynamic_range_db,
            colormap=config.plot_colormap,
        )
    return image


def main() -> int:
    run_bp_imaging(RUN_CONFIG)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
