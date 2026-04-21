"""Streaming CSV recording for IMU samples."""

from __future__ import annotations

import csv
import errno
import json
import shutil
from datetime import datetime
from pathlib import Path
from statistics import median

from imu_protocol import CSV_FIELDS, ImuSample, sample_to_csv_row


class ImuRecordingSession:
    def __init__(self, base_dir: Path) -> None:
        self._base_dir = Path(base_dir)
        self._file = None
        self._writer: csv.DictWriter | None = None
        self._temp_path: Path | None = None
        self.rows_written = 0

    @property
    def temp_path(self) -> Path | None:
        return self._temp_path

    def start(self) -> Path:
        self._base_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        temp_path, handle = self._create_temp_file(timestamp)
        self._file = handle
        self._temp_path = temp_path
        self._writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        self._writer.writeheader()
        handle.flush()
        self.rows_written = 0
        return self._temp_path

    def _create_temp_file(self, timestamp: str):
        for counter in range(1000):
            temp_path = self._base_dir / f"imu_recording_{timestamp}_{counter:03d}.tmp.csv"
            try:
                return temp_path, temp_path.open("x", newline="", encoding="utf-8")
            except FileExistsError:
                continue
        raise RuntimeError("could not create temporary IMU recording file")

    def write_sample(self, sample: ImuSample) -> None:
        if self._file is None or self._writer is None:
            raise RuntimeError("recording not started")
        self._writer.writerow(sample_to_csv_row(sample))
        self._file.flush()
        self.rows_written += 1

    def finalize(self, final_path: Path) -> None:
        if self._temp_path is None:
            raise RuntimeError("recording not started")

        source = self._temp_path
        self._close()
        final_path = Path(final_path)
        final_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            source.replace(final_path)
        except OSError as exc:
            if exc.errno != errno.EXDEV and getattr(exc, "winerror", None) != 17:
                raise
            shutil.move(str(source), str(final_path))
        self._temp_path = None

    def cancel(self) -> None:
        if self._temp_path is None:
            return
        source = self._temp_path
        self._close()
        if source.exists():
            source.unlink()
        self._temp_path = None

    def _close(self) -> None:
        if self._file is not None:
            self._file.close()
        self._file = None
        self._writer = None


MERGED_FIELDS = [
    "pair_index",
    "pair_time",
    "time_delta_ms",
    "A_host_time",
    "A_sequence",
    "A_device_time",
    "A_sync_time",
    "A_pitch",
    "A_roll",
    "A_yaw",
    "A_acc_x",
    "A_acc_y",
    "A_acc_z",
    "A_gyro_x",
    "A_gyro_y",
    "A_gyro_z",
    "B_host_time",
    "B_sequence",
    "B_device_time",
    "B_sync_time",
    "B_pitch",
    "B_roll",
    "B_yaw",
    "B_acc_x",
    "B_acc_y",
    "B_acc_z",
    "B_gyro_x",
    "B_gyro_y",
    "B_gyro_z",
]


class ImuSessionRecorder:
    """Record two IMU streams into one session directory."""

    def __init__(self, base_dir: Path, align_window_seconds: float = 0.05) -> None:
        self._base_dir = Path(base_dir)
        self._align_window_seconds = float(align_window_seconds)
        self._session_dir: Path | None = None
        self._files: dict[str, object] = {}
        self._writers: dict[str, csv.DictWriter] = {}
        self._samples: dict[str, list[ImuSample]] = {"A": [], "B": []}
        self._device_configs: dict[str, dict[str, object]] = {}
        self.rows_written_by_device: dict[str, int] = {"A": 0, "B": 0}
        self._started_at = ""
        self._metadata_filename = "session.json"
        self._merged_filename = "merged_aligned.csv"
        self._owns_session_dir = True
        self._note = ""

    @property
    def session_dir(self) -> Path | None:
        return self._session_dir

    @property
    def total_rows_written(self) -> int:
        return sum(self.rows_written_by_device.values())

    def start(
        self,
        device_configs: dict[str, dict[str, object]],
        timestamp: str | None = None,
    ) -> Path:
        self._base_dir.mkdir(parents=True, exist_ok=True)
        if timestamp is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._session_dir = self._create_session_dir(timestamp)
        self._owns_session_dir = True
        self._metadata_filename = "session.json"
        self._merged_filename = "merged_aligned.csv"
        self._note = ""
        self._open_session_files(device_configs, timestamp)
        return self._session_dir

    def start_in_directory(
        self,
        session_dir: Path,
        device_configs: dict[str, dict[str, object]],
        started_at: str,
        metadata_filename: str = "session.json",
        merged_filename: str = "merged_aligned.csv",
        note: str = "",
    ) -> Path:
        session_dir = Path(session_dir)
        session_dir.mkdir(parents=True, exist_ok=True)
        self._session_dir = session_dir
        self._owns_session_dir = False
        self._metadata_filename = metadata_filename
        self._merged_filename = merged_filename
        self._note = note
        self._open_session_files(device_configs, started_at)
        return self._session_dir

    def _open_session_files(
        self,
        device_configs: dict[str, dict[str, object]],
        started_at: str,
    ) -> None:
        if self._session_dir is None:
            raise RuntimeError("session directory not set")
        self._started_at = started_at
        self._device_configs = {device_key: dict(config) for device_key, config in device_configs.items()}
        self._samples = {"A": [], "B": []}
        self.rows_written_by_device = {"A": 0, "B": 0}

        for device_key in ("A", "B"):
            path = self._session_dir / f"imu_{device_key}.csv"
            handle = path.open("x", newline="", encoding="utf-8")
            writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
            writer.writeheader()
            handle.flush()
            self._files[device_key] = handle
            self._writers[device_key] = writer

    def _create_session_dir(self, timestamp: str) -> Path:
        for counter in range(1000):
            suffix = "" if counter == 0 else f"_{counter:03d}"
            path = self._base_dir / f"imu_session_{timestamp}{suffix}"
            try:
                path.mkdir()
                return path
            except FileExistsError:
                continue
        raise RuntimeError("could not create IMU session directory")

    def write_sample(self, device_key: str, sample: ImuSample) -> None:
        if self._session_dir is None:
            raise RuntimeError("session recording not started")
        if device_key not in self._writers:
            raise ValueError(f"unknown IMU device key: {device_key}")

        self._writers[device_key].writerow(sample_to_csv_row(sample))
        self._files[device_key].flush()
        self.rows_written_by_device[device_key] += 1
        self._samples[device_key].append(sample)

    def finalize(self) -> Path:
        if self._session_dir is None:
            raise RuntimeError("session recording not started")

        self._close()
        merged_rows = build_aligned_rows(
            self._samples["A"],
            self._samples["B"],
            self._align_window_seconds,
        )
        self._write_merged_rows(merged_rows)
        self._write_metadata(len(merged_rows))
        session_dir = self._session_dir
        self._session_dir = None
        return session_dir

    def cancel(self) -> None:
        if self._session_dir is None:
            return
        session_dir = self._session_dir
        self._close()
        shutil.rmtree(session_dir, ignore_errors=True)
        self._session_dir = None

    def _write_merged_rows(self, rows: list[dict[str, str]]) -> None:
        if self._session_dir is None:
            return
        with (self._session_dir / self._merged_filename).open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=MERGED_FIELDS)
            writer.writeheader()
            writer.writerows(rows)

    def _write_metadata(self, merged_rows: int) -> None:
        if self._session_dir is None:
            return
        metadata = {
            "started_at": self._started_at,
            "align_window_seconds": self._align_window_seconds,
            "devices": self._device_configs,
            "rows_written_by_device": self.rows_written_by_device,
            "merged_rows": merged_rows,
            "note": self._note,
        }
        with (self._session_dir / self._metadata_filename).open("w", encoding="utf-8") as handle:
            json.dump(metadata, handle, ensure_ascii=False, indent=2)

    def _close(self) -> None:
        for handle in self._files.values():
            handle.close()
        self._files = {}
        self._writers = {}


def build_aligned_rows(
    samples_a: list[ImuSample],
    samples_b: list[ImuSample],
    align_window_seconds: float,
) -> list[dict[str, str]]:
    if not samples_a or not samples_b:
        return []

    clock = _select_alignment_clock(samples_a, samples_b)
    offsets = {
        "A": _hardware_offset(samples_a, clock),
        "B": _hardware_offset(samples_b, clock),
    }
    rows: list[dict[str, str]] = []
    index_a = 0
    index_b = 0
    while index_a < len(samples_a) and index_b < len(samples_b):
        sample_a = samples_a[index_a]
        sample_b = samples_b[index_b]
        time_a = _effective_time(sample_a, clock, offsets["A"])
        time_b = _effective_time(sample_b, clock, offsets["B"])
        delta = time_b - time_a

        if abs(delta) <= align_window_seconds:
            rows.append(_merged_row(len(rows), sample_a, sample_b, time_a, delta))
            index_a += 1
            index_b += 1
        elif time_a < time_b:
            index_a += 1
        else:
            index_b += 1
    return rows


def _select_alignment_clock(samples_a: list[ImuSample], samples_b: list[ImuSample]) -> str:
    samples = samples_a + samples_b
    if all(sample.sync_time is not None for sample in samples):
        return "sync_time"
    if all(sample.device_time is not None for sample in samples):
        return "device_time"
    return "host_time"


def _hardware_offset(samples: list[ImuSample], clock: str) -> float:
    if clock == "host_time":
        return 0.0
    offsets = [
        sample.host_time - (getattr(sample, clock) or 0) / 1_000_000.0
        for sample in samples
    ]
    return median(offsets)


def _effective_time(sample: ImuSample, clock: str, offset: float) -> float:
    if clock == "host_time":
        return sample.host_time
    return ((getattr(sample, clock) or 0) / 1_000_000.0) + offset


def _merged_row(
    pair_index: int,
    sample_a: ImuSample,
    sample_b: ImuSample,
    pair_time: float,
    delta_s: float,
) -> dict[str, str]:
    row = dict.fromkeys(MERGED_FIELDS, "")
    row["pair_index"] = str(pair_index)
    row["pair_time"] = f"{pair_time:.6f}"
    row["time_delta_ms"] = f"{abs(delta_s) * 1000.0:.3f}"
    _write_prefixed_sample(row, "A", sample_a)
    _write_prefixed_sample(row, "B", sample_b)
    return row


def _write_prefixed_sample(row: dict[str, str], prefix: str, sample: ImuSample) -> None:
    row[f"{prefix}_host_time"] = f"{sample.host_time:.6f}"
    row[f"{prefix}_sequence"] = str(sample.sequence)
    row[f"{prefix}_device_time"] = "" if sample.device_time is None else str(sample.device_time)
    row[f"{prefix}_sync_time"] = "" if sample.sync_time is None else str(sample.sync_time)
    _write_prefixed_vec(row, prefix, ("pitch", "roll", "yaw"), sample.euler)
    _write_prefixed_vec(row, prefix, ("acc_x", "acc_y", "acc_z"), sample.accel)
    _write_prefixed_vec(row, prefix, ("gyro_x", "gyro_y", "gyro_z"), sample.gyro)


def _write_prefixed_vec(
    row: dict[str, str],
    prefix: str,
    keys: tuple[str, ...],
    values: tuple[float, ...] | None,
) -> None:
    if values is None:
        return
    for key, value in zip(keys, values):
        row[f"{prefix}_{key}"] = f"{value:.6f}"
