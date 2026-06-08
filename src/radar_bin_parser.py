"""谐波雷达 bin/xml 解析。"""

from __future__ import annotations

import csv
import json
from math import floor
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np


BIG_FRAME_BYTES = 130260
HEADER_BYTES = 16
FOOTER_BYTES = 4
AD_BYTES = BIG_FRAME_BYTES - HEADER_BYTES - FOOTER_BYTES
COMPLEX_SAMPLES_PER_FRAME = AD_BYTES // 4


def parse_radar_xml(path: Path) -> dict[str, object]:
    root = ET.parse(path).getroot()
    raw_values = {child.tag: (child.text or "").strip() for child in root}

    def _get_float(tag: str, default: float = 0.0) -> float:
        try:
            return float(raw_values.get(tag, default))
        except (TypeError, ValueError):
            return float(default)

    def _get_int(tag: str, default: int = 0) -> int:
        try:
            return int(float(raw_values.get(tag, default)))
        except (TypeError, ValueError):
            return int(default)

    return {
        "scan_start_frequency_hz": _get_float("扫描起始频率Hz"),
        "scan_stop_frequency_hz": _get_float("扫描截止频率Hz"),
        "scan_step_frequency_hz": _get_float("扫描步进频率Hz"),
        "scan_time_s": _get_float("扫描时间s"),
        "sweep_period_s": _get_float("扫频周期s"),
        "trigger_count": _get_int("触发数"),
        "single_trigger_interval_ms": _get_float("连续单次触发间隔ms"),
        "raw_xml_fields": raw_values,
    }


def parse_radar_recording(
    bin_path: Path,
    xml_path: Path,
    output_dir: Path,
    session_id: str,
    radar_start_session_elapsed_s: float,
    host_start_epoch_s: float,
    host_stop_epoch_s: float,
    radar_stop_session_elapsed_s: float | None = None,
) -> dict[str, object]:
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(bin_path, output_dir / "radar_recording.bin")
    shutil.copy2(xml_path, output_dir / "radar_config.xml")

    xml_info = parse_radar_xml(xml_path)
    scan_time_s = float(xml_info.get("scan_time_s", 0.0))
    sweep_period_s = float(xml_info.get("sweep_period_s", 0.0))
    if sweep_period_s <= 0.0:
        if scan_time_s <= 0.0:
            raise ValueError("invalid sweep_period_s")
        sweep_period_s = scan_time_s
    one_sweep_points = int(floor(scan_time_s / 176e-6) * 220)
    if one_sweep_points <= 0:
        raise ValueError("invalid oneSweepADPoints")

    raw_data = bin_path.read_bytes()
    if len(raw_data) % BIG_FRAME_BYTES != 0:
        raise ValueError("radar bin size is not aligned to frame size")

    frame_count = len(raw_data) // BIG_FRAME_BYTES
    complex_frames: list[np.ndarray] = []
    sweep_rows: list[dict[str, object]] = []
    global_sweep_index = 0

    for frame_index in range(frame_count):
        start = frame_index * BIG_FRAME_BYTES
        frame_bytes = raw_data[start:start + BIG_FRAME_BYTES]
        offset = frame_bytes[8] + frame_bytes[9] * 256
        iq_int16 = np.frombuffer(frame_bytes[HEADER_BYTES:HEADER_BYTES + AD_BYTES], dtype=">i2")
        i_values = iq_int16[0::2].astype(np.float32)
        q_values = iq_int16[1::2].astype(np.float32)
        complex_samples = i_values + 1j * q_values

        yushu = COMPLEX_SAMPLES_PER_FRAME % one_sweep_points
        if yushu < offset:
            temp = offset - yushu
        else:
            temp = offset + one_sweep_points - yushu
        matlab_offset = one_sweep_points - (temp - 2)

        usable_start = max(0, int(matlab_offset))
        usable = complex_samples[usable_start:]
        sweep_count = len(usable) // one_sweep_points
        if sweep_count <= 0:
            continue

        sweeps = usable[: sweep_count * one_sweep_points].reshape(sweep_count, one_sweep_points)
        complex_frames.append(sweeps)
        for sweep_index in range(sweep_count):
            radar_relative_time_s = global_sweep_index * sweep_period_s
            sweep_rows.append(
                {
                    "session_id": session_id,
                    "radar_frame_index": frame_index,
                    "radar_sweep_index": sweep_index,
                    "radar_global_sweep_index": global_sweep_index,
                    "radar_relative_time_s": radar_relative_time_s,
                    "radar_session_elapsed_s": radar_start_session_elapsed_s + radar_relative_time_s,
                    "sample_start_index": usable_start + sweep_index * one_sweep_points,
                    "sample_count": one_sweep_points,
                    "offset": matlab_offset,
                }
            )
            global_sweep_index += 1

    complex_data = np.vstack(complex_frames) if complex_frames else np.empty((0, one_sweep_points), dtype=np.complex64)
    np.savez(output_dir / "radar_complex.npz", complex_data=complex_data)

    with (output_dir / "radar_sweeps.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(sweep_rows[0].keys()) if sweep_rows else [
            "session_id",
            "radar_frame_index",
            "radar_sweep_index",
            "radar_global_sweep_index",
            "radar_relative_time_s",
            "radar_session_elapsed_s",
            "sample_start_index",
            "sample_count",
            "offset",
        ])
        writer.writeheader()
        writer.writerows(sweep_rows)

    with (output_dir / "radar_timeline.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=["event", "host_time_epoch_s", "session_elapsed_s", "filename"],
        )
        writer.writeheader()
        writer.writerow(
            {
                "event": "start",
                "host_time_epoch_s": host_start_epoch_s,
                "session_elapsed_s": radar_start_session_elapsed_s,
                "filename": "radar_recording.bin",
            }
        )
        writer.writerow(
            {
                "event": "stop",
                "host_time_epoch_s": host_stop_epoch_s,
                "session_elapsed_s": (
                    radar_stop_session_elapsed_s
                    if radar_stop_session_elapsed_s is not None
                    else radar_start_session_elapsed_s + max(0.0, (global_sweep_index - 1) * sweep_period_s)
                ),
                "filename": "radar_recording.bin",
            }
        )

    metadata = dict(xml_info)
    metadata.update(
        {
            "session_id": session_id,
            "host_start_epoch_s": host_start_epoch_s,
            "host_stop_epoch_s": host_stop_epoch_s,
            "radar_start_session_elapsed_s": radar_start_session_elapsed_s,
            "radar_stop_session_elapsed_s": radar_stop_session_elapsed_s,
            "scan_time_s": scan_time_s,
            "sweep_period_s": sweep_period_s,
            "frame_count": frame_count,
            "total_sweeps": int(global_sweep_index),
            "one_sweep_points": one_sweep_points,
        }
    )
    with (output_dir / "radar_metadata.json").open("w", encoding="utf-8") as handle:
        json.dump(metadata, handle, ensure_ascii=False, indent=2)

    return metadata
