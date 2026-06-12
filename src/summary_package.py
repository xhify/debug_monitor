"""汇总记录目录打包。"""

from __future__ import annotations

import csv
import json
from pathlib import Path
import shutil
import time
import zipfile

from alignment import write_alignment_outputs


RAW_FILE_MAP = {
    "encoder.csv": "serial_encoder.csv",
    "fastlio_odometry.csv": "fastlio_odometry.csv",
    "trajectory_odometry.csv": "trajectory_odometry.csv",
    "ros_odom.csv": "ros_odom.csv",
    "ros_imu.csv": "ros_imu.csv",
    "ros_active_imu.csv": "ros_active_imu.csv",
    "ros_power_voltage.csv": "ros_power_voltage.csv",
    "akm_state.csv": "akm_state.csv",
    "control_debug.csv": "control_debug.csv",
    "chassis_diagnostics.csv": "chassis_diagnostics.csv",
    "imu_A.csv": "imu_A.csv",
    "imu_B.csv": "imu_B.csv",
    "imu_session.json": "imu_session.json",
    "imu_merged_aligned.csv": "imu_merged_aligned.csv",
    "ros_imu_merged_aligned.csv": "ros_imu_merged_aligned.csv",
}


def build_summary_package(session_dir: Path) -> dict[str, object]:
    total_start = time.perf_counter()
    session_dir = Path(session_dir)
    raw_dir = session_dir / "raw"
    aligned_dir = session_dir / "aligned"
    raw_dir.mkdir(exist_ok=True)
    aligned_dir.mkdir(exist_ok=True)
    package_zip = session_dir / f"{session_dir.name}.zip"

    source_session = {}
    session_json_path = session_dir / "session.json"
    if session_json_path.exists():
        with session_json_path.open("r", encoding="utf-8") as handle:
            source_session = json.load(handle)

    generated_files: list[str] = []
    row_counts: dict[str, int] = {}
    warnings: list[str] = list(source_session.get("warnings", []))
    errors: list[str] = list(source_session.get("errors", []))
    timings_s: dict[str, float] = {}

    stage_start = time.perf_counter()
    for source_name, raw_name in RAW_FILE_MAP.items():
        source_path = session_dir / source_name
        if not source_path.exists():
            continue
        target_path = raw_dir / raw_name
        shutil.copy2(source_path, target_path)
        generated_files.append(str(target_path.relative_to(session_dir)).replace("\\", "/"))
        if source_path.suffix.lower() == ".csv":
            row_counts[raw_name] = _count_csv_rows(source_path)

    existing_radar_dir = raw_dir / "radar"
    if existing_radar_dir.exists():
        for path in existing_radar_dir.rglob("*"):
            if not path.is_file():
                continue
            generated_files.append(str(path.relative_to(session_dir)).replace("\\", "/"))
            if path.suffix.lower() == ".csv":
                row_counts[str(path.relative_to(raw_dir)).replace("\\", "/")] = _count_csv_rows(path)
    timings_s["copy_raw"] = _elapsed(stage_start)

    stage_start = time.perf_counter()
    try:
        alignment_stats = write_alignment_outputs(raw_dir=raw_dir, aligned_dir=aligned_dir)
    except Exception as exc:
        alignment_stats = {"trajectory_rows": 0, "chassis_rows": 0}
        warnings.append(f"aligned generation skipped: {exc}")
    else:
        for name in ("trajectory_aligned.csv", "chassis_100hz_aligned.csv"):
            path = aligned_dir / name
            if path.exists():
                generated_files.append(str(path.relative_to(session_dir)).replace("\\", "/"))
                row_counts[name] = _count_csv_rows(path)
    timings_s["alignment"] = _elapsed(stage_start)

    for relative in ("session.json", "manifest.json", package_zip.name):
        if relative not in generated_files:
            generated_files.append(relative)

    rosbag_manifest = _write_rosbag_manifest(session_dir, source_session)
    if rosbag_manifest is not None:
        manifest_relative = "raw/rosbag_manifest.json"
        if manifest_relative not in generated_files:
            generated_files.append(manifest_relative)

    if session_json_path.exists():
        source_session["generated_files"] = generated_files
        source_session["package_zip"] = str(package_zip)
        source_session["manifest_path"] = "manifest.json"
        with session_json_path.open("w", encoding="utf-8") as handle:
            json.dump(source_session, handle, ensure_ascii=False, indent=2)

    manifest = {
        "session_id": source_session.get("session_id", session_dir.name),
        "started_at_iso": source_session.get("started_at_iso", source_session.get("started_at", "")),
        "stopped_at_iso": source_session.get("stopped_at_iso", ""),
        "selected_sources": source_session.get("selected_sources", []),
        "generated_files": generated_files,
        "row_counts": row_counts,
        "estimated_topic_hz": source_session.get("estimated_topic_hz", {}),
        "warnings": warnings,
        "errors": errors,
        "package_zip": str(package_zip),
        "alignment": alignment_stats,
        "timings_s": timings_s,
    }
    if rosbag_manifest is not None:
        manifest["rosbag"] = {
            "enabled": bool(rosbag_manifest.get("enabled", False)),
            "session_id": rosbag_manifest.get("session_id", ""),
            "manifest_path": "raw/rosbag_manifest.json",
            "local_file_count": rosbag_manifest.get("local_file_count", 0),
            "local_total_bytes": rosbag_manifest.get("local_total_bytes", 0),
        }

    timings_s["zip"] = _write_zip(package_zip, session_dir, manifest, total_start)

    manifest_path = session_dir / "manifest.json"
    with manifest_path.open("w", encoding="utf-8") as handle:
        json.dump(manifest, handle, ensure_ascii=False, indent=2)

    return manifest


def _count_csv_rows(path: Path) -> int:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return max(0, sum(1 for _ in csv.reader(handle)) - 1)


def _elapsed(start: float) -> float:
    return round(time.perf_counter() - start, 6)


def _write_zip(package_zip: Path, session_dir: Path, manifest: dict[str, object], total_start: float) -> float:
    start = time.perf_counter()
    with zipfile.ZipFile(package_zip, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        path = session_dir / "session.json"
        if path.exists():
            archive.write(path, "session.json")
        for folder in ("raw", "aligned"):
            base = session_dir / folder
            if not base.exists():
                continue
            for path in base.rglob("*"):
                if path.is_file():
                    if path.suffix.lower() == ".bag":
                        continue
                    archive.write(path, str(path.relative_to(session_dir)))
        elapsed = _elapsed(start)
        manifest["timings_s"]["zip"] = elapsed
        manifest["timings_s"]["total"] = _elapsed(total_start)
        archive.writestr("manifest.json", json.dumps(manifest, ensure_ascii=False, indent=2))
    return elapsed


def _write_rosbag_manifest(session_dir: Path, source_session: dict[str, object]) -> dict[str, object] | None:
    rosbag = source_session.get("rosbag")
    if not isinstance(rosbag, dict):
        return None
    raw_dir = session_dir / "raw"
    raw_dir.mkdir(exist_ok=True)
    manifest_path = raw_dir / "rosbag_manifest.json"
    rosbag_manifest = dict(rosbag)
    local_files: list[dict[str, object]] = []
    local_dir_text = str(rosbag.get("local_dir", ""))
    local_dir = Path(local_dir_text) if local_dir_text else session_dir / "raw" / "rosbag"
    if local_dir.exists():
        for path in local_dir.rglob("*"):
            if not path.is_file():
                continue
            try:
                relative = str(path.relative_to(session_dir)).replace("\\", "/")
            except ValueError:
                relative = str(path)
            local_files.append(
                {
                    "path": relative,
                    "size_bytes": path.stat().st_size,
                }
            )
    rosbag_manifest["local_files"] = local_files
    rosbag_manifest["local_file_count"] = len(local_files)
    rosbag_manifest["local_total_bytes"] = sum(int(item["size_bytes"]) for item in local_files)
    with manifest_path.open("w", encoding="utf-8") as handle:
        json.dump(rosbag_manifest, handle, ensure_ascii=False, indent=2)
    return rosbag_manifest
