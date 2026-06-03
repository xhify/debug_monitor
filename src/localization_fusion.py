"""Frozen FAST-LIO map and trajectory fusion helpers."""

from __future__ import annotations

import csv
import json
import shutil
import tempfile
import zipfile
from dataclasses import dataclass, fields
from datetime import datetime
from html import escape
from math import ceil
from pathlib import Path
from typing import Iterable, Sequence

from localization_buffer import LOCALIZATION_CSV_HEADER, LocalizationSample


@dataclass(frozen=True, slots=True)
class MapPoint:
    x: float
    y: float
    z: float = 0.0
    intensity: float = 0.0


def read_map_points(path: Path, max_points: int | None = None) -> list[MapPoint]:
    """Read a supported frozen-map file into the internal point format."""
    path = Path(path)
    suffix = path.suffix.lower()
    if suffix == ".ply":
        return read_ascii_ply_xy(path, max_points=max_points)
    if suffix == ".csv":
        return read_csv_map_points(path, max_points=max_points)
    if suffix == ".pcd":
        return read_ascii_pcd_points(path, max_points=max_points)
    raise ValueError(f"不支持的地图格式: {path.suffix or path}")


def read_ascii_ply_xy(path: Path, max_points: int | None = None) -> list[MapPoint]:
    path = Path(path)
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        first = handle.readline().strip()
        if first != "ply":
            raise ValueError(f"不是 PLY 文件: {path}")

        vertex_count: int | None = None
        properties: list[str] = []
        reading_vertex_properties = False
        while True:
            line = handle.readline()
            if line == "":
                raise ValueError(f"PLY 文件缺少 end_header: {path}")
            stripped = line.strip()
            if stripped == "end_header":
                break
            parts = stripped.split()
            if len(parts) >= 3 and parts[0] == "element" and parts[1] == "vertex":
                vertex_count = int(parts[2])
                reading_vertex_properties = True
                continue
            if parts[:2] == ["element", "face"]:
                reading_vertex_properties = False
                continue
            if reading_vertex_properties and len(parts) >= 3 and parts[0] == "property":
                properties.append(parts[-1])

        if vertex_count is None:
            raise ValueError(f"PLY 文件缺少 vertex 数量: {path}")
        x_index = _property_index(properties, "x")
        y_index = _property_index(properties, "y")
        z_index = properties.index("z") if "z" in properties else None
        intensity_index = properties.index("intensity") if "intensity" in properties else None

        points: list[MapPoint] = []
        for _ in range(vertex_count):
            line = handle.readline()
            if line == "":
                break
            values = line.strip().split()
            if len(values) <= max(x_index, y_index):
                continue
            z = _float_at(values, z_index)
            intensity = _float_at(values, intensity_index)
            points.append(MapPoint(float(values[x_index]), float(values[y_index]), z, intensity))

    return _downsample(points, max_points)


def read_csv_map_points(path: Path, max_points: int | None = None) -> list[MapPoint]:
    points: list[MapPoint] = []
    with Path(path).open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"CSV 地图缺少表头: {path}")
        lowered = {name.lower(): name for name in reader.fieldnames}
        if "x" not in lowered or "y" not in lowered:
            raise ValueError(f"CSV 地图至少需要 x/y 字段: {path}")
        for row in reader:
            points.append(
                MapPoint(
                    float(row[lowered["x"]]),
                    float(row[lowered["y"]]),
                    _float_cell(row.get(lowered.get("z", ""))),
                    _float_cell(row.get(lowered.get("intensity", ""))),
                )
            )
    return _downsample(points, max_points)


def read_ascii_pcd_points(path: Path, max_points: int | None = None) -> list[MapPoint]:
    path = Path(path)
    fields_line = ""
    data_ascii = False
    points: list[MapPoint] = []
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            stripped = line.strip()
            if not stripped:
                continue
            lower = stripped.lower()
            if lower.startswith("fields "):
                fields_line = stripped
            if lower.startswith("data "):
                data_ascii = lower == "data ascii"
                break
        if not data_ascii:
            raise ValueError(f"仅支持 ASCII PCD: {path}")
        names = fields_line.split()[1:]
        x_index = _property_index(names, "x")
        y_index = _property_index(names, "y")
        z_index = names.index("z") if "z" in names else None
        intensity_index = names.index("intensity") if "intensity" in names else None
        for line in handle:
            values = line.strip().split()
            if len(values) <= max(x_index, y_index):
                continue
            points.append(
                MapPoint(
                    float(values[x_index]),
                    float(values[y_index]),
                    _float_at(values, z_index),
                    _float_at(values, intensity_index),
                )
            )
    return _downsample(points, max_points)


def save_fused_map_trajectory(
    map_ply: Path,
    trajectory_points: Sequence[tuple[float, float]],
    output_path: Path,
    *,
    size: int = 1000,
    padding: int = 60,
    point_radius: float = 0.9,
    max_map_points: int | None = 50000,
) -> dict[str, object]:
    map_points = read_map_points(Path(map_ply), max_points=max_map_points)
    return render_fused_map_trajectory_svg(
        map_points,
        trajectory_points,
        Path(output_path),
        size=size,
        padding=padding,
        point_radius=point_radius,
        map_source=str(map_ply),
    )


def render_fused_map_trajectory_svg(
    map_points: Sequence[MapPoint],
    trajectory_points: Sequence[tuple[float, float]],
    output_path: Path,
    *,
    size: int = 1000,
    padding: int = 60,
    point_radius: float = 0.9,
    map_source: str = "",
) -> dict[str, object]:
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    trajectory = [(float(x), float(y)) for x, y in trajectory_points]

    all_x = [point.x for point in map_points] + [point[0] for point in trajectory]
    all_y = [point.y for point in map_points] + [point[1] for point in trajectory]
    min_x, max_x, min_y, max_y = _bounds(all_x, all_y)

    plot_size = max(1.0, float(size - 2 * padding))
    x_span = max(max_x - min_x, 1e-9)
    y_span = max(max_y - min_y, 1e-9)
    scale = plot_size / max(x_span, y_span)
    x_offset = padding + (plot_size - x_span * scale) / 2.0
    y_offset = padding + (plot_size - y_span * scale) / 2.0

    def project_xy(x: float, y: float) -> tuple[float, float]:
        svg_x = x_offset + (x - min_x) * scale
        svg_y = size - (y_offset + (y - min_y) * scale)
        return svg_x, svg_y

    with output_path.open("w", encoding="utf-8", newline="\n") as handle:
        handle.write(
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{size}" height="{size}" '
            f'viewBox="0 0 {size} {size}" data-fused-map-trajectory="true" '
            f'data-map-points="{len(map_points)}" data-trajectory-points="{len(trajectory)}" '
            f'data-x-range="{min_x},{max_x}" data-y-range="{min_y},{max_y}">\n'
        )
        handle.write('<rect width="100%" height="100%" fill="#ffffff"/>\n')
        handle.write(
            f'<rect x="{padding}" y="{padding}" width="{plot_size}" height="{plot_size}" '
            'fill="none" stroke="#d6dde5" stroke-width="1"/>\n'
        )
        if map_source:
            handle.write(f"<title>{escape(map_source)}</title>\n")
        for point in map_points:
            svg_x, svg_y = project_xy(point.x, point.y)
            opacity = max(0.18, min(0.72, point.intensity / 255.0 if point.intensity else 0.32))
            handle.write(
                f'<circle cx="{svg_x:.3f}" cy="{svg_y:.3f}" r="{point_radius}" '
                f'fill="#64748b" fill-opacity="{opacity:.3f}"/>\n'
            )
        if trajectory:
            polyline = " ".join(
                f"{project_xy(x, y)[0]:.3f},{project_xy(x, y)[1]:.3f}"
                for x, y in trajectory
            )
            handle.write(
                f'<polyline points="{polyline}" fill="none" stroke="#16a34a" '
                'stroke-width="3" stroke-linecap="round" stroke-linejoin="round"/>\n'
            )
            start_x, start_y = project_xy(*trajectory[0])
            end_x, end_y = project_xy(*trajectory[-1])
            handle.write(f'<circle cx="{start_x:.3f}" cy="{start_y:.3f}" r="5" fill="#0284c7"/>\n')
            handle.write(f'<circle cx="{end_x:.3f}" cy="{end_y:.3f}" r="5" fill="#ea580c"/>\n')
        handle.write("</svg>\n")

    return {
        "map_points": len(map_points),
        "trajectory_points": len(trajectory),
        "output": str(output_path),
        "min_x": min_x,
        "max_x": max_x,
        "min_y": min_y,
        "max_y": max_y,
    }


def export_frozen_map_trajectory_zip(
    output_zip: Path,
    *,
    map_points: Sequence[MapPoint],
    trajectory_rows: Sequence[LocalizationSample],
    metadata: dict[str, object],
    raw_map_path: Path | None = None,
) -> dict[str, object]:
    output_zip = Path(output_zip)
    output_zip.parent.mkdir(parents=True, exist_ok=True)
    preview_name = "preview.svg"
    raw_map_archive_name = ""
    if raw_map_path:
        raw_map_archive_name = f"raw_map/{Path(raw_map_path).name}"

    with tempfile.TemporaryDirectory(prefix="frozen_map_trajectory_") as temp_name:
        temp_dir = Path(temp_name)
        map_csv = temp_dir / "frozen_map_points.csv"
        trajectory_csv = temp_dir / "trajectory_points.csv"
        preview_svg = temp_dir / preview_name
        metadata_json = temp_dir / "metadata.json"

        _write_map_csv(map_csv, map_points)
        _write_trajectory_csv(trajectory_csv, trajectory_rows)
        render_fused_map_trajectory_svg(
            map_points,
            [(row.x, row.y) for row in trajectory_rows],
            preview_svg,
            map_source=str(metadata.get("map_source", "")),
        )
        merged_metadata = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "coordinate_frame": metadata.get("coordinate_frame", "camera_init x-y top-down"),
            "odometry_topic": metadata.get("odometry_topic", "/Odometry"),
            "map_source": metadata.get("map_source", ""),
            "map_freeze_method": metadata.get("map_freeze_method", metadata.get("freeze_method", "")),
            "map_point_count": len(map_points),
            "trajectory_point_count": len(trajectory_rows),
            "use_aligned_xy": bool(metadata.get("use_aligned_xy", False)),
            "preview_file": preview_name,
            "raw_map_file": raw_map_archive_name,
        }
        metadata_json.write_text(
            json.dumps(merged_metadata, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

        with zipfile.ZipFile(output_zip, "w", compression=zipfile.ZIP_DEFLATED) as archive:
            archive.write(metadata_json, "metadata.json")
            archive.write(map_csv, "frozen_map_points.csv")
            archive.write(trajectory_csv, "trajectory_points.csv")
            archive.write(preview_svg, preview_name)
            if raw_map_path:
                raw_copy = temp_dir / Path(raw_map_archive_name)
                raw_copy.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(raw_map_path, raw_copy)
                archive.write(raw_copy, raw_map_archive_name)

    return {
        "output": str(output_zip),
        "map_points": len(map_points),
        "trajectory_points": len(trajectory_rows),
        "preview": preview_name,
    }


def _write_map_csv(path: Path, points: Sequence[MapPoint]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["x", "y", "z", "intensity"])
        writer.writeheader()
        for point in points:
            writer.writerow({
                "x": point.x,
                "y": point.y,
                "z": point.z,
                "intensity": point.intensity,
            })


def _write_trajectory_csv(path: Path, rows: Sequence[LocalizationSample]) -> None:
    valid_fields = {field.name for field in fields(LocalizationSample)}
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=LOCALIZATION_CSV_HEADER)
        writer.writeheader()
        for row in rows:
            writer.writerow({name: getattr(row, name) for name in LOCALIZATION_CSV_HEADER if name in valid_fields})


def _property_index(properties: list[str], name: str) -> int:
    if name not in properties:
        raise ValueError(f"点云缺少 {name} 字段")
    return properties.index(name)


def _downsample(points: list[MapPoint], max_points: int | None) -> list[MapPoint]:
    if max_points is None or max_points <= 0 or len(points) <= max_points:
        return points
    step = max(1, ceil(len(points) / max_points))
    return points[::step]


def _float_at(values: Sequence[str], index: int | None) -> float:
    if index is None or index >= len(values):
        return 0.0
    return float(values[index])


def _float_cell(value: object) -> float:
    if value is None:
        return 0.0
    text = str(value).strip()
    return float(text) if text else 0.0


def _bounds(xs: Iterable[float], ys: Iterable[float]) -> tuple[float, float, float, float]:
    x_values = list(xs)
    y_values = list(ys)
    if x_values and y_values:
        return min(x_values), max(x_values), min(y_values), max(y_values)
    return 0.0, 0.0, 0.0, 0.0
