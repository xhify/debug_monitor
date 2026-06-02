"""PLY map and FAST-LIO trajectory fusion rendering."""

from __future__ import annotations

from dataclasses import dataclass
from html import escape
from math import ceil
from pathlib import Path
from typing import Sequence


@dataclass(frozen=True, slots=True)
class MapPoint:
    x: float
    y: float
    intensity: float = 0.0


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
        intensity_index = properties.index("intensity") if "intensity" in properties else None

        points: list[MapPoint] = []
        for _ in range(vertex_count):
            line = handle.readline()
            if line == "":
                break
            values = line.strip().split()
            if len(values) <= max(x_index, y_index):
                continue
            intensity = 0.0
            if intensity_index is not None and len(values) > intensity_index:
                intensity = float(values[intensity_index])
            points.append(MapPoint(float(values[x_index]), float(values[y_index]), intensity))

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
    map_points = read_ascii_ply_xy(Path(map_ply), max_points=max_map_points)
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
    if all_x and all_y:
        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)
    else:
        min_x = max_x = min_y = max_y = 0.0

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


def _property_index(properties: list[str], name: str) -> int:
    if name not in properties:
        raise ValueError(f"PLY vertex 缺少 {name} 属性")
    return properties.index(name)


def _downsample(points: list[MapPoint], max_points: int | None) -> list[MapPoint]:
    if max_points is None or max_points <= 0 or len(points) <= max_points:
        return points
    step = max(1, ceil(len(points) / max_points))
    return points[::step]
