import csv
import json
import os
import shutil
import sys
import unittest
import uuid
import zipfile
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from localization_buffer import LocalizationSample
from localization_fusion import (
    export_frozen_map_trajectory_zip,
    read_ascii_ply_xy,
    read_map_points,
    save_fused_map_trajectory,
)


TEST_TMP_ROOT = Path(__file__).resolve().parents[1] / ".test_tmp"


def temp_dir():
    path = TEST_TMP_ROOT / f"fusion_{uuid.uuid4().hex}"
    path.mkdir(parents=True)
    return path


class LocalizationFusionTests(unittest.TestCase):
    def test_reads_ascii_ply_xy_points_and_renders_fused_svg(self) -> None:
        tmp = temp_dir()
        try:
            map_path = tmp / "map.ply"
            map_path.write_text(
                "\n".join([
                    "ply",
                    "format ascii 1.0",
                    "element vertex 3",
                    "property float x",
                    "property float y",
                    "property float z",
                    "property float intensity",
                    "end_header",
                    "0 0 0 10",
                    "1 0 0 20",
                    "0 1 0 30",
                    "",
                ]),
                encoding="utf-8",
            )
            output_path = tmp / "fused.svg"

            points = read_ascii_ply_xy(map_path)
            summary = save_fused_map_trajectory(
                map_path,
                [(0.0, 0.0), (0.5, 0.25), (1.0, 0.5)],
                output_path,
                size=320,
            )

            self.assertEqual(len(points), 3)
            self.assertEqual(summary["map_points"], 3)
            self.assertEqual(summary["trajectory_points"], 3)
            svg = output_path.read_text(encoding="utf-8")
            self.assertIn('data-fused-map-trajectory="true"', svg)
            self.assertIn('data-map-points="3"', svg)
            self.assertIn('data-trajectory-points="3"', svg)
            self.assertIn("<polyline", svg)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_reads_csv_map_points_through_common_adapter(self) -> None:
        tmp = temp_dir()
        try:
            map_path = tmp / "map.csv"
            map_path.write_text(
                "x,y,z,intensity\n"
                "1.0,2.0,3.0,42\n"
                "4.0,5.0,6.0,\n",
                encoding="utf-8",
            )

            points = read_map_points(map_path)

            self.assertEqual([(p.x, p.y, p.z, p.intensity) for p in points], [
                (1.0, 2.0, 3.0, 42.0),
                (4.0, 5.0, 6.0, 0.0),
            ])
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_export_frozen_map_trajectory_zip_contains_replayable_data(self) -> None:
        tmp = temp_dir()
        try:
            raw_map = tmp / "frozen_map.csv"
            raw_map.write_text("x,y,z,intensity\n100,50,0,10\n101,50,0,20\n", encoding="utf-8")
            output_zip = tmp / "frozen_map_trajectory.zip"
            trajectory = [
                LocalizationSample(
                    ros_time=1.0,
                    recv_time=10.0,
                    source="/Odometry",
                    frame_id="camera_init",
                    child_frame_id="body",
                    x=100.0,
                    y=50.0,
                    z=0.0,
                    qx=0.0,
                    qy=0.0,
                    qz=0.0,
                    qw=1.0,
                    roll=0.0,
                    pitch=0.0,
                    yaw=0.0,
                    x0_aligned=0.0,
                    y0_aligned=0.0,
                ),
                LocalizationSample(
                    ros_time=2.0,
                    recv_time=11.0,
                    source="/Odometry",
                    frame_id="camera_init",
                    child_frame_id="body",
                    x=101.0,
                    y=50.25,
                    z=0.0,
                    qx=0.0,
                    qy=0.0,
                    qz=0.0,
                    qw=1.0,
                    roll=0.0,
                    pitch=0.0,
                    yaw=0.1,
                    yaw0_aligned=0.1,
                    x0_aligned=1.0,
                    y0_aligned=0.25,
                    lateral_error=0.25,
                    trajectory_length=1.03,
                ),
            ]

            summary = export_frozen_map_trajectory_zip(
                output_zip,
                map_points=read_map_points(raw_map),
                trajectory_rows=trajectory,
                metadata={
                    "coordinate_frame": "map xy top-down",
                    "odometry_topic": "/Odometry",
                    "map_source": "remote file",
                    "freeze_method": "rosparam:/mapping/map_update_enable=false",
                    "use_aligned_xy": False,
                },
                raw_map_path=raw_map,
            )

            self.assertEqual(summary["map_points"], 2)
            self.assertEqual(summary["trajectory_points"], 2)
            with zipfile.ZipFile(output_zip) as archive:
                names = set(archive.namelist())
                self.assertIn("metadata.json", names)
                self.assertIn("frozen_map_points.csv", names)
                self.assertIn("trajectory_points.csv", names)
                self.assertIn("preview.svg", names)
                self.assertIn("raw_map/frozen_map.csv", names)
                metadata = json.loads(archive.read("metadata.json").decode("utf-8"))
                self.assertEqual(metadata["map_point_count"], 2)
                self.assertEqual(metadata["trajectory_point_count"], 2)
                self.assertFalse(metadata["use_aligned_xy"])
                self.assertEqual(metadata["preview_file"], "preview.svg")
                self.assertEqual(metadata["raw_map_file"], "raw_map/frozen_map.csv")
                preview = archive.read("preview.svg").decode("utf-8")
                self.assertIn('data-x-range="100.0,101.0"', preview)
                self.assertIn('data-y-range="50.0,50.25"', preview)
                rows = list(csv.DictReader(
                    archive.read("trajectory_points.csv").decode("utf-8").splitlines()
                ))
                self.assertEqual(rows[1]["x0_aligned"], "1.0")
                self.assertEqual(rows[1]["lateral_error"], "0.25")
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
