import os
import shutil
import sys
import unittest
import uuid
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from localization_fusion import read_ascii_ply_xy, save_fused_map_trajectory


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


if __name__ == "__main__":
    unittest.main()
