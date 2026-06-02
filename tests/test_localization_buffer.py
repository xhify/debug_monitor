import csv
import math
import os
import shutil
import sys
import unittest
import uuid
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from localization_buffer import LocalizationBuffer, LocalizationSample


TEST_TMP_ROOT = Path(__file__).resolve().parents[1] / ".test_tmp"


def temp_dir():
    path = TEST_TMP_ROOT / f"localization_{uuid.uuid4().hex}"
    path.mkdir(parents=True)
    return path


def make_sample(ros_time: float, x: float, y: float, yaw: float) -> LocalizationSample:
    half = yaw / 2.0
    return LocalizationSample(
        ros_time=ros_time,
        recv_time=100.0 + ros_time,
        source="/Odometry",
        frame_id="camera_init",
        child_frame_id="body",
        x=x,
        y=y,
        z=0.0,
        qx=0.0,
        qy=0.0,
        qz=math.sin(half),
        qw=math.cos(half),
        roll=0.0,
        pitch=0.0,
        yaw=yaw,
    )


class LocalizationBufferTests(unittest.TestCase):
    def test_aligns_to_first_pose_and_computes_straight_line_metrics(self) -> None:
        buffer = LocalizationBuffer()

        buffer.append(make_sample(1.0, 10.0, 20.0, math.radians(30.0)))
        buffer.append(make_sample(2.0, 10.8660254, 20.5, math.radians(35.0)))

        latest = buffer.latest()
        stats = buffer.stats()

        self.assertAlmostEqual(latest.x0_aligned, 1.0, places=5)
        self.assertAlmostEqual(latest.y0_aligned, 0.0, places=5)
        self.assertAlmostEqual(latest.yaw0_aligned, math.radians(5.0), places=5)
        self.assertAlmostEqual(stats.trajectory_length, 1.0, places=5)
        self.assertAlmostEqual(stats.endpoint_distance, 1.0, places=5)
        self.assertAlmostEqual(stats.lateral_error_current, 0.0, places=5)
        self.assertAlmostEqual(stats.yaw_rms, math.sqrt(math.radians(5.0) ** 2 / 2.0), places=5)
        self.assertAlmostEqual(stats.estimated_speed_mean, 1.0, places=5)

    def test_csv_and_markdown_report_include_control_placeholders(self) -> None:
        tmp = temp_dir()
        try:
            buffer = LocalizationBuffer()
            buffer.set_control_state(
                enabled=True,
                mode="heading_assist",
                target_speed=0.2,
                target_yaw=0.1,
                correction_vx=0.01,
                correction_vz=-0.02,
                safety_state="preview_only",
            )
            buffer.start_recording()
            buffer.append(make_sample(1.0, 0.0, 0.0, 0.0))
            buffer.append(make_sample(2.0, 1.0, 0.1, 0.02))

            csv_path = buffer.stop_recording(tmp / "fastlio.csv")
            report_path = buffer.write_report(tmp / "report.md")

            with csv_path.open("r", encoding="utf-8", newline="") as fh:
                rows = list(csv.DictReader(fh))
            self.assertEqual(rows[0]["source"], "/Odometry")
            self.assertEqual(rows[1]["control_enabled"], "true")
            self.assertEqual(rows[1]["control_mode"], "heading_assist")
            self.assertEqual(rows[1]["radar_quality"], "")
            self.assertEqual(rows[1]["safety_state"], "preview_only")

            report = report_path.read_text(encoding="utf-8")
            self.assertIn("FAST-LIO2 定位稳定性测试报告", report)
            self.assertIn("当前无 ground truth", report)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
