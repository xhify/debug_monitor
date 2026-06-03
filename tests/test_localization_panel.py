import os
import shutil
import sys
import unittest
import uuid
from pathlib import Path

from PySide6.QtWidgets import QApplication

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from localization_buffer import LocalizationSample
from localization_fusion import MapPoint
from widgets.localization_panel import LocalizationPanel


TEST_TMP_ROOT = Path(__file__).resolve().parents[1] / ".test_tmp"


def temp_dir():
    path = TEST_TMP_ROOT / f"localization_panel_{uuid.uuid4().hex}"
    path.mkdir(parents=True)
    return path


class FakeMappingUpdateClient:
    def __init__(self, fail: bool = False) -> None:
        self.fail = fail
        self.enabled_values: list[bool] = []

    def set_map_update_enabled(self, enabled: bool) -> dict[str, object]:
        if self.fail:
            raise RuntimeError("freeze failed")
        self.enabled_values.append(enabled)
        return {"enabled": enabled, "method": "mock"}


class FakeMapFetchClient:
    def __init__(self, points: list[MapPoint] | None = None) -> None:
        self.points = points or [MapPoint(0.0, 0.0, 0.0), MapPoint(1.0, 0.0, 0.0)]
        self.fetch_count = 0

    def fetch_once(self, cache_dir: Path):
        self.fetch_count += 1

        class Result:
            local_path = cache_dir / "frozen.csv"
            source = "mock"
            method = "mock"
            raw_file_name = "frozen.csv"

        return Result()

    def read_points(self, path: Path) -> list[MapPoint]:
        return list(self.points)


def make_sample(x: float, y: float) -> LocalizationSample:
    return LocalizationSample(
        ros_time=x + 1.0,
        recv_time=x + 10.0,
        source="/Odometry",
        frame_id="camera_init",
        child_frame_id="body",
        x=x,
        y=y,
        z=0.0,
        qx=0.0,
        qy=0.0,
        qz=0.0,
        qw=1.0,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
    )


class LocalizationPanelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_default_map_fetch_config_uses_actual_laser_map_topic(self) -> None:
        panel = LocalizationPanel()

        fetcher = panel._current_map_fetch_client()

        self.assertEqual(fetcher.config.map_topic, "/Laser_map")
        self.assertEqual(fetcher.config.remote_map_path, "")

    def test_freeze_failure_does_not_toggle_button_or_fetch_map(self) -> None:
        fetcher = FakeMapFetchClient()
        panel = LocalizationPanel(
            mapping_update_client=FakeMappingUpdateClient(fail=True),
            map_fetch_client=fetcher,
        )

        panel._toggle_mapping_freeze()

        self.assertEqual(panel._mapping_freeze_btn.text(), "冻结建图")
        self.assertFalse(panel._map_frozen)
        self.assertEqual(fetcher.fetch_count, 0)
        self.assertIn("失败", panel._record_label.text())

    def test_freeze_success_fetches_one_map_snapshot_and_resume_fetches_none(self) -> None:
        mapping = FakeMappingUpdateClient()
        fetcher = FakeMapFetchClient()
        panel = LocalizationPanel(mapping_update_client=mapping, map_fetch_client=fetcher)

        panel._toggle_mapping_freeze()
        panel._toggle_mapping_freeze()

        self.assertEqual(mapping.enabled_values, [False, True])
        self.assertEqual(fetcher.fetch_count, 1)
        self.assertEqual(panel._mapping_freeze_btn.text(), "冻结建图")
        self.assertFalse(panel._map_frozen)

    def test_save_package_requires_frozen_map_and_trajectory(self) -> None:
        tmp = temp_dir()
        try:
            panel = LocalizationPanel()

            self.assertIsNone(panel._save_frozen_package(tmp / "out.zip"))
            self.assertIn("请先冻结建图并获取地图", panel._record_label.text())

            panel._frozen_map_points = [MapPoint(0.0, 0.0, 0.0)]
            panel._map_frozen = True
            self.assertIsNone(panel._save_frozen_package(tmp / "out.zip"))
            self.assertIn("当前没有轨迹数据", panel._record_label.text())
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_save_package_writes_zip_after_frozen_map_and_trajectory(self) -> None:
        tmp = temp_dir()
        try:
            panel = LocalizationPanel()
            panel._frozen_map_points = [MapPoint(0.0, 0.0, 0.0), MapPoint(1.0, 0.0, 0.0)]
            panel._map_frozen = True
            panel._map_fetch_metadata = {
                "map_source": "mock",
                "map_freeze_method": "mock",
                "raw_map_file": "",
            }
            panel._buffer.append(make_sample(0.0, 0.0))
            panel._buffer.append(make_sample(1.0, 0.25))

            saved = panel._save_frozen_package(tmp / "out.zip")

            self.assertEqual(saved, tmp / "out.zip")
            self.assertTrue((tmp / "out.zip").exists())
            self.assertIn("已保存", panel._record_label.text())
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
