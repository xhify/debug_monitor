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

    def test_default_map_fetch_config_uses_current_rosbridge_host_port_and_laser_map_topic(self) -> None:
        panel = LocalizationPanel()
        panel._host_edit.setText("robot.local")
        panel._port_spin.setValue(19090)

        mapping = panel._current_mapping_update_client()
        fetcher = panel._current_map_fetch_client()

        self.assertEqual(mapping.config.host, "robot.local")
        self.assertEqual(mapping.config.port, 19090)
        self.assertEqual(mapping.config.parameter, "/mapping/map_update_enable")
        self.assertEqual(fetcher.config.host, "robot.local")
        self.assertEqual(fetcher.config.port, 19090)
        self.assertEqual(fetcher.config.map_topic, "/Laser_map")

    def test_fastlio_topic_edit_emits_normalized_topic(self) -> None:
        panel = LocalizationPanel()
        topics: list[str] = []
        panel.fastlio_topic_changed.connect(topics.append)
        panel._topic_edit.setText("Odometry2")

        panel._topic_edit.editingFinished.emit()

        self.assertEqual(panel.fastlio_odometry_topic(), "/Odometry2")
        self.assertEqual(topics, ["/Odometry2"])

    def test_shared_localization_sample_updates_buffer(self) -> None:
        panel = LocalizationPanel()

        panel.accept_localization_sample(make_sample(2.0, 0.1))

        self.assertAlmostEqual(panel._buffer.latest().x, 2.0)
        self.assertIn("在线", panel._online_label.text())

    def test_calibration_launch_uses_current_fastlio_topic(self) -> None:
        panel = LocalizationPanel()
        commands: list[str] = []
        panel.launch_manager_command_requested.connect(commands.append)
        panel.set_fastlio_odometry_topic("/fastlio/odom")
        panel.set_shared_ros_connected(True)

        panel._calibration_launch_start_btn.click()

        self.assertEqual(
            commands,
            [
                "restart pid_control simple_follower pid_control_lidar_assisted.launch "
                "imu_topic:=/active_imu lidar_odom_topic:=/fastlio/odom"
            ],
        )
        self.assertFalse(panel._calibration_launch_start_btn.isEnabled())
        self.assertFalse(panel._calibration_launch_stop_btn.isEnabled())

    def test_calibration_motion_requires_confirmed_running_status(self) -> None:
        panel = LocalizationPanel()
        controls: list[tuple[float, bool, bool]] = []
        panel.line_follow_control_requested.connect(
            lambda speed, forward, backward: controls.append((speed, forward, backward))
        )
        panel.set_shared_ros_connected(True)

        panel._calibration_forward_btn.click()

        self.assertEqual(controls, [])
        self.assertFalse(panel._calibration_forward_btn.isEnabled())

    def test_calibration_forward_backward_stop_emit_line_follow_controls(self) -> None:
        panel = LocalizationPanel()
        controls: list[tuple[float, bool, bool]] = []
        panel.line_follow_control_requested.connect(
            lambda speed, forward, backward: controls.append((speed, forward, backward))
        )
        panel.set_shared_ros_connected(True)
        panel.update_launch_manager_status(
            {
                "running": ["pid_control"],
                "detail": {
                    "pid_control": {
                        "package": "simple_follower",
                        "launch": "pid_control_lidar_assisted.launch",
                    }
                },
            }
        )
        panel._calibration_speed_spin.setValue(0.3)

        panel._calibration_forward_btn.click()
        panel._calibration_backward_btn.click()
        panel._calibration_stop_btn.click()

        self.assertEqual(
            controls,
            [
                (0.3, True, False),
                (0.3, False, True),
                (0.0, False, False),
            ],
        )

    def test_shutdown_stops_active_calibration_motion(self) -> None:
        panel = LocalizationPanel()
        controls: list[tuple[float, bool, bool]] = []
        panel.line_follow_control_requested.connect(
            lambda speed, forward, backward: controls.append((speed, forward, backward))
        )
        panel.set_shared_ros_connected(True)
        panel.update_launch_manager_status(
            {
                "running": ["pid_control"],
                "detail": {
                    "pid_control": {
                        "launch": "pid_control_lidar_assisted.launch",
                    }
                },
            }
        )
        panel._calibration_forward_btn.click()

        panel.shutdown()

        self.assertEqual(controls[-1], (0.0, False, False))

    def test_trajectory_plot_keeps_equal_xy_scale_for_map_overlay(self) -> None:
        panel = LocalizationPanel()

        view_box = panel._trajectory_plot.getPlotItem().getViewBox()

        self.assertEqual(view_box.state["aspectLocked"], 1.0)

    def test_lidar_launch_buttons_publish_launch_manager_commands(self) -> None:
        panel = LocalizationPanel()
        commands: list[str] = []
        panel._worker.publish_launch_manager_command = commands.append

        panel._set_connected(True)
        panel._lidar_launch_start_btn.click()
        panel._lidar_launch_stop_btn.click()

        self.assertEqual(
            commands,
            [
                "start lidar turn_on_wheeltec_robot wheeltec_lidar.launch",
                "stop lidar",
            ],
        )
        self.assertIn("雷达", panel._lidar_launch_label.text())

    def test_launch_buttons_request_status_after_commands(self) -> None:
        panel = LocalizationPanel()
        queries: list[str] = []
        panel.status_query_requested.connect(lambda: queries.append("query"))
        panel._set_connected(True)

        panel._fastlio_launch_start_btn.click()
        panel._fastlio_launch_stop_btn.click()
        panel._lidar_launch_start_btn.click()
        panel._lidar_launch_stop_btn.click()

        self.assertEqual(queries, ["query", "query", "query", "query"])

    def test_launch_manager_status_updates_fastlio_and_lidar_buttons(self) -> None:
        panel = LocalizationPanel()
        panel._set_connected(True)

        panel.update_launch_manager_status(
            {
                "running": ["fastlio", "lidar"],
                "detail": {
                    "fastlio": {"package": "fast_lio", "launch": "mapping_c16.launch"},
                    "lidar": {"package": "turn_on_wheeltec_robot", "launch": "wheeltec_lidar.launch"},
                },
            }
        )

        self.assertFalse(panel._fastlio_launch_start_btn.isEnabled())
        self.assertTrue(panel._fastlio_launch_stop_btn.isEnabled())
        self.assertFalse(panel._lidar_launch_start_btn.isEnabled())
        self.assertTrue(panel._lidar_launch_stop_btn.isEnabled())
        self.assertIn("运行中", panel._fastlio_launch_label.text())
        self.assertIn("mapping_c16.launch", panel._fastlio_launch_label.text())
        self.assertIn("运行中", panel._lidar_launch_label.text())
        self.assertIn("wheeltec_lidar.launch", panel._lidar_launch_label.text())

        panel.update_launch_manager_status(
            {
                "running": ["lidar"],
                "detail": {"lidar": {"package": "turn_on_wheeltec_robot", "launch": "wheeltec_lidar.launch"}},
            }
        )

        self.assertTrue(panel._fastlio_launch_start_btn.isEnabled())
        self.assertFalse(panel._fastlio_launch_stop_btn.isEnabled())
        self.assertFalse(panel._lidar_launch_start_btn.isEnabled())
        self.assertTrue(panel._lidar_launch_stop_btn.isEnabled())
        self.assertIn("未运行", panel._fastlio_launch_label.text())

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

    def test_frozen_map_overlay_uses_camera_init_trajectory_coordinates(self) -> None:
        panel = LocalizationPanel()
        panel._map_frozen = True
        panel._frozen_map_points = [MapPoint(100.0, 50.0, 0.0)]
        panel._buffer.append(make_sample(100.0, 50.0))
        panel._buffer.append(make_sample(101.0, 50.25))

        panel._refresh_view()

        xs, ys = panel._trajectory_curve.getData()
        self.assertEqual(list(xs), [100.0, 101.0])
        self.assertEqual(list(ys), [50.0, 50.25])


if __name__ == "__main__":
    unittest.main()
