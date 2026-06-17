import os
import sys
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication, QGroupBox

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from main_window import MainWindow
from runtime_ui_optimizations import apply_runtime_ui_optimizations
from widgets.localization_panel import LocalizationPanel


PATCHED_MAIN_WINDOW_METHODS = (
    "_setup_ui",
    "_build_summary_source_group",
    "_make_summary_source_checkbox",
    "_build_summary_device_group",
    "_connect_summary_device",
    "_disconnect_summary_device",
    "_on_ros_connection_changed",
    "_on_ros_message",
    "_update_summary_row",
    "_apply_summary_check_results",
)


class RuntimeUiOptimizationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls._main_window_originals = {
            name: getattr(MainWindow, name)
            for name in PATCHED_MAIN_WINDOW_METHODS
            if hasattr(MainWindow, name)
        }
        cls._localization_original_build_side_panel = LocalizationPanel._build_side_panel
        cls._localization_original_build_control_group = LocalizationPanel._build_control_group
        apply_runtime_ui_optimizations(MainWindow)
        cls.app = QApplication.instance() or QApplication([])

    @classmethod
    def tearDownClass(cls) -> None:
        for name, method in cls._main_window_originals.items():
            setattr(MainWindow, name, method)
        for flag in ("_optimized_summary_ui",):
            if hasattr(MainWindow, flag):
                delattr(MainWindow, flag)
        LocalizationPanel._build_side_panel = cls._localization_original_build_side_panel
        LocalizationPanel._build_control_group = cls._localization_original_build_control_group
        if hasattr(LocalizationPanel, "_optimized_no_error_stats"):
            delattr(LocalizationPanel, "_optimized_no_error_stats")

    def tearDown(self) -> None:
        for widget in QApplication.topLevelWidgets():
            widget.close()
            widget.deleteLater()
        self.app.processEvents()

    def test_rosbridge_status_is_visible_on_ros_related_pages(self) -> None:
        window = MainWindow()
        window._ros_worker._error_count = 3

        window._on_ros_connection_changed(True)

        expected = "ROSbridge: 已连接 192.168.0.14:9090 / 网络延迟: --- / 错误 3"
        self.assertEqual(window._summary_rosbridge_status_label.text(), expected)
        self.assertEqual(window._ros_panel._status_label.text(), expected)
        self.assertEqual(window._ros_imu_panel._status_label.text(), expected)
        self.assertEqual(window._localization_panel._connection_label.text(), expected)

    def test_localization_panel_receives_odometry_from_unified_rosbridge(self) -> None:
        window = MainWindow()
        now = time.time()

        window._on_ros_message(
            {
                "topic": "/Odometry",
                "message_type": "nav_msgs/Odometry",
                "recv_time_epoch_s": now,
                "message": {
                    "header": {
                        "stamp": {"secs": 10, "nsecs": 500_000_000},
                        "frame_id": "camera_init",
                    },
                    "child_frame_id": "body",
                    "pose": {
                        "pose": {
                            "position": {"x": 1.25, "y": -0.5, "z": 0.1},
                            "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
                        }
                    },
                },
            }
        )

        latest = window._localization_panel._buffer.latest()
        self.assertIsNotNone(latest)
        self.assertEqual(latest.source, "/Odometry")
        self.assertAlmostEqual(latest.x, 1.25)
        self.assertAlmostEqual(latest.y, -0.5)
        self.assertEqual(window._localization_panel._online_label.text(), "/Odometry: 在线")

    def test_ros_related_ui_timers_refresh_at_one_hz(self) -> None:
        window = MainWindow()

        self.assertEqual(window._ros_panel._refresh_timer.interval(), 1000)
        self.assertEqual(window._ros_imu_panel._refresh_timer.interval(), 1000)
        self.assertEqual(window._localization_panel._refresh_timer.interval(), 1000)

    def test_topic_frequency_labels_refresh_at_one_hz(self) -> None:
        window = MainWindow()

        window._on_ros_message({"topic": "/active_imu", "recv_time_epoch_s": 100.0, "message": {}})
        window._on_ros_message({"topic": "/imu", "recv_time_epoch_s": 100.001, "message": {}})
        window._on_ros_message({"topic": "/active_imu", "recv_time_epoch_s": 100.01, "message": {}})
        self.assertEqual(window._summary_source_rate_labels["ros_active_imu"].text(), "0 Hz")
        window._on_ros_message({"topic": "/active_imu", "recv_time_epoch_s": 101.0, "message": {}})

        self.assertEqual(window._summary_source_rate_labels["ros_active_imu"].text(), "3 Hz")

    def test_rosbridge_status_shows_network_latency_or_clock_delta(self) -> None:
        window = MainWindow()

        window._ros_worker.network_latency_measured.emit(18.4)
        self.assertIn("网络延迟: 18.4 ms", window._summary_rosbridge_status_label.text())

        window._on_ros_message(
            {
                "topic": "/imu",
                "recv_time_epoch_s": 100.0,
                "message": {"header": {"stamp": {"secs": 99, "nsecs": 990_000_000}}},
            }
        )
        self.assertIn("网络延迟: 18.4 ms / 消息时差: 10.0 ms", window._summary_rosbridge_status_label.text())

        window._on_ros_message(
            {
                "topic": "/imu",
                "recv_time_epoch_s": 100.2,
                "message": {"header": {"stamp": {"secs": 100, "nsecs": 250_000_000}}},
            }
        )
        self.assertIn("网络延迟: 18.4 ms / 消息时差: 10.0 ms", window._summary_rosbridge_status_label.text())

        window._on_ros_message(
            {
                "topic": "/imu",
                "recv_time_epoch_s": 101.1,
                "message": {"header": {"stamp": {"secs": 100, "nsecs": 250_000_000}}},
            }
        )
        self.assertIn("网络延迟: 18.4 ms / 消息时差: 850.0 ms", window._summary_rosbridge_status_label.text())

        window._on_ros_message(
            {
                "topic": "/imu",
                "recv_time_epoch_s": 102.2,
                "message": {"header": {"stamp": {"secs": 102, "nsecs": 300_000_000}}},
            }
        )
        self.assertIn("网络延迟: 18.4 ms / 时钟差: -100.0 ms", window._summary_rosbridge_status_label.text())

    def test_rosbridge_status_ignores_outlier_stamp_after_good_latency(self) -> None:
        window = MainWindow()

        window._on_ros_message(
            {
                "topic": "/odom",
                "recv_time_epoch_s": 100.0,
                "message": {"header": {"stamp": {"secs": 99, "nsecs": 990_000_000}}},
            }
        )
        self.assertIn("消息时差: 10.0 ms", window._summary_rosbridge_status_label.text())

        window._on_ros_message(
            {
                "topic": "/active_imu",
                "recv_time_epoch_s": 101.1,
                "message": {"header": {"stamp": {"secs": 107, "nsecs": 0}}},
            }
        )

        self.assertIn("消息时差: 10.0 ms", window._summary_rosbridge_status_label.text())
        self.assertAlmostEqual(window._ros_timestamp_outliers["/active_imu"], -5900.0)

    def test_localization_config_fields_keep_readable_width(self) -> None:
        window = MainWindow()

        self.assertGreaterEqual(window._localization_panel._map_update_param_edit.minimumWidth(), 280)
        self.assertGreaterEqual(window._localization_panel._map_topic_edit.minimumWidth(), 280)
        self.assertGreaterEqual(window._localization_panel._local_map_path_edit.minimumWidth(), 280)
        self.assertGreaterEqual(window._localization_panel._map_update_param_edit.minimumHeight(), 24)

        window.resize(1920, 1030)
        window._switch_module("localization")
        window.show()
        self.app.processEvents()

        first = window._localization_panel._map_update_param_edit.geometry()
        second = window._localization_panel._map_topic_edit.geometry()
        third = window._localization_panel._local_map_path_edit.geometry()
        self.assertGreaterEqual(first.height(), 20)
        self.assertLessEqual(first.bottom(), second.y())
        self.assertLessEqual(second.bottom(), third.y())
        config_group = window._localization_panel._map_update_param_edit.parent()
        while config_group is not None and not hasattr(config_group, "title"):
            config_group = config_group.parent()
        self.assertIsNotNone(config_group)
        self.assertGreaterEqual(config_group.minimumHeight(), 300)
        following_groups = [
            group
            for group in window._localization_panel.findChildren(QGroupBox)
            if group.parent() is config_group.parent()
            and group is not config_group
            and group.geometry().y() > config_group.geometry().y()
        ]
        self.assertTrue(following_groups)
        self.assertLessEqual(config_group.geometry().bottom(), following_groups[0].geometry().y())

    def test_localization_refresh_keeps_removed_stats_labels_safe(self) -> None:
        window = MainWindow()

        window._localization_panel._refresh_view()

        self.assertIn("lateral_error_current", window._localization_panel._labels)

    def test_summary_row_update_tolerates_unpatched_summary_labels(self) -> None:
        window = MainWindow()
        delattr(window, "_summary_source_rate_labels")

        window._update_summary_row("encoder", connected=True, frame_count=1, error_count=0)

        self.assertEqual(window._summary_rows["encoder"]["frame_label"].text(), "1")

    def test_localization_connection_state_and_launch_commands_use_shared_ros_worker(self) -> None:
        window = MainWindow()
        commands: list[str] = []
        window._ros_worker.publish_launch_manager_command = commands.append

        window._on_ros_connection_changed(True)
        window._localization_panel._fastlio_launch_start_btn.click()

        self.assertFalse(window._localization_panel._connect_btn.isEnabled())
        self.assertTrue(window._localization_panel._disconnect_btn.isEnabled())
        self.assertEqual(commands, ["start fastlio fast_lio mapping_c16.launch"])

    def test_localization_connect_subscribes_its_odometry_topic_on_shared_rosbridge(self) -> None:
        window = MainWindow()
        requests: list[tuple[str, int, list[str]]] = []
        window._ros_worker.open_bridge = lambda host, port, topics=None: requests.append((host, port, list(topics or [])))
        window._ros_panel._topic_checkboxes["/imu"].setChecked(False)
        window._localization_panel._topic_edit.setText("/Odometry")

        window._localization_panel._connect_btn.click()

        self.assertEqual(requests, [("192.168.0.14", 9090, ["/Odometry"])])


if __name__ == "__main__":
    unittest.main()
