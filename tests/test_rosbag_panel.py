import os
import sys
import unittest

from PySide6.QtWidgets import QApplication

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from rosbag_models import RemoteRosbagSession, RosbagRecordingStatus
from widgets.rosbag_panel import RosbagPanel


class RosbagPanelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_current_config_uses_fastlio_defaults_and_emits_start(self) -> None:
        panel = RosbagPanel()
        starts = []
        panel.start_requested.connect(starts.append)

        panel._start_btn.click()

        self.assertRegex(starts[0]["session_id"], r"^session_\d{8}_\d{6}$")
        self.assertEqual(starts[0]["prefix"], "fastlio")
        self.assertIn("/point_cloud_filtered", starts[0]["topics"])
        self.assertNotIn("/point_cloud_raw", starts[0]["topics"])
        self.assertEqual(starts[0]["split_size_mb"], 2048)
        self.assertEqual(starts[0]["compression"], "lz4")

    def test_mode_combo_shows_chinese_descriptions_but_keeps_mode_keys(self) -> None:
        panel = RosbagPanel()

        fastlio_index = panel._mode_combo.findData("fastlio")
        fallback_index = panel._mode_combo.findData("fallback_no_fastlio")
        custom_index = panel._mode_combo.findData("custom")

        self.assertGreaterEqual(fastlio_index, 0)
        self.assertGreaterEqual(fallback_index, 0)
        self.assertGreaterEqual(custom_index, 0)
        self.assertEqual(panel._mode_combo.itemData(fastlio_index), "fastlio")
        self.assertEqual(panel._mode_combo.itemText(fastlio_index), "fastlio - FAST-LIO 定位常用")
        self.assertIn("无 FAST-LIO 备用记录", panel._mode_combo.itemText(fallback_index))
        self.assertEqual(panel._mode_combo.itemText(custom_index), "custom - 手动选择 topic")

        panel._mode_combo.setCurrentIndex(fastlio_index)
        config = panel.current_config()

        self.assertEqual(panel._prefix_edit.text(), "fastlio")
        self.assertEqual(config["prefix"], "fastlio")
        self.assertIn("/point_cloud_filtered", config["topics"])

    def test_stop_button_does_not_emit_without_session_id(self) -> None:
        panel = RosbagPanel()
        stops = []
        panel.stop_requested.connect(stops.append)

        panel._stop_btn.click()

        self.assertEqual(stops, [])

    def test_recording_status_controls_start_and_stop_buttons(self) -> None:
        panel = RosbagPanel()

        self.assertTrue(panel._start_btn.isEnabled())
        self.assertFalse(panel._stop_btn.isEnabled())

        panel.update_recording_status(RosbagRecordingStatus(active=True, session_id="session_running"))
        self.assertFalse(panel._start_btn.isEnabled())
        self.assertTrue(panel._stop_btn.isEnabled())

        panel.update_recording_status(RosbagRecordingStatus(active=False, session_id="session_done"))
        self.assertTrue(panel._start_btn.isEnabled())
        self.assertFalse(panel._stop_btn.isEnabled())

    def test_trajectory_environment_preset_records_motion_and_environment_topics(self) -> None:
        panel = RosbagPanel()

        index = panel._mode_combo.findData("trajectory_environment")
        self.assertGreaterEqual(index, 0)
        panel._mode_combo.setCurrentIndex(index)
        config = panel.current_config()

        self.assertEqual(config["prefix"], "trajectory_environment")
        self.assertEqual(
            config["topics"],
            [
                "/point_cloud_filtered",
                "/Laser_map",
                "/imu",
                "/active_imu",
                "/Odometry",
                "/odom",
                "/path",
                "/tf",
                "/tf_static",
                "/cmd_vel",
                "/wheeltec/akm_state",
                "/PowerVoltage",
            ],
        )

    def test_no_fastlio_fallback_preset_excludes_fastlio_outputs(self) -> None:
        panel = RosbagPanel()

        index = panel._mode_combo.findData("fallback_no_fastlio")
        self.assertGreaterEqual(index, 0)
        panel._mode_combo.setCurrentIndex(index)
        config = panel.current_config()

        self.assertEqual(config["prefix"], "fallback_no_fastlio")
        self.assertEqual(
            config["topics"],
            [
                "/point_cloud_filtered",
                "/imu",
                "/active_imu",
                "/odom",
                "/tf",
                "/tf_static",
                "/cmd_vel",
                "/line_follow_control",
                "/PowerVoltage",
                "/wheeltec/akm_state",
                "/wheeltec/control_debug",
                "/wheeltec/chassis_diagnostics",
            ],
        )
        self.assertNotIn("/Odometry", config["topics"])
        self.assertNotIn("/path", config["topics"])
        self.assertNotIn("/Laser_map", config["topics"])

    def test_topic_editor_uses_multicolumn_checkboxes_and_counts_topics(self) -> None:
        panel = RosbagPanel()

        self.assertIn("/point_cloud_filtered", panel._topic_checks)
        self.assertNotIn("/point_cloud_raw", panel._topic_checks)
        self.assertIn("/Laser_map", panel._topic_checks)
        self.assertTrue(panel._topic_checks["/point_cloud_filtered"].isChecked())
        self.assertIn("7 个 topic", panel._topic_count_label.text())

    def test_topic_checkboxes_control_recorded_topics_and_custom_topics_append(self) -> None:
        panel = RosbagPanel()

        panel._topic_checks["/point_cloud_filtered"].setChecked(False)
        panel._custom_topics_edit.setText("/custom_a, /custom_b\n/custom_c")

        topics = panel.current_config()["topics"]

        self.assertNotIn("/point_cloud_raw", topics)
        self.assertIn("/imu", topics)
        self.assertEqual(topics[-3:], ["/custom_a", "/custom_b", "/custom_c"])
        self.assertIn("9 个 topic", panel._topic_count_label.text())

    def test_delete_guard_rejects_active_sessions_and_allows_stopped_without_confirm_text(self) -> None:
        panel = RosbagPanel()
        deletes = []
        panel.delete_requested.connect(lambda session_id, confirm: deletes.append((session_id, confirm)))

        active = RemoteRosbagSession(session_id="session_2", status="recording")
        panel.update_recording_status(RosbagRecordingStatus(active=True, session_id="session_2"))
        self.assertFalse(panel.request_delete_for_test(active))
        self.assertEqual(deletes, [])

        panel.update_recording_status(RosbagRecordingStatus(active=False, session_id=""))
        recording = RemoteRosbagSession(session_id="session_3", status="recording")
        self.assertFalse(panel.request_delete_for_test(recording))
        self.assertEqual(deletes, [])

        stopped = RemoteRosbagSession(session_id="session_1", status="stopped")
        self.assertTrue(panel.request_delete_for_test(stopped))
        self.assertEqual(deletes, [("session_1", "session_1")])

    def test_delete_confirmation_message_shows_session_details_and_risk(self) -> None:
        panel = RosbagPanel()
        session = RemoteRosbagSession(
            session_id="session_1",
            status="stopped",
            remote_dir="/home/wheeltec/bags",
            size_bytes=1024,
        )

        message = panel._delete_confirmation_message(session)

        self.assertIn("session_id: session_1", message)
        self.assertIn("状态: stopped", message)
        self.assertIn("大小: 1.0 KB", message)
        self.assertIn("远程目录: /home/wheeltec/bags", message)
        self.assertIn("永久删除不可恢复", message)
        self.assertIn("未同步的数据会丢失", message)


if __name__ == "__main__":
    unittest.main()
