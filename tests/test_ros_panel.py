import os
import csv
import shutil
import sys
import unittest
import uuid
from pathlib import Path

from PySide6.QtWidgets import QApplication

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from ros_bridge_worker import RosSnapshot
from widgets.ros_panel import RosPanel


class RosPanelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def tearDown(self) -> None:
        for widget in QApplication.topLevelWidgets():
            widget.close()
            widget.deleteLater()
        self.app.processEvents()

    def test_connect_button_emits_host_port_and_selected_data_topics(self) -> None:
        panel = RosPanel()
        requests: list[tuple[str, int, list[str]]] = []
        panel.connect_requested.connect(lambda host, port, topics: requests.append((host, port, list(topics))))
        panel._host_edit.setText("192.168.0.14")
        panel._port_spin.setValue(9090)
        panel._topic_checkboxes["/odom"].setChecked(True)
        panel._topic_checkboxes["/imu"].setChecked(True)

        panel._connect_btn.click()

        self.assertEqual(requests, [("192.168.0.14", 9090, ["/odom", "/imu"])])

    def test_data_topic_checkboxes_default_to_unchecked(self) -> None:
        panel = RosPanel()

        self.assertEqual(panel.selected_data_topics(), [])
        self.assertTrue(panel._topic_checkboxes)
        self.assertTrue(all(not checkbox.isChecked() for checkbox in panel._topic_checkboxes.values()))

    def test_data_topic_presets_select_expected_topics(self) -> None:
        panel = RosPanel()

        panel._preset_buttons["none"].click()
        self.assertEqual(panel.selected_data_topics(), [])

        panel._preset_buttons["basic_low_bandwidth"].click()
        self.assertEqual(panel.selected_data_topics(), ["/odom", "/imu"])

        panel._preset_buttons["full"].click()
        self.assertEqual(set(panel.selected_data_topics()), set(panel._topic_checkboxes))
        self.assertNotIn("/launch_manager/status", panel.selected_data_topics())

    def test_apply_data_subscriptions_emits_selected_topics(self) -> None:
        panel = RosPanel()
        requests: list[list[str]] = []
        panel.data_subscriptions_changed.connect(lambda topics: requests.append(list(topics)))
        panel._topic_checkboxes["/PowerVoltage"].setChecked(True)
        panel._topic_checkboxes["/wheeltec/control_debug"].setChecked(True)

        panel._apply_subscriptions_btn.click()

        self.assertEqual(requests, [["/PowerVoltage", "/wheeltec/control_debug"]])

    def test_cmd_vel_buttons_emit_velocity_commands(self) -> None:
        panel = RosPanel()
        commands: list[tuple[float, float]] = []
        panel.cmd_vel_requested.connect(lambda linear_x, angular_z: commands.append((linear_x, angular_z)))
        panel.set_connected(True)
        panel._linear_x_spin.setValue(0.25)
        panel._angular_z_spin.setValue(-0.5)

        panel._send_cmd_vel_btn.click()
        panel._stop_cmd_vel_btn.click()

        self.assertEqual(commands[0], (0.25, -0.5))
        self.assertEqual(commands[1], (0.0, 0.0))

    def test_pid_control_buttons_emit_line_follow_commands(self) -> None:
        panel = RosPanel()
        commands: list[tuple[float, bool, bool]] = []
        panel.pid_control_requested.connect(
            lambda linear_x, forward, backward: commands.append((linear_x, forward, backward))
        )
        panel.set_connected(True)
        panel._pid_linear_x_spin.setValue(0.2)

        panel._pid_forward_btn.click()
        panel._pid_backward_btn.click()
        panel._pid_stop_btn.click()

        self.assertEqual(commands[0], (0.2, True, False))
        self.assertEqual(commands[1], (0.2, False, True))
        self.assertEqual(commands[2], (0.0, False, False))

    def test_pid_launch_buttons_emit_launch_manager_commands(self) -> None:
        panel = RosPanel()
        commands: list[str] = []
        panel.launch_manager_command_requested.connect(commands.append)
        panel.set_connected(True)

        panel._pid_launch_start_btn.click()
        panel._pid_launch_stop_btn.click()

        self.assertEqual(
            commands,
            [
                "start pid_control simple_follower pid_control.launch",
                "stop pid_control",
            ],
        )

    def test_radar_calibration_launch_buttons_emit_launch_manager_commands(self) -> None:
        panel = RosPanel()
        commands: list[str] = []
        panel.launch_manager_command_requested.connect(commands.append)
        panel.set_connected(True)

        panel._radar_calibration_launch_start_btn.click()
        panel._radar_calibration_launch_stop_btn.click()

        self.assertEqual(
            commands,
            [
                "restart pid_control simple_follower pid_control_lidar_assisted.launch "
                "imu_topic:=/active_imu lidar_odom_topic:=/Odometry",
                "stop pid_control",
            ],
        )

    def test_pid_and_radar_calibration_launch_buttons_are_mutually_guarded(self) -> None:
        panel = RosPanel()
        panel.set_connected(True)

        panel._pid_launch_start_btn.click()

        self.assertTrue(panel._pid_launch_start_btn.isEnabled())
        self.assertFalse(panel._radar_calibration_launch_start_btn.isEnabled())

        panel._pid_launch_stop_btn.click()

        self.assertTrue(panel._pid_launch_start_btn.isEnabled())
        self.assertTrue(panel._radar_calibration_launch_start_btn.isEnabled())

        panel._radar_calibration_launch_start_btn.click()

        self.assertFalse(panel._pid_launch_start_btn.isEnabled())
        self.assertTrue(panel._radar_calibration_launch_start_btn.isEnabled())

    def test_disconnected_state_disables_all_launch_buttons(self) -> None:
        panel = RosPanel()

        panel.set_connected(False)

        self.assertFalse(panel._pid_launch_start_btn.isEnabled())
        self.assertFalse(panel._pid_launch_stop_btn.isEnabled())
        self.assertFalse(panel._radar_calibration_launch_start_btn.isEnabled())
        self.assertFalse(panel._radar_calibration_launch_stop_btn.isEnabled())

    def test_update_snapshot_refreshes_ros_values(self) -> None:
        panel = RosPanel()

        panel.update_snapshot(
            RosSnapshot(
                frame_count=12,
                linear_x=0.3,
                linear_y=0.1,
                angular_z=-0.2,
                pose_x=1.2,
                pose_y=3.4,
                accel_z=9.81,
                gyro_z=0.04,
                voltage=25.8,
            )
        )
        panel._flush_pending_snapshot()

        self.assertEqual(panel._labels["frame_count"].text(), "12")
        self.assertEqual(panel._field_labels["linear_x"].text(), "电机A左轮速度")
        self.assertEqual(panel._field_labels["linear_y"].text(), "电机B右轮速度")
        self.assertEqual(panel._labels["linear_x"].text(), "0.3000")
        self.assertEqual(panel._labels["linear_y"].text(), "0.1000")
        self.assertEqual(panel._labels["angular_z"].text(), "-0.2000")
        self.assertEqual(panel._labels["pose_x"].text(), "1.2000")
        self.assertEqual(panel._labels["accel_z"].text(), "9.8100")
        self.assertEqual(panel._labels["gyro_z"].text(), "0.0400")
        self.assertEqual(panel._labels["voltage"].text(), "25.80")
        self.assertEqual(panel._speed_labels["actual_left"].text(), "0.3000")
        self.assertEqual(panel._speed_labels["actual_right"].text(), "0.1000")

    def test_speed_monitor_tracks_last_commanded_targets(self) -> None:
        panel = RosPanel()
        panel.set_connected(True)
        panel._linear_x_spin.setValue(0.25)

        panel._send_cmd_vel_btn.click()

        self.assertEqual(panel._speed_labels["target_left"].text(), "0.2500")
        self.assertEqual(panel._speed_labels["target_right"].text(), "0.2500")

        panel._pid_linear_x_spin.setValue(0.2)
        panel._pid_backward_btn.click()

        self.assertEqual(panel._speed_labels["target_left"].text(), "-0.2000")
        self.assertEqual(panel._speed_labels["target_right"].text(), "-0.2000")

        panel._pid_stop_btn.click()

        self.assertEqual(panel._speed_labels["target_left"].text(), "0.0000")
        self.assertEqual(panel._speed_labels["target_right"].text(), "0.0000")

    def test_ros_panel_adds_speed_plot_and_keeps_control_groups(self) -> None:
        panel = RosPanel()

        self.assertTrue(hasattr(panel, "_speed_plot"))
        self.assertIn("actual_left", panel._speed_curves)
        self.assertIn("target_right", panel._speed_curves)
        self.assertEqual(panel._data_group.title(), "ROS 数据")
        self.assertEqual(panel._cmd_group.title(), "/cmd_vel")
        self.assertEqual(panel._pid_group.title(), "PID 直行控制 /line_follow_control")

    def test_speed_plot_tracks_actual_and_target_history(self) -> None:
        panel = RosPanel()
        original_append = panel._buffer.append
        timestamps = iter([10.0, 10.1])

        def append_with_timestamp(snapshot: RosSnapshot) -> float:
            return original_append(snapshot, timestamp=next(timestamps))

        panel._buffer.append = append_with_timestamp
        panel.set_connected(True)
        panel._linear_x_spin.setValue(0.25)

        panel._send_cmd_vel_btn.click()
        panel.update_snapshot(RosSnapshot(frame_count=1, linear_x=0.10, linear_y=0.12))
        panel.update_snapshot(RosSnapshot(frame_count=2, linear_x=0.20, linear_y=0.22))
        panel._flush_pending_snapshot()

        _x, actual_left = panel._speed_curves["actual_left"].getData()
        _x, actual_right = panel._speed_curves["actual_right"].getData()
        _x, target_left = panel._speed_curves["target_left"].getData()
        _x, target_right = panel._speed_curves["target_right"].getData()

        self.assertEqual(actual_left.tolist(), [0.10, 0.20])
        self.assertEqual(actual_right.tolist(), [0.12, 0.22])
        self.assertEqual(target_left.tolist(), [0.25, 0.25])
        self.assertEqual(target_right.tolist(), [0.25, 0.25])

    def test_high_rate_updates_are_coalesced_before_value_refresh(self) -> None:
        panel = RosPanel()

        for frame_count in range(1, 21):
            panel.update_snapshot(
                RosSnapshot(
                    frame_count=frame_count,
                    linear_x=frame_count * 0.01,
                    voltage=24.0,
                )
            )

        self.assertEqual(panel._labels["frame_count"].text(), "---")

        panel._flush_pending_snapshot()

        self.assertEqual(panel._labels["frame_count"].text(), "20")
        self.assertEqual(panel._labels["linear_x"].text(), "0.2000")

    def test_recording_writes_received_snapshots_to_csv(self) -> None:
        panel = RosPanel()
        base_dir = Path(__file__).resolve().parents[1] / ".test_tmp" / f"ros_panel_{uuid.uuid4().hex}"
        base_dir.mkdir(parents=True)
        try:
            final_path = base_dir / "ros.csv"
            panel.start_recording_for_test(base_dir=base_dir)

            panel.update_snapshot(RosSnapshot(frame_count=7, linear_x=0.7, voltage=26.0))
            panel.stop_recording_for_test(final_path=final_path)

            self.assertTrue(final_path.exists())
            with final_path.open("r", encoding="utf-8", newline="") as fh:
                rows = list(csv.reader(fh))
            self.assertEqual(rows[1][1], "7")
            self.assertEqual(rows[1][2], "0.7")
            self.assertEqual(rows[1][-1], "26.0")
        finally:
            shutil.rmtree(base_dir, ignore_errors=True)

    def test_shutdown_cancels_active_recording(self) -> None:
        panel = RosPanel()
        base_dir = Path(__file__).resolve().parents[1] / ".test_tmp" / f"ros_panel_{uuid.uuid4().hex}"
        base_dir.mkdir(parents=True)
        try:
            panel.start_recording_for_test(base_dir=base_dir)
            temp_path = panel._recording_session.temp_path

            panel.shutdown()

            self.assertIsNone(panel._recording_session)
            self.assertFalse(temp_path.exists())
        finally:
            shutil.rmtree(base_dir, ignore_errors=True)

    def test_connected_state_disables_connection_inputs(self) -> None:
        panel = RosPanel()

        panel.set_connected(True)

        self.assertFalse(panel._connect_btn.isEnabled())
        self.assertTrue(panel._disconnect_btn.isEnabled())
        self.assertFalse(panel._host_edit.isEnabled())
        self.assertTrue(panel._pid_forward_btn.isEnabled())
        self.assertTrue(panel._pid_backward_btn.isEnabled())
        self.assertTrue(panel._pid_stop_btn.isEnabled())
        self.assertEqual(panel._status_label.text(), "已连接")


if __name__ == "__main__":
    unittest.main()
