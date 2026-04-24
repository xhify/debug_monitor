import os
import json
import shutil
import sys
import unittest
import uuid
from contextlib import contextmanager
from pathlib import Path

from PySide6.QtWidgets import QApplication

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from imu_protocol import ImuSample
from main_window import MainWindow
from protocol import DataFrame
from ros_bridge_worker import RosImuReading, RosSnapshot


TEST_TMP_ROOT = Path(__file__).resolve().parents[1] / ".test_tmp"


@contextmanager
def temp_dir():
    TEST_TMP_ROOT.mkdir(exist_ok=True)
    path = TEST_TMP_ROOT / f"main_window_{uuid.uuid4().hex}"
    path.mkdir()
    try:
        yield path
    finally:
        shutil.rmtree(path, ignore_errors=True)


def make_encoder_frame() -> DataFrame:
    return DataFrame(
        t_raw_A=1.0,
        t_raw_B=2.0,
        m_raw_A=3.0,
        m_raw_B=4.0,
        final_A=5.0,
        final_B=6.0,
        target_A=7.0,
        target_B=8.0,
        output_A=9,
        output_B=10,
        afc_output_A=11.0,
        afc_output_B=12.0,
    )


def make_imu_sample(sequence: int) -> ImuSample:
    return ImuSample(
        host_time=sequence / 1000.0,
        sequence=sequence,
        accel=(1.0, 2.0, 3.0),
        gyro=(4.0, 5.0, 6.0),
        euler=(7.0, 8.0, 9.0),
        sync_time=sequence * 1000,
    )


class FakeRadarClient:
    def __init__(self, fail_start: bool = False) -> None:
        self.fail_start = fail_start
        self.identify_calls = 0
        self.started: list[str] = []
        self.stopped = 0

    def identify(self) -> str:
        self.identify_calls += 1
        return "PHASELOCK,FakeRadar"

    def start_recording(self, timestamp: str) -> str:
        if self.fail_start:
            raise RuntimeError("radar unavailable")
        self.started.append(timestamp)
        return "2026_04_24_15_30_00.bin"

    def stop_recording(self) -> None:
        self.stopped += 1


class MainWindowReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_main_window_uses_shorter_default_height(self) -> None:
        window = MainWindow()
        screen = window.screen() or self.app.primaryScreen()
        available = screen.availableGeometry()

        self.assertLessEqual(window.size().width(), available.width())
        self.assertLessEqual(window.size().height(), available.height())

    def test_main_window_starts_centered_on_available_screen(self) -> None:
        window = MainWindow()
        screen = window.screen() or self.app.primaryScreen()
        available = screen.availableGeometry()
        geometry = window.frameGeometry()

        self.assertEqual(geometry.center().x(), available.center().x())
        self.assertEqual(geometry.center().y(), available.center().y())

    def test_shown_main_window_frame_fits_available_screen(self) -> None:
        window = MainWindow()
        window.show()
        self.app.processEvents()
        screen = window.screen() or self.app.primaryScreen()
        available = screen.availableGeometry()
        frame = window.frameGeometry()

        self.assertLessEqual(frame.width(), available.width())
        self.assertLessEqual(frame.height(), available.height())

    def test_monitor_panels_move_into_tabs_with_param_default(self) -> None:
        window = MainWindow()

        self.assertEqual(window._monitor_tabs.count(), 3)
        self.assertEqual(window._monitor_tabs.tabText(0), "固件参数")
        self.assertIs(window._monitor_tabs.currentWidget(), window._param_panel)
        self.assertIs(window._command_panel.parentWidget(), window._right_sidebar)

    def test_main_window_exposes_encoder_and_imu_module_switches(self) -> None:
        window = MainWindow()

        self.assertEqual(window._module_stack.currentWidget(), window._encoder_page)
        self.assertEqual(window._summary_module_btn.text(), "汇总")
        self.assertEqual(window._encoder_module_btn.text(), "编码器")
        self.assertEqual(window._imu_module_btn.text(), "IMU")
        self.assertEqual(window._ros_module_btn.text(), "ROS")
        self.assertEqual(window._ros_imu_module_btn.text(), "ROS IMU")

    def test_main_window_switches_to_imu_module(self) -> None:
        window = MainWindow()

        window._imu_module_btn.click()

        self.assertEqual(window._module_stack.currentWidget(), window._imu_panel)
        self.assertTrue(window._imu_module_btn.isChecked())
        self.assertFalse(window._encoder_module_btn.isChecked())

    def test_imu_module_has_two_independent_device_connections_and_shared_record_button(self) -> None:
        window = MainWindow()
        imu_panel = window._imu_panel

        self.assertEqual(set(imu_panel._devices.keys()), {"A", "B"})
        self.assertEqual(imu_panel._devices["A"].group.title(), "IMU A")
        self.assertEqual(imu_panel._devices["B"].group.title(), "IMU B")
        self.assertEqual(imu_panel._record_btn.text(), "开始记录")

    def test_summary_module_switches_to_summary_page(self) -> None:
        window = MainWindow()

        window._summary_module_btn.click()

        self.assertEqual(window._module_stack.currentWidget(), window._summary_page)
        self.assertTrue(window._summary_module_btn.isChecked())

    def test_ros_module_switches_to_ros_page(self) -> None:
        window = MainWindow()

        window._ros_module_btn.click()

        self.assertEqual(window._module_stack.currentWidget(), window._ros_panel)
        self.assertTrue(window._ros_module_btn.isChecked())

    def test_ros_imu_module_switches_to_ros_imu_page(self) -> None:
        window = MainWindow()

        window._ros_imu_module_btn.click()

        self.assertEqual(window._module_stack.currentWidget(), window._ros_imu_panel)
        self.assertTrue(window._ros_imu_module_btn.isChecked())

    def test_ros_snapshot_only_updates_visible_ros_page(self) -> None:
        window = MainWindow()
        ros_updates = 0
        ros_imu_updates = 0

        def count_ros(_snapshot) -> None:
            nonlocal ros_updates
            ros_updates += 1

        def count_ros_imu(_snapshot) -> None:
            nonlocal ros_imu_updates
            ros_imu_updates += 1

        window._ros_panel.update_snapshot = count_ros
        window._ros_imu_panel.update_snapshot = count_ros_imu
        snapshot = RosSnapshot(
            frame_count=1,
            imu=RosImuReading(frame_count=1),
            active_imu=RosImuReading(frame_count=1),
        )

        window._on_ros_snapshot(snapshot)
        self.assertEqual((ros_updates, ros_imu_updates), (0, 0))

        window._ros_imu_module_btn.click()
        window._on_ros_snapshot(snapshot)
        self.assertEqual((ros_updates, ros_imu_updates), (0, 1))

        window._ros_module_btn.click()
        window._on_ros_snapshot(snapshot)
        self.assertEqual((ros_updates, ros_imu_updates), (1, 1))

    def test_ros_panel_pid_control_signal_reaches_worker(self) -> None:
        window = MainWindow()
        commands: list[tuple[float, bool, bool]] = []
        window._ros_worker.publish_line_follow_control = (
            lambda linear_x, forward, backward: commands.append((linear_x, forward, backward))
        )
        window._ros_panel.set_connected(True)
        window._ros_panel._pid_linear_x_spin.setValue(0.2)

        window._ros_panel._pid_forward_btn.click()

        self.assertEqual(commands, [(0.2, True, False)])

    def test_summary_recording_writes_encoder_and_imu_files_to_one_directory(self) -> None:
        window = MainWindow()
        with temp_dir() as tmp:
            window._summary_note_edit.setPlainText("室内地面，低速直线，PWM 3000")
            session_dir = window._start_summary_recording(
                base_dir=tmp,
                timestamp="20260420_170000",
            )
            window._buffer.append(make_encoder_frame())
            window._imu_panel._on_sample("A", make_imu_sample(1))
            window._imu_panel._on_sample("B", make_imu_sample(2))

            window._stop_summary_recording(save=True)

            self.assertEqual(session_dir.name, "session_20260420_170000")
            self.assertTrue((session_dir / "encoder.csv").exists())
            self.assertTrue((session_dir / "imu_A.csv").exists())
            self.assertTrue((session_dir / "imu_B.csv").exists())
            self.assertTrue((session_dir / "imu_session.json").exists())
            self.assertTrue((session_dir / "imu_merged_aligned.csv").exists())
            self.assertTrue((session_dir / "session.json").exists())
            with (session_dir / "session.json").open("r", encoding="utf-8") as fh:
                metadata = json.load(fh)
            self.assertEqual(metadata["note"], "室内地面，低速直线，PWM 3000")
            self.assertEqual(metadata["files"]["encoder"], "encoder.csv")

    def test_summary_module_exposes_serial_and_ros_source_selectors(self) -> None:
        window = MainWindow()

        encoder_source = window._summary_rows["encoder"]["source_combo"]
        imu_source = window._summary_rows["imu_A"]["source_combo"]

        self.assertGreaterEqual(encoder_source.findData("serial"), 0)
        self.assertGreaterEqual(encoder_source.findData("ros_odom"), 0)
        self.assertGreaterEqual(imu_source.findData("serial"), 0)
        self.assertGreaterEqual(imu_source.findData("ros_imu"), 0)

    def test_summary_recording_can_use_ros_odom_and_received_ros_imu_sources(self) -> None:
        window = MainWindow()
        window._summary_rows["encoder"]["source_combo"].setCurrentIndex(
            window._summary_rows["encoder"]["source_combo"].findData("ros_odom")
        )
        window._summary_rows["imu_A"]["source_combo"].setCurrentIndex(
            window._summary_rows["imu_A"]["source_combo"].findData("ros_imu")
        )
        window._summary_rows["imu_B"]["source_combo"].setCurrentIndex(
            window._summary_rows["imu_B"]["source_combo"].findData("ros_imu")
        )

        with temp_dir() as tmp:
            session_dir = window._start_summary_recording(
                base_dir=tmp,
                timestamp="20260423_110000",
            )
            window._on_ros_snapshot(
                RosSnapshot(
                    frame_count=1,
                    last_topic="/odom",
                    linear_x=0.2,
                    angular_z=0.03,
                    pose_x=1.0,
                    pose_y=2.0,
                )
            )
            window._on_ros_snapshot(
                RosSnapshot(
                    frame_count=2,
                    last_topic="/imu",
                    imu=RosImuReading(frame_count=1, accel_z=9.8, yaw_deg=12.0),
                )
            )

            window._stop_summary_recording(save=True)

            self.assertTrue((session_dir / "ros_odom.csv").exists())
            self.assertTrue((session_dir / "ros_imu.csv").exists())
            self.assertTrue((session_dir / "ros_active_imu.csv").exists())
            self.assertTrue((session_dir / "ros_imu_merged_aligned.csv").exists())
            self.assertFalse((session_dir / "encoder.csv").exists())
            self.assertFalse((session_dir / "imu_A.csv").exists())
            with (session_dir / "session.json").open("r", encoding="utf-8") as fh:
                metadata = json.load(fh)
            self.assertEqual(metadata["devices"]["encoder"]["source"], "ros_odom")
            self.assertEqual(metadata["devices"]["encoder"]["topic"], "/odom")
            self.assertEqual(metadata["devices"]["imu_A"]["source"], "ros_imu")
            self.assertEqual(metadata["files"]["ros_odom"], "ros_odom.csv")
            self.assertEqual(metadata["files"]["ros_imu"], "ros_imu.csv")
            self.assertEqual(metadata["files"]["ros_active_imu"], "ros_active_imu.csv")
            self.assertEqual(metadata["files"]["ros_imu_aligned"], "ros_imu_merged_aligned.csv")

    def test_radar_sync_checkbox_is_enabled_only_after_successful_connection_test(self) -> None:
        window = MainWindow()
        radar = FakeRadarClient()
        window._radar_client = radar

        self.assertFalse(window._summary_radar_sync_cb.isEnabled())

        window._test_summary_radar_connection()

        self.assertEqual(radar.identify_calls, 1)
        self.assertTrue(window._summary_radar_sync_cb.isEnabled())
        self.assertIn("PHASELOCK", window._summary_radar_status_label.text())

    def test_summary_recording_starts_and_stops_radar_when_sync_is_checked(self) -> None:
        window = MainWindow()
        radar = FakeRadarClient()
        window._radar_client = radar
        window._test_summary_radar_connection()
        window._summary_radar_sync_cb.setChecked(True)

        with temp_dir() as tmp:
            window._start_summary_recording(base_dir=tmp, timestamp="20260424_153000")
            window._stop_summary_recording(save=True)

        self.assertEqual(radar.started, ["20260424_153000"])
        self.assertEqual(radar.stopped, 1)

    def test_summary_recording_does_not_touch_radar_when_sync_is_unchecked(self) -> None:
        window = MainWindow()
        radar = FakeRadarClient()
        window._radar_client = radar

        with temp_dir() as tmp:
            window._start_summary_recording(base_dir=tmp, timestamp="20260424_153000")
            window._stop_summary_recording(save=True)

        self.assertEqual(radar.started, [])
        self.assertEqual(radar.stopped, 0)

    def test_summary_recording_aborts_when_checked_radar_start_fails(self) -> None:
        window = MainWindow()
        radar = FakeRadarClient(fail_start=True)
        window._radar_client = radar
        window._summary_radar_sync_cb.setEnabled(True)
        window._summary_radar_sync_cb.setChecked(True)

        with temp_dir() as tmp:
            with self.assertRaises(RuntimeError):
                window._start_summary_recording(base_dir=tmp, timestamp="20260424_153000")

            self.assertFalse(any(tmp.iterdir()))
            self.assertFalse(window._is_summary_recording())
            self.assertFalse(window._buffer.recording)

    def test_recording_default_filename_includes_both_motor_pid_values(self) -> None:
        window = MainWindow()
        window._command_panel._a_kp_spin.setValue(4000)
        window._command_panel._a_ki_spin.setValue(175)
        window._command_panel._a_kd_spin.setValue(0)
        window._command_panel._b_kp_spin.setValue(4100)
        window._command_panel._b_ki_spin.setValue(200)
        window._command_panel._b_kd_spin.setValue(1)

        filename = window._recording_default_name("20260415_181442")

        self.assertEqual(
            filename,
            "debug_data_Akp4000_Aki175_Akd0_Bkp4100_Bki200_Bkd1_20260415_181442.csv",
        )

    def test_switching_to_replay_enables_replay_mode(self) -> None:
        window = MainWindow()
        window._set_replay_loaded_for_test(
            time_values=[0.0, 0.1],
            rows=[
                {
                    "final_a": 1.0,
                    "final_b": 2.0,
                    "target_a": 1.0,
                    "target_b": 2.0,
                    "output_a": 10,
                    "output_b": 20,
                },
                {
                    "final_a": 1.1,
                    "final_b": 2.1,
                    "target_a": 1.0,
                    "target_b": 2.0,
                    "output_a": 11,
                    "output_b": 21,
                },
            ],
        )
        window._set_data_mode_for_test("replay")

        self.assertEqual(window.current_data_mode(), "replay")

    def test_replay_refresh_keeps_plot_showing_first_10_seconds_before_cursor_reaches_10s(self) -> None:
        window = MainWindow()
        window._set_replay_loaded_for_test(
            time_values=[0.0, 2.0, 4.0, 6.0, 8.0, 10.0],
            rows=[
                {"final_a": 1.0, "final_b": 2.0, "target_a": 1.0, "target_b": 2.0, "output_a": 10, "output_b": 20},
                {"final_a": 1.1, "final_b": 2.1, "target_a": 1.0, "target_b": 2.0, "output_a": 11, "output_b": 21},
                {"final_a": 1.2, "final_b": 2.2, "target_a": 1.0, "target_b": 2.0, "output_a": 12, "output_b": 22},
                {"final_a": 1.3, "final_b": 2.3, "target_a": 1.0, "target_b": 2.0, "output_a": 13, "output_b": 23},
                {"final_a": 1.4, "final_b": 2.4, "target_a": 1.0, "target_b": 2.0, "output_a": 14, "output_b": 24},
                {"final_a": 1.5, "final_b": 2.5, "target_a": 1.0, "target_b": 2.0, "output_a": 15, "output_b": 25},
            ],
        )
        window._set_data_mode_for_test("replay")
        window._plot_panel._speed_plot.setXRange(0.0, 1.0, padding=0.0)
        window._replay_current_time = 2.0

        window._on_refresh()

        x_min, x_max = window._plot_panel._speed_plot.getPlotItem().viewRange()[0]
        self.assertAlmostEqual(x_min, 0.0, places=6)
        self.assertAlmostEqual(x_max, 10.0, places=6)

    def test_replay_refresh_keeps_plot_following_current_time_with_10_second_window(self) -> None:
        window = MainWindow()
        window._set_replay_loaded_for_test(
            time_values=[0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0],
            rows=[
                {"final_a": 1.0, "final_b": 2.0, "target_a": 1.0, "target_b": 2.0, "output_a": 10, "output_b": 20},
                {"final_a": 1.1, "final_b": 2.1, "target_a": 1.0, "target_b": 2.0, "output_a": 11, "output_b": 21},
                {"final_a": 1.2, "final_b": 2.2, "target_a": 1.0, "target_b": 2.0, "output_a": 12, "output_b": 22},
                {"final_a": 1.3, "final_b": 2.3, "target_a": 1.0, "target_b": 2.0, "output_a": 13, "output_b": 23},
                {"final_a": 1.4, "final_b": 2.4, "target_a": 1.0, "target_b": 2.0, "output_a": 14, "output_b": 24},
                {"final_a": 1.5, "final_b": 2.5, "target_a": 1.0, "target_b": 2.0, "output_a": 15, "output_b": 25},
                {"final_a": 1.6, "final_b": 2.6, "target_a": 1.0, "target_b": 2.0, "output_a": 16, "output_b": 26},
            ],
        )
        window._set_data_mode_for_test("replay")
        window._plot_panel._speed_plot.setXRange(0.0, 1.0, padding=0.0)
        window._replay_current_time = 12.0

        window._on_refresh()

        x_min, x_max = window._plot_panel._speed_plot.getPlotItem().viewRange()[0]
        self.assertAlmostEqual(x_min, 2.0, places=6)
        self.assertAlmostEqual(x_max, 12.0, places=6)
