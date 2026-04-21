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
