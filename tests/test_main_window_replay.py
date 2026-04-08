import os
import sys
import unittest

from PySide6.QtWidgets import QApplication

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from main_window import MainWindow


class MainWindowReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_monitor_panels_move_into_tabs_with_param_default(self) -> None:
        window = MainWindow()

        self.assertEqual(window._monitor_tabs.count(), 3)
        self.assertEqual(window._monitor_tabs.tabText(0), "固件参数")
        self.assertIs(window._monitor_tabs.currentWidget(), window._param_panel)
        self.assertIs(window._command_panel.parentWidget(), window._right_sidebar)

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
