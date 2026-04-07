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
