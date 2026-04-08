import os
import sys
import unittest

from PySide6.QtWidgets import QApplication

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from widgets.analysis_panel import AnalysisPanel


class AnalysisPanelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_update_metrics_renders_values(self) -> None:
        panel = AnalysisPanel()
        panel.update_metrics(
            mode_label="最近 10 秒",
            metrics_a={"mean": 1.23, "std": 0.1},
            metrics_b={"mean": 2.34, "std": 0.2},
        )

        self.assertIn("1.230", panel.metric_text("mean", "A"))
        self.assertIn("2.340", panel.metric_text("mean", "B"))
