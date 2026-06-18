import os
import sys
import unittest
from pathlib import Path

from PySide6.QtGui import QIcon
from PySide6.QtWidgets import QApplication

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from main import app_icon_path, load_app_icon


class AppIconTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_app_icon_path_points_to_png_asset(self) -> None:
        path = app_icon_path()

        self.assertEqual(path.name, "app_icon.png")
        self.assertEqual(path.parent.name, "assets")
        self.assertTrue(path.exists())

    def test_runtime_png_icon_loads(self) -> None:
        icon = load_app_icon()

        self.assertFalse(icon.isNull())

    def test_windows_ico_icon_loads_for_packaging(self) -> None:
        icon_path = Path(__file__).resolve().parents[1] / "src" / "assets" / "app_icon.ico"

        self.assertTrue(icon_path.exists())
        self.assertFalse(QIcon(str(icon_path)).isNull())


if __name__ == "__main__":
    unittest.main()
