"""
WHEELTEC C50X 调试监视器 — 入口点

用法：
    python main.py
"""

import sys
from PySide6.QtWidgets import QApplication
from main_window import MainWindow


def main() -> None:
    app = QApplication(sys.argv)
    app.setApplicationName("WHEELTEC C50X Debug Monitor")
    app.setStyle('Fusion')

    window = MainWindow()
    window.show()

    sys.exit(app.exec())


if __name__ == '__main__':
    main()
