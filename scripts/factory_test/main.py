# -*- coding: utf-8 -*-
"""Executable entry point for the factory test tool."""

import sys
from pathlib import Path

from PyQt5 import QtWidgets

if __package__:
    from .main_window import FactoryTestWindow
else:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from factory_test.main_window import FactoryTestWindow


def main():
    app = QtWidgets.QApplication(sys.argv)
    win = FactoryTestWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
