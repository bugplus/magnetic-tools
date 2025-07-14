"""
File: main.py
Purpose: Main application entry point
Description: Launches the magnetic compass calibration system
"""

import sys
from PyQt5.QtWidgets import QApplication
from compass_ui import CompassUI

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = CompassUI()
    ex.show()
    sys.exit(app.exec_())