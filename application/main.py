"""
File: main.py
Purpose: Main application entry point
Description: Launches the magnetic compass calibration system
"""

import sys
from compass_app import CalibrationApp

if __name__ == "__main__":
    app = CalibrationApp()
    app.run()