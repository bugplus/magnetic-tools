# compass_ui.py

import sys
import time
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox
from PyQt5.QtCore import QTimer
from serial.tools import list_ports

class CompassUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Compass Viewer with Control Panel")

        self.port_combo = QComboBox()
        self.baud_combo = QComboBox()
        self.start_button = QPushButton("Start")
        self.stop_button = QPushButton("Stop")
        self.stop_button.setEnabled(False)
        self.cleanup_timer = None

        self.initUI()
        self.refresh_ports()

    def initUI(self):
        layout = QVBoxLayout()

        port_layout = QHBoxLayout()
        port_layout.addWidget(QLabel("Serial Port:"))
        port_layout.addWidget(self.port_combo)

        baud_layout = QHBoxLayout()
        baud_layout.addWidget(QLabel("Baud Rate:"))
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.baud_combo.setCurrentText("115200")
        baud_layout.addWidget(self.baud_combo)

        button_layout = QHBoxLayout()
        button_layout.addWidget(self.start_button)
        button_layout.addWidget(self.stop_button)

        self.start_button.clicked.connect(self.start_plotting)
        self.stop_button.clicked.connect(self.stop_plotting)

        layout.addLayout(port_layout)
        layout.addLayout(baud_layout)
        layout.addLayout(button_layout)
        self.setLayout(layout)

    def refresh_ports(self):
        self.port_combo.clear()
        ports = list_ports.comports()
        for port in ports:
            self.port_combo.addItem(port.device)

    def start_plotting(self):
        from compass_core import CompassCore
        from compass_plot import CompassPlot

        port = self.port_combo.currentText()
        baud_rate = int(self.baud_combo.currentText())

        if self.cleanup_timer and self.cleanup_timer.isActive():
            self.cleanup_timer.stop()

        if hasattr(self, 'core'):
            del self.core

        self.core = CompassCore()
        self.core.connect_serial(port, baud_rate)

        if not hasattr(self, 'plotter'):
            self.plotter = CompassPlot(self.core)

        self.core.data_collection_started = True
        self.core.start_time = time.time()

        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)

    def stop_plotting(self):
        if hasattr(self, 'core'):
            self.core.data_collection_started = False
            self.core.disconnect_serial()

        if self.cleanup_timer and self.cleanup_timer.isActive():
            self.cleanup_timer.stop()

        # 立即清理 core 和 plotter，不需要等待 50 秒
        self.clear_core()

        self.stop_button.setEnabled(False)
        self.start_button.setEnabled(True)

    def clear_core(self):
        if hasattr(self, 'plotter'):
            # 清理 matplotlib figure 防止窗口卡住
            self.plotter.fig = None
            del self.plotter

        if hasattr(self, 'core'):
            del self.core