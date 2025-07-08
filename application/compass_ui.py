# compass_ui.py
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox
from PyQt5.QtCore import QTimer
import time
from serial.tools import list_ports
from compass_app import CompassApp
import config

class CompassUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Compass Viewer with Control Panel")
        self.port_combo = QComboBox()
        self.baud_combo = QComboBox()
        self.start_button = QPushButton("Calibration")  # 修改按钮名字为 Calibration
        self.stop_button = QPushButton("Stop")
        self.stop_button.setEnabled(False)
        self.cleanup_timer = None
        self.app_instance = None

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
        port = self.port_combo.currentText()
        baud_rate = int(self.baud_combo.currentText())

        if self.cleanup_timer and self.cleanup_timer.isActive():
            self.cleanup_timer.stop()

        if self.app_instance is not None:
            del self.app_instance

        self.app_instance = CompassApp()
        self.app_instance.port = port
        self.app_instance.baud_rate = baud_rate
        self.app_instance.connect_serial()

        self.app_instance.data_collection_started = True
        self.app_instance.start_time = time.time()

        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)

        self.cleanup_timer = QTimer(self)
        self.cleanup_timer.setSingleShot(True)
        self.cleanup_timer.timeout.connect(self.stop_plotting)
        self.cleanup_timer.start(config.CALIBRATION_DURATION * 1000)

    def stop_plotting(self):
        if self.app_instance is not None:
            self.app_instance.stop_serial()
            self.app_instance.calibration_done = True
            self.app_instance.calibrate_data()
            print("[INFO] 串口已暂停")

        self.stop_button.setEnabled(False)
        self.start_button.setEnabled(True)
        print("[INFO] Buttons updated: Stop disabled, Start enabled immediately.")

        if self.cleanup_timer and self.cleanup_timer.isActive():
            self.cleanup_timer.stop()

    def clear_app_instance(self):
        if self.app_instance is not None:
            del self.app_instance
            print("[INFO] 数据实例已释放")

        self.stop_button.setEnabled(False)
        self.start_button.setEnabled(True)
        print("[INFO] Buttons updated: Stop disabled, Start enabled after timer.")