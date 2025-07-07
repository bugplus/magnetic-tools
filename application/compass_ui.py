# 文件名: compass_ui.py
# 说明: 包含 CompassUI 类，负责创建用户界面和控制程序的启动/停止。

import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox
from PyQt5.QtCore import QTimer
from compass_app import CompassApp
from serial.tools import list_ports
import time
from config import CALIBRATION_DURATION  # 从 config.py 导入 CALIBRATION_DURATION

class CompassUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Compass Viewer with Control Panel")
        self.port_combo = QComboBox()
        self.baud_combo = QComboBox()
        self.start_button = QPushButton("Start")
        self.stop_button = QPushButton("Stop")
        self.stop_button.setEnabled(False)  # 初始化时禁用Stop按钮
        self.cleanup_timer = QTimer(self)
        self.app_instance = None

        self.initUI()
        self.refresh_ports()

    def initUI(self):
        layout = QVBoxLayout()

        # Serial Port Selection
        port_layout = QHBoxLayout()
        port_layout.addWidget(QLabel("Serial Port:"))
        port_layout.addWidget(self.port_combo)

        # Baud Rate Selection
        baud_layout = QHBoxLayout()
        baud_layout.addWidget(QLabel("Baud Rate:"))
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.baud_combo.setCurrentText("115200")
        baud_layout.addWidget(self.baud_combo)

        # Buttons
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.start_button)
        button_layout.addWidget(self.stop_button)

        layout.addLayout(port_layout)
        layout.addLayout(baud_layout)
        layout.addLayout(button_layout)
        self.setLayout(layout)

        self.start_button.clicked.connect(self.start_plotting)
        self.stop_button.clicked.connect(self.stop_plotting)

    def refresh_ports(self):
        self.port_combo.clear()
        ports = list_ports.comports()
        for port in ports:
            self.port_combo.addItem(port.device)

    def start_plotting(self):
        port = self.port_combo.currentText()
        baud_rate = int(self.baud_combo.currentText())

        if self.cleanup_timer.isActive():
            self.cleanup_timer.stop()

        if self.app_instance:
            self.app_instance.stop_serial()
            del self.app_instance

        self.app_instance = CompassApp(port, baud_rate, self)  # 传递UI实例
        self.app_instance.data_collection_started = True
        self.app_instance.start_time = time.time()

        self.start_button.setEnabled(False)  # 禁用Start按钮
        self.stop_button.setEnabled(True)   # 启用Stop按钮

        self.cleanup_timer.timeout.connect(self.on_timeout)  # 连接计时器到超时处理方法
        self.cleanup_timer.start(CALIBRATION_DURATION * 1000)  # 设置计时器时间

    def stop_plotting(self):
        if self.app_instance:
            self.app_instance.stop_data_collection()
            self.app_instance.stop_serial()
            self.app_instance.calibration_done = True
            self.app_instance.calibrate_data()

        self.stop_button.setEnabled(False)  # 禁用Stop按钮
        self.start_button.setEnabled(True)  # 启用Start按钮

        if self.cleanup_timer.isActive():
            self.cleanup_timer.stop()

    def on_timeout(self):
        if self.app_instance:
            self.app_instance.stop_data_collection()
        self.stop_plotting()

    def update_buttons(self):
        self.stop_button.setEnabled(False)
        self.start_button.setEnabled(True)