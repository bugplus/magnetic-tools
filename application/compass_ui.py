# compass_ui.py
# 说明: 包含 CompassUI 类，负责创建用户界面和控制程序的启动/停止。

import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox
from PyQt5.QtCore import QTimer
from compass_app import CompassApp
from serial.tools import list_ports
import time
from config import CALIBRATION_DURATION, BAUD_RATE_OPTIONS, DEFAULT_BAUD_RATE

class CompassUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Compass Viewer with Control Panel")
        self.port_combo = QComboBox()
        self.baud_combo = QComboBox()
        self.calibration_button = QPushButton("Calibration")
        self.run_button = QPushButton("Run")
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
        self.baud_combo.addItems(BAUD_RATE_OPTIONS)
        self.baud_combo.setCurrentText(DEFAULT_BAUD_RATE)
        baud_layout.addWidget(self.baud_combo)

        # Buttons
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.calibration_button)
        button_layout.addWidget(self.run_button)
        button_layout.addWidget(self.stop_button)

        layout.addLayout(port_layout)
        layout.addLayout(baud_layout)
        layout.addLayout(button_layout)
        self.setLayout(layout)

        self.calibration_button.clicked.connect(self.start_calibration)
        self.run_button.clicked.connect(self.run_calibration)
        self.stop_button.clicked.connect(self.stop_calibration)

    def refresh_ports(self):
        self.port_combo.clear()
        ports = list_ports.comports()
        for port in ports:
            self.port_combo.addItem(port.device)

    def start_calibration(self):
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

        self.calibration_button.setEnabled(False)  # 禁用Calibration按钮
        self.run_button.setEnabled(True)   # 启用Run按钮
        self.stop_button.setEnabled(True)   # 启用Stop按钮

        self.cleanup_timer.timeout.connect(self.on_timeout)  # 连接计时器到超时处理方法
        self.cleanup_timer.start(CALIBRATION_DURATION * 1000)  # 设置计时器时间

    def run_calibration(self):
        if self.app_instance:
            new_data = self.app_instance.get_new_data()  # 获取新数据
            calibrated_data = self.app_instance.process_new_data(new_data)  # 处理新数据
            self.app_instance.plot_data(new_data, calibrated_data)  # 绘制数据

        self.run_button.setEnabled(False)
        self.calibration_button.setEnabled(True)
        self.stop_button.setEnabled(True)

    def stop_calibration(self):
        if self.app_instance:
            self.app_instance.stop_serial()
            self.app_instance.calibration_done = True
            self.app_instance.calibrate_data()

        self.stop_button.setEnabled(False)
        self.calibration_button.setEnabled(True)

    def on_timeout(self):
        if self.app_instance:
            self.app_instance.stop_data_collection()
        self.stop_plotting()

    def stop_plotting(self):
        if self.app_instance:
            self.app_instance.stop_serial()
        self.run_button.setEnabled(False)
        self.calibration_button.setEnabled(True)
        self.stop_button.setEnabled(False)

        if self.cleanup_timer.isActive():
            self.cleanup_timer.stop()

    def update_buttons(self):
        self.stop_button.setEnabled(False)
        self.start_button.setEnabled(True)