# compass_ui.py
# compass_ui.py
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox, QMessageBox
from PyQt5.QtCore import QTimer
import time
from serial.tools import list_ports
from compass_app import CompassApp
import config
import numpy as np
import matplotlib.pyplot as plt

class CompassUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Compass Viewer with Control Panel")
        self.port_combo = QComboBox()
        self.baud_combo = QComboBox()
        self.calibration_button = QPushButton("Calibration")
        self.run_button = QPushButton("Run")
        self.stop_button = QPushButton("Stop")
        self.stop_button.setEnabled(False)
        self.app_instance = None
        self.is_calibrated = False

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
        button_layout.addWidget(self.calibration_button)
        button_layout.addWidget(self.run_button)
        button_layout.addWidget(self.stop_button)

        self.calibration_button.clicked.connect(self.start_calibration)
        self.run_button.clicked.connect(self.start_running)
        self.stop_button.clicked.connect(self.stop_operation)

        layout.addLayout(port_layout)
        layout.addLayout(baud_layout)
        layout.addLayout(button_layout)
        self.setLayout(layout)

    def refresh_ports(self):
        self.port_combo.clear()
        ports = list_ports.comports()
        for port in ports:
            self.port_combo.addItem(port.device)

    def start_calibration(self):
        if self.app_instance is not None:
            self.stop_operation()
            time.sleep(0.5)
            self.app_instance = None

        port = self.port_combo.currentText()
        baud_rate = int(self.baud_combo.currentText())

        self.app_instance = CompassApp(port, baud_rate)
        self.app_instance.connect_serial()

        self.app_instance.data_collection_started = True
        self.app_instance.start_time = time.time()

        self.calibration_button.setEnabled(False)
        self.run_button.setEnabled(False)
        self.stop_button.setEnabled(True)

        # 启动校准定时器
        self.calibration_timer = QTimer(self)
        self.calibration_timer.setSingleShot(True)
        self.calibration_timer.timeout.connect(self.complete_calibration)
        self.calibration_timer.start(config.CALIBRATION_DURATION * 1000)

    def complete_calibration(self):
        if self.app_instance is not None:
            self.app_instance.calibrate_data()
            self.is_calibrated = True
            self.calibration_button.setEnabled(True)
            self.run_button.setEnabled(True)
            self.stop_button.setEnabled(False)
            QMessageBox.information(self, "Calibration Complete", "Calibration is complete. You can now run the algorithm.")
        else:
            QMessageBox.warning(self, "Calibration Failed", "Calibration failed. Please try again.")

    def start_running(self):
        if not self.is_calibrated:
            QMessageBox.warning(self, "Warning", "Calibration is required before running the algorithm.")
            return

        self.run_button.setEnabled(False)
        self.calibration_button.setEnabled(False)
        self.stop_button.setEnabled(True)

        # 直接显示算法结果
        self.show_algorithm_results()

    def show_algorithm_results(self):
        if not self.is_calibrated or not self.app_instance.raw_data:
            QMessageBox.warning(self, "数据不足", "请先完成校准并收集足够数据")
            return

        # 获取校准参数
        scale_x = self.app_instance.scale_x
        scale_y = self.app_instance.scale_y
        center_x = self.app_instance.center_x
        center_y = self.app_instance.center_y

        # 获取原始数据
        raw_data = self.app_instance.raw_data
        
        # 应用算法
        processed_data = [
            (x * scale_x - center_x, y * scale_y - center_y)
            for x, y in raw_data
        ]

        # 创建新画布
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
        fig.suptitle("算法处理结果对比", fontsize=14)

        # 绘制原始数据
        ax1.scatter(*zip(*raw_data), color='red', s=10, label="原始数据")
        ax1.set_title("原始数据分布")
        ax1.set_xlabel("Mag X")
        ax1.set_ylabel("Mag Y")
        ax1.legend()
        ax1.axis('equal')

        # 绘制处理后数据
        ax2.scatter(*zip(*processed_data), color='blue', s=10, label="处理后数据")
        ax2.set_title("算法处理结果")
        ax2.set_xlabel("Processed X")
        ax2.set_ylabel("Processed Y")
        ax2.legend()
        ax2.axis('equal')

        # 显示新窗口
        plt.tight_layout()
        plt.show()

        # 恢复按钮状态
        self.calibration_button.setEnabled(True)
        self.run_button.setEnabled(True)
        self.stop_button.setEnabled(False)

    def stop_operation(self):
        if hasattr(self, 'calibration_timer') and self.calibration_timer.isActive():
            self.calibration_timer.stop()
        if self.app_instance is not None:
            self.app_instance.stop_serial()

        self.calibration_button.setEnabled(True)
        self.run_button.setEnabled(self.is_calibrated)
        self.stop_button.setEnabled(False)