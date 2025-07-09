# compass_ui.py
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox, QMessageBox
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
        self.calibration_button = QPushButton("Calibration")  # 校准按钮
        self.run_button = QPushButton("Run")  # 运行算法按钮
        self.stop_button = QPushButton("Stop")  # 停止按钮
        self.stop_button.setEnabled(False)  # 初始状态禁用 Stop 按钮
        self.app_instance = None
        self.is_calibrated = False  # 标记是否完成校准

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
            self.stop_operation()  # 如果已经有实例在运行，先停止
            del self.app_instance

        port = self.port_combo.currentText()
        baud_rate = int(self.baud_combo.currentText())

        self.app_instance = CompassApp()
        self.app_instance.port = port
        self.app_instance.baud_rate = baud_rate
        self.app_instance.connect_serial()

        self.app_instance.data_collection_started = True
        self.app_instance.start_time = time.time()

        self.calibration_button.setEnabled(False)
        self.run_button.setEnabled(False)
        self.stop_button.setEnabled(True)

        # 启动一个定时器，用于在指定时间后完成校准
        self.calibration_timer = QTimer(self)
        self.calibration_timer.setSingleShot(True)
        self.calibration_timer.timeout.connect(self.complete_calibration)
        self.calibration_timer.start(config.CALIBRATION_DURATION * 1000)

    def complete_calibration(self):
        if self.app_instance is not None:
            self.app_instance.calibrate_data()
            self.is_calibrated = True  # 校准完成后启用 Calibration 按钮
            self.calibration_button.setEnabled(True)
            self.run_button.setEnabled(True)  # 校准完成后启用 Run 按钮
            self.stop_button.setEnabled(False)  # 校准完成后禁用 Stop 按钮
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

        # 调用 CompassApp 的 run_algorithm 方法
        if self.app_instance is not None:
            self.app_instance.run_algorithm()
        else:
            QMessageBox.warning(self, "Warning", "No data available to process.")

        self.stop_operation()

    def stop_operation(self):
        if hasattr(self, 'calibration_timer'):
            self.calibration_timer.stop()
        if self.app_instance is not None:
            self.app_instance.stop_serial()
            self.app_instance.calibration_done = False
            self.app_instance.data_collection_started = False

        self.calibration_button.setEnabled(True)
        self.run_button.setEnabled(self.is_calibrated)  # 如果已经校准，则启用 Run 按钮
        self.stop_button.setEnabled(False)

    def clear_app_instance(self):
        if self.app_instance is not None:
            del self.app_instance
            self.app_instance = None
            print("[INFO] 数据实例已释放")