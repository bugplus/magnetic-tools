# compass_ui.py
# compass_ui.py
import sys
import time
import numpy as np
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                            QLabel, QPushButton, QComboBox, QMessageBox, QFileDialog)
from PyQt5.QtCore import QTimer
from serial.tools import list_ports
from compass_app import CompassApp
import config
import os
import datetime

class CompassUI(QWidget):
    def __init__(self):
        super().__init__()
        # 英文界面设置
        self.setWindowTitle("Compass Calibration System")
        self.setMinimumSize(600, 250)
        
        # UI组件（英文）
        self.port_combo = QComboBox()
        self.refresh_button = QPushButton("Refresh Ports")
        self.baud_combo = QComboBox()
        self.calibration_button = QPushButton("Start Calibration")
        self.results_button = QPushButton("Show Results")
        self.status_label = QLabel("Status: Ready")
        
        # 初始状态设置
        self.results_button.setEnabled(False)
        self.app_instance = None
        self.is_calibrated = False
        self.calibration_timer = None

        self.initUI()
        self.refresh_ports()

    def initUI(self):
        main_layout = QVBoxLayout()
        
        # 串口选择区域
        port_layout = QHBoxLayout()
        port_layout.addWidget(QLabel("Serial Port:"))
        port_layout.addWidget(self.port_combo, 4)
        port_layout.addWidget(self.refresh_button, 2)
        
        # 波特率选择区域
        baud_layout = QHBoxLayout()
        baud_layout.addWidget(QLabel("Baud Rate:"))
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.baud_combo.setCurrentText("115200")
        baud_layout.addWidget(self.baud_combo, 4)
        baud_layout.addWidget(self.calibration_button, 2)
        
        # 按钮区域
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.results_button, 1)
        
        # 状态区域
        status_layout = QHBoxLayout()
        status_layout.addWidget(self.status_label)
        
        # 组合所有布局
        main_layout.addLayout(port_layout)
        main_layout.addLayout(baud_layout)
        main_layout.addLayout(button_layout)
        main_layout.addLayout(status_layout)
        self.setLayout(main_layout)
        
        # 连接信号
        self.refresh_button.clicked.connect(self.refresh_ports)
        self.calibration_button.clicked.connect(self.start_calibration)
        self.results_button.clicked.connect(self.show_algorithm_results)

    def refresh_ports(self):
        """刷新可用串口列表"""
        self.port_combo.clear()
        ports = list_ports.comports()
        if ports:
            for port in ports:
                self.port_combo.addItem(port.device)
            self.status_label.setText("Status: Ports refreshed")
        else:
            self.port_combo.addItem("No ports available")
            self.status_label.setText("Status: No serial ports found")

    def start_calibration(self):
        """开始校准流程"""
        selected_port = self.port_combo.currentText()
        if not selected_port or "No ports" in selected_port:
            QMessageBox.warning(self, "Port Error", "Please select a valid serial port")
            return
            
        # 清理之前的实例
        if self.app_instance:
            self.app_instance.stop()
            self.app_instance = None
            time.sleep(0.5)
            
        # 创建新的应用实例
        baud_rate = int(self.baud_combo.currentText())
        self.app_instance = CompassApp(selected_port, baud_rate)
        
        # 连接串口
        if not self.app_instance.connect_serial():
            QMessageBox.critical(self, "Connection Error", 
                                f"Failed to connect to {selected_port} at {baud_rate} baud")
            self.app_instance = None
            return
            
        # 启动校准
        self.calibration_button.setEnabled(False)
        self.results_button.setEnabled(False)
        self.status_label.setText(f"Status: Calibrating... {config.CALIBRATION_DURATION} seconds remaining")
        
        # 启动数据收集和绘图
        self.app_instance.start_data_collection()
        
        # 设置校准计时器
        self.calibration_time_remaining = config.CALIBRATION_DURATION
        self.calibration_timer = QTimer(self)
        self.calibration_timer.timeout.connect(self.update_calibration_timer)
        self.calibration_timer.start(1000)

    def update_calibration_timer(self):
        """更新校准计时器"""
        self.calibration_time_remaining -= 1
        self.status_label.setText(f"Status: Calibrating... {self.calibration_time_remaining} seconds remaining")
        
        if self.calibration_time_remaining <= 0:
            self.calibration_timer.stop()
            self.complete_calibration()

    def complete_calibration(self):
        """完成校准流程"""
        if not self.app_instance or not self.app_instance.raw_data:
            QMessageBox.warning(self, "Calibration Failed", "No valid data collected")
            self.cleanup()
            return
            
        # 停止数据收集
        self.app_instance.stop_data_collection()
        
        # 执行校准计算并更新视图
        success = self.app_instance.calibrate_data()
        if not success:
            QMessageBox.warning(self, "Calibration Failed", "Insufficient data for calibration")
            self.cleanup()
            return
        
        # 更新状态
        self.is_calibrated = True
        self.results_button.setEnabled(True)
        self.calibration_button.setEnabled(True)
        self.status_label.setText("Status: Calibration complete! Click 'Show Results' to view")
        
        # 显示完成对话框（英文）
        data_count = len(self.app_instance.raw_data)
        scale_x = self.app_instance.scale_x
        scale_y = self.app_instance.scale_y
        center_x = self.app_instance.center_x
        center_y = self.app_instance.center_y
        
        msg = QMessageBox()
        msg.setWindowTitle("Calibration Complete")
        msg.setText(f"Calibration completed successfully in {config.CALIBRATION_DURATION} seconds!")
        msg.setInformativeText(
            f"Data points: {data_count}\n"
            f"Scale X: {scale_x:.4f}\n"
            f"Scale Y: {scale_y:.4f}\n"
            f"Center X: {center_x:.2f}\n"
            f"Center Y: {center_y:.2f}"
        )
        msg.setStandardButtons(QMessageBox.Ok)
        msg.exec_()

    def show_algorithm_results(self):
        """显示算法处理结果（新窗口显示图4和图5）"""
        if not self.is_calibrated or not self.app_instance or not self.app_instance.raw_data:
            QMessageBox.warning(self, "Data Error", "Please complete calibration first")
            return
            
        # 获取校准参数
        scale_x = self.app_instance.scale_x
        scale_y = self.app_instance.scale_y
        center_x = self.app_instance.center_x
        center_y = self.app_instance.center_y
        
        # 获取原始数据
        raw_data = self.app_instance.raw_data
        
        # 应用算法生成图5数据
        processed_data = [
            (x * scale_x - center_x, y * scale_y - center_y)
            for x, y in raw_data
        ]
        
        # 计算原始坐标系中的圆心位置
        raw_center_x = center_x / scale_x
        raw_center_y = center_y / scale_y
        
        # 创建新画布（图4和图5）- 全英文标注
        fig, (ax4, ax5) = plt.subplots(1, 2, figsize=(14, 6))
        fig.suptitle("Calibration Results Analysis", fontsize=16)
        
        # 图4：原始数据（红色点）
        xs_raw, ys_raw = zip(*raw_data)
        ax4.scatter(xs_raw, ys_raw, c='red', s=15, alpha=0.6, label="Raw Data")
        ax4.scatter([raw_center_x], [raw_center_y], c='black', s=80, marker='x', label="Center")
        ax4.set_title("Source Data Distribution (Fig 4)")
        ax4.set_xlabel("Mag X")
        ax4.set_ylabel("Mag Y")
        ax4.grid(True)
        ax4.legend()
        ax4.axis('equal')
        
        # 自动调整坐标轴范围
        x_min, x_max = min(xs_raw), max(xs_raw)
        y_min, y_max = min(ys_raw), max(ys_raw)
        margin = max((x_max - x_min)*0.1, (y_max - y_min)*0.1, 50)
        ax4.set_xlim(x_min - margin, x_max + margin)
        ax4.set_ylim(y_min - margin, y_max + margin)
        
        # 图5：应用算法处理后的数据（蓝色点）
        xs_proc, ys_proc = zip(*processed_data)
        ax5.scatter(xs_proc, ys_proc, c='blue', s=15, alpha=0.6, label="Processed Data")
        ax5.scatter([0], [0], c='black', s=80, marker='x', label="Center (0,0)")
        ax5.set_title("Calibrated Result (Fig 5)")
        ax5.set_xlabel("Calibrated X")
        ax5.set_ylabel("Calibrated Y")
        ax5.grid(True)
        ax5.axhline(0, color='black', linewidth=0.8)
        ax5.axvline(0, color='black', linewidth=0.8)
        ax5.legend()
        ax5.axis('equal')
        
        # 对称坐标轴范围
        max_val = max(abs(x) for x in (*xs_proc, *ys_proc)) * 1.2
        ax5.set_xlim(-max_val, max_val)
        ax5.set_ylim(-max_val, max_val)
        
        # 显示参数信息（英文）
        param_text = (
            f"Calibration Parameters:\n"
            f"Scale X: {scale_x:.4f}\n"
            f"Scale Y: {scale_y:.4f}\n"
            f"Center X: {center_x:.2f}\n"
            f"Center Y: {center_y:.2f}\n"
            f"Data Points: {len(raw_data)}"
        )
        plt.figtext(0.5, 0.01, param_text, ha="center", fontsize=11, 
                    bbox={"facecolor":"#FF88FF", "alpha":0.8, "pad":5})
        
        plt.tight_layout(rect=[0, 0.05, 1, 0.95])
        
        # 保存数据到文本文件（包含算法描述）
        self.save_data_to_file(raw_data, processed_data, scale_x, scale_y, center_x, center_y)
        
        plt.show()

    def save_data_to_file(self, raw_data, processed_data, scale_x, scale_y, center_x, center_y):
        """将原始数据、处理后的数据和算法描述保存到文本文件"""
        # 创建时间戳
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 让用户选择保存位置
        options = QFileDialog.Options()
        file_path, _ = QFileDialog.getSaveFileName(
            self, 
            "Save Calibration Data", 
            f"compass_calibration_{timestamp}.txt", 
            "Text Files (*.txt);;All Files (*)", 
            options=options
        )
        
        if not file_path:
            return  # 用户取消了保存
        
        try:
            with open(file_path, 'w') as f:
                # 1. 保存校准参数
                f.write("=== CALIBRATION PARAMETERS ===\n")
                f.write(f"Scale X: {scale_x:.6f}\n")
                f.write(f"Scale Y: {scale_y:.6f}\n")
                f.write(f"Center X: {center_x:.6f}\n")
                f.write(f"Center Y: {center_y:.6f}\n")
                f.write(f"Data Points: {len(raw_data)}\n")
                f.write(f"Timestamp: {timestamp}\n\n")
                
                # 2. 保存源数据和处理后数据（一行对应一个数据点）
                f.write("=== RAW AND CALIBRATED DATA ===\n")
                f.write("Index, Raw_X, Raw_Y, Calibrated_X, Calibrated_Y\n")
                
                # 添加行号并合并数据
                for i, ((raw_x, raw_y), (cal_x, cal_y)) in enumerate(zip(raw_data, processed_data)):
                    f.write(f"{i+1}, {raw_x:.6f}, {raw_y:.6f}, {cal_x:.6f}, {cal_y:.6f}\n")
                
                f.write("\n\n")
                
                # 3. 算法描述（包括缩放因子和偏移值计算）
                f.write("=== CALIBRATION ALGORITHM ===\n")
                f.write("Calibration formula:\n")
                f.write("Calibrated_X = (Raw_X × Scale_X) - Center_X\n")
                f.write("Calibrated_Y = (Raw_Y × Scale_Y) - Center_Y\n\n")
                
                f.write("Calculation details:\n")
                f.write("1. Collected raw magnetometer data over full 360° rotation\n")
                
                # 计算数据范围
                xs = [x for x, y in raw_data]
                ys = [y for x, y in raw_data]
                x_min, x_max = min(xs), max(xs)
                y_min, y_max = min(ys), max(ys)
                x_range = x_max - x_min
                y_range = y_max - y_min
                
                f.write(f"   X_min = {x_min:.2f}, X_max = {x_max:.2f}, Range = {x_range:.2f}\n")
                f.write(f"   Y_min = {y_min:.2f}, Y_max = {y_max:.2f}, Range = {y_range:.2f}\n\n")
                
                f.write("2. Scale factors calculation:\n")
                if x_range > y_range:
                    f.write(f"   Scale_X = 1.0 (fixed)\n")
                    f.write(f"   Scale_Y = X_range / Y_range = {x_range:.4f} / {y_range:.4f} = {scale_y:.6f}\n")
                else:
                    f.write(f"   Scale_X = Y_range / X_range = {y_range:.4f} / {x_range:.4f} = {scale_x:.6f}\n")
                    f.write(f"   Scale_Y = 1.0 (fixed)\n")
                f.write("\n")
                
                f.write("3. Center point calculation:\n")
                scaled_xs = [x * scale_x for x in xs]
                scaled_ys = [y * scale_y for y in ys]
                center_x_calc = (max(scaled_xs) + min(scaled_xs)) / 2
                center_y_calc = (max(scaled_ys) + min(scaled_ys)) / 2
                f.write(f"   Center_X = (max(Scaled_X) + min(Scaled_X)) / 2 = ({max(scaled_xs):.2f} + {min(scaled_xs):.2f}) / 2 = {center_x_calc:.6f}\n")
                f.write(f"   Center_Y = (max(Scaled_Y) + min(Scaled_Y)) / 2 = ({max(scaled_ys):.2f} + {min(scaled_ys):.2f}) / 2 = {center_y_calc:.6f}\n")
                
            QMessageBox.information(self, "Save Successful", f"Calibration data saved to:\n{file_path}")
        except Exception as e:
            QMessageBox.critical(self, "Save Failed", f"Error saving data:\n{str(e)}")
    def cleanup(self):
        """清理资源"""
        if self.calibration_timer and self.calibration_timer.isActive():
            self.calibration_timer.stop()
        if self.app_instance:
            self.app_instance.stop()
        self.calibration_button.setEnabled(True)
        self.status_label.setText("Status: Operation stopped")

    def closeEvent(self, event):
        """关闭窗口时清理资源"""
        self.cleanup()
        event.accept()
