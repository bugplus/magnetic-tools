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
        # 严格按图片界面设置
        self.setWindowTitle("磁力计校准系统")
        self.setMinimumSize(600, 250)
        
        # UI组件（完全按图片设置）
        self.port_combo = QComboBox()
        self.refresh_button = QPushButton("刷新串口")
        self.baud_combo = QComboBox()
        self.calibration_button = QPushButton("开始校准")
        self.results_button = QPushButton("显示结果")
        self.status_label = QLabel("状态: 就绪")
        
        # 初始状态设置
        self.results_button.setEnabled(False)
        self.app_instance = None
        self.is_calibrated = False
        self.calibration_timer = None

        self.initUI()
        self.refresh_ports()

    def initUI(self):
        main_layout = QVBoxLayout()
        
        # 串口选择区域（图片左侧）
        port_layout = QHBoxLayout()
        port_layout.addWidget(QLabel("串口:"))
        port_layout.addWidget(self.port_combo, 4)
        port_layout.addWidget(self.refresh_button, 2)
        
        # 波特率选择区域（图片中部）
        baud_layout = QHBoxLayout()
        baud_layout.addWidget(QLabel("波特率:"))
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.baud_combo.setCurrentText("115200")
        baud_layout.addWidget(self.baud_combo, 4)
        baud_layout.addWidget(self.calibration_button, 2)
        
        # 底部按钮区域（图片下方）
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.results_button, 1)
        
        # 状态区域（图片底部）
        status_layout = QHBoxLayout()
        status_layout.addWidget(self.status_label)
        
        # 组合所有布局（按图片结构）
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
            self.status_label.setText("状态: 串口已刷新")
        else:
            self.port_combo.addItem("无可用串口")
            self.status_label.setText("状态: 未检测到串口")

    def start_calibration(self):
        """开始校准流程"""
        selected_port = self.port_combo.currentText()
        if not selected_port or "无可用串口" in selected_port:
            QMessageBox.warning(self, "串口错误", "请选择有效串口")
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
            QMessageBox.critical(self, "连接错误", f"无法连接串口 {selected_port}@{baud_rate} 波特")
            self.app_instance = None
            return
            
        # 启动校准
        self.calibration_button.setEnabled(False)
        self.results_button.setEnabled(False)
        self.status_label.setText(f"状态: 校准中 - 剩余时间: {config.CALIBRATION_DURATION}秒")
        
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
        self.status_label.setText(f"状态: 校准中 - 剩余时间: {self.calibration_time_remaining}秒")
        
        if self.calibration_time_remaining <= 0:
            self.calibration_timer.stop()
            self.complete_calibration()

    def complete_calibration(self):
        """完成校准流程"""
        if not self.app_instance or not self.app_instance.raw_data:
            QMessageBox.warning(self, "校准失败", "未收集到有效数据")
            self.cleanup()
            return
            
        # 停止数据收集
        self.app_instance.stop_data_collection()
        
        # 执行校准计算并更新三视图
        success = self.app_instance.calibrate_data()
        if not success:
            QMessageBox.warning(self, "校准失败", "数据不足，无法计算校准参数")
            self.cleanup()
            return
        
        # 更新状态
        self.is_calibrated = True
        self.results_button.setEnabled(True)
        self.calibration_button.setEnabled(True)
        self.status_label.setText("状态: 校准完成! 点击'显示结果'查看")
        
        # 显示完成对话框（严格按图片格式）
        data_count = len(self.app_instance.raw_data)
        scale_x = self.app_instance.scale_x
        scale_y = self.app_instance.scale_y
        center_x = self.app_instance.center_x
        center_y = self.app_instance.center_y
        
        msg = QMessageBox()
        msg.setWindowTitle("校准完成")
        msg.setText(f"校准在{config.CALIBRATION_DURATION}秒内成功完成！")
        msg.setInformativeText(
            f"数据点数量: {data_count}\n"
            f"X轴缩放因子: {scale_x:.4f}\n"
            f"Y轴缩放因子: {scale_y:.4f}\n"
            f"中心点X: {center_x:.2f}\n"
            f"中心点Y: {center_y:.2f}"
        )
        msg.setStandardButtons(QMessageBox.Ok)
        msg.exec_()

    def show_algorithm_results(self):
        """显示算法处理结果（新窗口显示图4和图5）"""
        if not self.is_calibrated or not self.app_instance or not self.app_instance.raw_data:
            QMessageBox.warning(self, "数据错误", "请先完成校准并收集足够数据")
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
        
        # 创建新画布（图4和图5）
        fig, (ax4, ax5) = plt.subplots(1, 2, figsize=(14, 6))
        fig.suptitle("校准结果分析", fontsize=16)
        
        # 图4：原始数据（红色点）
        xs_raw, ys_raw = zip(*raw_data)
        ax4.scatter(xs_raw, ys_raw, c='red', s=15, alpha=0.6, label="原始数据")
        ax4.scatter([raw_center_x], [raw_center_y], c='black', s=80, marker='x', label="圆心")
        ax4.set_title("源数据分布 (图4)")
        ax4.set_xlabel("磁力计 X")
        ax4.set_ylabel("磁力计 Y")
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
        ax5.scatter(xs_proc, ys_proc, c='blue', s=15, alpha=0.6, label="处理后数据")
        ax5.scatter([0], [0], c='black', s=80, marker='x', label="圆心 (0,0)")
        ax5.set_title("处理结果 (图5)")
        ax5.set_xlabel("校准 X")
        ax5.set_ylabel("校准 Y")
        ax5.grid(True)
        ax5.axhline(0, color='black', linewidth=0.8)
        ax5.axvline(0, color='black', linewidth=0.8)
        ax5.legend()
        ax5.axis('equal')
        
        # 对称坐标轴范围
        max_val = max(abs(x) for x in (*xs_proc, *ys_proc)) * 1.2
        ax5.set_xlim(-max_val, max_val)
        ax5.set_ylim(-max_val, max_val)
        
        # 显示参数信息（按图片格式在底部）
        param_text = (
            f"校准参数:\n"
            f"X轴缩放因子: {scale_x:.4f}\n"
            f"Y轴缩放因子: {scale_y:.4f}\n"
            f"中心点 X: {center_x:.2f}\n"
            f"中心点 Y: {center_y:.2f}\n"
            f"数据点数: {len(raw_data)}"
        )
        plt.figtext(0.5, 0.01, param_text, ha="center", fontsize=11, 
                    bbox={"facecolor":"#F0F8FF", "alpha":0.8, "pad":5})
        
        plt.tight_layout(rect=[0, 0.05, 1, 0.95])
        
        # 保存数据到文本文件
        self.save_data_to_file(raw_data, processed_data, scale_x, scale_y, center_x, center_y)
        
        plt.show()

    def save_data_to_file(self, raw_data, processed_data, scale_x, scale_y, center_x, center_y):
        """将原始数据、处理后的数据和校准参数保存到文本文件"""
        # 创建时间戳
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 让用户选择保存位置
        options = QFileDialog.Options()
        file_path, _ = QFileDialog.getSaveFileName(
            self, 
            "保存校准数据", 
            f"compass_calibration_{timestamp}.txt", 
            "Text Files (*.txt);;All Files (*)", 
            options=options
        )
        
        if not file_path:
            return  # 用户取消了保存
        
        try:
            with open(file_path, 'w') as f:
                # 写入校准参数
                f.write("=== 校准参数 ===\n")
                f.write(f"X轴缩放因子: {scale_x:.6f}\n")
                f.write(f"Y轴缩放因子: {scale_y:.6f}\n")
                f.write(f"中心点 X: {center_x:.6f}\n")
                f.write(f"中心点 Y: {center_y:.6f}\n")
                f.write(f"数据点数: {len(raw_data)}\n")
                f.write(f"校准时间: {timestamp}\n\n")
                
                # 写入原始数据
                f.write("=== 原始数据 ===\n")
                f.write("Mag_X, Mag_Y\n")
                for x, y in raw_data:
                    f.write(f"{x:.6f}, {y:.6f}\n")
                f.write("\n")
                
                # 写入处理后的数据
                f.write("=== 处理后的数据 ===\n")
                f.write("Calibrated_X, Calibrated_Y\n")
                for x, y in processed_data:
                    f.write(f"{x:.6f}, {y:.6f}\n")
                
            QMessageBox.information(self, "保存成功", f"校准数据已保存到:\n{file_path}")
        except Exception as e:
            QMessageBox.critical(self, "保存失败", f"保存数据时出错:\n{str(e)}")

    def cleanup(self):
        """清理资源"""
        if self.calibration_timer and self.calibration_timer.isActive():
            self.calibration_timer.stop()
        if self.app_instance:
            self.app_instance.stop()
        self.calibration_button.setEnabled(True)
        self.status_label.setText("状态: 操作已终止")

    def closeEvent(self, event):
        """关闭窗口时清理资源"""
        self.cleanup()
        event.accept()
