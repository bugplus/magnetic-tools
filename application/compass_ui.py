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
import matplotlib as mpl

class CompassUI(QWidget):
    def __init__(self):
        super().__init__()
        self.main_layout = None  # 1. 在构造函数中声明主布局变量
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
        # 新增UI元素
        self.run_button = QPushButton("Run Real-time")
        self.stop_button = QPushButton("Stop")
        self.pause_button = QPushButton("Pause")
        
        # 状态标签
        self.realtime_status = QLabel("Real-time: Stopped")
        
        # 初始化状态
        self.realtime_active = False
        self.realtime_paused = False
        self.realtime_data = []  # 存储实时数据点
        # 新增校准模式选择
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["Horizontal Calibration", "Tilt Calibration"])
        
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
        
        # 校准模式选择
        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("Calibration Mode:"))
        mode_layout.addWidget(self.mode_combo, 4)
        
        # 按钮区域
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.calibration_button, 1)
        button_layout.addWidget(self.results_button, 1)
        
        # 状态区域
        status_layout = QHBoxLayout()
        status_layout.addWidget(self.status_label)
        
        # 组合所有布局
        main_layout.addLayout(port_layout)
        main_layout.addLayout(baud_layout)
        main_layout.addLayout(mode_layout)
        main_layout.addLayout(button_layout)
        main_layout.addLayout(status_layout)
        self.setLayout(main_layout)
        
        # 连接信号
        self.refresh_button.clicked.connect(self.refresh_ports)
        self.calibration_button.clicked.connect(self.start_calibration)
        self.results_button.clicked.connect(self.show_algorithm_results)

        # 新增实时控制区域
        realtime_layout = QHBoxLayout()
        realtime_layout.addWidget(self.run_button)
        realtime_layout.addWidget(self.pause_button)
        realtime_layout.addWidget(self.stop_button)
        realtime_layout.addWidget(self.realtime_status)
        
        # 添加到主布局
        main_layout.addLayout(realtime_layout)
        
        # 连接信号
        self.run_button.clicked.connect(self.start_realtime)
        self.pause_button.clicked.connect(self.toggle_pause)
        self.stop_button.clicked.connect(self.stop_realtime)

    def start_realtime(self):
        """启动实时模式"""
        if not self.is_calibrated:
            QMessageBox.warning(self, "Error", "Please calibrate first!")
            return
            
        # 确保串口连接
        if not self.app_instance or not self.app_instance.ser or not self.app_instance.ser.is_open:
            self.start_calibration(connect_only=True)
            
        if not self.app_instance.ser.is_open:
            QMessageBox.critical(self, "Connection Error", "Serial port not connected")
            return
            
        # 创建实时绘图窗口
        self.realtime_fig, (self.ax_raw, self.ax_cal) = plt.subplots(1, 2, figsize=(14, 6))
        self.realtime_fig.suptitle("Real-time Compass Data", fontsize=16)
        
        # 设置原始数据图
        self.ax_raw.set_title("Raw Data")
        self.ax_raw.set_xlabel("Mag X")
        self.ax_raw.set_ylabel("Mag Y")
        self.ax_raw.grid(True)
        self.ax_raw.axis('equal')
        self.ax_raw.set_xlim(-300, 300)
        self.ax_raw.set_ylim(-300, 300)
        self.scatter_raw = self.ax_raw.scatter([], [], c='red', s=15, alpha=0.6, label="Raw")
        
        # 设置校准后数据图
        self.ax_cal.set_title("Calibrated Data")
        self.ax_cal.set_xlabel("Calibrated X")
        self.ax_cal.set_ylabel("Calibrated Y")
        self.ax_cal.grid(True)
        self.ax_cal.axhline(0, color='black', linewidth=0.8)
        self.ax_cal.axvline(0, color='black', linewidth=0.8)
        self.ax_cal.axis('equal')
        self.ax_cal.set_xlim(-200, 200)
        self.ax_cal.set_ylim(-200, 200)
        self.scatter_cal = self.ax_cal.scatter([], [], c='blue', s=15, alpha=0.6, label="Calibrated")
        
        plt.legend()
        plt.tight_layout(rect=[0, 0, 1, 0.95])
        plt.show(block=False)
        
        # 初始化实时数据存储
        self.realtime_data = []
        self.realtime_active = True
        self.realtime_paused = False
        self.realtime_status.setText("Real-time: Running")
        
        # 启动定时器
        self.realtime_timer = QTimer(self)
        self.realtime_timer.timeout.connect(self.update_realtime_plot)
        self.realtime_timer.start(100)  # 10Hz更新频率
        
    def toggle_pause(self):
        """暂停/继续实时模式"""
        if not self.realtime_active:
            return
            
        self.realtime_paused = not self.realtime_paused
        status = "Paused" if self.realtime_paused else "Running"
        self.realtime_status.setText(f"Real-time: {status}")
        
    def stop_realtime(self):
        """停止实时模式"""
        if self.realtime_timer and self.realtime_timer.isActive():
            self.realtime_timer.stop()
            
        self.realtime_active = False
        self.realtime_paused = False
        self.realtime_status.setText("Real-time: Stopped")
        
        # 关闭绘图窗口
        if hasattr(self, 'realtime_fig') and self.realtime_fig:
            plt.close(self.realtime_fig)
            
    def update_realtime_plot(self):
        """更新实时绘图"""
        if not self.realtime_active or self.realtime_paused:
            return
            
        if not self.app_instance or not self.app_instance.ser or not self.app_instance.ser.is_open:
            return
            
        try:
            # 读取串口数据
            if self.app_instance.ser.in_waiting:
                line = self.app_instance.ser.readline().decode('ascii', errors='ignore').strip()
                
                if not line:
                    return
                    
                # 解析数据
                data_dict = {}
                parts = line.split(',')
                for part in parts:
                    part = part.strip()
                    if '=' in part:
                        key, value = part.split('=', 1)
                        key = key.strip().lower()
                        try:
                            data_dict[key] = float(value)
                        except ValueError:
                            pass
                
                # 检查是否包含所需数据
                if 'mag_x' in data_dict and 'mag_y' in data_dict and 'mag_z' in data_dict and 'pitch' in data_dict and 'roll' in data_dict:
                    mag_x = data_dict['mag_x']
                    mag_y = data_dict['mag_y']
                    mag_z = data_dict['mag_z']
                    pitch = data_dict['pitch']
                    roll = data_dict['roll']
                    
                    # 应用倾角补偿
                    x_comp, y_comp = self.app_instance.tilt_compensation(mag_x, mag_y, mag_z, pitch, roll)
                    
                    # 应用校准参数
                    x_cal = (x_comp * self.app_instance.scale_x) - self.app_instance.center_x
                    y_cal = (y_comp * self.app_instance.scale_y) - self.app_instance.center_y
                    
                    # 存储数据点
                    self.realtime_data.append({
                        'raw': (mag_x, mag_y),
                        'calibrated': (x_cal, y_cal)
                    })
                    
                    # 限制数据点数量
                    if len(self.realtime_data) > 100:
                        self.realtime_data.pop(0)
                        
                    # 更新散点图数据
                    raw_x = [d['raw'][0] for d in self.realtime_data]
                    raw_y = [d['raw'][1] for d in self.realtime_data]
                    cal_x = [d['calibrated'][0] for d in self.realtime_data]
                    cal_y = [d['calibrated'][1] for d in self.realtime_data]
                    
                    self.scatter_raw.set_offsets(np.column_stack([raw_x, raw_y]))
                    self.scatter_cal.set_offsets(np.column_stack([cal_x, cal_y]))
                    
                    # 重绘图
                    self.realtime_fig.canvas.draw_idle()
                    
        except Exception as e:
            print(f"Realtime error: {e}")
            
    # 在closeEvent中添加清理
    def closeEvent(self, event):
        if self.realtime_active:
            self.stop_realtime()
        if self.app_instance:
            self.app_instance.stop()
        event.accept()

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

    def start_calibration(self, connect_only=False):
        """开始校准流程，增加connect_only参数"""
        selected_port = self.port_combo.currentText()
        if not selected_port or "No ports" in selected_port:
            if not connect_only:
                QMessageBox.warning(self, "Port Error", "Please select a valid serial port")
            return False
            
        # 清理之前的实例
        if self.app_instance:
            self.app_instance.stop()
            self.app_instance = None
            time.sleep(0.5)
            
        # 创建新的应用实例
        baud_rate = int(self.baud_combo.currentText())
        self.app_instance = CompassApp(selected_port, baud_rate)
        
        # 连接串口
        if not self.app_instance.connect_serial(connect_only):
            if not connect_only:
                QMessageBox.critical(self, "Connection Error", 
                                    f"Failed to connect to {selected_port} at {baud_rate} baud")
            return False
            
        # return True
            
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

        return True  # 成功启动时返回True（可选）

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
        
        # 处理数据（根据校准模式）
        processed_data = []
        if "Tilt" in self.mode_combo.currentText():
            # 倾角校准模式
            for mag_x, mag_y, mag_z, pitch, roll in raw_data:
                # 应用倾角补偿
                x_comp, y_comp = self.app_instance.tilt_compensation(
                    mag_x, mag_y, mag_z, pitch, roll
                )
                # 应用校准参数
                x_cal = x_comp * scale_x - center_x
                y_cal = y_comp * scale_y - center_y
                processed_data.append((x_cal, y_cal))
        else:
            # 水平校准模式
            for mag_x, mag_y, mag_z, pitch, roll in raw_data:
                # 应用校准参数
                x_cal = mag_x * scale_x - center_x
                y_cal = mag_y * scale_y - center_y
                processed_data.append((x_cal, y_cal))
        
        # 计算原始坐标系中的圆心位置
        raw_center_x = center_x / scale_x
        raw_center_y = center_y / scale_y
        
        # 创建新画布（图4和图5）- 全英文标注
        fig, (ax4, ax5) = plt.subplots(1, 2, figsize=(14, 6))
        fig.suptitle("Calibration Results Analysis", fontsize=16)
        
        # 图4：原始数据（红色点）
        xs_raw = [x for x, y, z, p, r in raw_data]
        ys_raw = [y for x, y, z, p, r in raw_data]
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

        """显示增强的3D可视化效果"""
        # ... [获取校准参数和数据] ...
        
        # 创建画布布局
        fig = plt.figure(figsize=(20, 12))
        
        # === 1. 3D主视图（动态交互） ===
        ax_main = fig.add_subplot(231, projection='3d', facecolor='#f0f0f0')
        ax_main.set_title("Enhanced 3D Magnetic Field", fontsize=14, pad=15)
        
        # 提取原始磁力数据
        xs = [x for x, y, z, p, r in self.app_instance.raw_data]
        ys = [y for x, y, z, p, r in self.app_instance.raw_data]
        zs = [z for x, y, z, p, r in self.app_instance.raw_data]
        
        # 创建色彩映射（按矢量大小）
        magnitudes = np.sqrt(np.square(xs) + np.square(ys) + np.square(zs))
        color_map = plt.cm.viridis
        norm = plt.Normalize(min(magnitudes), max(magnitudes))
        
        # 点云绘制（半透明效果）
        scatter = ax_main.scatter(xs, ys, zs, 
                                c=color_map(norm(magnitudes)), 
                                s=15, 
                                alpha=0.7, 
                                depthshade=True)
        
        # 添加色彩条
        cbar = fig.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=color_map),
                   ax=ax_main, shrink=0.7)
        cbar.set_label('Magnetic Magnitude (μT)', fontsize=10)
        
        # === 2. 理论参考球面 ===
        # 计算平均半径作为参考
        avg_radius = np.mean(magnitudes)
        
        # 生成参考球面
        u = np.linspace(0, 2 * np.pi, 50)
        v = np.linspace(0, np.pi, 25)
        x_ref = avg_radius * np.outer(np.cos(u), np.sin(v))
        y_ref = avg_radius * np.outer(np.sin(u), np.sin(v))
        z_ref = avg_radius * np.outer(np.ones(np.size(u)), np.cos(v))
        
        # 绘制透明参考球面
        ax_main.plot_surface(x_ref, y_ref, z_ref, 
                            color='red', 
                            alpha=0.15, 
                            edgecolor='darkred', 
                            linewidth=0.5)
        
        # === 3. 正交投影视图 ===
        def create_ortho_view(ax, plane='xy'):
            """创建正交投影视图"""
            if plane == 'xy':
                x, y = xs, ys
                z = zs
                labels = ('Mag X', 'Mag Y')
            elif plane == 'xz':
                x, y = xs, zs
                z = ys
                labels = ('Mag X', 'Mag Z')
            else:  # 'yz'
                x, y = ys, zs
                z = xs
                labels = ('Mag Y', 'Mag Z')
                
            # 密度热力图
            hb = ax.hexbin(x, y, 
                            gridsize=30,
                            cmap='viridis',
                            bins='log',
                            mincnt=1)
            
            # 添加颜色条
            fig.colorbar(hb, ax=ax, label='Density')
            
            ax.set_title(f"{plane.upper()} Plane Projection", fontsize=12)
            ax.set_xlabel(labels[0], fontsize=10)
            ax.set_ylabel(labels[1], fontsize=10)
            ax.grid(True, linestyle='--', alpha=0.7)
            
            # 标记原点
            ax.scatter([0], [0], color='red', s=50, marker='+')
        
        # XY平面投影
        ax_xy = fig.add_subplot(234)
        create_ortho_view(ax_xy, 'xy')
        
        # XZ平面投影
        ax_xz = fig.add_subplot(235)
        create_ortho_view(ax_xz, 'xz')
        
        # YZ平面投影
        ax_yz = fig.add_subplot(236)
        create_ortho_view(ax_yz, 'yz')
        
        # === 4. 径向偏差分析 ===
        ax_radial = fig.add_subplot(232, projection='polar')
        
        # 计算角度和半径偏差
        radii = np.sqrt(np.square(xs) + np.square(ys) + np.square(zs))
        mean_radius = np.mean(radii)
        deviations = (radii - mean_radius) / mean_radius * 100  # 百分比偏差
        
        # 计算方位角
        azimuth = np.arctan2(ys, xs)  # 弧度
        
        # 分箱统计
        azimuth_bins = np.linspace(0, 2*np.pi, 24)
        bin_means = []
        for i in range(len(azimuth_bins)-1):
            mask = (azimuth >= azimuth_bins[i]) & (azimuth < azimuth_bins[i+1])
            if np.any(mask):
                bin_means.append(np.mean(deviations[mask]))
            else:
                bin_means.append(0)
        
        # 绘制径向偏差
        bars = ax_radial.bar(azimuth_bins[:-1], 
                            bin_means, 
                            width=2*np.pi/24, 
                            alpha=0.7,
                            color='blue')
        
        # 美化图表
        ax_radial.set_title("Radial Deviation by Azimuth (°)", fontsize=12, pad=15)
        ax_radial.set_rlabel_position(315)  # 移动半径标签位置
        ax_radial.grid(True, linestyle='--', alpha=0.7)
        
        # === 5. 偏差值分布直方图 ===
        ax_hist = fig.add_subplot(233)
        ax_hist.hist(deviations, bins=30, color='green', alpha=0.7)
        ax_hist.axvline(0, color='red', linestyle='--')
        ax_hist.set_title("Radial Deviation Distribution", fontsize=12)
        ax_hist.set_xlabel("Deviation (%)")
        ax_hist.set_ylabel("Count")
        ax_hist.grid(True, alpha=0.3)
        
        # 添加统计信息
        mean_dev = np.mean(deviations)
        std_dev = np.std(deviations)
        ax_hist.text(0.05, 0.9, 
                    f"Mean: {mean_dev:.2f}%\nStd: {std_dev:.2f}%", 
                    transform=ax_hist.transAxes,
                    bbox=dict(facecolor='white', alpha=0.8))
        
        # 添加整体描述
        fig.text(0.5, 0.92, 
                f"3D Magnetic Field Analysis | Points: {len(xs)} | Avg Magnitude: {np.mean(magnitudes):.1f}μT ± {np.std(magnitudes):.1f}μT", 
                fontsize=14, ha='center')
        
        # 添加图例和说明
        fig.text(0.1, 0.02, 
                "Blue points: Raw magnetic measurements | Red sphere: Theoretical reference sphere", 
                fontsize=10)
        
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        
        # 保存数据到文本文件
        self.save_data_to_file(raw_data, processed_data, scale_x, scale_y, center_x, center_y)
        
        plt.show()

    # compass_ui.py (修正后的save_data_to_file方法)
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
                # 1. 保存校准参数（完全按照图片格式）
                f.write("=== CALIBRATION PARAMETERS ===\n")
                f.write(f"Scale X: {scale_x:.6f}\n")
                f.write(f"Scale Y: {scale_y:.6f}\n")
                f.write(f"Center X: {center_x:.6f}\n")
                f.write(f"Center Y: {center_y:.6f}\n")
                f.write(f"Data Points: {len(raw_data)}\n")
                f.write(f"Timestamp: {timestamp}\n\n")
                
                # 2. 保存源数据和处理后数据（一行对应一个数据点）
                f.write("=== RAW AND CALIBRATED DATA ===\n")
                f.write("Index, Raw_X, Raw_Y, Raw_Z, Pitch, Roll, Calibrated_X, Calibrated_Y\n")
                
                # 添加行号并合并数据
                for i, ((raw_x, raw_y, raw_z, pitch, roll), (cal_x, cal_y)) in enumerate(zip(raw_data, processed_data)):
                    # 使用f-string格式化每行数据（保留6位小数）
                    f.write(f"{i+1}, {raw_x:.6f}, {raw_y:.6f}, {raw_z:.6f}, {pitch:.6f}, {roll:.6f}, {cal_x:.6f}, {cal_y:.6f}\n")
                
                f.write("\n")
                
                # 3. 算法描述（包括倾角补偿公式）
                f.write("=== CALIBRATION ALGORITHM ===\n")
                f.write("Calibration formula:\n")
                f.write("Calibrated_X = (TiltCompensated_X × Scale_X) - Center_X\n")
                f.write("Calibrated_Y = (TiltCompensated_Y × Scale_Y) - Center_Y\n\n")
                
                f.write("Tilt Compensation formula:\n")
                f.write("pitch_rad = math.radians(pitch)\n")
                f.write("roll_rad = math.radians(roll)\n")
                f.write("TiltCompensated_X = mag_x * cos(pitch_rad) + mag_z * sin(pitch_rad)\n")
                f.write("TiltCompensated_Y = mag_x * sin(roll_rad)*sin(pitch_rad) + mag_y * cos(roll_rad) - mag_z * sin(roll_rad)*cos(pitch_rad)\n")
            
            QMessageBox.information(self, "Save Successful", f"Calibration data saved to:\n{file_path}")
        except Exception as e:
            QMessageBox.critical(self, "Save Failed", f"Error saving data:\n{str(e)}")