import sys
import serial
import threading
import time
import numpy as np
import re
from PyQt5.QtWidgets import QApplication
from compass_ui import CompassMainWindow

import matplotlib.pyplot as plt
from config import MAG_AXIS_MAP

CALIBRATION_DURATION = 30  # seconds

class SerialReader(threading.Thread):
    def __init__(self, port, baudrate, callback):
        threading.Thread.__init__(self)
        self.port = port
        self.baudrate = baudrate
        self.callback = callback
        self.running = False
    def run(self):
        self.running = True
        try:
            ser = serial.Serial(self.port, self.baudrate, timeout=1)
            while self.running:
                try:
                    line = ser.readline().decode(errors='ignore')
                    self.callback(line)
                except Exception as e:
                    print(f"Serial read error: {e}")
        except Exception as e:
            print(f"Serial error: {e}")
    def stop(self):
        self.running = False

def fit_circle_least_squares(xy):
    x = xy[:, 0]
    y = xy[:, 1]
    A = np.c_[2*x, 2*y, np.ones(x.shape)]
    b = x**2 + y**2
    c, resid, rank, s = np.linalg.lstsq(A, b, rcond=None)
    center_x, center_y = c[0], c[1]
    radius = np.sqrt(c[2] + center_x**2 + center_y**2)
    return np.array([center_x, center_y]), radius

def calibrate_magnetometer_standard(xy):
    # 1. Soft iron: scale y to match x radius (about origin)
    radius_x = (np.max(xy[:, 0]) - np.min(xy[:, 0])) / 2
    radius_y = (np.max(xy[:, 1]) - np.min(xy[:, 1])) / 2
    scale = radius_x / radius_y if radius_y != 0 else 1.0
    soft_calibrated = xy.copy()
    soft_calibrated[:, 1] *= scale

    # 2. Hard iron: fit circle center after scaling, then shift to origin
    center, _ = fit_circle_least_squares(soft_calibrated)
    hard_soft_calibrated = soft_calibrated - center

    return soft_calibrated, hard_soft_calibrated, center, scale

def calibrate_magnetometer_least_squares(xy):
    center, _ = fit_circle_least_squares(xy)
    shifted = xy - center
    radius_x = (np.max(shifted[:, 0]) - np.min(shifted[:, 0])) / 2
    radius_y = (np.max(shifted[:, 1]) - np.min(shifted[:, 1])) / 2
    scale = radius_x / radius_y if radius_y != 0 else 1.0
    calibrated = shifted.copy()
    calibrated[:, 1] *= scale
    return shifted, calibrated, center, scale

def generate_c_code(center, scale):
    c_code = f"""
// Auto-generated calibration code
void calibrate(float* x, float* y) {{
    // Hard iron offset
    *x -= {center[0]:.6f}f;
    *y -= {center[1]:.6f}f;
    // Soft iron scale
    *y *= {scale:.6f}f;
}}
"""
    return c_code

def plot_two(raw_data, calibrated, center, scale):
    plt.figure(figsize=(10, 4))
    ax1 = plt.subplot(1, 2, 1)
    ax1.set_title("Raw Data")
    ax1.axis('equal')
    if raw_data is not None and len(raw_data) > 0:
        ax1.plot(raw_data[:, 0], raw_data[:, 1], 'b.', alpha=0.7)
    ax1.plot(0, 0, 'k+', markersize=10, mew=2)
    ax1.plot(center[0], center[1], 'rx', markersize=10, mew=2)
    ax2 = plt.subplot(1, 2, 2)
    ax2.set_title(f"Calibrated Data\nScale: {scale:.4f}")
    ax2.axis('equal')
    if calibrated is not None and len(calibrated) > 0:
        ax2.plot(calibrated[:, 0], calibrated[:, 1], 'g.', alpha=0.7)
    # 校准后数据的圆心就是原点
    ax2.plot(0, 0, 'k+', markersize=10, mew=2)
    ax2.plot(0, 0, 'rx', markersize=10, mew=2)
    plt.tight_layout()
    plt.show()

class CalibrationApp:
    def __init__(self):
        self.app = QApplication(sys.argv)
        self.window = CompassMainWindow()
        self.window.start_calibration_signal.connect(self.start_calibration)
        self.window.view_result_signal.connect(self.view_result)
        self.serial_thread = None
        self.raw_mag_data = []  # [mx, my, mz, roll, pitch, yaw]
        self.calibrated = False
        self.calibration_params = None
        self.fig1 = None
        self.fig2 = None
        self.ax_original = None
        self.ax_projected = None
        self.ax_soft_iron = None
        self.ax_final = None
        self.original_plot = None
        self.projected_plot = None
        self._pending_mag = None  # 用于暂存mag行
        self._pending_euler = None # 用于暂存欧拉角

    def start_calibration(self, port, baudrate):
        try:
            if self.fig1 is not None:
                plt.close(self.fig1)
            if self.fig2 is not None:
                plt.close(self.fig2)
        except Exception:
            pass
        self.fig1 = None
        self.fig2 = None
        self.ax_original = None
        self.ax_projected = None
        self.ax_soft_iron = None
        self.ax_final = None
        self.original_plot = None
        self.projected_plot = None

        self.raw_mag_data.clear()
        self.calibrated = False
        self.calibration_params = None
        self._pending_euler = None  # 清空待处理的欧拉角数据
        self.window.clear_all_canvases()
        self.window.set_status("Calibrating...")
        self.serial_thread = SerialReader(port, baudrate, self.on_serial_data)
        self.serial_thread.start()
        self.calibration_start_time = time.time()
        # 创建两个画布：第一个显示原始数据和投影数据，第二个显示校准过程
        self.fig1, (self.ax_original, self.ax_projected) = plt.subplots(1, 2, figsize=(12, 5))
        self.fig2, (self.ax_soft_iron, self.ax_final) = plt.subplots(1, 2, figsize=(12, 5))
        
        # 第一个画布：原始数据和倾斜补偿数据
        self.ax_original.set_title("Original Data (mx, my)")
        self.ax_projected.set_title("Tilt Compensated Data")
        self.ax_original.axis('equal')
        self.ax_projected.axis('equal')
        
        # 第二个画布：校准过程
        self.ax_soft_iron.set_title("Soft Iron Calibrated")
        self.ax_final.set_title("Final Calibrated (Soft + Hard Iron)")
        self.ax_soft_iron.axis('equal')
        self.ax_final.axis('equal')
        
        # 实时绘图对象
        self.original_plot, = self.ax_original.plot([], [], 'r.', alpha=0.7)
        self.projected_plot, = self.ax_projected.plot([], [], 'b.', alpha=0.7)
        
        plt.tight_layout()
        self.fig1.show()
        self.fig2.show()
        threading.Thread(target=self._calibration_timer, daemon=True).start()

    def _calibration_timer(self):
        while time.time() - self.calibration_start_time < CALIBRATION_DURATION:
            time.sleep(0.1)
        if self.serial_thread:
            self.serial_thread.stop()
        self.perform_calibration()

    def extract_mag(self, mag_line):
        return [float(x) for x in re.findall(r'-?\d+\.?\d*', mag_line)]

    def map_mag_to_imu(self, mx, my, mz):
        mag = np.stack([mx, my, mz], axis=-1)
        mag_imu = mag @ np.array(MAG_AXIS_MAP).T
        return mag_imu[..., 0], mag_imu[..., 1], mag_imu[..., 2]

    def on_serial_data(self, line):
        try:
            # 适配你的数据格式：simples行、pitch行、mag_x行
            if not line.strip():
                return
            if line.startswith('simples:'):
                # 解析simples行：simples:62.463959,-1.012322,-90.374313,27.030861
                vals = re.findall(r'simples:([\-\d\.]+),([\-\d\.]+),([\-\d\.]+),([\-\d\.]+)', line)
                if vals:
                    pitch, roll, yaw, testyaw = map(float, vals[0])
                    self._pending_euler = (pitch, roll, yaw)
            elif line.startswith('pitch='):
                # 解析pitch行：pitch= 62.463959, roll= -1.012322,yaw= -90.374313, testyaw= 27.030861
                vals = re.findall(r'pitch=\s*([\-\d\.]+),\s*roll=\s*([\-\d\.]+),\s*yaw=\s*([\-\d\.]+)', line)
                if vals:
                    pitch, roll, yaw = map(float, vals[0])
                    self._pending_euler = (pitch, roll, yaw)
            elif line.startswith('mag_x='):
                # 解析mag_x行：mag_x=247, mag_y=137, mag_z=-57
                vals = re.findall(r'mag_x=\s*([\-\d\.]+),\s*mag_y=\s*([\-\d\.]+),\s*mag_z=\s*([\-\d\.]+)', line)
                if vals and self._pending_euler is not None:
                    mx, my, mz = map(float, vals[0])
                    pitch, roll, yaw = self._pending_euler
                    # 坐标系映射
                    mx, my, mz = self.map_mag_to_imu(np.array([mx]), np.array([my]), np.array([mz]))
                    mx, my, mz = mx[0], my[0], mz[0]
                    self.raw_mag_data.append([mx, my, mz, roll, pitch, yaw])
                    self._pending_euler = None
                    print(f"Data paired: mag=({mx:.1f}, {my:.1f}, {mz:.1f}), euler=({roll:.1f}, {pitch:.1f}, {yaw:.1f})")
                arr = np.array(self.raw_mag_data)
                if arr.shape[0] > 0 and self.original_plot is not None and self.projected_plot is not None:
                    # 显示原始数据（红色）
                    mx, my, mz, roll, pitch, yaw = arr.T
                    self.original_plot.set_data(mx, my)
                    
                    # 计算并显示倾斜补偿后的数据（蓝色）
                    roll_rad = np.radians(roll)
                    pitch_rad = np.radians(pitch)
                    mxh, myh = [], []
                    for i in range(len(mx)):
                        # 使用标准的倾斜补偿公式
                        cos_p = np.cos(pitch_rad[i])
                        sin_p = np.sin(pitch_rad[i])
                        cos_r = np.cos(roll_rad[i])
                        sin_r = np.sin(roll_rad[i])
                        
                        # 标准倾斜补偿公式（使用旋转矩阵）
                        # 先绕X轴旋转（pitch），再绕Y轴旋转（roll）
                        mx_comp = mx[i] * cos_p + mz[i] * sin_p
                        my_comp = mx[i] * sin_p * sin_r + my[i] * cos_r - mz[i] * cos_p * sin_r
                        
                        mxh.append(mx_comp)
                        myh.append(my_comp)
                    xy = np.stack([mxh, myh], axis=1)
                    self.projected_plot.set_data(xy[:, 0], xy[:, 1])
                    
                    # 显示统计信息
                    print(f"Total points: {len(arr)}")
                    print(f"Roll range: {roll.min():.1f}° to {roll.max():.1f}°")
                    print(f"Pitch range: {pitch.min():.1f}° to {pitch.max():.1f}°")
                    
                    # 调试信息：显示一些数据点的对比
                    if len(mx) > 10:
                        print(f"Sample data point 0:")
                        print(f"  Raw: mx={mx[0]:.1f}, my={my[0]:.1f}, mz={mz[0]:.1f}")
                        print(f"  Euler: roll={roll[0]:.1f}°, pitch={pitch[0]:.1f}°, yaw={yaw[0]:.1f}°")
                    
                    # 更新坐标轴 - 确保能看到形状差异
                    if self.ax_original is not None:
                        self.ax_original.relim()
                        self.ax_original.autoscale_view()
                        self.ax_original.set_aspect('equal', adjustable='box')
                    if self.ax_projected is not None:
                        self.ax_projected.relim()
                        self.ax_projected.autoscale_view()
                        self.ax_projected.set_aspect('equal', adjustable='box')
                    
                    # 刷新画布
                    if self.fig1 is not None:
                        self.fig1.canvas.draw_idle()
                        self.fig1.canvas.flush_events()
        except Exception as e:
            print(f"Serial data parsing error: {e}")
            print(f"Problematic line: {line.strip()}")

    def perform_calibration(self):
        self.window.set_status("Calibration finished. Processing data...")
        if len(self.raw_mag_data) < 10:
            self.window.set_status("Not enough data for calibration.")
            self.window.start_btn.setEnabled(True)
            self.window.port_combo.setEnabled(True)
            self.window.baud_combo.setEnabled(True)
            self.window.refresh_btn.setEnabled(True)
            return
        arr = np.array(self.raw_mag_data)
        mx, my, mz, roll, pitch, yaw = arr.T
        
        # 调试信息：显示欧拉角的统计
        print(f"Roll range: {roll.min():.2f} to {roll.max():.2f} degrees")
        print(f"Pitch range: {pitch.min():.2f} to {pitch.max():.2f} degrees")
        print(f"Yaw range: {yaw.min():.2f} to {yaw.max():.2f} degrees")
        
        # 倾角补偿（pitch/roll转弧度）
        roll_rad = np.radians(roll)
        pitch_rad = np.radians(pitch)
        mxh, myh = [], []
        for i in range(len(mx)):
            # 使用标准的倾斜补偿公式
            cos_p = np.cos(pitch_rad[i])
            sin_p = np.sin(pitch_rad[i])
            cos_r = np.cos(roll_rad[i])
            sin_r = np.sin(roll_rad[i])
            
            # 标准倾斜补偿公式（使用旋转矩阵）
            # 先绕X轴旋转（pitch），再绕Y轴旋转（roll）
            mx_comp = mx[i] * cos_p + mz[i] * sin_p
            my_comp = mx[i] * sin_p * sin_r + my[i] * cos_r - mz[i] * cos_p * sin_r
            
            mxh.append(mx_comp)
            myh.append(my_comp)
        xy = np.stack([mxh, myh], axis=1)
        # 用xy做椭圆拟合
        raw_center, _ = fit_circle_least_squares(xy)
        a, b = raw_center
        radius_x = (np.max(xy[:, 0] - a))
        radius_y = (np.max(xy[:, 1] - b))
        scale = radius_x / radius_y if radius_y != 0 else 1.0
        soft_calibrated = xy.copy()
        soft_calibrated[:, 0] = a + (soft_calibrated[:, 0] - a)
        soft_calibrated[:, 1] = b + (soft_calibrated[:, 1] - b) * scale
        center_after_soft, _ = fit_circle_least_squares(soft_calibrated)
        final_calibrated = soft_calibrated - center_after_soft
        dist = np.linalg.norm(final_calibrated, axis=1)
        median_dist = np.median(dist)
        std_dist = np.std(dist)
        mask = np.abs(dist - median_dist) < 3 * std_dist
        filtered_final_calibrated = final_calibrated[mask]
        filtered_xy = xy[mask]
        # Plotting - 更新第二个画布（校准过程）
        if self.ax_soft_iron is not None:
            self.ax_soft_iron.cla()
            self.ax_soft_iron.set_title("Soft Iron Calibrated")
            self.ax_soft_iron.axis('equal')
            self.ax_soft_iron.plot(soft_calibrated[:, 0], soft_calibrated[:, 1], 'r.', alpha=0.7)
            self.ax_soft_iron.plot(a, b, 'rx', markersize=10, mew=2)
            self.ax_soft_iron.plot(0, 0, 'k+', markersize=10, mew=2)
        if self.ax_final is not None:
            self.ax_final.cla()
            self.ax_final.set_title(f"Final Calibrated (Soft + Hard Iron)\nScale: {scale:.4f}")
            self.ax_final.axis('equal')
            self.ax_final.plot(filtered_final_calibrated[:, 0], filtered_final_calibrated[:, 1], 'g.', alpha=0.7)
            self.ax_final.plot(0, 0, 'rx', markersize=10, mew=2)
            self.ax_final.plot(0, 0, 'k+', markersize=10, mew=2)
        R = max(
            np.abs(filtered_xy[:, 0]).max(), np.abs(filtered_xy[:, 1]).max(),
            np.abs(soft_calibrated[:, 0]).max(), np.abs(soft_calibrated[:, 1]).max(),
            np.abs(filtered_final_calibrated[:, 0]).max(), np.abs(filtered_final_calibrated[:, 1]).max()
        )
        # 设置第一个画布的坐标轴
        for ax in [self.ax_original, self.ax_projected]:
            if ax is not None:
                ax.set_xlim(-R, R)
                ax.set_ylim(-R, R)
                ax.set_aspect('equal', adjustable='box')
        # 设置第二个画布的坐标轴
        for ax in [self.ax_soft_iron, self.ax_final]:
            if ax is not None:
                ax.set_xlim(-R, R)
                ax.set_ylim(-R, R)
                ax.set_aspect('equal', adjustable='box')
        # 刷新两个画布
        if self.fig1 is not None:
            self.fig1.canvas.draw_idle()
            self.fig1.canvas.flush_events()
        if self.fig2 is not None:
            self.fig2.canvas.draw_idle()
            self.fig2.canvas.flush_events()
        self.window.set_status("Calibration finished. Click View Result to see results.")
        self.window.enable_view_btn(True)
        self.calibration_params = (center_after_soft, scale, filtered_xy, filtered_final_calibrated)
        self.window.start_btn.setEnabled(True)
        self.window.port_combo.setEnabled(True)
        self.window.baud_combo.setEnabled(True)
        self.window.refresh_btn.setEnabled(True)

    def view_result(self):
        if not self.calibration_params:
            self.window.set_status("Please finish calibration first.")
            return
        center, scale, data, calibrated = self.calibration_params
        c_code = generate_c_code(center, scale)
        plot_two(data, calibrated, center, scale)
        self.window.show_result_dialog(c_code)

    def run(self):
        self.window.show()
        sys.exit(self.app.exec_())