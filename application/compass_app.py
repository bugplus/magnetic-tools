import sys
import serial
import threading
import time
import numpy as np
import re
from PyQt5.QtWidgets import QApplication
from compass_ui import CompassMainWindow

import matplotlib.pyplot as plt

CALIBRATION_DURATION = 30  # seconds

class SerialReader(threading.Thread):
    def __init__(self, port, baudrate, callback):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.callback = callback
        self.running = False

    def run(self):
        self.running = True
        try:
            ser = serial.Serial(self.port, self.baudrate, timeout=1)
            while self.running:
                line = ser.readline().decode(errors='ignore').strip()
                if 'mag_x' in line:
                    self.callback(line)
        except Exception as e:
            print("Serial error:", e)

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
        self.raw_mag_data = []
        self.calibrated = False
        self.calibration_params = None
        self.fig = None
        self.ax_raw = None
        self.ax_shifted = None
        self.ax_calibrated = None
        self.raw_plot = None

    def start_calibration(self, port, baudrate):
        try:
            if self.fig is not None:
                plt.close(self.fig)
        except Exception:
            pass
        self.fig = None
        self.ax_raw = None
        self.ax_shifted = None
        self.ax_calibrated = None
        self.raw_plot = None

        self.raw_mag_data.clear()
        self.calibrated = False
        self.calibration_params = None
        self.window.clear_all_canvases()
        self.window.set_status("Calibrating...")
        self.serial_thread = SerialReader(port, baudrate, self.on_serial_data)
        self.serial_thread.start()
        self.calibration_start_time = time.time()
        self.fig, (self.ax_raw, self.ax_shifted, self.ax_calibrated) = plt.subplots(1, 3, figsize=(15, 4))
        self.ax_raw.set_title("Raw Data")
        self.ax_shifted.set_title("Soft Iron Calibrated")
        self.ax_calibrated.set_title("Final Calibrated")
        self.ax_raw.axis('equal')
        self.ax_shifted.axis('equal')
        self.ax_calibrated.axis('equal')
        self.raw_plot, = self.ax_raw.plot([], [], 'b.', alpha=0.7)
        plt.tight_layout()
        plt.show(block=False)
        threading.Thread(target=self._calibration_timer, daemon=True).start()

    def _calibration_timer(self):
        while time.time() - self.calibration_start_time < CALIBRATION_DURATION:
            time.sleep(0.1)
        if self.serial_thread:
            self.serial_thread.stop()
        self.perform_calibration()

    def extract_mag(self, mag_line):
        return [float(x) for x in re.findall(r'-?\d+\.?\d*', mag_line)]

    def on_serial_data(self, mag_line):
        mag_values = self.extract_mag(mag_line)
        if len(mag_values) >= 2:
            self.raw_mag_data.append(mag_values[:2])
            if self.ax_raw is not None and self.raw_plot is not None:
                data = np.array(self.raw_mag_data)
                self.raw_plot.set_data(data[:, 0], data[:, 1])
                self.ax_raw.relim()
                self.ax_raw.autoscale_view()
                if self.fig is not None:
                    self.fig.canvas.draw_idle()
                    self.fig.canvas.flush_events()

    def perform_calibration(self):
        self.window.set_status("Calibration finished. Processing data...")
        if len(self.raw_mag_data) < 10:
            self.window.set_status("Not enough data for calibration.")
            self.window.start_btn.setEnabled(True)
            self.window.port_combo.setEnabled(True)
            self.window.baud_combo.setEnabled(True)
            self.window.refresh_btn.setEnabled(True)
            return
        data = np.array(self.raw_mag_data)
        # 1. Fit center of raw data
        raw_center, _ = fit_circle_least_squares(data)
        a, b = raw_center
        # 2. Soft iron: scale y about the center (not the origin)
        radius_x = (np.max(data[:, 0] - a))
        radius_y = (np.max(data[:, 1] - b))
        scale = radius_x / radius_y if radius_y != 0 else 1.0
        soft_calibrated = data.copy()
        soft_calibrated[:, 0] = a + (soft_calibrated[:, 0] - a)
        soft_calibrated[:, 1] = b + (soft_calibrated[:, 1] - b) * scale
        # 3. Fit center after scaling, then shift to origin (hard iron)
        center_after_soft, _ = fit_circle_least_squares(soft_calibrated)
        final_calibrated = soft_calibrated - center_after_soft
        # 过滤偏离圆心过大的异常点
        dist = np.linalg.norm(final_calibrated, axis=1)
        median_dist = np.median(dist)
        std_dist = np.std(dist)
        mask = np.abs(dist - median_dist) < 3 * std_dist
        filtered_final_calibrated = final_calibrated[mask]
        filtered_data = data[mask]
        # Plotting
        if self.ax_shifted is not None:
            self.ax_shifted.cla()
            self.ax_shifted.set_title("Soft Iron Calibrated (center unchanged)")
            self.ax_shifted.axis('equal')
            self.ax_shifted.plot(soft_calibrated[:, 0], soft_calibrated[:, 1], 'r.', alpha=0.7)
            self.ax_shifted.plot(a, b, 'rx', markersize=10, mew=2)  # Center should be unchanged
            self.ax_shifted.plot(0, 0, 'k+', markersize=10, mew=2)
        if self.ax_calibrated is not None:
            self.ax_calibrated.cla()
            self.ax_calibrated.set_title(f"Final Calibrated\nScale: {scale:.4f}")
            self.ax_calibrated.axis('equal')
            self.ax_calibrated.plot(filtered_final_calibrated[:, 0], filtered_final_calibrated[:, 1], 'g.', alpha=0.7)
            self.ax_calibrated.plot(0, 0, 'rx', markersize=10, mew=2)  # Center now at origin
            self.ax_calibrated.plot(0, 0, 'k+', markersize=10, mew=2)
        if self.ax_raw is not None:
            self.ax_raw.plot(0, 0, 'k+', markersize=10, mew=2)
            self.ax_raw.plot(a, b, 'rx', markersize=10, mew=2)
        # 设置三个子图的坐标范围和比例一致，消除视觉误差
        R = max(
            np.abs(filtered_data[:, 0]).max(), np.abs(filtered_data[:, 1]).max(),
            np.abs(soft_calibrated[:, 0]).max(), np.abs(soft_calibrated[:, 1]).max(),
            np.abs(filtered_final_calibrated[:, 0]).max(), np.abs(filtered_final_calibrated[:, 1]).max()
        )
        for ax in [self.ax_raw, self.ax_shifted, self.ax_calibrated]:
            if ax is not None:
                ax.set_xlim(-R, R)
                ax.set_ylim(-R, R)
                ax.set_aspect('equal', adjustable='box')
        if self.fig is not None:
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
        self.window.set_status("Calibration finished. Click View Result to see results.")
        self.window.enable_view_btn(True)
        self.calibration_params = (center_after_soft, scale, filtered_data, filtered_final_calibrated)
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