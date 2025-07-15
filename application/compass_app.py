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

def calibrate_magnetometer_simple(xy):
    x_min, x_max = np.min(xy[:, 0]), np.max(xy[:, 0])
    y_min, y_max = np.min(xy[:, 1]), np.max(xy[:, 1])
    center = np.array([(x_max + x_min) / 2, (y_max + y_min) / 2])
    shifted = xy - center
    radius_x = (x_max - x_min) / 2
    radius_y = (y_max - y_min) / 2
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

def plot_two(raw_data, calibrated):
    plt.figure(figsize=(10, 4))
    plt.subplot(1, 2, 1)
    plt.title("Raw Data")
    plt.axis('equal')
    if raw_data is not None and len(raw_data) > 0:
        plt.plot(raw_data[:, 0], raw_data[:, 1], 'b.', alpha=0.7)
    plt.subplot(1, 2, 2)
    plt.title("Calibrated Data")
    plt.axis('equal')
    if calibrated is not None and len(calibrated) > 0:
        plt.plot(calibrated[:, 0], calibrated[:, 1], 'g.', alpha=0.7)
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
        # Close previous figure if exists
        if self.fig is not None:
            plt.close(self.fig)
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
        # Open one figure with three subplots
        self.fig, (self.ax_raw, self.ax_shifted, self.ax_calibrated) = plt.subplots(1, 3, figsize=(15, 4))
        self.ax_raw.set_title("Raw Data")
        self.ax_shifted.set_title("Hard Iron Calibrated")
        self.ax_calibrated.set_title("Final Calibrated")
        self.ax_raw.axis('equal')
        self.ax_shifted.axis('equal')
        self.ax_calibrated.axis('equal')
        self.raw_plot, = self.ax_raw.plot([], [], 'b.', alpha=0.7)
        self.ax_shifted.cla()
        self.ax_shifted.set_title("Hard Iron Calibrated")
        self.ax_shifted.axis('equal')
        self.ax_calibrated.cla()
        self.ax_calibrated.set_title("Final Calibrated")
        self.ax_calibrated.axis('equal')
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
                self.fig.canvas.draw_idle()
                self.fig.canvas.flush_events()

    def perform_calibration(self):
        self.window.set_status("Calibration finished. Processing data...")
        if len(self.raw_mag_data) < 10:
            self.window.set_status("Not enough data for calibration.")
            return
        data = np.array(self.raw_mag_data)
        shifted, calibrated, center, scale = calibrate_magnetometer_simple(data)
        # Update shifted and calibrated plots
        self.ax_shifted.cla()
        self.ax_shifted.set_title("Hard Iron Calibrated")
        self.ax_shifted.axis('equal')
        if shifted is not None and len(shifted) > 0:
            self.ax_shifted.plot(shifted[:, 0], shifted[:, 1], 'r.', alpha=0.7)
        self.ax_calibrated.cla()
        self.ax_calibrated.set_title("Final Calibrated")
        self.ax_calibrated.axis('equal')
        if calibrated is not None and len(calibrated) > 0:
            self.ax_calibrated.plot(calibrated[:, 0], calibrated[:, 1], 'g.', alpha=0.7)
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        self.window.set_status("Calibration finished. Click View Result to see results.")
        self.window.enable_view_btn(True)
        self.calibration_params = (center, scale, data, calibrated)

    def view_result(self):
        if not self.calibration_params:
            self.window.set_status("Please finish calibration first.")
            return
        center, scale, data, calibrated = self.calibration_params
        c_code = generate_c_code(center, scale)
        plot_two(data, calibrated)
        self.window.show_result_dialog(c_code)

    def run(self):
        self.window.show()
        sys.exit(self.app.exec_())