# compass_app.py
import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import config

class CompassApp:
    def __init__(self):
        self.raw_data = []
        self.ser = None
        self.calibration_done = False
        self.data_collection_started = False
        self.start_time = None
        self.scale_x = self.scale_y = 1.0
        self.center_x = self.center_y = 0.0

        # 初始化串口
        self.port = config.PORT
        self.baud_rate = config.BAUD_RATE
        self.connect_serial()

        # 初始化绘图
        self.init_plot()

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=config.TIMEOUT)
            print(f"[INFO] 已连接到串口 {self.port}")
        except Exception as e:
            print(f"[ERROR] 无法打开串口 {self.port}，错误：{e}")

    def init_plot(self):
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(1, 3, figsize=(18, 6))
        self.line1, = self.ax1.plot([], [], 'r.', markersize=3)
        self.line2, = self.ax2.plot([], [], 'g.', markersize=3)
        self.line3, = self.ax3.plot([], [], 'b.', markersize=3)
        self.ax1.set_title("Raw Magnetometer X-Y Data\n(Collecting for Calibration)")
        self.ax2.set_title("After Scaling Only")
        self.ax3.set_title("Fully Calibrated")
        self.ax1.set_xlabel("mag_x")
        self.ax1.set_ylabel("mag_y")
        self.ax2.set_xlabel("mag_x (scaled)")
        self.ax2.set_ylabel("mag_y (scaled)")
        self.ax3.set_xlabel("mag_x (calibrated)")
        self.ax3.set_ylabel("mag_y (calibrated)")
        self.ax1.axhline(0, color='black', lw=0.5)
        self.ax1.axvline(0, color='black', lw=0.5)
        self.ax2.axhline(0, color='black', lw=0.5)
        self.ax2.axvline(0, color='black', lw=0.5)
        self.ax3.axhline(0, color='black', lw=0.5)
        self.ax3.axvline(0, color='black', lw=0.5)
        self.ax1.grid(True)
        self.ax2.grid(True)
        self.ax3.grid(True)
        self.ax1.axis('equal')
        self.ax2.axis('equal')
        self.ax3.axis('equal')
        self.ax1.set_xlim(-300, 300)
        self.ax1.set_ylim(-300, 300)
        self.ax2.set_xlim(-300, 300)
        self.ax2.set_ylim(-300, 300)
        self.ax3.set_xlim(-300, 300)
        self.ax3.set_ylim(-300, 300)
        self.ani = FuncAnimation(self.fig, self.update, frames=None, interval=config.UPDATE_INTERVAL, blit=False, cache_frame_data=False)
        plt.tight_layout()
        plt.show()

    def update(self, frame):
        if self.ser and self.ser.is_open and self.data_collection_started and not self.calibration_done:
            while self.ser.in_waiting:
                line_str = self.ser.readline().decode('utf-8', errors='replace').strip()
                try:
                    data = line_str.split(',')
                    if len(data) >= 2:
                        x = int(data[0].split('=')[1])
                        y = int(data[1].split('=')[1])
                        if (x, y) not in self.raw_data:
                            self.raw_data.append((x, y))
                            if len(self.raw_data) > config.MAX_POINTS:
                                self.raw_data.pop(0)
                            if self.data_collection_started and (time.time() - self.start_time <= config.CALIBRATION_DURATION):
                                print(f"Received Data: mag_x={x}, mag_y={y}")
                            if not self.data_collection_started:
                                self.start_time = time.time()
                                self.data_collection_started = True
                except Exception as e:
                    print(f"[ERROR] 数据解析失败: {e}")
                    continue

        if len(self.raw_data) >= 2 and not self.calibration_done:
            xs = np.array([x[0] for x in self.raw_data])
            ys = np.array([y[1] for y in self.raw_data])
            self.line1.set_data(xs, ys)
            x_min, x_max = min(xs), max(xs)
            y_min, y_max = min(ys), max(ys)
            margin = 50
            self.ax1.set_xlim(x_min - margin, x_max + margin)
            self.ax1.set_ylim(y_min - margin, y_max + margin)

        if self.data_collection_started and not self.calibration_done and (time.time() - self.start_time > config.CALIBRATION_DURATION):
            self.calibrate_data()
            self.calibration_done = True
            print("[INFO] 校准完成，已切换至校准图")

        return self.line1, self.line2, self.line3

    def calibrate_data(self):
        if len(self.raw_data) >= 6:
            xs = np.array([x[0] for x in self.raw_data])
            ys = np.array([y[1] for y in self.raw_data])
            x_min, x_max = min(xs), max(xs)
            y_min, y_max = min(ys), max(ys)
            margin = 50
            self.x_range_final = (x_min - margin, x_max + margin)
            self.y_range_final = (y_min - margin, y_max + margin)
            x_range = x_max - x_min
            y_range = y_max - y_min
            if x_range > y_range:
                self.scale_x = 1.0
                self.scale_y = x_range / y_range
            else:
                self.scale_x = y_range / x_range
                self.scale_y = 1.0
            scaled_xs = xs * self.scale_x
            scaled_ys = ys * self.scale_y
            self.center_x = (max(scaled_xs) + min(scaled_xs)) / 2
            self.center_y = (max(scaled_ys) + min(scaled_ys)) / 2
            calibrated_xs = scaled_xs - self.center_x
            calibrated_ys = scaled_ys - self.center_y
            self.line2.set_data(scaled_xs, scaled_ys)
            self.ax2.set_title(f"After Scaling Only\n(Scale: x={self.scale_x:.3f}, y={self.scale_y:.3f})")
            self.line3.set_data(calibrated_xs, calibrated_ys)
            self.ax3.set_title(f"Fully Calibrated\n(Offset: ({self.center_x:.1f}, {self.center_y:.1f}), Scale: x={self.scale_x:.3f}, y={self.scale_y:.3f})")
            self.ax2.set_xlim(min(scaled_xs) - 50, max(scaled_xs) + 50)
            self.ax2.set_ylim(min(scaled_ys) - 50, max(scaled_ys) + 50)
            self.ax3.set_xlim(min(calibrated_xs) - 50, max(calibrated_xs) + 50)
            self.ax3.set_ylim(min(calibrated_ys) - 50, max(calibrated_ys) + 50)
            self.ax2.plot(self.center_x, self.center_y, 'ro')
            self.ax3.plot(0, 0, 'ro')
            self.ax1.plot(self.center_x, self.center_y, 'ro')

    def get_calibrated_data(self):
        if self.calibration_done:
            xs = np.array([x[0] for x in self.raw_data])
            ys = np.array([y[1] for y in self.raw_data])
            scaled_xs = xs * self.scale_x
            scaled_ys = ys * self.scale_y
            calibrated_xs = scaled_xs - self.center_x
            calibrated_ys = scaled_ys - self.center_y
            return list(zip(calibrated_xs, calibrated_ys))
        else:
            return []

    def stop_serial(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[INFO] 串口已关闭")