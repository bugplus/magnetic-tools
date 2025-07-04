# compass_core.py

import serial
import time
import numpy as np
from config import PORT, BAUD_RATE, TIMEOUT, MAX_POINTS, CALIBRATION_DURATION

class CompassCore:
    def __init__(self):
        self.raw_data = []
        self.ser = None
        self.calibration_done = False
        self.data_collection_started = False
        self.start_time = None
        self.scale_x = self.scale_y = 1.0
        self.center_x = self.center_y = 0.0
        self.x_range_final = self.y_range_final = None

    def connect_serial(self, port=PORT, baud_rate=BAUD_RATE):
        try:
            self.ser = serial.Serial(port, baud_rate, timeout=TIMEOUT)
            print(f"[INFO] 已连接到串口 {port}")
        except Exception as e:
            print(f"[ERROR] 无法打开串口 {port}，错误：{e}")

    def disconnect_serial(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[INFO] 串口已关闭")

    def collect_data(self):
        if self.ser and self.ser.is_open and self.data_collection_started and not self.calibration_done:
            while self.ser.in_waiting:
                line_str = self.ser.readline().decode('utf-8', errors='replace').strip()
                try:
                    data = line_str.split(',')
                    if len(data) >= 2:
                        x = int(data[0].split('=')[1])
                        y = int(data[1].split('=')[1])
                        self.raw_data.append((x, y))
                        if len(self.raw_data) > MAX_POINTS:
                            self.raw_data.pop(0)

                        if self.data_collection_started and (time.time() - self.start_time <= CALIBRATION_DURATION):
                            print(f"Received Data: mag_x={x}, mag_y={y}")

                except Exception as e:
                    print(f"[ERROR] 数据解析失败: {e}")
                    continue

    def calibrate(self):
        if self.data_collection_started and not self.calibration_done and (time.time() - self.start_time > CALIBRATION_DURATION):
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

                if x_range >= y_range:
                    self.scale_x = 1.0
                    self.scale_y = x_range / y_range
                else:
                    self.scale_x = y_range / x_range
                    self.scale_y = 1.0

                scaled_xs = xs * self.scale_x
                scaled_ys = ys * self.scale_y

                self.center_x = (max(scaled_xs) + min(scaled_xs)) / 2
                self.center_y = (max(scaled_ys) + min(scaled_ys)) / 2

                self.calibration_done = True
                print("[INFO] 校准完成，已切换至校准图")