# compass_core.py
import serial
import numpy as np
import time
from config import MAX_POINTS, CALIBRATION_DURATION

class CompassApp:
    def __init__(self, port, baud_rate):
        self.raw_data = []
        self.ser = None
        self.calibration_done = False
        self.data_collection_started = False
        self.start_time = None
        self.scale_x = self.scale_y = 1.0
        self.center_x = self.center_y = 0.0

        # 初始化串口
        self.port = port
        self.baud_rate = baud_rate
        self.connect_serial()

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
            print(f"[INFO] 已连接到串口 {self.port}")
        except Exception as e:
            print(f"[ERROR] 无法打开串口 {self.port}，错误：{e}")

    def collect_data(self):
        if self.ser and self.ser.is_open and self.data_collection_started and not self.calibration_done:
            while self.ser.in_waiting:
                line_str = self.ser.readline().decode('utf-8', errors='replace').strip()
                try:
                    data = line_str.split(',')
                    if len(data) >= 2:
                        x = int(data[0].split('=')[1])
                        y = int(data[1].split('=')[1])
                        # 检查点是否已存在，如果存在则忽略
                        if (x, y) not in self.raw_data:
                            self.raw_data.append((x, y))
                            if len(self.raw_data) > MAX_POINTS:
                                self.raw_data.pop(0)

                            if self.data_collection_started and (time.time() - self.start_time <= CALIBRATION_DURATION):
                                print(f"Received Data: mag_x={x}, mag_y={y}")

                            if not self.data_collection_started:
                                self.start_time = time.time()
                                self.data_collection_started = True

                except Exception as e:
                    print(f"[ERROR] 数据解析失败: {e}")
                    continue
                print(f"Collected data: {line_str}")  # Debug print

    def calibrate_data(self):
        if len(self.raw_data) >= 6:
            xs = np.array([x[0] for x in self.raw_data])
            ys = np.array([y[1] for y in self.raw_data])

            x_min, x_max = min(xs), max(xs)
            y_min, y_max = min(ys), max(ys)

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

            calibrated_xs = scaled_xs - self.center_x
            calibrated_ys = scaled_ys - self.center_y

            return calibrated_xs, calibrated_ys

    def stop_serial(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[INFO] 串口已关闭")