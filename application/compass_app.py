# compass_app.py
# compass_app.py
import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import config

class CompassApp:
    def __init__(self, port=None, baud_rate=None):
        self.raw_data = []
        self.ser = None
        self.calibration_done = False
        self.data_collection_started = False
        self.start_time = None
        self.scale_x = self.scale_y = 1.0
        self.center_x = self.center_y = 0.0
        
        # 使用传入参数或默认配置
        self.port = port or config.PORT
        self.baud_rate = baud_rate or config.BAUD_RATE
        
        # 初始化绘图
        self.init_plot()

    def connect_serial(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=config.TIMEOUT)
            print(f"Connected to {self.port} at {self.baud_rate} baud")
        except serial.SerialException as e:
            print(f"Failed to open {self.port}: {e}")

    def init_plot(self):
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(1, 3, figsize=(18, 6))
        
        # 初始化散点图
        self.scatter1 = self.ax1.scatter([], [], c='red', s=10)
        self.scatter2 = self.ax2.scatter([], [], c='green', s=10)
        self.scatter3 = self.ax3.scatter([], [], c='blue', s=10)
        
        # 设置图表属性
        titles = [
            "Raw Magnetometer X-Y Data\n(Collecting for Calibration)",
            "After Scaling Only",
            "Fully Calibrated"
        ]
        for ax, title in zip([self.ax1, self.ax2, self.ax3], titles):
            ax.set_title(title)
            ax.set_xlabel("mag_x")
            ax.set_ylabel("mag_y")
            ax.axhline(0, color='black', lw=0.5)
            ax.axvline(0, color='black', lw=0.5)
            ax.grid(True)
            ax.axis('equal')
            ax.set_xlim(-300, 300)
            ax.set_ylim(-300, 300)
        
        # 启动动画
        self.ani = FuncAnimation(
            self.fig, 
            self.update, 
            interval=config.UPDATE_INTERVAL, 
            cache_frame_data=False
        )
        plt.tight_layout()
        plt.show(block=False)

    def update(self, frame):
        if self.ser and self.ser.is_open and self.data_collection_started:
            while self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode('utf-8').strip()
                    if not line:
                        continue
                    
                    # 解析数据格式：mag_x=123, mag_y=456
                    parts = line.split(',')
                    if len(parts) >= 2:
                        x = int(parts[0].split('=')[1])
                        y = int(parts[1].split('=')[1])
                        self.raw_data.append((x, y))
                        
                        # 限制数据点数量
                        if len(self.raw_data) > config.MAX_POINTS:
                            self.raw_data.pop(0)
                except Exception as e:
                    print(f"Data error: {e}")
        
        # 更新图表
        if self.raw_data:
            xs, ys = zip(*self.raw_data)
            
            # 更新原始数据图
            self.scatter1.set_offsets(np.column_stack([xs, ys]))
            
            # 更新校准后视图
            if self.calibration_done:
                scaled_xs = [x * self.scale_x for x in xs]
                scaled_ys = [y * self.scale_y for y in ys]
                calibrated_xs = [x - self.center_x for x in scaled_xs]
                calibrated_ys = [y - self.center_y for y in scaled_ys]
                
                self.scatter2.set_offsets(np.column_stack([scaled_xs, scaled_ys]))
                self.scatter3.set_offsets(np.column_stack([calibrated_xs, calibrated_ys]))
                
                # 更新坐标轴范围
                self.ax2.set_xlim(min(scaled_xs)-50, max(scaled_xs)+50)
                self.ax2.set_ylim(min(scaled_ys)-50, max(scaled_ys)+50)
                self.ax3.set_xlim(min(calibrated_xs)-50, max(calibrated_xs)+50)
                self.ax3.set_ylim(min(calibrated_ys)-50, max(calibrated_ys)+50)

        return self.scatter1, self.scatter2, self.scatter3

    def calibrate_data(self):
        if len(self.raw_data) < 6:
            print("Need at least 6 points for calibration")
            return
        
        xs = [x for x, _ in self.raw_data]
        ys = [y for _, y in self.raw_data]
        
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)
        
        x_range = x_max - x_min
        y_range = y_max - y_min
        
        if x_range > y_range:
            self.scale_x = 1.0
            self.scale_y = x_range / y_range
        else:
            self.scale_x = y_range / x_range
            self.scale_y = 1.0
        
        scaled_xs = [x * self.scale_x for x in xs]
        scaled_ys = [y * self.scale_y for y in ys]
        
        self.center_x = (max(scaled_xs) + min(scaled_xs)) / 2
        self.center_y = (max(scaled_ys) + min(scaled_ys)) / 2
        self.calibration_done = True

        print(f"Calibration complete: scale_x={self.scale_x:.2f}, scale_y={self.scale_y:.2f}")
        print(f"Center: x={self.center_x:.2f}, y={self.center_y:.2f}")

    def stop_serial(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.data_collection_started = False
        print("Serial connection closed")