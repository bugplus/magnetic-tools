# compass_app.py
import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import config
import time

class CompassApp:
    def __init__(self, port, baud_rate):
        self.port = port
        self.baud_rate = baud_rate
        self.ser = None
        self.raw_data = []  # 原始数据
        self.scale_x = 1.0  # X轴缩放因子
        self.scale_y = 1.0  # Y轴缩放因子
        self.center_x = 0.0  # X轴中心偏移
        self.center_y = 0.0  # Y轴中心偏移
        
        # 三视图画布相关
        self.fig = None
        self.ax1 = None  # 图1: 原始数据
        self.ax2 = None  # 图2: 缩放数据
        self.ax3 = None  # 图3: 校准数据
        self.scatter1 = None
        self.scatter2 = None
        self.scatter3 = None
        self.ani = None
        self.data_collection_active = False
        self.calibration_complete = False

    def connect_serial(self):
        """连接串口"""
        try:
            self.ser = serial.Serial(
                self.port, 
                self.baud_rate, 
                timeout=config.TIMEOUT
            )
            print(f"已连接串口: {self.port}@{self.baud_rate}波特")
            return True
        except Exception as e:
            print(f"串口连接失败: {e}")
            return False

    def start_data_collection(self):
        """启动数据收集和三视图显示"""
        # 重置状态
        self.raw_data = []
        self.data_collection_active = True
        self.calibration_complete = False
        
        # 创建三视图画布 (1行3列)
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(1, 3, figsize=(18, 6))
        plt.get_current_fig_manager().set_window_title("磁力计数据收集")
        
        # 设置图1: 原始数据
        self.ax1.set_title("原始数据 (图1)")
        self.ax1.set_xlabel("磁力计 X")
        self.ax1.set_ylabel("磁力计 Y")
        self.ax1.grid(True)
        self.scatter1 = self.ax1.scatter([], [], c='blue', s=15, alpha=0.6)
        
        # 设置图2: 缩放数据 (初始为空)
        self.ax2.set_title("缩放数据 (图2)")
        self.ax2.set_xlabel("缩放 X")
        self.ax2.set_ylabel("缩放 Y")
        self.ax2.grid(True)
        self.scatter2 = self.ax2.scatter([], [], c='green', s=15, alpha=0.6)
        
        # 设置图3: 校准数据 (初始为空)
        self.ax3.set_title("校准数据 (图3)")
        self.ax3.set_xlabel("校准 X")
        self.ax3.set_ylabel("校准 Y")
        self.ax3.grid(True)
        self.scatter3 = self.ax3.scatter([], [], c='red', s=15, alpha=0.6)
        
        # 设置所有图的坐标轴范围
        for ax in [self.ax1, self.ax2, self.ax3]:
            ax.set_xlim(-300, 300)
            ax.set_ylim(-300, 300)
            ax.axis('equal')
        
        plt.tight_layout()
        
        # 启动动画（只更新图1）
        self.ani = FuncAnimation(
            self.fig, 
            self.update_plot, 
            interval=config.UPDATE_INTERVAL, 
            cache_frame_data=False,
            blit=False
        )
        
        plt.show(block=False)

    def update_plot(self, frame):
        """更新三视图（校准期间只更新图1）"""
        if not self.data_collection_active or not self.ser or not self.ser.is_open:
            return self.scatter1, self.scatter2, self.scatter3
            
        try:
            # 从串口读取数据
            while self.ser.in_waiting:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                
                # 解析格式: mag_x=123, mag_y=456
                if 'mag_x' in line and 'mag_y' in line:
                    parts = line.split(',')
                    if len(parts) >= 2:
                        x_str = parts[0].split('=')[1].strip()
                        y_str = parts[1].split('=')[1].strip()
                        
                        try:
                            x = float(x_str)
                            y = float(y_str)
                            self.raw_data.append((x, y))
                            
                            # 限制数据点数量
                            if len(self.raw_data) > config.MAX_POINTS:
                                self.raw_data.pop(0)
                        except ValueError:
                            pass
        except serial.SerialException:
            return self.scatter1, self.scatter2, self.scatter3
        
        # 更新图1（原始数据）
        if self.raw_data:
            xs, ys = zip(*self.raw_data)
            self.scatter1.set_offsets(np.column_stack([xs, ys]))
            
            # 自动调整图1的坐标轴范围
            x_min, x_max = min(xs), max(xs)
            y_min, y_max = min(ys), max(ys)
            margin = max((x_max - x_min) * 0.1, (y_max - y_min) * 0.1, 50)
            self.ax1.set_xlim(x_min - margin, x_max + margin)
            self.ax1.set_ylim(y_min - margin, y_max + margin)
        
        return self.scatter1, self.scatter2, self.scatter3

    def stop_data_collection(self):
        """停止数据收集"""
        self.data_collection_active = False
        if self.ani:
            self.ani.event_source.stop()
            print("数据收集已停止")
    
    def calibrate_data(self):
        """计算校准参数并更新图2和图3"""
        if len(self.raw_data) < 10:
            print("校准失败: 数据不足")
            return False
        
        # 提取X/Y数据
        xs = [x for x, _ in self.raw_data]
        ys = [y for _, y in self.raw_data]
        
        # 计算范围
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)
        
        # 计算缩放因子（关键算法）
        x_range = x_max - x_min
        y_range = y_max - y_min
        
        if x_range > y_range:
            self.scale_x = 1.0
            self.scale_y = x_range / y_range
        else:
            self.scale_x = y_range / x_range
            self.scale_y = 1.0
        
        # 计算中心点（关键算法）
        scaled_xs = [x * self.scale_x for x in xs]
        scaled_ys = [y * self.scale_y for y in ys]
        
        self.center_x = (max(scaled_xs) + min(scaled_xs)) / 2
        self.center_y = (max(scaled_ys) + min(scaled_ys)) / 2
        
        # 更新图2（缩放数据）
        self.scatter2.set_offsets(np.column_stack([scaled_xs, scaled_ys]))
        
        # 更新图3（校准数据）
        calibrated_xs = [x - self.center_x for x in scaled_xs]
        calibrated_ys = [y - self.center_y for y in scaled_ys]
        self.scatter3.set_offsets(np.column_stack([calibrated_xs, calibrated_ys]))
        
        # 添加圆心标记（所有视图）
        self.ax1.scatter([(self.center_x / self.scale_x)], [self.center_y / self.scale_y], 
                         c='black', s=80, marker='x')
        self.ax2.scatter([self.center_x], [self.center_y], 
                         c='black', s=80, marker='x')
        self.ax3.scatter([0], [0], 
                         c='black', s=80, marker='x')
        
        # 调整坐标轴范围
        self.ax2.set_xlim(min(scaled_xs)-50, max(scaled_xs)+50)
        self.ax2.set_ylim(min(scaled_ys)-50, max(scaled_ys)+50)
        self.ax3.set_xlim(min(calibrated_xs)-50, max(calibrated_xs)+50)
        self.ax3.set_ylim(min(calibrated_ys)-50, max(calibrated_ys)+50)
        
        # 重绘图2和图3
        self.fig.canvas.draw_idle()
        
        print(f"校准参数计算完成: X缩放={self.scale_x:.4f}, Y缩放={self.scale_y:.4f}")
        print(f"中心点: X={self.center_x:.2f}, Y={self.center_y:.2f}")
        
        self.calibration_complete = True
        return True

    def stop(self):
        """完全停止所有操作"""
        self.stop_data_collection()
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("串口连接已关闭")
        if self.fig:
            plt.close(self.fig)