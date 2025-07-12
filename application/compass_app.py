# compass_app.py
import math
import numpy as np
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import config
import time

class CompassApp:
    def __init__(self, port, baud_rate):
        self.port = port
        self.baud_rate = baud_rate
        self.ser = None
        self.raw_data = []  # 原始数据: (mag_x, mag_y, mag_z, pitch, roll)
        self.scale_x = 1.0  # X轴缩放因子
        self.scale_y = 1.0  # Y轴缩放因子
        self.center_x = 0.0  # X轴中心偏移
        self.center_y = 0.0  # Y轴中心偏移
        self.partial_data = {}  # 用于存储部分解析的数据
        
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

    def connect_serial(self, connect_only=False):
        """连接串口，增加connect_only参数"""
        try:
            self.ser = serial.Serial(
                self.port, 
                self.baud_rate, 
                timeout=config.TIMEOUT
            )
            print(f"已连接串口: {self.port}@{self.baud_rate}波特")
            return True
        except Exception as e:
            if not connect_only:
                print(f"串口连接失败: {e}")
            return False
    def start_data_collection(self):
        """启动数据收集和三视图显示"""
        # 重置状态
        self.raw_data = []
        self.partial_data = {}
        self.data_collection_active = True
        self.calibration_complete = False
        
        # 创建三视图画布 (1行3列)
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(1, 3, figsize=(18, 6))
        plt.get_current_fig_manager().set_window_title("mag-compass data collector")
        
        # 设置图1: 原始数据
        self.ax1.set_title("Raw Data (Fig1)")
        self.ax1.set_xlabel("Mag X")
        self.ax1.set_ylabel("Mag Y")
        self.ax1.grid(True)
        self.scatter1 = self.ax1.scatter([], [], c='blue', s=15, alpha=0.6)
        
        # 设置图2: 缩放数据 (初始为空)
        self.ax2.set_title("Scaled Data (Fig2)")
        self.ax2.set_xlabel("Scaled X")
        self.ax2.set_ylabel("Scaled Y")
        self.ax2.grid(True)
        self.scatter2 = self.ax2.scatter([], [], c='green', s=15, alpha=0.6)
        
        # 设置图3: 校准数据 (初始为空)
        self.ax3.set_title("Calibrated Data (Fig3)")
        self.ax3.set_xlabel("Calibrated X")
        self.ax3.set_ylabel("Calibrated Y")
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

    def tilt_compensation(self, mag_x, mag_y, mag_z, pitch, roll):
        """倾角补偿：将磁力计数据从载体坐标系转换到水平坐标系"""
        # 将角度转换为弧度
        pitch_rad = math.radians(pitch)
        roll_rad = math.radians(roll)
        
        # 计算旋转矩阵分量
        cos_pitch = math.cos(pitch_rad)
        sin_pitch = math.sin(pitch_rad)
        cos_roll = math.cos(roll_rad)
        sin_roll = math.sin(roll_rad)
        
        # 计算补偿后的磁力值
        mag_x_comp = mag_x * cos_pitch + mag_z * sin_pitch
        mag_y_comp = mag_x * sin_roll * sin_pitch + mag_y * cos_roll - mag_z * sin_roll * cos_pitch
        
        return mag_x_comp, mag_y_comp

    def update_plot(self, frame):
        """更新三视图（校准期间只更新图1）"""
        if not self.data_collection_active or not self.ser or not self.ser.is_open:
            return self.scatter1, self.scatter2, self.scatter3
            
        try:
            # 从串口读取数据
            while self.ser.in_waiting:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                
                # 跳过空行
                if not line:
                    continue
                
                # 解析数据格式
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
                            # 忽略无法转换的值
                            pass
                
                # 处理磁力计数据行
                if 'mag_x' in data_dict and 'mag_y' in data_dict and 'mag_z' in data_dict:
                    self.partial_data['mag_x'] = data_dict['mag_x']
                    self.partial_data['mag_y'] = data_dict['mag_y']
                    self.partial_data['mag_z'] = data_dict['mag_z']
                
                # 处理姿态数据行
                if 'pitch' in data_dict and 'roll' in data_dict:
                    self.partial_data['pitch'] = data_dict['pitch']
                    self.partial_data['roll'] = data_dict['roll']
                
                # 检查是否已收集到完整数据
                if all(key in self.partial_data for key in ['mag_x', 'mag_y', 'mag_z', 'pitch', 'roll']):
                    # 存储完整数据
                    self.raw_data.append((
                        self.partial_data['mag_x'],
                        self.partial_data['mag_y'],
                        self.partial_data['mag_z'],
                        self.partial_data['pitch'],
                        self.partial_data['roll']
                    ))
                    
                    # 限制数据点数量
                    if len(self.raw_data) > config.MAX_POINTS:
                        self.raw_data.pop(0)
                    
                    # 重置部分数据
                    self.partial_data = {}
                    
        except Exception as e:
            print(f"数据处理错误: {e}")
            return self.scatter1, self.scatter2, self.scatter3
        
        # 更新图1（原始数据）
        if self.raw_data:
            # 提取XY数据（原始磁力计XY值）
            xs = [x for x, y, z, p, r in self.raw_data]
            ys = [y for x, y, z, p, r in self.raw_data]
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
        
        # 提取所有数据
        xs = []
        ys = []
        
        # 进行倾角补偿
        for mag_x, mag_y, mag_z, pitch, roll in self.raw_data:
            # 应用倾角补偿
            x_comp, y_comp = self.tilt_compensation(mag_x, mag_y, mag_z, pitch, roll)
            xs.append(x_comp)
            ys.append(y_comp)
        
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