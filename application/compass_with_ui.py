import sys
import serial
from serial.tools import list_ports
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox
from PyQt5.QtCore import QTimer

# ===================== 配置区域 =====================
PORT = 'COM4'               # 默认串口号
BAUD_RATE = 115200          # 波特率
TIMEOUT = 1                 # 串口超时时间
UPDATE_INTERVAL = 50        # 更新间隔（毫秒）
MAX_POINTS = 600            # 最大数据点数
CALIBRATION_DURATION = 60   # 采集持续时间（秒）
TOLERANCE_THRESHOLD = 1     # 点的容忍阈值，单位：像素
# ===================================================

class CompassApp:
    def __init__(self):
        self.raw_data = []
        self.ser = None
        self.calibration_done = False
        self.data_collection_started = False
        self.start_time = None
        self.scale_x = self.scale_y = 1.0
        self.center_x = self.center_y = 0.0
        self.x_range_final = self.y_range_final = None

        # 初始化串口
        self.port = PORT
        self.baud_rate = BAUD_RATE
        self.connect_serial()

        # 初始化绘图
        self.init_plot()

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=TIMEOUT)
            print(f"[INFO] 已连接到串口 {self.port}")
        except Exception as e:
            print(f"[ERROR] 无法打开串口 {self.port}，错误：{e}")

    def init_plot(self):
        # 创建三个画布
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(1, 3, figsize=(18, 6))

        # 图1：原始数据
        self.line1, = self.ax1.plot([], [], 'r.', markersize=3)
        self.ax1.set_title("Raw Magnetometer X-Y Data\n(Collecting for Calibration)")
        self.ax1.set_xlabel("mag_x")
        self.ax1.set_ylabel("mag_y")
        self.ax1.axhline(0, color='black', lw=0.5)
        self.ax1.axvline(0, color='black', lw=0.5)
        self.ax1.grid(True)
        self.ax1.axis('equal')
        self.ax1.set_xlim(-300, 300)
        self.ax1.set_ylim(-300, 300)

        # 图2：缩放后
        self.line2, = self.ax2.plot([], [], 'g.', markersize=3)
        self.ax2.set_title("After Scaling Only\n(Scale: x=?, y=?)")
        self.ax2.set_xlabel("mag_x (scaled)")
        self.ax2.set_ylabel("mag_y (scaled)")
        self.ax2.axhline(0, color='black', lw=0.5)
        self.ax2.axvline(0, color='black', lw=0.5)
        self.ax2.grid(True)
        self.ax2.axis('equal')

        # 图3：校准后
        self.line3, = self.ax3.plot([], [], 'b.', markersize=3)
        self.ax3.set_title("Fully Calibrated\n(Offset: (0.0, 0.0), Scale: x=1.000, y=1.000)")
        self.ax3.set_xlabel("mag_x (calibrated)")
        self.ax3.set_ylabel("mag_y (calibrated)")
        self.ax3.axhline(0, color='black', lw=0.5)
        self.ax3.axvline(0, color='black', lw=0.5)
        self.ax3.grid(True)
        self.ax3.axis('equal')

        # 启动动画
        self.ani = FuncAnimation(self.fig, self.update, frames=None,
                                 interval=UPDATE_INTERVAL, blit=False, cache_frame_data=False)
        plt.tight_layout()
        plt.show()

    def update(self, frame):
        current_time = time.time()

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

                            if self.data_collection_started and (current_time - self.start_time <= CALIBRATION_DURATION):
                                print(f"Received Data: mag_x={x}, mag_y={y}")

                            if not self.data_collection_started:
                                self.start_time = current_time
                                self.data_collection_started = True

                except Exception as e:
                    print(f"[ERROR] 数据解析失败: {e}")
                    continue

        # 更新原始数据图（前30秒持续更新）
        if len(self.raw_data) >= 2 and not self.calibration_done:
            xs = np.array([x[0] for x in self.raw_data])
            ys = np.array([y[1] for y in self.raw_data])
            self.line1.set_data(xs, ys)

            x_min, x_max = min(xs), max(xs)
            y_min, y_max = min(ys), max(ys)
            margin = 50
            self.ax1.set_xlim(x_min - margin, x_max + margin)
            self.ax1.set_ylim(y_min - margin, y_max + margin)

        # 如果已经开始采集，并且还没校准完成
        if self.data_collection_started and not self.calibration_done and (current_time - self.start_time > CALIBRATION_DURATION):
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

            # 计算缩放比例，确保椭圆变正圆
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
            self.ax3.set_title(f"Fully Calibrated\n(Offset: ({self.center_x:.1f}, {self.center_y:.1f}), "
                                f"Scale: x={self.scale_x:.3f}, y={self.scale_y:.3f})")

            # 动态设置坐标范围
            self.ax2.set_xlim(min(scaled_xs) - 50, max(scaled_xs) + 50)
            self.ax2.set_ylim(min(scaled_ys) - 50, max(scaled_ys) + 50)
            self.ax3.set_xlim(min(calibrated_xs) - 50, max(calibrated_xs) + 50)
            self.ax3.set_ylim(min(calibrated_ys) - 50, max(calibrated_ys) + 50)

            # 绘制椭圆中心
            self.ax2.plot(self.center_x, self.center_y, 'ro')
            self.ax3.plot(0, 0, 'ro')  # 在图三绘制圆心在原点
            self.ax1.plot(self.center_x, self.center_y, 'ro')  # 在图一也绘制圆心

    def stop_serial(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[INFO] 串口已关闭")

class CompassUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Compass Viewer with Control Panel")

        # UI 元素
        self.port_combo = QComboBox()
        self.baud_combo = QComboBox()
        self.start_button = QPushButton("Start")
        self.stop_button = QPushButton("Stop")
        self.stop_button.setEnabled(False)
        self.cleanup_timer = None  # 定时器用于延迟清理

        self.initUI()
        self.refresh_ports()

    def initUI(self):
        layout = QVBoxLayout()

        # Port selection
        port_layout = QHBoxLayout()
        port_layout.addWidget(QLabel("Serial Port:"))
        port_layout.addWidget(self.port_combo)

        # Baud rate selection
        baud_layout = QHBoxLayout()
        baud_layout.addWidget(QLabel("Baud Rate:"))
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.baud_combo.setCurrentText("115200")
        baud_layout.addWidget(self.baud_combo)

        # Buttons
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.start_button)
        button_layout.addWidget(self.stop_button)

        # Connect signals
        self.start_button.clicked.connect(self.start_plotting)
        self.stop_button.clicked.connect(self.stop_plotting)

        # Add to main layout
        layout.addLayout(port_layout)
        layout.addLayout(baud_layout)
        layout.addLayout(button_layout)
        self.setLayout(layout)

    def refresh_ports(self):
        self.port_combo.clear()  # 清空当前的串口号
        ports = list_ports.comports()
        for port in ports:
            self.port_combo.addItem(port.device)

    def start_plotting(self):
        port = self.port_combo.currentText()
        baud_rate = int(self.baud_combo.currentText())

        # 如果已有定时器正在运行，则先停止它
        if self.cleanup_timer and self.cleanup_timer.isActive():
            self.cleanup_timer.stop()

        # 如果已有实例，先清空
        if hasattr(self, 'app_instance'):
            del self.app_instance

        # 创建新实例
        self.app_instance = CompassApp()
        self.app_instance.port = port
        self.app_instance.baud_rate = baud_rate
        self.app_instance.connect_serial()

        # 只在第一次启动时调用 init_plot
        if not hasattr(self.app_instance, 'fig'):
            self.app_instance.init_plot()

        self.app_instance.data_collection_started = True
        self.app_instance.start_time = time.time()

        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)

        # 设置一个定时器，在 60 秒后自动停止
        self.cleanup_timer = QTimer(self)
        self.cleanup_timer.setSingleShot(True)
        self.cleanup_timer.timeout.connect(self.stop_plotting)
        self.cleanup_timer.start(CALIBRATION_DURATION * 1000)

    def stop_plotting(self):
        if hasattr(self, 'app_instance'):
            self.app_instance.stop_serial()
            self.app_instance.calibration_done = True  # 标记为校准完成
            self.app_instance.calibrate_data()  # 执行校准
            print("[INFO] 串口已暂停")

        # 立即禁用 Stop，启用 Start（等待定时器完成后再清空）
        self.stop_button.setEnabled(False)
        self.start_button.setEnabled(True)
        print("[INFO] Buttons updated: Stop disabled, Start enabled immediately.")

        if self.cleanup_timer and self.cleanup_timer.isActive():
            self.cleanup_timer.stop()

    def clear_app_instance(self):
        if hasattr(self, 'app_instance'):
            del self.app_instance
            print("[INFO] 数据实例已释放")

        # 更新按钮状态
        self.stop_button.setEnabled(False)
        self.start_button.setEnabled(True)
        print("[INFO] Buttons updated: Stop disabled, Start enabled after timer.")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = CompassUI()
    window.resize(400, 200)
    window.show()
    sys.exit(app.exec_())