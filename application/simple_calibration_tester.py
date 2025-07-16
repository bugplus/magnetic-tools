import sys
import serial
import threading
import time
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QTextEdit, QComboBox, QFileDialog
)
from PyQt5.QtCore import pyqtSignal, QObject
import re
import math
from collections import deque
import csv
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

# ========== 串口扫描工具 ==========
def list_serial_ports():
    import serial.tools.list_ports
    return [port.device for port in serial.tools.list_ports.comports()]

# ========== 倾角补偿 ==========
def tilt_compensate(mx, my, mz, pitch, roll):
    mx_comp = mx * math.cos(pitch) + mz * math.sin(pitch)
    my_comp = mx * math.sin(roll) * math.sin(pitch) + my * math.cos(roll) - mz * math.sin(roll) * math.cos(pitch)
    return mx_comp, my_comp

# ========== 二维椭圆拟合 ==========
def fit_circle_least_squares(xy):
    x = xy[:, 0]
    y = xy[:, 1]
    A = np.c_[2*x, 2*y, np.ones(x.shape)]
    b = x**2 + y**2
    c, resid, rank, s = np.linalg.lstsq(A, b, rcond=None)
    center_x, center_y = c[0], c[1]
    radius = np.sqrt(c[2] + center_x**2 + center_y**2)
    return np.array([center_x, center_y]), radius

def calibrate_2d_ellipse(xy):
    # 软铁：y轴缩放到x轴半径
    radius_x = (np.max(xy[:, 0]) - np.min(xy[:, 0])) / 2
    radius_y = (np.max(xy[:, 1]) - np.min(xy[:, 1])) / 2
    scale = radius_x / radius_y if radius_y != 0 else 1.0
    soft_calibrated = xy.copy()
    soft_calibrated[:, 1] *= scale
    # 硬铁：拟合圆心，平移到原点
    center, _ = fit_circle_least_squares(soft_calibrated)
    calibrated = soft_calibrated - center
    return calibrated, center, scale

# ========== heading计算 ==========
def calculate_heading(mx, my):
    heading_rad = math.atan2(-my, mx)
    heading_deg = math.degrees(heading_rad)
    if heading_deg < 0:
        heading_deg += 360
    return heading_deg

def heading_array(mx, my):
    headings = np.degrees(np.arctan2(-my, mx))
    headings[headings < 0] += 360
    return headings

# ========== 串口读取线程 ==========
class SerialReader(QObject, threading.Thread):
    data_signal = pyqtSignal(str)
    def __init__(self, port, baudrate, callback):
        QObject.__init__(self)
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
                    line = ser.readline().decode(errors='ignore').strip()
                    self.callback(line)  # 任何行都传递
                except Exception as e:
                    print("Serial read error:", e)
        except Exception as e:
            print("Serial error:", e)
    def stop(self):
        self.running = False

# ========== 主界面 ==========
class SimpleCalibrationTester(QWidget):
    append_text_signal = pyqtSignal(str)
    DATA_COUNT_THRESHOLD = 100
    def __init__(self):
        super().__init__()
        self.setWindowTitle("简易地磁二维校准验证工具")
        self.resize(500, 400)
        self.main_layout = QVBoxLayout()
        self.setLayout(self.main_layout)

        # 串口选择
        hbox = QHBoxLayout()
        self.port_combo = QComboBox()
        self.refresh_btn = QPushButton("刷新串口")
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["115200", "9600", "57600", "38400"])
        hbox.addWidget(QLabel("串口:"))
        hbox.addWidget(self.port_combo)
        hbox.addWidget(self.refresh_btn)
        hbox.addWidget(QLabel("波特率:"))
        hbox.addWidget(self.baud_combo)
        self.main_layout.addLayout(hbox)

        # 按钮
        btn_box = QHBoxLayout()
        self.btn_level = QPushButton("采集水平数据")
        self.btn_30deg = QPushButton("采集30度数据")
        self.btn_test = QPushButton("测试")
        btn_box.addWidget(self.btn_level)
        btn_box.addWidget(self.btn_30deg)
        btn_box.addWidget(self.btn_test)
        self.main_layout.addLayout(btn_box)

        # 状态与结果
        self.status_label = QLabel("状态: 等待操作")
        self.main_layout.addWidget(self.status_label)
        self.result_text = QTextEdit()
        self.result_text.setReadOnly(True)
        self.main_layout.addWidget(self.result_text)

        # 数据缓存
        self.data_level = []
        self.data_30deg = []
        self.serial_thread = None
        self.collecting = None  # 'level' or '30deg'
        self._pending_mag = None  # 用于暂存mag行
        self.data_log = []  # 新增：用于存储最近N条原始数据
        self.mag_queue = deque()
        self.pitch_queue = deque()
        self.append_text_signal.connect(self._append_data_text)

        # 新增：实时数据显示区
        self.data_text = QTextEdit()
        self.data_text.setReadOnly(True)
        self.main_layout.addWidget(QLabel("串口实时数据："))
        self.main_layout.addWidget(self.data_text)

        # 新增：mag_x/mag_y实时散点图
        self.fig, self.ax = plt.subplots(figsize=(3,3))
        self.canvas = FigureCanvas(self.fig)
        self.main_layout.addWidget(QLabel("mag_x/mag_y实时分布："))
        self.main_layout.addWidget(self.canvas)
        self.scatter_plot = None

        # 新增：导出数据按钮
        export_box = QHBoxLayout()
        self.btn_export_level = QPushButton("导出水平数据")
        self.btn_export_30deg = QPushButton("导出30度数据")
        self.btn_export_level.setEnabled(False)
        self.btn_export_30deg.setEnabled(False)
        export_box.addWidget(self.btn_export_level)
        export_box.addWidget(self.btn_export_30deg)
        self.main_layout.addLayout(export_box)
        self.btn_export_level.clicked.connect(self.export_level_data)
        self.btn_export_30deg.clicked.connect(self.export_30deg_data)

        # 事件绑定
        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.btn_level.clicked.connect(self.collect_level)
        self.btn_30deg.clicked.connect(self.collect_30deg)
        self.btn_test.clicked.connect(self.run_test)
        self.btn_30deg.setEnabled(False)
        self.btn_test.setEnabled(False)
        self.refresh_ports()

    def refresh_ports(self):
        self.port_combo.clear()
        ports = list_serial_ports()
        self.port_combo.addItems(ports)

    def collect_level(self):
        self.data_level.clear()
        self.collecting = 'level'
        self.btn_level.setStyleSheet("background-color: yellow")
        self.btn_level.setEnabled(False)
        self.btn_30deg.setEnabled(False)
        self.btn_test.setEnabled(False)
        self.start_serial_collection()
        self.status_label.setText("状态: 采集水平数据中，请水平旋转1-2圈...")

    def collect_30deg(self):
        self.data_30deg.clear()
        self.collecting = '30deg'
        self.btn_30deg.setStyleSheet("background-color: yellow")
        self.btn_30deg.setEnabled(False)
        self.btn_test.setEnabled(False)
        self.start_serial_collection()
        self.status_label.setText("状态: 采集30度倾斜数据中，请倾斜30°旋转1-2圈...")

    def start_serial_collection(self):
        if self.serial_thread:
            self.serial_thread.stop()
            time.sleep(0.2)
        port = self.port_combo.currentText()
        baud = int(self.baud_combo.currentText())
        self.serial_thread = SerialReader(port, baud, self.on_serial_data)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        # 移除自动3秒停采逻辑
        # threading.Thread(target=self._auto_stop, daemon=True).start()

    def _auto_stop(self):
        time.sleep(3)
        if self.serial_thread:
            self.serial_thread.stop()
        self.status_label.setText("状态: 采集完成，可进行下一步")

    def export_level_data(self):
        self.save_data_to_csv(self.data_level, "水平数据.csv")

    def export_30deg_data(self):
        self.save_data_to_csv(self.data_30deg, "30度数据.csv")

    def save_data_to_csv(self, data, filename_hint):
        path, _ = QFileDialog.getSaveFileName(self, "保存数据", filename_hint, "CSV Files (*.csv)")
        if path:
            with open(path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['mag_x', 'mag_y', 'mag_z', 'pitch', 'roll'])
                writer.writerows(data)

    def update_scatter(self, data):
        self.ax.cla()
        if len(data) > 0:
            arr = np.array(data)
            self.ax.scatter(arr[:,0], arr[:,1], s=10, label='mag_x/mag_y')
            self.ax.set_title('mag_x/mag_y')
            self.ax.axis('equal')
            self.ax.legend()
        self.canvas.draw_idle()

    def polar_bin_filter(self, arr, bins=36):
        # 极坐标分箱，保留每个方向最远的点
        if len(arr) == 0:
            return arr
        x, y = arr[:,0], arr[:,1]
        theta = np.arctan2(y, x)
        r = np.sqrt(x**2 + y**2)
        bin_edges = np.linspace(-np.pi, np.pi, bins+1)
        idx = np.digitize(theta, bin_edges) - 1
        keep = []
        for b in range(bins):
            mask = (idx == b)
            if np.any(mask):
                i = np.argmax(r[mask])
                keep.append(np.where(mask)[0][i])
        return arr[keep]

    def check_distribution_uniform(self, arr, bins=36, min_bins=36, min_radius=20):  # 调小阈值
        if len(arr) == 0:
            return False
        x, y = arr[:,0], arr[:,1]
        theta = np.arctan2(y, x)
        r = np.sqrt(x**2 + y**2)
        bin_edges = np.linspace(-np.pi, np.pi, bins+1)
        idx = np.digitize(theta, bin_edges) - 1
        covered_bins = set(idx[(r > min_radius)])
        return len(covered_bins) >= min_bins

    def get_covered_bins(self, arr, bins=36):
        if len(arr) == 0:
            return 0
        x, y = arr[:,0], arr[:,1]
        theta = np.arctan2(y, x)
        bin_edges = np.linspace(-np.pi, np.pi, bins+1)
        idx = np.digitize(theta, bin_edges) - 1
        return len(np.unique(idx))

    def on_serial_data(self, line):
        try:
            self.append_text_signal.emit(line)  # 用信号发到主线程
            if not line.strip() or line.strip() == 'mag':
                return
            if 'mag_x' in line:
                vals = [float(x) for x in re.findall(r'-?\d+\.?\d*', line)]
                if len(vals) == 3:
                    self.mag_queue.append(vals)
            elif 'pitch' in line:
                vals = [float(x) for x in re.findall(r'-?\d+\.?\d*', line)]
                if len(vals) >= 2:
                    self.pitch_queue.append(vals)
            # 队列配对采集
            while self.mag_queue and self.pitch_queue:
                try:
                    mag_vals = self.mag_queue.popleft()
                    pitch_vals = self.pitch_queue.popleft()
                    mx, my, mz = mag_vals
                    pitch, roll = pitch_vals[0], pitch_vals[1]
                    row = [mx, my, mz, pitch, roll]
                    if self.collecting == 'level':
                        self.data_level.append(row)
                        self.update_scatter(self.data_level)
                        arr = np.array(self.data_level)
                        filtered = self.polar_bin_filter(arr[:, :2])
                        covered_bins = self.get_covered_bins(filtered)
                        self.status_label.setText(f"已采集{len(self.data_level)}条，已覆盖方向{covered_bins}/36")
                        if self.check_distribution_uniform(filtered, bins=36, min_bins=36):
                            if self.serial_thread:
                                self.serial_thread.stop()
                            self.btn_30deg.setEnabled(True)
                            self.btn_level.setEnabled(True)
                            self.btn_export_level.setEnabled(True)
                            self.collecting = None
                    elif self.collecting == '30deg':
                        self.data_30deg.append(row)
                        self.update_scatter(self.data_30deg)
                        arr = np.array(self.data_30deg)
                        filtered = self.polar_bin_filter(arr[:, :2])
                        covered_bins = self.get_covered_bins(filtered)
                        self.status_label.setText(f"已采集{len(self.data_30deg)}条，已覆盖方向{covered_bins}/36")
                        if self.check_distribution_uniform(filtered, bins=36, min_bins=36):
                            if self.serial_thread:
                                self.serial_thread.stop()
                            self.btn_test.setEnabled(True)
                            self.btn_30deg.setEnabled(True)
                            self.btn_export_30deg.setEnabled(True)
                            self.collecting = None
                except Exception as e:
                    self.append_text_signal.emit(f"[配对异常] {e}")
                    print("Pairing error:", e)
                    import traceback
                    traceback.print_exc()
        except Exception as e:
            self.append_text_signal.emit(f"[解析异常] {e}")
            print("Data parse error:", e)
            import traceback
            traceback.print_exc()

    def _append_data_text(self, line):
        import datetime
        now = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
        self.data_text.append(f"[{now}] {line}")
        # 限制显示最近100条
        doc = self.data_text.document()
        if doc.blockCount() > 100:
            cursor = self.data_text.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.select(cursor.LineUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()

    def collect_level_enable(self):
        self.btn_level.setEnabled(True)
        self.btn_level.setStyleSheet("")

    def collect_30deg_enable(self):
        self.btn_30deg.setEnabled(True)
        self.btn_30deg.setStyleSheet("")

    def run_test(self):
        try:
            if len(self.data_level) < 30 or len(self.data_30deg) < 30:
                self.result_text.setText("请先采集足够的水平和30度数据（每组建议不少于30条）！")
                return
            # 1. 倾角补偿
            data1 = np.array(self.data_level)
            data2 = np.array(self.data_30deg)
            mx1, my1, mz1, pitch1, roll1 = data1.T
            mx2, my2, mz2, pitch2, roll2 = data2.T
            mxh1, myh1 = [], []
            for i in range(len(mx1)):
                x, y = tilt_compensate(mx1[i], my1[i], mz1[i], pitch1[i], roll1[i])
                mxh1.append(x)
                myh1.append(y)
            mxh2, myh2 = [], []
            for i in range(len(mx2)):
                x, y = tilt_compensate(mx2[i], my2[i], mz2[i], pitch2[i], roll2[i])
                mxh2.append(x)
                myh2.append(y)
            mxh = np.concatenate([mxh1, mxh2])
            myh = np.concatenate([myh1, myh2])
            xy = np.stack([mxh, myh], axis=1)
            # 2. 二维椭圆拟合
            calibrated, center, scale = calibrate_2d_ellipse(xy)
            # 3. 分组heading
            n1 = len(mxh1)
            n2 = len(mxh2)
            cal1 = calibrated[:n1]
            cal2 = calibrated[n1:]
            heading1 = heading_array(cal1[:,0], cal1[:,1])
            heading2 = heading_array(cal2[:,0], cal2[:,1])
            mean1 = np.mean(heading1)
            mean2 = np.mean(heading2)
            heading_drift = abs(mean1 - mean2)
            # 4. 输出结果
            result = f"水平均值: {mean1:.2f}°\n30度均值: {mean2:.2f}°\nheading_drift: {heading_drift:.2f}°\n"
            if heading_drift < 3:
                result += "\n结论：适合二维校准量产！"
            else:
                result += "\n结论：建议升级三轴椭球拟合或优化安装！"
            self.result_text.setText(result)
        except Exception as e:
            self.result_text.setText(f"测试异常: {e}")
            import traceback
            traceback.print_exc()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = SimpleCalibrationTester()
    win.show()
    sys.exit(app.exec_()) 