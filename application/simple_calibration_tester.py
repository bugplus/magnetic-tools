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
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas

# ========== 串口扫描工具 ==========
def list_serial_ports():
    import serial.tools.list_ports
    return [port.device for port in serial.tools.list_ports.comports()]

# ========== 倾角补偿 ==========
def tilt_compensate(mx, my, mz, pitch, roll):
    # 标准三轴磁力计倾角补偿
    mx_comp = mx * np.cos(pitch) + mz * np.sin(pitch)
    my_comp = mx * np.sin(roll) * np.sin(pitch) + my * np.cos(roll) - mz * np.sin(roll) * np.cos(pitch)
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
    scale = radius_x / radius_y if radius_y != 0 else 10
    soft_calibrated = xy.copy()
    soft_calibrated[:, 1] *= scale
    # 硬铁：拟合圆心，平移到原点
    center, _ = fit_circle_least_squares(soft_calibrated)
    calibrated = soft_calibrated - center
    return calibrated, center, scale

# ========== heading计算 ==========
def heading_schemes(mx, my):
    import math
    # 返回所有常见heading方案
    h1 = (math.degrees(math.atan2(-my, mx)) + 360) % 360
    h2 = (math.degrees(math.atan2(my, mx)) + 360) % 360
    h3 = (math.degrees(math.atan2(mx, my)) + 360) % 360
    h4 = (math.degrees(math.atan2(-my, -mx)) + 360) % 360
    h5 = (math.degrees(math.atan2(my, -mx)) + 360) % 360
    return h1, h2, h3, h4, h5

def calculate_heading(mx, my):
    # 默认用方案1
    return (math.degrees(math.atan2(-my, mx)) + 360) % 360

def heading_array(mx, my):
    headings = np.degrees(np.arctan2(-my, mx))
    headings[headings < 0] += 360
    return headings

def fit_ellipse(xy):
    cov = np.cov(xy.T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    order = np.argsort(eigvals)[::-1]
    eigvals = eigvals[order]
    eigvecs = eigvecs[:, order]
    theta = np.arctan2(eigvecs[0,0], eigvecs[0,1])
    a = np.sqrt(eigvals[0])
    b = np.sqrt(eigvals[1])
    return theta, a, b

def rotate(xy, theta):
    R = np.array([[np.cos(theta), -np.sin(theta)],
                [np.sin(theta),  np.cos(theta)]])
    return xy @ R.T

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
                    line = ser.readline().decode(errors='ignore')
                    self.callback(line)  # 任何行都传递
                except Exception as e:
                    print(f"Serial read error: {e}")
        except Exception as e:
            print(f"Serial error: {e}")
    def stop(self):
        self.running = False

# ========== 主界面 ==========
class SimpleCalibrationTester(QWidget):
    append_text_signal = pyqtSignal(str)
    DATA_COUNT_THRESHOLD = 100
    def __init__(self):
        super().__init__()
        self.setWindowTitle("简易地磁二维校准验证工具")
        self.resize(500,400)
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
        self.btn_30eg = QPushButton("采集30度数据")
        self.btn_test = QPushButton("测试")
        btn_box.addWidget(self.btn_level)
        btn_box.addWidget(self.btn_30eg)
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
        self.data_30 = []
        self.serial_thread = None
        self.collecting = None  # 'level' or '30eg'
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
        self.btn_export_30eg = QPushButton("导出30度数据")
        self.btn_export_level.setEnabled(False)
        self.btn_export_30eg.setEnabled(False)
        export_box.addWidget(self.btn_export_level)
        export_box.addWidget(self.btn_export_30eg)
        self.main_layout.addLayout(export_box)
        self.btn_export_level.clicked.connect(self.export_level_data)
        self.btn_export_30eg.clicked.connect(self.export_30deg_data)

        # 事件绑定
        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.btn_level.clicked.connect(self.collect_level)
        self.btn_30eg.clicked.connect(self.collect_30)
        self.btn_test.clicked.connect(self.run_test)
        self.btn_30eg.setEnabled(False)
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
        self.btn_30eg.setEnabled(False)
        self.btn_test.setEnabled(False)
        self.start_serial_collection()
        self.status_label.setText("状态: 采集水平数据中，请水平旋转1")

    def collect_30(self):
        self.data_30.clear()
        self.collecting = '30'
        self.btn_30eg.setStyleSheet("background-color: yellow")
        self.btn_30eg.setEnabled(False)
        self.btn_test.setEnabled(False)
        self.start_serial_collection()
        self.status_label.setText("状态: 采集30度倾斜数据中，请倾斜301.")

    def start_serial_collection(self):
        if self.serial_thread:
            self.serial_thread.stop()
            time.sleep(0.2)
        port = self.port_combo.currentText()
        baud = int(self.baud_combo.currentText())
        self.serial_thread = SerialReader(port, baud, self.on_serial_data)
        self.serial_thread.daemon = True
        self.serial_thread.start()

    def export_level_data(self):
        self.save_data_to_csv(self.data_level, "水平数据.csv")

    def export_30deg_data(self):
        self.save_data_to_csv(self.data_30, "30度数据.csv")

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

    def check_distribution_uniform(self, arr, bins=36, min_bins=36, min_radius=20):
        if len(arr) == 0:
            return False
        x, y = arr[:,0], arr[:,1]
        theta = np.arctan2(y, x)
        r = np.sqrt(x**2 + y**2)
        bin_edges = np.linspace(-np.pi, np.pi, bins+1)
        idx = np.digitize(theta, bin_edges) -1
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
            self.append_text_signal.emit(line)
            if not line.strip():
                return
            # 新格式：mag_x=...在前，roll=...在后
            if line.startswith('mag_x='):
                vals = re.findall(r'mag_x=\s*([\-\d\.]+),\s*mag_y=\s*([\-\d\.]+),\s*mag_z=\s*([\-\d\.]+)', line)
                if vals:
                    mx, my, mz = map(float, vals[0])
                    self._pending_mag = (mx, my, mz)
            elif line.startswith('roll='):
                vals = re.findall(r'roll=\s*([\-\d\.]+),\s*pitch=\s*([\-\d\.]+)', line)
                if vals and hasattr(self, '_pending_mag') and self._pending_mag is not None:
                    roll, pitch = map(float, vals[0])
                    mx, my, mz = self._pending_mag
                    row = (mx, my, mz, pitch, roll)
                    if self.collecting == 'level':
                        self.data_level.append(row)
                        self.update_scatter(self.data_level)
                        arr = np.array(self.data_level)
                        # 放宽条件：只要采集到10条就允许测试
                        if len(self.data_level) >= 10:
                            self.btn_test.setEnabled(True)
                        filtered = self.polar_bin_filter(arr[:, :2])
                        covered_bins = self.get_covered_bins(filtered)
                        self.status_label.setText(f"已采集{len(self.data_level)}条，已覆盖方向{covered_bins}/36")
                        if self.check_distribution_uniform(filtered, bins=36):
                            if self.serial_thread:
                                self.serial_thread.stop()
                            self.btn_30eg.setEnabled(True)
                            self.btn_level.setEnabled(True)
                            self.btn_export_level.setEnabled(True)
                            self.collecting = None
                    elif self.collecting == '30':
                        self.data_30.append(row)
                        self.update_scatter(self.data_30)
                        arr = np.array(self.data_30)
                        if len(self.data_30) >= 10:
                            self.btn_test.setEnabled(True)
                        filtered = self.polar_bin_filter(arr[:, :2])
                        covered_bins = self.get_covered_bins(filtered)
                        self.status_label.setText(f"已采集{len(self.data_30)}条，已覆盖方向{covered_bins}/36")
                        if self.check_distribution_uniform(filtered, bins=36):
                            if self.serial_thread:
                                self.serial_thread.stop()
                            self.btn_test.setEnabled(True)
                            self.btn_30eg.setEnabled(True)
                            self.btn_export_30eg.setEnabled(True)
                            self.collecting = None
                    self._pending_mag = None  # 用完清空
        except Exception as e:
            self.append_text_signal.emit(f"[解析异常] {e}")
            print("Data parse error:", e)
            import traceback
            traceback.print_exc()

    def _append_data_text(self, line):
        import datetime
        now = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.data_text.append(f"[{now}] {line}")
        # 限制显示最近100
        doc = self.data_text.document()
        if doc and doc.blockCount() > 100:
            cursor = self.data_text.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.select(cursor.LineUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()

    def run_test(self):
        try:
            if len(self.data_level) < 30:
                self.result_text.setText("请先采集至少30条旋转数据！")
                return

            data = np.array(self.data_level)
            mx, my, mz, pitch, roll = data.T
            pitch = np.radians(pitch)
            roll = np.radians(roll)

            # 倾角补偿
            mxh, myh = [], []
            for i in range(len(mx)):
                x, y = tilt_compensate(mx[i], my[i], mz[i], pitch[i], roll[i])
                mxh.append(x)
                myh.append(y)
            mxh = np.array(mxh)
            myh = np.array(myh)

            # 椭圆拟合
            xy = np.stack([mxh, myh], axis=1)
            theta, a, b = fit_ellipse(xy)
            xy_rot = rotate(xy, -theta)
            xy_calib = xy_rot.copy()
            xy_calib[:,1] *= a / b
            center = np.mean(xy_calib, axis=0)
            hard_iron_offset = np.sqrt(center[0]**2 + center[1]**2)
            soft_iron_ratio = a / b

            # 调试输出
            result = f"=== 旋转校准结果 ===\n"
            result += f"采集数据量: {len(mx)}\n"
            result += f"软铁比例(a/b): {soft_iron_ratio:.3f}\n"
            result += f"硬铁偏移(center/长轴): {hard_iron_offset/a:.3f}\n"
            result += f"椭圆参数: a={a:.2f}, b={b:.2f}, theta={np.degrees(theta):.2f}°\n"
            result += f"硬铁偏移: center=({center[0]:.2f}, {center[1]:.2f}), 大小={hard_iron_offset:.2f}\n"

            # 分档评估
            if 0.75 < soft_iron_ratio < 1.3 and hard_iron_offset/a < 0.3:
                result += "\n结论：环境优秀，适合做软铁/硬铁校准！"
            elif 0.6 < soft_iron_ratio < 1.5 and hard_iron_offset/a < 0.5:
                result += "\n结论：环境良好，校准后可用。"
            else:
                result += "\n结论：环境有较强干扰，校准效果可能受影响。"

            self.result_text.setText(result)
        except Exception as e:
            self.result_text.setText(f"校准异常: {e}")
            import traceback
            traceback.print_exc()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = SimpleCalibrationTester()
    win.show()
    sys.exit(app.exec_()) 