import sys
import serial
import threading
import time
import numpy as np
from PyQt5tWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QTextEdit, QComboBox, QFileDialog
)
from PyQt5QtCore import pyqtSignal, QObject
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
    
    倾角补偿：将三轴磁力计数据根据pitch/roll旋转到水平面
    使用小角度近似，更适合实际应用中的小角度误差
    输入：mx, my, mz (磁力计三轴数据), pitch, roll (弧度)
    输出：补偿后的水平面磁力计数据 mx_comp, my_comp
    小角度近似：sin(θ) ≈ θ, cos(θ) ≈ 1，适用于pitch≈0, roll≈3°的情况
       mx_comp = mx - mz * pitch
    my_comp = my - mx * roll + mz * pitch * roll
    return mx_comp, my_comp

# ========== 二维椭圆拟合 ==========
def fit_circle_least_squares(xy):
    x = xy[:, 0
    y = xy[:, 1
    A = np.c_[2*x, 2*y, np.ones(x.shape)]
    b = x**2 + y**2
    c, resid, rank, s = np.linalg.lstsq(A, b, rcond=None)
    center_x, center_y = c[0], c[1  radius = np.sqrt(c[2nter_x**2 + center_y**2)
    return np.array([center_x, center_y]), radius

def calibrate_2d_ellipse(xy):
    # 软铁：y轴缩放到x轴半径
    radius_x = (np.max(xy:, 0]) - np.min(xy[:, 0])) / 2    radius_y = (np.max(xy:, 1]) - np.min(xy[:, 1)) / 2
    scale = radius_x / radius_y if radius_y != 0 else 10   soft_calibrated = xy.copy()
    soft_calibrated[:, 1] *= scale
    # 硬铁：拟合圆心，平移到原点
    center, _ = fit_circle_least_squares(soft_calibrated)
    calibrated = soft_calibrated - center
    return calibrated, center, scale

# ========== heading计算 ==========
def calculate_heading(mx, my):

    计算方位角（heading）
    输入：水平面磁力计数据 mx, my
    输出：方位角（度，0 
    heading_rad = math.atan2(-my, mx)
    heading_deg = math.degrees(heading_rad)
    if heading_deg < 0:
        heading_deg += 360
    return heading_deg

def heading_array(mx, my):
    headings = np.degrees(np.arctan2(-my, mx))
    headings[headings < 0] += 360
    return headings

def fit_ellipse(xy):
    cov = np.cov(xy.T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    order = np.argsort(eigvals)[::-1]
    eigvals = eigvalsorder]
    eigvecs = eigvecs[:, order]
    theta = np.arctan2(eigvecs10 eigvecs[0,0])
    a = np.sqrt(eigvals[0])
    b = np.sqrt(eigvals[1])
    return theta, a, b

def rotate(xy, theta):
    R = np.array([[np.cos(theta), -np.sin(theta)],
                np.sin(theta),  np.cos(theta)]])
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
        self.running = true  try:
            ser = serial.Serial(self.port, self.baudrate, timeout=1)
            while self.running:
                try:
                    line = ser.readline().decode(errors=ignore                   self.callback(line)  # 任何行都传递
                except Exception as e:
                    print(Serial read error:", e)
        except Exception as e:
            print(Serial error:, e)
    def stop(self):
        self.running = False

# ========== 主界面 ==========
class SimpleCalibrationTester(QWidget):
    append_text_signal = pyqtSignal(str)
    DATA_COUNT_THRESHOLD = 100
    def __init__(self):
        super().__init__()
        self.setWindowTitle("简易地磁二维校准验证工具")
        self.resize(500,400
        self.main_layout = QVBoxLayout()
        self.setLayout(self.main_layout)

        # 串口选择
        hbox = QHBoxLayout()
        self.port_combo = QComboBox()
        self.refresh_btn = QPushButton("刷新串口)
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["115200, 960, 576003840)
        hbox.addWidget(QLabel("串口:"))
        hbox.addWidget(self.port_combo)
        hbox.addWidget(self.refresh_btn)
        hbox.addWidget(QLabel(波特率:"))
        hbox.addWidget(self.baud_combo)
        self.main_layout.addLayout(hbox)

        # 按钮
        btn_box = QHBoxLayout()
        self.btn_level = QPushButton("采集水平数据")
        self.btn_30eg = QPushButton(采集30度数据")
        self.btn_test = QPushButton("测试)       btn_box.addWidget(self.btn_level)
        btn_box.addWidget(self.btn_30g)
        btn_box.addWidget(self.btn_test)
        self.main_layout.addLayout(btn_box)

        # 状态与结果
        self.status_label = QLabel("状态: 等待操作")
        self.main_layout.addWidget(self.status_label)
        self.result_text = QTextEdit()
        self.result_text.setReadOnly(true
        self.main_layout.addWidget(self.result_text)

        # 数据缓存
        self.data_level = []
        self.data_30 =       self.serial_thread = None
        self.collecting = None  #level' or '30eg'
        self._pending_mag = None  # 用于暂存mag行
        self.data_log =]  # 新增：用于存储最近N条原始数据
        self.mag_queue = deque()
        self.pitch_queue = deque()
        self.append_text_signal.connect(self._append_data_text)

        # 新增：实时数据显示区
        self.data_text = QTextEdit()
        self.data_text.setReadOnly(true
        self.main_layout.addWidget(QLabel("串口实时数据："))
        self.main_layout.addWidget(self.data_text)

        # 新增：mag_x/mag_y实时散点图
        self.fig, self.ax = plt.subplots(figsize=(3)
        self.canvas = FigureCanvas(self.fig)
        self.main_layout.addWidget(QLabel("mag_x/mag_y实时分布："))
        self.main_layout.addWidget(self.canvas)
        self.scatter_plot = None

        # 新增：导出数据按钮
        export_box = QHBoxLayout()
        self.btn_export_level = QPushButton("导出水平数据")
        self.btn_export_30eg = QPushButton(导出30度数据")
        self.btn_export_level.setEnabled(False)
        self.btn_export_30deg.setEnabled(False)
        export_box.addWidget(self.btn_export_level)
        export_box.addWidget(self.btn_export_30
        self.main_layout.addLayout(export_box)
        self.btn_export_level.clicked.connect(self.export_level_data)
        self.btn_export_30deg.clicked.connect(self.export_30deg_data)

        # 事件绑定
        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.btn_level.clicked.connect(self.collect_level)
        self.btn_30deg.clicked.connect(self.collect_30)
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
        self.collecting = 'level    self.btn_level.setStyleSheet(background-color: yellow")
        self.btn_level.setEnabled(False)
        self.btn_30deg.setEnabled(False)
        self.btn_test.setEnabled(False)
        self.start_serial_collection()
        self.status_label.setText(状态: 采集水平数据中，请水平旋转1)

    def collect_30deg(self):
        self.data_30deg.clear()
        self.collecting = '30
        self.btn_30eg.setStyleSheet(background-color: yellow")
        self.btn_30deg.setEnabled(False)
        self.btn_test.setEnabled(False)
        self.start_serial_collection()
        self.status_label.setText("状态: 采集30度倾斜数据中，请倾斜301.)  def start_serial_collection(self):
        if self.serial_thread:
            self.serial_thread.stop()
            time.sleep(00.2       port = self.port_combo.currentText()
        baud = int(self.baud_combo.currentText())
        self.serial_thread = SerialReader(port, baud, self.on_serial_data)
        self.serial_thread.daemon = True
        self.serial_thread.start()

    def export_level_data(self):
        self.save_data_to_csv(self.data_level, "水平数据.csv)

    def export_30ata(self):
        self.save_data_to_csv(self.data_30deg,30度数据.csv")

    def save_data_to_csv(self, data, filename_hint):
        path, _ = QFileDialog.getSaveFileName(self, 保存数据", filename_hint,CSV Files (*.csv))
        if path:
            with open(path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['mag_x,mag_y,mag_z', pitch])
                writer.writerows(data)

    def update_scatter(self, data):
        self.ax.cla()
        if len(data) > 0
            arr = np.array(data)
            self.ax.scatter(arr:,0], arr[:,1], s=10, label='mag_x/mag_y')
            self.ax.set_title('mag_x/mag_y')
            self.ax.axis('equal')
            self.ax.legend()
        self.canvas.draw_idle()

    def polar_bin_filter(self, arr, bins=36):
        # 极坐标分箱，保留每个方向最远的点
        if len(arr) == 0:
            return arr
        x, y = arr[:,0], arr[:,1        theta = np.arctan2(y, x)
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

    def check_distribution_uniform(self, arr, bins=36, min_bins=36, min_radius=20        if len(arr) == 0:
            return False
        x, y = arr[:,0], arr[:,1        theta = np.arctan2(y, x)
        r = np.sqrt(x**2 + y**2)
        bin_edges = np.linspace(-np.pi, np.pi, bins+1)
        idx = np.digitize(theta, bin_edges) -1
        covered_bins = set(idx[(r > min_radius)])
        return len(covered_bins) >= min_bins

    def get_covered_bins(self, arr, bins=36        if len(arr) == 0:
            return 0
        x, y = arr[:,0], arr[:,1        theta = np.arctan2(y, x)
        bin_edges = np.linspace(-np.pi, np.pi, bins+1)
        idx = np.digitize(theta, bin_edges) - 1
        return len(np.unique(idx))

    def on_serial_data(self, line):
        try:
            self.append_text_signal.emit(line)
            if not line.strip() or line.strip() == 'mag:            return
            ifmag_xe:
                vals = [float(x) for x in re.findall(r'-?\d+\.?\d*', line)]
                if len(vals) == 3:
                    self.mag_queue.append(vals)
            elifpitche:
                vals = [float(x) for x in re.findall(r'-?\d+\.?\d*', line)]
                if len(vals) >= 2:
                    self.pitch_queue.append(vals)
            # 队列配对采集
            while self.mag_queue and self.pitch_queue:
                try:
                    mag_vals = self.mag_queue.popleft()
                    pitch_vals = self.pitch_queue.popleft()
                    mx, my, mz = mag_vals
                    pitch, roll = pitch_vals[0                   row = mx, my, mz, pitch, roll]
                    if self.collecting == 'level':
                        self.data_level.append(row)
                        self.update_scatter(self.data_level)
                        arr = np.array(self.data_level)
                        filtered = self.polar_bin_filter(arr[:, :2])
                        covered_bins = self.get_covered_bins(filtered)
                        self.status_label.setText(f已采集{len(self.data_level)}条，已覆盖方向{covered_bins}/36")
                        if self.check_distribution_uniform(filtered, bins=36                   if self.serial_thread:
                                self.serial_thread.stop()
                            self.btn_30deg.setEnabled(True)
                            self.btn_level.setEnabled(True)
                            self.btn_export_level.setEnabled(True)
                            self.collecting = None
                    elif self.collecting == '30deg':
                        self.data_30                   self.update_scatter(self.data_30deg)
                        arr = np.array(self.data_30deg)
                        filtered = self.polar_bin_filter(arr[:, :2])
                        covered_bins = self.get_covered_bins(filtered)
                        self.status_label.setText(f已采集{len(self.data_30deg)}条，已覆盖方向{covered_bins}/36")
                        if self.check_distribution_uniform(filtered, bins=36                   if self.serial_thread:
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
        now = datetime.datetime.now().strftime(%H:%M:%S.%f')[:-3]
        self.data_text.append(f[{now}] [object Object]line}")
        # 限制显示最近100        doc = self.data_text.document()
        if doc and doc.blockCount() > 100:
            cursor = self.data_text.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.select(cursor.LineUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()

    def run_test(self):
        try:
            if len(self.data_level) < 30 or len(self.data_30deg) < 30              self.result_text.setText("请先采集足够的水平和30数据（每组建议不少于30条）！)            return
            
            # 1. 数据准备和单位检查
            data1 = np.array(self.data_level)
            data2 = np.array(self.data_30eg)
            mx1, my1, mz1tch1, roll1 = data1.T
            mx2, my2, mz2tch2a2.T
            
            # 检查pitch/roll单位，如果是弧度需要转换
            if np.max(np.abs(pitch1) > np.pi/2:  # 如果pitch超过90度，说明是弧度
                pitch1 = np.radians(pitch1              roll1 = np.radians(roll1             pitch2 = np.radians(pitch2              roll2 = np.radians(roll2)
            
            #2. 倾角补偿（移除硬编码的坐标系修正）
            mxh1, myh1 = 
            for i in range(len(mx1)):
                # 直接使用原始数据，不做坐标系修正
                mx_corr = mx1[i]
                my_corr = my1[i]
                mz_corr = mz1[i]
                x, y = tilt_compensate(mx_corr, my_corr, mz_corr, pitch1[i], roll1[i])
                mxh1.append(x)
                myh1.append(y)
            
            mxh2, myh2 = 
            for i in range(len(mx2)):
                mx_corr = mx2[i]
                my_corr = my2[i]
                mz_corr = mz2[i]
                x, y = tilt_compensate(mx_corr, my_corr, mz_corr, pitch2[i], roll2[i])
                mxh2.append(x)
                myh2.append(y)
            
            # 3. 合并数据并做椭圆拟合
            mxh = np.concatenate([mxh1, mxh2])
            myh = np.concatenate([myh1, myh2)
            xy = np.stack([mxh, myh], axis=1)
            
            # 椭圆拟合+软铁补偿+硬铁补偿
            theta, a, b = fit_ellipse(xy)
            xy_rot = rotate(xy, -theta)
            
            # 软铁补偿：缩放
            xy_calib = xy_rot.copy()
            xy_calib[:,1] *= a / b
            
            # 硬铁补偿：平移到原点
            center = np.mean(xy_calib, axis=0)
            xy_calib = xy_calib - center
            
            # 4. 分组heading计算
            n1 = len(mxh1)
            n2 = len(mxh2)
            cal1b[:n1]
            cal2n1:]
            
            heading1 = np.array([calculate_heading(x, y) for x, y in cal1])
            heading2 = np.array([calculate_heading(x, y) for x, y in cal2])
            
            mean1 = np.mean(heading1)
            mean2 = np.mean(heading2)
            heading_drift = abs(mean1 - mean2)
            
            # 5. 输出详细结果
            result = f水平均值:[object Object]mean1:.2f}°\n30度均值: {mean2f}°\nheading_drift: [object Object]heading_drift:.2f}°\n"
            result += f"椭圆参数: a={a:.2f}, b={b:.2 theta={np.degrees(theta):.2f}°\n"
            result += f"硬铁偏移: center=({center0:.2f}, {center[1]:.2f})\n"
            
            if heading_drift < 3            result += \n结论：适合二维校准量产！（椭圆软铁补偿+硬铁补偿）            else:
                result +=\n结论：建议升级三轴椭球拟合或优化安装！（椭圆软铁补偿+硬铁补偿）"
            
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