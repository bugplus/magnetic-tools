# compass_app.py
import sys
import serial
import threading
import numpy as np
import re
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QPushButton, QLabel,
    QComboBox, QHBoxLayout, QMessageBox, QFileDialog, QDialog, QTextEdit
)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from compass_ui import CompassMainWindow, ResultDialog
from PyQt5.QtCore import (
    QTimer, pyqtSlot, pyqtSignal, QObject, QThread, Qt
)

CALIBRATION_DURATION = 30
MIN_3D_POINTS = 100

# -----------------------------
# 1. 线程安全串口读取
# -----------------------------
class DataBridge(QObject):
    new_data = pyqtSignal(float, float, float)

class SerialThread(QThread):
    def __init__(self, port, baud, bridge):
        super().__init__()
        self.bridge = bridge
        self.port = port
        self.baud = baud
        self.running = True

    def run(self):
        try:
            with serial.Serial(self.port, self.baud, timeout=1) as ser:
                while self.running:
                    line = ser.readline().decode(errors='ignore')
                    if line.startswith('mag_x='):
                        vals = re.findall(
                            r'mag_x=\s*([\-\d\.]+),\s*mag_y=\s*([\-\d\.]+),\s*mag_z=\s*([\-\d\.]+)', line
                        )
                        if vals:
                            mx, my, mz = map(float, vals[0])
                            self.bridge.new_data.emit(mx, my, mz)
        except Exception as e:
            print(f"Serial error: {e}")

    def stop(self):
        self.running = False

# -----------------------------
# 2. 功能函数
# -----------------------------
def fit_ellipsoid_3d(points):
    try:
        if not points or len(points) < MIN_3D_POINTS:
            raise ValueError(f"Insufficient data points: {len(points)} (required: {MIN_3D_POINTS})")
        pts = np.array(points)
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        D = np.column_stack([x*x, y*y, z*z, x*y, x*z, y*z, x, y, z, np.ones_like(x)])
        coeffs, *_ = np.linalg.lstsq(D, np.ones_like(x), rcond=None)
        A, B, C, D, E, F, G, H, I, J = coeffs
        Q = np.array([[A, D/2, E/2], [D/2, B, F/2], [E/2, F/2, C]])
        b = -np.linalg.solve(Q, [G, H, I]) / 2
        Ainv = np.linalg.inv(Q)
        scale = (1.0 / abs(np.linalg.det(Ainv)))**(1/3)
        A_cal = Ainv * scale
        return b, A_cal
    except Exception as e:
        print(f"Error in fit_ellipsoid_3d: {e}")
        return None, None

def generate_c_code_3d(b, A):
    if b is None or A is None:
        return "/* Error: Calibration failed */"
    bx, by, bz = b
    lines = [
        "/* 3D mag calibration (auto) */",
        f"const float HARD_IRON[3] = {{{bx:.6f}f, {by:.6f}f, {bz:.6f}f}};",
        "const float SOFT_IRON[3][3] = {",
        f"  {{{A[0, 0]:.6f}f, {A[0, 1]:.6f}f, {A[0, 2]:.6f}f}},",
        f"  {{{A[1, 0]:.6f}f, {A[1, 1]:.6f}f, {A[1, 2]:.6f}f}},",
        f"  {{{A[2, 0]:.6f}f, {A[2, 1]:.6f}f, {A[2, 2]:.6f}f}}",
        "};"
    ]
    return "\n".join(lines)

# -----------------------------
# 3. CalibrationApp
# -----------------------------
class CalibrationApp(QObject):
    def __init__(self):
        super().__init__()
        self.app = QApplication(sys.argv)
        self.window = CompassMainWindow()

        # 实时 3D 画布
        self.fig3d = plt.figure()
        self.canvas3d = FigureCanvas(self.fig3d)
        self.ax3d = self.fig3d.add_subplot(111, projection='3d')
        self.ax3d.set_title("3D Real-time Mag")
        self.canvas3d.setParent(self.window.central)
        self.window.central_layout.addWidget(self.canvas3d)

        # 校准后 3D 画布
        self.fig3d_cal = plt.figure()
        self.canvas3d_cal = FigureCanvas(self.fig3d_cal)
        self.ax3d_cal = self.fig3d_cal.add_subplot(111, projection='3d')
        self.ax3d_cal.set_title("Calibrated 3D Mag")
        self.window.central_layout.addWidget(self.canvas3d_cal)

        # DataBridge
        self.data_bridge = DataBridge()
        self.data_bridge.new_data[float, float, float].connect(
            self.handle_new_data, Qt.QueuedConnection
        )

        # 数据 & 线程
        self.mag3d_data = []
        self.thread = None

        # 冻结数据缓存
        self.freeze_data = None
        self.freeze_b = None
        self.freeze_A = None

        # 定时器（仅用于实时刷新）
        self.timer = QTimer()
        self.timer.timeout.connect(self._update_3d_plot_safe)
        self.timer.start(200)

        # 按钮连接
        self.window.step0_btn.clicked.connect(self.on_step0_clicked)
        self.window.step1_btn.clicked.connect(self.start_step1)
        self.window.view3d_btn.clicked.connect(self.view_result_3d)

    @pyqtSlot(float, float, float)
    def handle_new_data(self, mx, my, mz):
        if self.freeze_data is None:          # 未冻结前一直追加
            self.mag3d_data.append([mx, my, mz])

    def _update_3d_plot_safe(self):
        if self.freeze_data is not None:      # 冻结后不再刷新
            return
        if self.mag3d_data:
            self._update_3d_plot()

    def _update_3d_plot(self):
        try:
            self.ax3d.clear()
            xyz = np.array(self.mag3d_data)
            self.ax3d.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], c='b', s=5)
            self.ax3d.set_title(f"Raw 3D Mag ({len(xyz)} pts)")
            self.canvas3d.draw()
        except Exception as e:
            print(f"Error in _update_3d_plot: {e}")

    def on_step0_clicked(self):
        port = self.window.port_combo.currentText()
        baud = int(self.window.baud_combo.currentText())
        if "No" in port:
            QMessageBox.warning(self.window, "Error", "No port")
            return
        self.window.step0_btn.setEnabled(False)
        self.start_step0(port, baud)

    def start_step0(self, port, baud):
        try:
            self.mag3d_data.clear()
            self.freeze_data = None
            self.window.set_status("3D Step1: level 30s...")
            self.thread = SerialThread(port, baud, self.data_bridge)
            self.thread.start()
            QTimer.singleShot(CALIBRATION_DURATION * 1000, self.thread.stop)
            QTimer.singleShot(CALIBRATION_DURATION * 1000,
                              lambda: self.window.enable_step1_btn(True))
        except Exception as e:
            print(f"Error in start_step0: {e}")
            self.window.set_status("3D Step1 failed")

    def start_step1(self):
        if len(self.mag3d_data) < MIN_3D_POINTS // 2:
            self.window.set_status(f"Step1 need ≥{MIN_3D_POINTS//2}")
            return
        port = self.window.port_combo.currentText()
        baud = int(self.window.baud_combo.currentText())
        self.thread = SerialThread(port, baud, self.data_bridge)
        self.thread.start()
        self.window.set_status("3D Step2: nose-up 30s...")
        QTimer.singleShot(CALIBRATION_DURATION * 1000, self.finish_steps)

    def finish_steps(self):
        if self.thread:
            self.thread.stop()
        if len(self.mag3d_data) < MIN_3D_POINTS:
            self.window.set_status(f"3D Need ≥{MIN_3D_POINTS}")
            return

        # 1. 冻结数据
        self.freeze_data = list(self.mag3d_data)

        # 2. 计算校准矩阵
        self.freeze_b, self.freeze_A = fit_ellipsoid_3d(self.freeze_data)
        if self.freeze_b is None or self.freeze_A is None:
            self.window.set_status("3D Calibration failed")
            return

        # 3. 停实时定时器
        self.timer.stop()

        # 4. 画两幅图
        xyz = np.array(self.freeze_data)
        # 原始
        self.ax3d.clear()
        self.ax3d.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], c='b', s=5)
        self.ax3d.set_title(f"Raw 3D Mag ({len(xyz)} pts)")
        self.canvas3d.draw()
        # 校准
        pts_centered = xyz - self.freeze_b
        pts_cal = (self.freeze_A @ pts_centered.T).T
        self.ax3d_cal.clear()
        self.ax3d_cal.scatter(pts_cal[:, 0], pts_cal[:, 1], pts_cal[:, 2], c='r', s=5)
        self.ax3d_cal.set_title("Calibrated 3D Mag")
        self.canvas3d_cal.draw()

        # 5. 弹窗 C 代码
        c_code = generate_c_code_3d(self.freeze_b, self.freeze_A)
        self.window.show_result_dialog(c_code)
        self.window.enable_view3d_btn(True)
        self.window.set_status("3D Done")

    def view_result_3d(self):
        if self.freeze_data is None:
            self.window.set_status("No frozen data. Run calibration first.")
            return
        # 仅重绘已冻结的两幅图
        xyz = np.array(self.freeze_data)
        # 原始
        self.ax3d.clear()
        self.ax3d.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], c='b', s=5)
        self.ax3d.set_title(f"Raw 3D Mag ({len(xyz)} pts)")
        self.canvas3d.draw()
        # 校准
        pts_centered = xyz - self.freeze_b
        pts_cal = (self.freeze_A @ pts_centered.T).T
        self.ax3d_cal.clear()
        self.ax3d_cal.scatter(pts_cal[:, 0], pts_cal[:, 1], pts_cal[:, 2], c='r', s=5)
        self.ax3d_cal.set_title("Calibrated 3D Mag")
        self.canvas3d_cal.draw()

    def run(self):
        self.window.show()
        sys.exit(self.app.exec_())

if __name__ == "__main__":
    app = CalibrationApp()
    app.run()