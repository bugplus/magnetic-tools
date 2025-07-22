import sys
import serial
import threading
import time
import numpy as np
import re
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QPushButton, QLabel, QComboBox, QHBoxLayout, QMessageBox, QFileDialog, QDialog, QTextEdit
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from compass_ui import CompassMainWindow
from config import MAG_AXIS_MAP_A, MAG_AXIS_MAP_B, SENSOR_TYPE

CALIBRATION_DURATION = 30
MIN_3D_POINTS = 100

class SerialReader(threading.Thread):
    def __init__(self, port, baudrate, callback):
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
                    self.callback(line)
                except Exception as e:
                    print(f"Serial read error: {e}")
        except Exception as e:
            print(f"Serial error: {e}")

    def stop(self):
        self.running = False

def fit_ellipsoid_3d(points):
    pts = np.array(points)
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    # 使用 np.ones_like 生成与 x, y, z 形状相同的数组
    D = np.column_stack([x*x, y*y, z*z, x*y, x*z, y*z, x, y, z, np.ones_like(x)])
    coeffs, *_ = np.linalg.lstsq(D, 1, rcond=None)
    A, B, C, D, E, F, G, H, I, J = coeffs
    Q = np.array([[A, D/2, E/2], [D/2, B, F/2], [E/2, F/2, C]])
    b = -np.linalg.solve(Q, [G, H, I]) / 2
    Ainv = np.linalg.inv(Q)
    scale = (1.0 / abs(np.linalg.det(Ainv)))**(1/3)
    A_cal = Ainv * scale
    return b, A_cal

def generate_c_code_3d(b, A):
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

class CalibrationApp:
    def __init__(self):
        self.app = QApplication(sys.argv)
        self.window = CompassMainWindow()

        # 3D 信号
        self.window.step0_done.connect(self.start_step1)
        self.window.step1_done.connect(self.finish_steps)
        self.window.view3d_done.connect(self.view_result_3d)

        # 按钮
        self.window.step0_btn.clicked.connect(self.on_step0_clicked)
        self.window.step1_btn.clicked.connect(self.start_step1)
        self.window.view3d_btn.clicked.connect(self.view_result_3d)

        # 数据
        self.mag3d_data = []
        self.serial_thread = None
        self.step_stage = 0

        # 画布
        self.fig3d = plt.figure()
        self.canvas3d = FigureCanvas(self.fig3d)
        self.ax3d = self.fig3d.add_subplot(111, projection='3d')
        self.ax3d.set_title("3D Real-time Mag")
        self.ax3d.set_xlim([-100, 100])
        self.ax3d.set_ylim([-100, 100])
        self.ax3d.set_zlim([-100, 100])
        self.canvas3d.setParent(self.window.central)
        self.window.central_layout.addWidget(self.canvas3d)

    def _update_3d_plot(self):
        if self.ax3d:
            self.ax3d.clear()
            xyz = np.array(self.mag3d_data)
            if len(xyz) > 0:
                x_min, x_max = np.min(xyz[:, 0]), np.max(xyz[:, 0])
                y_min, y_max = np.min(xyz[:, 1]), np.max(xyz[:, 1])
                z_min, z_max = np.min(xyz[:, 2]), np.max(xyz[:, 2])
                self.ax3d.set_xlim([x_min - 10, x_max + 10])
                self.ax3d.set_ylim([y_min - 10, y_max + 10])
                self.ax3d.set_zlim([z_min - 10, z_max + 10])
            self.ax3d.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], c='b', s=5)
            self.ax3d.set_title(f"3D points: {len(xyz)}")
            self.canvas3d.draw()

    def on_mag_only(self, line):
        try:
            if line.startswith('mag_x='):
                vals = re.findall(r'mag_x=\s*([\-\d\.]+),\s*mag_y=\s*([\-\d\.]+),\s*mag_z=\s*([\-\d\.]+)', line)
                if vals:
                    mx, my, mz = map(float, vals[0])
                    self.mag3d_data.append([mx, my, mz])
                    self._update_3d_plot()
        except Exception as e:
            print(f"3D Serial: {e}")

    def on_step0_clicked(self):
        port = self.window.port_combo.currentText()
        baud = int(self.window.baud_combo.currentText())
        if "No" in port:
            QMessageBox.warning(self.window, "Error", "No port")
            return
        self.window.step0_btn.setEnabled(False)
        self.start_step0(port, baud)

    def start_step0(self, port, baud):
        self.step_stage = 1
        self.mag3d_data.clear()
        self.window.set_status("3D Step1: level 30s...")
        self.serial_thread = SerialReader(port, baud, self.on_mag_only)
        self.serial_thread.start()
        threading.Timer(CALIBRATION_DURATION, self.step_timeout).start()

    def start_step1(self):
        if len(self.mag3d_data) < MIN_3D_POINTS // 2:
            self.window.set_status(f"Step1 need ≥{MIN_3D_POINTS//2}")
            return
        self.step_stage = 2
        self.window.set_status("3D Step2: nose-up 30s...")
        self.serial_thread = SerialReader(
            self.window.port_combo.currentText(),
            int(self.window.baud_combo.currentText()),
            self.on_mag_only)
        self.serial_thread.start()
        threading.Timer(CALIBRATION_DURATION, self.step_timeout).start()

    def step_timeout(self):
        if self.serial_thread:
            self.serial_thread.stop()
        if self.step_stage == 1:
            self.window.enable_step1_btn(True)
        elif self.step_stage == 2:
            self.finish_steps()

    def finish_steps(self):
        if len(self.mag3d_data) < MIN_3D_POINTS:
            self.window.set_status(f"3D Need ≥{MIN_3D_POINTS}")
            return
        b, A = fit_ellipsoid_3d(self.mag3d_data)
        c_code = generate_c_code_3d(b, A)
        self.window.show_result_dialog(c_code)
        self.window.enable_view3d_btn(True)
        self.window.set_status("3D Done")

    def view_result_3d(self):
        if len(self.mag3d_data) < MIN_3D_POINTS:
            self.window.set_status(f"3D Need ≥{MIN_3D_POINTS}")
            return
        b, A = fit_ellipsoid_3d(self.mag3d_data)
        c_code = generate_c_code_3d(b, A)
        self.window.show_result_dialog(c_code)

    def run(self):
        self.window.show()
        sys.exit(self.app.exec_())

if __name__ == "__main__":
    app = CalibrationApp()
    app.run()