# compass_app.py
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
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.colors import LightSource
from config import CALIBRATION_DURATION, MIN_3D_POINTS

# -----------------------------
# 1. 线程安全串口读取（带角度门控+稀疏化）
# -----------------------------
class DataBridge(QObject):
    new_data = pyqtSignal(float, float, float, float, float)  # mx,my,mz,pitch,roll

class SerialThread(QThread):
    def __init__(self, port, baud, bridge):
        super().__init__()
        self.bridge = bridge
        self.port, self.baud = port, baud
        self.running = True
        self.cache_mag = None
        self.cache_ang = None

    def run(self):
        import numpy as np, re, serial
        try:
            with serial.Serial(self.port, self.baud, timeout=1) as ser:
                while self.running:
                    line = ser.readline().decode(errors='ignore')
                    m = re.search(r'mag_x=\s*([-\d\.]+),\s*mag_y=\s*([-\d\.]+),\s*mag_z=\s*([-\d\.]+)', line)
                    if m:
                        self.cache_mag = list(map(float, m.groups()))
                    a = re.search(r'pitch=\s*([-\d\.]+).*roll=\s*([-\d\.]+).*yaw=\s*([-\d\.]+)', line)
                    if a:
                        self.cache_ang = list(map(float, a.groups()))
                    if self.cache_mag and self.cache_ang:
                        mx, my, mz = -self.cache_mag[1], -self.cache_mag[0], self.cache_mag[2]
                        pitch, roll, _ = self.cache_ang
                        self.bridge.new_data.emit(mx, my, mz, pitch, roll)
                        self.cache_mag = self.cache_ang = None
        except Exception as e:
            print("Serial error:", e)

    def stop(self):
        self.running = False

# -----------------------------
# 2. 功能函数
# -----------------------------
def fit_ellipsoid_3d(points):
    """
    鲁棒椭球拟合：硬铁 + 软铁
    返回 b, A；失败返回 None, None
    """
    pts = np.asarray(points, dtype=float)
    if pts.shape[0] < 10:
        print("点数不足 10")
        return None, None

    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    D = np.column_stack([x*x, y*y, z*z, x*y, x*z, y*z, x, y, z, np.ones_like(x)])

    # Tikhonov 正则化
    lam = 1e-6 * np.trace(D.T @ D) / D.shape[1]
    DTD = D.T @ D + lam * np.eye(10)
    DTy = D.T @ np.ones_like(x)
    try:
        coeffs = np.linalg.solve(DTD, DTy)
    except np.linalg.LinAlgError:
        print("正则化后仍无法求解")
        return None, None

    Aq, Bq, Cq, Dq, Eq, Fq, G, H, I, J = coeffs
    Q = np.array([[Aq, Dq/2, Eq/2],
                  [Dq/2, Bq, Fq/2],
                  [Eq/2, Fq/2, Cq]])

    # 强制正定
    eig_vals, eig_vecs = np.linalg.eigh(Q)
    eig_vals = np.maximum(eig_vals, 1e-6)
    if np.any(eig_vals <= 0):
        print("强制正定后仍失败")
        return None, None

    b = -np.linalg.solve(Q, [G, H, I]) / 2
    scale = np.sqrt(1.0 / eig_vals)
    A_cal = eig_vecs @ np.diag(scale) @ eig_vecs.T
    A_cal = np.linalg.inv(A_cal)
    return b, A_cal

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

def draw_unit_sphere(ax, r=1.0):
    u = np.linspace(0, 2 * np.pi, 60)
    v = np.linspace(0, np.pi, 30)
    x = r * np.outer(np.cos(u), np.sin(v))
    y = r * np.outer(np.sin(u), np.sin(v))
    z = r * np.outer(np.ones_like(u), np.cos(v))
    ls = LightSource(azdeg=45, altdeg=45)
    rgb = ls.shade(z, cmap=plt.cm.coolwarm, vert_exag=0.1, blend_mode='soft')
    ax.plot_surface(x, y, z, facecolors=rgb, alpha=0.4, shade=True, antialiased=True)

# -----------------------------
# 3. CalibrationApp（三步校准）
# -----------------------------
class CalibrationApp(QObject):
    def __init__(self):
        super().__init__()
        self.app = QApplication(sys.argv)
        self.window = CompassMainWindow()

        # 实时 3D
        self.fig3d = plt.figure()
        self.canvas3d = FigureCanvas(self.fig3d)
        self.ax3d = self.fig3d.add_subplot(111, projection='3d')
        self.ax3d.set_title("3D Real-time Mag")
        self.canvas3d.setParent(self.window.central)
        self.window.central_layout.addWidget(self.canvas3d)

        self.fig3d_cal = plt.figure()
        self.canvas3d_cal = FigureCanvas(self.fig3d_cal)
        self.ax3d_cal = self.fig3d_cal.add_subplot(111, projection='3d')
        self.ax3d_cal.set_title("Calibrated 3D Mag")
        self.window.central_layout.addWidget(self.canvas3d_cal)

        # 数据桥
        self.data_bridge = DataBridge()
        self.data_bridge.new_data[float, float, float, float, float].connect(
            self.handle_new_data, Qt.QueuedConnection)

        self.mag3d_data = []
        self.freeze_data = self.freeze_b = self.freeze_A = None
        self.timer = QTimer()
        self.timer.timeout.connect(self._update_3d_plot_safe)
        self.timer.start(200)

        # 按钮连接
        self.window.step0_btn.clicked.connect(self.on_step0_clicked)
        self.window.step1_btn.clicked.connect(self.start_step1)
        self.window.step2_btn.clicked.connect(self.start_step2)
        self.window.view3d_btn.clicked.connect(self.view_result_3d)

    @pyqtSlot(float, float, float, float, float)
    def handle_new_data(self, mx, my, mz, pitch, roll):
        if self.freeze_data is None:
            self.mag3d_data.append([mx, my, mz])

    def _update_3d_plot_safe(self):
        if self.freeze_data is not None:
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
            print(f"Error _update_3d_plot: {e}")

    def on_step0_clicked(self):
        port = self.window.port_combo.currentText()
        baud = int(self.window.baud_combo.currentText())
        if "No" in port:
            QMessageBox.warning(self.window, "Error", "No port")
            return
        self.window.step0_btn.setEnabled(False)
        self.start_step0(port, baud)

    def start_step0(self, port, baud):
        self.mag3d_data.clear()
        self.freeze_data = None
        self.window.set_status("3D Step1: level 60s...")
        self.thread = SerialThread(port, baud, self.data_bridge)
        self.thread.start()
        QTimer.singleShot(CALIBRATION_DURATION * 1000, self.thread.stop)
        QTimer.singleShot(CALIBRATION_DURATION * 1000,
                          lambda: self.window.step1_btn.setEnabled(True))

    def start_step1(self):
        if len(self.mag3d_data) < MIN_3D_POINTS:
            self.window.set_status(f"Step1 need ≥{MIN_3D_POINTS}")
            return
        port = self.window.port_combo.currentText()
        baud = int(self.window.baud_combo.currentText())
        self.thread = SerialThread(port, baud, self.data_bridge)
        self.thread.start()
        self.window.set_status("3D Step2: nose-up 60s...")
        QTimer.singleShot(CALIBRATION_DURATION * 1000, self.thread.stop)
        QTimer.singleShot(CALIBRATION_DURATION * 1000,
                          lambda: self.window.step2_btn.setEnabled(True))

    def start_step2(self):
        if len(self.mag3d_data) < int(MIN_3D_POINTS * 1.5):
            self.window.set_status(f"Step2 need ≥{int(MIN_3D_POINTS * 1.5)}")
            return
        port = self.window.port_combo.currentText()
        baud = int(self.window.baud_combo.currentText())
        self.thread = SerialThread(port, baud, self.data_bridge)
        self.thread.start()
        self.window.set_status("3D Step3: stern-up 60s...")
        QTimer.singleShot(CALIBRATION_DURATION * 1000, self.finish_steps)

    def finish_steps(self):
        if self.thread:
            self.thread.stop()
        if len(self.mag3d_data) < MIN_3D_POINTS * 2:
            self.window.set_status(f"3D Need ≥{MIN_3D_POINTS * 2}")
            return
        self.freeze_data = list(self.mag3d_data)
        self.freeze_b, self.freeze_A = fit_ellipsoid_3d(self.freeze_data)
        if self.freeze_b is None or self.freeze_A is None:
            self.window.set_status("3D Calibration failed")
            return
        self.timer.stop()
        xyz = np.array(self.freeze_data)
        self.ax3d.clear()
        self.ax3d.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], c='b', s=5)
        self.ax3d.set_title(f"Raw 3D Mag ({len(xyz)} pts)")
        self.canvas3d.draw()

        pts_centered = xyz - self.freeze_b
        pts_cal = (self.freeze_A @ pts_centered.T).T
        scale = np.linalg.norm(pts_cal, axis=1).mean()
        pts_cal_unit = pts_cal / scale

        self.ax3d_cal.clear()
        draw_unit_sphere(self.ax3d_cal, r=1.0)
        self.ax3d_cal.scatter(
            pts_cal_unit[:, 0], pts_cal_unit[:, 1], pts_cal_unit[:, 2],
            c=np.linalg.norm(pts_cal_unit, axis=1),
            s=6, cmap='coolwarm', vmin=0.9, vmax=1.1)
        self.ax3d_cal.set_title("Calibrated on Unit Sphere")
        self.ax3d_cal.set_box_aspect([1, 1, 1])
        self.canvas3d_cal.draw()

        c_code = generate_c_code_3d(self.freeze_b, self.freeze_A)
        csv_path = QFileDialog.getSaveFileName(
            self.window, "保存原始数据 CSV", "raw_mag.csv", "CSV (*.csv)")[0]
        if csv_path:
            np.savetxt(csv_path, np.array(self.freeze_data),
                       delimiter=',', fmt='%.6f')
        self.window.show_result_dialog(c_code)
        self.window.enable_view3d_btn(True)
        self.window.set_status("3D Three-Step Done")

    def view_result_3d(self):
        if self.freeze_data is None:
            self.window.set_status("No frozen data. Run calibration first.")
            return
        xyz = np.array(self.freeze_data)
        half = len(xyz) // 3
        level_raw, tilt_raw, stern_raw = xyz[:half], xyz[half:2*half], xyz[2*half:]
        pts_centered = xyz - self.freeze_b
        pts_cal = (self.freeze_A @ pts_centered.T).T
        norms = np.linalg.norm(pts_cal, axis=1, keepdims=True)
        pts_cal_unit = pts_cal / norms
        level_cal, tilt_cal, stern_cal = pts_cal_unit[:half], pts_cal_unit[half:2*half], pts_cal_unit[2*half:]

        self.ax3d.clear()
        if len(level_raw):
            self.ax3d.scatter(level_raw[:, 0], level_raw[:, 1], level_raw[:, 2],
                              c='#ff0080', s=8, label='Level', depthshade=True)
        if len(tilt_raw):
            self.ax3d.scatter(tilt_raw[:, 0], tilt_raw[:, 1], tilt_raw[:, 2],
                              c='#00e5ff', s=8, label='Tilt', depthshade=True)
        if len(stern_raw):
            self.ax3d.scatter(stern_raw[:, 0], stern_raw[:, 1], stern_raw[:, 2],
                              c='#00ff80', s=8, label='Stern', depthshade=True)
        self.ax3d.set_title("Raw Mag Data: Level / Tilt / Stern")
        self.ax3d.set_box_aspect([1, 1, 1])
        self.ax3d.legend()

        self.ax3d_cal.clear()
        draw_unit_sphere(self.ax3d_cal, r=1.0)
        if len(level_cal):
            self.ax3d_cal.scatter(level_cal[:, 0], level_cal[:, 1], level_cal[:, 2],
                                  c='#ff0080', s=4, label='Level (cal)', depthshade=True)
        if len(tilt_cal):
            self.ax3d_cal.scatter(tilt_cal[:, 0], tilt_cal[:, 1], tilt_cal[:, 2],
                                  c='#00e5ff', s=4, label='Tilt (cal)', depthshade=True)
        if len(stern_cal):
            self.ax3d_cal.scatter(stern_cal[:, 0], stern_cal[:, 1], stern_cal[:, 2],
                                  c='#00ff80', s=4, label='Stern (cal)', depthshade=True)
        self.ax3d_cal.set_title("Calibrated Mag on Sphere")
        self.ax3d_cal.set_box_aspect([1, 1, 1])
        self.ax3d_cal.legend()

        from matplotlib.animation import FuncAnimation
        self.anim = FuncAnimation(self.fig3d, lambda i: self.ax3d.view_init(20, i % 360),
                                  frames=360, interval=50, repeat=True)
        self.anim2 = FuncAnimation(self.fig3d_cal, lambda i: self.ax3d_cal.view_init(20, i % 360),
                                   frames=360, interval=50, repeat=True)
        self.canvas3d.draw()
        self.canvas3d_cal.draw()

    def run(self):
        self.window.show()
        sys.exit(self.app.exec_())

if __name__ == "__main__":
    app = CalibrationApp()
    app.run()