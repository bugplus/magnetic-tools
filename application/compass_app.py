# compass_app.py
# 磁力计三步校准上位机
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
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas2D
from PyQt5.QtWidgets import QDialog, QVBoxLayout
from config import (
    CALIBRATION_DURATION, MIN_3D_POINTS,
    ANGLE_GATE_DEG, DIST_GATE_CM,
    UNIT_SPHERE_SCALE, DECIMAL_PRECISION
)
import os
CALIB_DIR = os.path.join(os.path.dirname(__file__), "calibration_mag")
os.makedirs(CALIB_DIR, exist_ok=True)

# -----------------------------
# 1. 线程安全串口读取（角度门控+稀疏化）
# -----------------------------
class DataBridge(QObject):
    new_data = pyqtSignal(float, float, float, float, float, float)

class SerialThread(QThread):
    def __init__(self, port, baud, bridge):
        super().__init__()
        self.bridge = bridge
        self.port, self.baud = port, baud
        self.running = True

    def run(self):
        try:
            with serial.Serial(self.port, self.baud, timeout=1) as ser:
                last_ang = None
                last_xyz = None
                while self.running:
                    line = ser.readline().decode(errors='ignore')
                    m = re.search(r'mag_x=\s*([-\d\.eE+-]+),\s*mag_y=\s*([-\d\.eE+-]+),\s*mag_z=\s*([-\d\.eE+-]+)', line)
                    if m:
                        mx, my, mz = -float(m.group(2)), -float(m.group(1)), float(m.group(3))
                    a = re.search(r'pitch=\s*([-\d\.eE+-]+).*roll=\s*([-\d\.eE+-]+).*yaw=\s*([-\d\.eE+-]+)', line)
                    if a:
                        pitch, roll, yaw = map(float, a.groups())
                    if 'mx' in locals() and 'pitch' in locals():
                        xyz = np.array([mx, my, mz])
                        ang = np.array([pitch, roll, yaw])
                        if last_ang is None or np.any(np.abs(ang - last_ang) >= ANGLE_GATE_DEG):
                            last_ang = ang
                        else:
                            continue
                        if last_xyz is None or np.linalg.norm(xyz - last_xyz) >= DIST_GATE_CM * 0.01:
                            last_xyz = xyz
                            self.bridge.new_data.emit(mx, my, mz, pitch, roll, yaw)
        except Exception as e:
            QMessageBox.critical(None, "Serial Error", str(e))

    def stop(self):
        self.running = False

# -----------------------------
# 2. 核心函数
# -----------------------------
def fit_ellipsoid_3d(points):
    pts = np.asarray(points, dtype=float)[:, :3]
    if pts.shape[0] < 10:
        return None, None
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    D = np.column_stack([x*x, y*y, z*z, x*y, x*z, y*z, x, y, z, np.ones_like(x)])
    lam = 1e-6 * np.trace(D.T @ D) / D.shape[1]
    coeffs = np.linalg.solve(D.T @ D + lam * np.eye(10), D.T @ np.ones_like(x))
    Aq, Bq, Cq, Dq, Eq, Fq, G, H, I, J = coeffs
    Q = np.array([[Aq, Dq/2, Eq/2], [Dq/2, Bq, Fq/2], [Eq/2, Fq/2, Cq]])
    eig_vals, eig_vecs = np.linalg.eigh(Q)
    eig_vals = np.maximum(eig_vals, 1e-6)
    b = -np.linalg.solve(Q, [G, H, I]) / 2
    A_cal = eig_vecs @ np.diag(np.sqrt(eig_vals)) @ eig_vecs.T
    if np.linalg.det(A_cal) < 0:
        A_cal = -A_cal
    return b, A_cal

def generate_c_code_3d(b, A):
    if b is None or A is None:
        return "/* Error: Calibration failed */"
    bx, by, bz = b
    A = np.linalg.inv(A)
    lines = [
        "/* 3D mag calibration (auto) */",
        f"const float HARD_IRON[3] = {{{bx:.8f}f, {by:.8f}f, {bz:.8f}f}};",
        "const float SOFT_IRON[3][3] = {",
        f"  {{{A[0,0]:.8f}f, {A[0,1]:.8f}f, {A[0,2]:.8f}f}},",
        f"  {{{A[1,0]:.8f}f, {A[1,1]:.8f}f, {A[1,2]:.8f}f}},",
        f"  {{{A[2,0]:.8f}f, {A[2,1]:.8f}f, {A[2,2]:.8f}f}}",
        "};"
    ]
    return "\n".join(lines)

def raw_to_unit(raw_xyz, b):
    centered = raw_xyz[:, :3] - b
    length = np.linalg.norm(centered, axis=1, keepdims=True)
    length[length == 0] = 1
    return centered / length

def ellipsoid_to_sphere(raw_xyz, b, A):
    centered = raw_xyz[:, :3] - b
    sphere = centered @ A
    norm = np.linalg.norm(sphere, axis=1, keepdims=True)
    norm[norm == 0] = 1
    return sphere / norm

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
# 3. CalibrationApp
# -----------------------------
class CalibrationApp(QObject):
    def __init__(self):
        super().__init__()
        self.app = QApplication(sys.argv)
        self.window = CompassMainWindow()

        # 3D 画布
        self.fig3d = plt.figure()
        self.canvas3d = FigureCanvas(self.fig3d)
        self.ax3d = self.fig3d.add_subplot(111, projection='3d')
        self.ax3d.set_title("Raw 3D Mag (μT)")
        self.window.central_layout.addWidget(self.canvas3d)

        self.fig_raw_sphere = plt.figure()
        self.canvas_raw_sphere = FigureCanvas(self.fig_raw_sphere)
        self.ax_raw_sphere = self.fig_raw_sphere.add_subplot(111, projection='3d')
        self.ax_raw_sphere.set_title("Raw Unit Sphere")

        self.fig3d_cal = plt.figure()
        self.canvas3d_cal = FigureCanvas(self.fig3d_cal)
        self.ax3d_cal = self.fig3d_cal.add_subplot(111, projection='3d')
        self.ax3d_cal.set_title("Calibrated on Unit Sphere")

        hbox = QHBoxLayout()
        hbox.addWidget(self.canvas_raw_sphere)
        hbox.addWidget(self.canvas3d_cal)
        self.window.central_layout.addLayout(hbox)

        # 数据桥 & 定时器
        self.data_bridge = DataBridge()
        self.data_bridge.new_data[float, float, float, float, float, float].connect(
            self.handle_new_data, Qt.QueuedConnection)
        self.mag3d_data = []
        self.freeze_data = None
        self.freeze_b = None
        self.freeze_A = None
        self.timer = QTimer()
        self.timer.timeout.connect(self._update_3d_plot_safe)
        self.timer.start(50)

        # 按钮信号
        self.window.step0_btn.clicked.connect(lambda: self.start_step(0))
        self.window.step1_btn.clicked.connect(lambda: self.start_step(1))
        self.window.step2_btn.clicked.connect(lambda: self.start_step(2))
        self.window.view3d_btn.clicked.connect(self.view_result_3d)
        self.window.algo3d_btn.clicked.connect(self.on_algo3d)
        self.window.reset_btn.clicked.connect(self.reset_calibration)

    @pyqtSlot(float, float, float, float, float, float)
    def handle_new_data(self, mx, my, mz, pitch, roll, yaw):
        if self.freeze_data is None:
            self.mag3d_data.append([mx, my, mz, pitch, roll, yaw])

    def _update_3d_plot_safe(self):
        if self.freeze_data is not None:
            return
        if self.mag3d_data:
            self._update_3d_plot()

    def _update_3d_plot(self):
        xyz = np.array(self.mag3d_data)[:, :3]
        self.ax3d.clear()
        self.ax3d.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], c='b', s=5)
        self.ax3d.set_title(f"Raw 3D Mag ({len(xyz)} pts)")
        self.canvas3d.draw()

    def start_step(self, step: int):
        port = self.window.port_combo.currentText()
        baud = int(self.window.baud_combo.currentText())
        if "No" in port:
            QMessageBox.warning(self.window, "Error", "No port")
            return
        need = {0: 0, 1: MIN_3D_POINTS, 2: int(MIN_3D_POINTS * 1.5)}[step]
        if step and len(self.mag3d_data) < need:
            self.window.set_status(f"Step{step} need ≥{need}")
            return
        if step == 0:
            self.mag3d_data.clear()
            self.freeze_data = None

        btn_list = [self.window.step0_btn, self.window.step1_btn, self.window.step2_btn]
        btn_list[step].setEnabled(False)

        self.thread = SerialThread(port, baud, self.data_bridge)
        self.thread.start()
        QTimer.singleShot(CALIBRATION_DURATION * 1000, self.thread.stop)
        if step < 2:
            QTimer.singleShot(CALIBRATION_DURATION * 1000,
                              lambda: btn_list[step + 1].setEnabled(True))
        else:
            QTimer.singleShot(CALIBRATION_DURATION * 1000, self.finish_steps)
        self.window.set_status(f"Step {step + 1} running {CALIBRATION_DURATION}s")

    def reset_calibration(self):
        self.mag3d_data.clear()
        self.freeze_data = None
        self.freeze_b = None
        self.freeze_A = None
        self.timer.start(50)
        self.window.step0_btn.setEnabled(True)
        self.window.step1_btn.setEnabled(False)
        self.window.step2_btn.setEnabled(False)
        self.window.view3d_btn.setEnabled(False)
        self.window.set_status("Reset complete. Start new calibration.")

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
        xyz = np.array(self.freeze_data)[:, :3]
        pts_cal_unit = ellipsoid_to_sphere(xyz, self.freeze_b, self.freeze_A)

        os.makedirs(CALIB_DIR, exist_ok=True)
        np.savetxt(os.path.join(CALIB_DIR, "calibrated_mag.csv"), pts_cal_unit, delimiter=',', fmt='%.8f')
        np.savetxt(os.path.join(CALIB_DIR, "raw_mag_with_orientation.csv"), np.array(self.freeze_data), delimiter=',', fmt='%.6f')
        c_code = generate_c_code_3d(self.freeze_b, np.linalg.inv(self.freeze_A))
        with open(os.path.join(CALIB_DIR, "mag_calibration.h"), 'w', encoding='utf-8') as f:
            f.write(c_code)

        self.ax3d_cal.clear()
        draw_unit_sphere(self.ax3d_cal, r=1.0)
        self.ax3d_cal.scatter(pts_cal_unit[:, 0], pts_cal_unit[:, 1], pts_cal_unit[:, 2],
                              c=np.linalg.norm(pts_cal_unit, axis=1), s=6, cmap='coolwarm', vmin=0.9, vmax=1.1)
        self.ax3d_cal.set_title("Calibrated on Unit Sphere")
        self.ax3d_cal.set_box_aspect([1, 1, 1])
        self.canvas3d_cal.draw()

        self.window.show_result_dialog(c_code)
        self.window.enable_view3d_btn(True)
        self.window.set_status("3D Three-Step Done")

    def view_result_3d(self):
        if self.freeze_data is None:
            return

        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation

        xyz = np.array(self.freeze_data)[:, :3]
        n = len(xyz)
        k = n // 3
        colors = ['#ff0080', '#00e5ff', '#8000FF']
        labels = ['Level', 'Tilt', 'Stern']

        # Raw 3D
        self.ax3d.clear()
        for i, (c, lab) in enumerate(zip(colors, labels)):
            seg = xyz[i * k:(i + 1) * k]
            if len(seg):
                self.ax3d.scatter(*seg.T, c=c, s=8, label=lab, depthshade=False)
        self.ax3d.set_title("Raw 3D Mag (μT)")
        self.ax3d.legend()
        self.ax3d.set_box_aspect([1, 1, 1])
        self.canvas3d.draw()

        # Raw Unit Sphere
        raw_unit = raw_to_unit(xyz, self.freeze_b)
        self.ax_raw_sphere.clear()
        draw_unit_sphere(self.ax_raw_sphere, r=1.0)
        for i, (c, lab) in enumerate(zip(colors, labels)):
            seg = raw_unit[i * k:(i + 1) * k]
            if len(seg):
                self.ax_raw_sphere.scatter(*seg.T, c=c, s=4, label=lab, depthshade=False)
        self.ax_raw_sphere.set_title("Raw Unit Sphere")
        self.ax_raw_sphere.set_box_aspect([1, 1, 1])
        self.ax_raw_sphere.legend()
        self.canvas_raw_sphere.draw()

        # Calibrated Unit Sphere
        pts_centered = xyz - self.freeze_b
        pts_cal = (self.freeze_A @ pts_centered.T).T
        pts_cal_unit = pts_cal / np.linalg.norm(pts_cal, axis=1, keepdims=True)

        self.ax3d_cal.clear()
        draw_unit_sphere(self.ax3d_cal, r=1.0)
        for i, (c, lab) in enumerate(zip(colors, labels)):
            seg = pts_cal_unit[i * k:(i + 1) * k]
            if len(seg):
                self.ax3d_cal.scatter(*seg.T, c=c, s=4, label=lab, depthshade=False)
        self.ax3d_cal.set_title("Calibrated on Unit Sphere")
        self.ax3d_cal.set_box_aspect([1, 1, 1])
        self.ax3d_cal.legend()
        self.canvas3d_cal.draw()

        # 3D 动画
        elev = 20
        self.anim1 = FuncAnimation(self.fig3d, lambda i: self.ax3d.view_init(elev, i % 360), frames=360, interval=50)
        self.anim2 = FuncAnimation(self.fig_raw_sphere, lambda i: self.ax_raw_sphere.view_init(elev, i % 360), frames=360, interval=50)
        self.anim3 = FuncAnimation(self.fig3d_cal, lambda i: self.ax3d_cal.view_init(elev, i % 360), frames=360, interval=50)

        # XY 平面图（模态，不会闪退）
        dlg2d = QDialog(self.window)
        dlg2d.setWindowTitle("Calibrated XY Projection")
        dlg2d.resize(450, 450)
        layout = QVBoxLayout(dlg2d)
        fig2d, ax2d = plt.subplots()
        ax2d.set_aspect('equal')
        ax2d.scatter(pts_cal_unit[:, 0], pts_cal_unit[:, 1], s=4)
        circle = plt.Circle((0, 0), 1, color='r', fill=False, linestyle='--')
        ax2d.add_patch(circle)
        ax2d.set_title("Calibrated XY Plane Projection")
        canvas2d = FigureCanvas2D(fig2d)
        layout.addWidget(canvas2d)
        dlg2d.setLayout(layout)
        dlg2d.exec_()

    @pyqtSlot()
    def on_algo3d(self):
        fname, _ = QFileDialog.getOpenFileName(self.window, "Select CSV", "", "CSV (*.csv)")
        if not fname:
            return
        try:
            raw = np.loadtxt(fname, delimiter=',', ndmin=2)
            if raw.shape[1] not in (3, 6):
                raise ValueError("CSV must be N×3 or N×6")

            b, A = fit_ellipsoid_3d(raw)
            if b is None or A is None:
                QMessageBox.warning(self.window, "Error", "Algorithm failed")
                return

            pts_raw_unit = raw_to_unit(raw, b)
            pts_cal_unit = ellipsoid_to_sphere(raw, b, A)

            n = len(pts_raw_unit)
            k = n // 3
            colors = ['#ff0080', '#00e5ff', '#8000FF']
            sections = ['Level', 'Tilt', 'Stern']

            dlg = QDialog(self.window)
            dlg.setWindowTitle("Algorithm 3D View")
            dlg.resize(900, 600)

            fig = plt.figure(figsize=(9, 6))
            ax_raw = fig.add_subplot(121, projection='3d')
            ax_raw.set_title("Raw Unit Sphere (Hard-Iron Only)")
            ax_raw.set_box_aspect([1, 1, 1])
            draw_unit_sphere(ax_raw, r=1.0)
            for i, (c, lab) in enumerate(zip(colors, sections)):
                seg = pts_raw_unit[i*k:(i+1)*k]
                if len(seg):
                    ax_raw.scatter(*seg.T, c=c, s=4, label=lab, depthshade=False)
            ax_raw.legend()

            ax_cal = fig.add_subplot(122, projection='3d')
            ax_cal.set_title("Calibrated Unit Sphere")
            ax_cal.set_box_aspect([1, 1, 1])
            draw_unit_sphere(ax_cal, r=1.0)
            for i, (c, lab) in enumerate(zip(colors, sections)):
                seg = pts_cal_unit[i*k:(i+1)*k]
                if len(seg):
                    ax_cal.scatter(*seg.T, c=c, s=4, label=lab, depthshade=False)
            ax_cal.legend()

            canvas = FigureCanvas(fig)
            lay = QVBoxLayout(dlg)
            lay.addWidget(canvas)

            # 3D 动画
            from matplotlib.animation import FuncAnimation
            FuncAnimation(fig, lambda i: ax_raw.view_init(20, i % 360), frames=360, interval=50)
            FuncAnimation(fig, lambda i: ax_cal.view_init(20, i % 360), frames=360, interval=50)

            # XY 平面图（模态）
            dlg2d = QDialog(self.window)
            dlg2d.setWindowTitle("Calibrated XY Projection")
            dlg2d.resize(450, 450)
            layout = QVBoxLayout(dlg2d)
            fig2d, ax2d = plt.subplots()
            ax2d.set_aspect('equal')
            ax2d.scatter(pts_cal_unit[:, 0], pts_cal_unit[:, 1], s=4)
            circle = plt.Circle((0, 0), 1, color='r', fill=False, linestyle='--')
            ax2d.add_patch(circle)
            ax2d.set_title("Calibrated XY Plane Projection")
            canvas2d = FigureCanvas2D(fig2d)
            layout.addWidget(canvas2d)
            dlg2d.setLayout(layout)
            dlg2d.exec_()

            dlg.exec_()
        except Exception as e:
            QMessageBox.critical(self.window, "Error", str(e))

    def run(self):
        self.window.show()
        sys.exit(self.app.exec_())

if __name__ == "__main__":
    app = CalibrationApp()
    app.run()# compass_app.py
# 磁力计三步校准上位机
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
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas2D
from PyQt5.QtWidgets import QDialog, QVBoxLayout
from config import (
    CALIBRATION_DURATION, MIN_3D_POINTS,
    ANGLE_GATE_DEG, DIST_GATE_CM,
    UNIT_SPHERE_SCALE, DECIMAL_PRECISION
)
import os
CALIB_DIR = os.path.join(os.path.dirname(__file__), "calibration_mag")
os.makedirs(CALIB_DIR, exist_ok=True)

# -----------------------------
# 1. 线程安全串口读取（角度门控+稀疏化）
# -----------------------------
class DataBridge(QObject):
    new_data = pyqtSignal(float, float, float, float, float, float)

class SerialThread(QThread):
    def __init__(self, port, baud, bridge):
        super().__init__()
        self.bridge = bridge
        self.port, self.baud = port, baud
        self.running = True

    def run(self):
        try:
            with serial.Serial(self.port, self.baud, timeout=1) as ser:
                last_ang = None
                last_xyz = None
                while self.running:
                    line = ser.readline().decode(errors='ignore')
                    m = re.search(r'mag_x=\s*([-\d\.eE+-]+),\s*mag_y=\s*([-\d\.eE+-]+),\s*mag_z=\s*([-\d\.eE+-]+)', line)
                    if m:
                        mx, my, mz = -float(m.group(2)), -float(m.group(1)), float(m.group(3))
                    a = re.search(r'pitch=\s*([-\d\.eE+-]+).*roll=\s*([-\d\.eE+-]+).*yaw=\s*([-\d\.eE+-]+)', line)
                    if a:
                        pitch, roll, yaw = map(float, a.groups())
                    if 'mx' in locals() and 'pitch' in locals():
                        xyz = np.array([mx, my, mz])
                        ang = np.array([pitch, roll, yaw])
                        if last_ang is None or np.any(np.abs(ang - last_ang) >= ANGLE_GATE_DEG):
                            last_ang = ang
                        else:
                            continue
                        if last_xyz is None or np.linalg.norm(xyz - last_xyz) >= DIST_GATE_CM * 0.01:
                            last_xyz = xyz
                            self.bridge.new_data.emit(mx, my, mz, pitch, roll, yaw)
        except Exception as e:
            QMessageBox.critical(None, "Serial Error", str(e))

    def stop(self):
        self.running = False

# -----------------------------
# 2. 核心函数
# -----------------------------
def fit_ellipsoid_3d(points):
    pts = np.asarray(points, dtype=float)[:, :3]
    if pts.shape[0] < 10:
        return None, None
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    D = np.column_stack([x*x, y*y, z*z, x*y, x*z, y*z, x, y, z, np.ones_like(x)])
    lam = 1e-6 * np.trace(D.T @ D) / D.shape[1]
    coeffs = np.linalg.solve(D.T @ D + lam * np.eye(10), D.T @ np.ones_like(x))
    Aq, Bq, Cq, Dq, Eq, Fq, G, H, I, J = coeffs
    Q = np.array([[Aq, Dq/2, Eq/2], [Dq/2, Bq, Fq/2], [Eq/2, Fq/2, Cq]])
    eig_vals, eig_vecs = np.linalg.eigh(Q)
    eig_vals = np.maximum(eig_vals, 1e-6)
    b = -np.linalg.solve(Q, [G, H, I]) / 2
    A_cal = eig_vecs @ np.diag(np.sqrt(eig_vals)) @ eig_vecs.T
    if np.linalg.det(A_cal) < 0:
        A_cal = -A_cal
    return b, A_cal

def generate_c_code_3d(b, A):
    if b is None or A is None:
        return "/* Error: Calibration failed */"
    bx, by, bz = b
    A = np.linalg.inv(A)
    lines = [
        "/* 3D mag calibration (auto) */",
        f"const float HARD_IRON[3] = {{{bx:.8f}f, {by:.8f}f, {bz:.8f}f}};",
        "const float SOFT_IRON[3][3] = {",
        f"  {{{A[0,0]:.8f}f, {A[0,1]:.8f}f, {A[0,2]:.8f}f}},",
        f"  {{{A[1,0]:.8f}f, {A[1,1]:.8f}f, {A[1,2]:.8f}f}},",
        f"  {{{A[2,0]:.8f}f, {A[2,1]:.8f}f, {A[2,2]:.8f}f}}",
        "};"
    ]
    return "\n".join(lines)

def raw_to_unit(raw_xyz, b):
    centered = raw_xyz[:, :3] - b
    length = np.linalg.norm(centered, axis=1, keepdims=True)
    length[length == 0] = 1
    return centered / length

def ellipsoid_to_sphere(raw_xyz, b, A):
    centered = raw_xyz[:, :3] - b
    sphere = centered @ A
    norm = np.linalg.norm(sphere, axis=1, keepdims=True)
    norm[norm == 0] = 1
    return sphere / norm

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
# 3. CalibrationApp
# -----------------------------
class CalibrationApp(QObject):
    def __init__(self):
        super().__init__()
        self.app = QApplication(sys.argv)
        self.window = CompassMainWindow()

        # 3D 画布
        self.fig3d = plt.figure()
        self.canvas3d = FigureCanvas(self.fig3d)
        self.ax3d = self.fig3d.add_subplot(111, projection='3d')
        self.ax3d.set_title("Raw 3D Mag (μT)")
        self.window.central_layout.addWidget(self.canvas3d)

        self.fig_raw_sphere = plt.figure()
        self.canvas_raw_sphere = FigureCanvas(self.fig_raw_sphere)
        self.ax_raw_sphere = self.fig_raw_sphere.add_subplot(111, projection='3d')
        self.ax_raw_sphere.set_title("Raw Unit Sphere")

        self.fig3d_cal = plt.figure()
        self.canvas3d_cal = FigureCanvas(self.fig3d_cal)
        self.ax3d_cal = self.fig3d_cal.add_subplot(111, projection='3d')
        self.ax3d_cal.set_title("Calibrated on Unit Sphere")

        hbox = QHBoxLayout()
        hbox.addWidget(self.canvas_raw_sphere)
        hbox.addWidget(self.canvas3d_cal)
        self.window.central_layout.addLayout(hbox)

        # 数据桥 & 定时器
        self.data_bridge = DataBridge()
        self.data_bridge.new_data[float, float, float, float, float, float].connect(
            self.handle_new_data, Qt.QueuedConnection)
        self.mag3d_data = []
        self.freeze_data = None
        self.freeze_b = None
        self.freeze_A = None
        self.timer = QTimer()
        self.timer.timeout.connect(self._update_3d_plot_safe)
        self.timer.start(50)

        # 按钮信号
        self.window.step0_btn.clicked.connect(lambda: self.start_step(0))
        self.window.step1_btn.clicked.connect(lambda: self.start_step(1))
        self.window.step2_btn.clicked.connect(lambda: self.start_step(2))
        self.window.view3d_btn.clicked.connect(self.view_result_3d)
        self.window.algo3d_btn.clicked.connect(self.on_algo3d)
        self.window.reset_btn.clicked.connect(self.reset_calibration)

    @pyqtSlot(float, float, float, float, float, float)
    def handle_new_data(self, mx, my, mz, pitch, roll, yaw):
        if self.freeze_data is None:
            self.mag3d_data.append([mx, my, mz, pitch, roll, yaw])

    def _update_3d_plot_safe(self):
        if self.freeze_data is not None:
            return
        if self.mag3d_data:
            self._update_3d_plot()

    def _update_3d_plot(self):
        xyz = np.array(self.mag3d_data)[:, :3]
        self.ax3d.clear()
        self.ax3d.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], c='b', s=5)
        self.ax3d.set_title(f"Raw 3D Mag ({len(xyz)} pts)")
        self.canvas3d.draw()

    def start_step(self, step: int):
        port = self.window.port_combo.currentText()
        baud = int(self.window.baud_combo.currentText())
        if "No" in port:
            QMessageBox.warning(self.window, "Error", "No port")
            return
        need = {0: 0, 1: MIN_3D_POINTS, 2: int(MIN_3D_POINTS * 1.5)}[step]
        if step and len(self.mag3d_data) < need:
            self.window.set_status(f"Step{step} need ≥{need}")
            return
        if step == 0:
            self.mag3d_data.clear()
            self.freeze_data = None

        btn_list = [self.window.step0_btn, self.window.step1_btn, self.window.step2_btn]
        btn_list[step].setEnabled(False)

        self.thread = SerialThread(port, baud, self.data_bridge)
        self.thread.start()
        QTimer.singleShot(CALIBRATION_DURATION * 1000, self.thread.stop)
        if step < 2:
            QTimer.singleShot(CALIBRATION_DURATION * 1000,
                              lambda: btn_list[step + 1].setEnabled(True))
        else:
            QTimer.singleShot(CALIBRATION_DURATION * 1000, self.finish_steps)
        self.window.set_status(f"Step {step + 1} running {CALIBRATION_DURATION}s")

    def reset_calibration(self):
        self.mag3d_data.clear()
        self.freeze_data = None
        self.freeze_b = None
        self.freeze_A = None
        self.timer.start(50)
        self.window.step0_btn.setEnabled(True)
        self.window.step1_btn.setEnabled(False)
        self.window.step2_btn.setEnabled(False)
        self.window.view3d_btn.setEnabled(False)
        self.window.set_status("Reset complete. Start new calibration.")

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
        xyz = np.array(self.freeze_data)[:, :3]
        pts_cal_unit = ellipsoid_to_sphere(xyz, self.freeze_b, self.freeze_A)

        os.makedirs(CALIB_DIR, exist_ok=True)
        np.savetxt(os.path.join(CALIB_DIR, "calibrated_mag.csv"), pts_cal_unit, delimiter=',', fmt='%.8f')
        np.savetxt(os.path.join(CALIB_DIR, "raw_mag_with_orientation.csv"), np.array(self.freeze_data), delimiter=',', fmt='%.6f')
        c_code = generate_c_code_3d(self.freeze_b, np.linalg.inv(self.freeze_A))
        with open(os.path.join(CALIB_DIR, "mag_calibration.h"), 'w', encoding='utf-8') as f:
            f.write(c_code)

        self.ax3d_cal.clear()
        draw_unit_sphere(self.ax3d_cal, r=1.0)
        self.ax3d_cal.scatter(pts_cal_unit[:, 0], pts_cal_unit[:, 1], pts_cal_unit[:, 2],
                              c=np.linalg.norm(pts_cal_unit, axis=1), s=6, cmap='coolwarm', vmin=0.9, vmax=1.1)
        self.ax3d_cal.set_title("Calibrated on Unit Sphere")
        self.ax3d_cal.set_box_aspect([1, 1, 1])
        self.canvas3d_cal.draw()

        self.window.show_result_dialog(c_code)
        self.window.enable_view3d_btn(True)
        self.window.set_status("3D Three-Step Done")

    def view_result_3d(self):
        if self.freeze_data is None:
            return

        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation

        xyz = np.array(self.freeze_data)[:, :3]
        n = len(xyz)
        k = n // 3
        colors = ['#ff0080', '#00e5ff', '#8000FF']
        labels = ['Level', 'Tilt', 'Stern']

        # Raw 3D
        self.ax3d.clear()
        for i, (c, lab) in enumerate(zip(colors, labels)):
            seg = xyz[i * k:(i + 1) * k]
            if len(seg):
                self.ax3d.scatter(*seg.T, c=c, s=8, label=lab, depthshade=False)
        self.ax3d.set_title("Raw 3D Mag (μT)")
        self.ax3d.legend()
        self.ax3d.set_box_aspect([1, 1, 1])
        self.canvas3d.draw()

        # Raw Unit Sphere
        raw_unit = raw_to_unit(xyz, self.freeze_b)
        self.ax_raw_sphere.clear()
        draw_unit_sphere(self.ax_raw_sphere, r=1.0)
        for i, (c, lab) in enumerate(zip(colors, labels)):
            seg = raw_unit[i * k:(i + 1) * k]
            if len(seg):
                self.ax_raw_sphere.scatter(*seg.T, c=c, s=4, label=lab, depthshade=False)
        self.ax_raw_sphere.set_title("Raw Unit Sphere")
        self.ax_raw_sphere.set_box_aspect([1, 1, 1])
        self.ax_raw_sphere.legend()
        self.canvas_raw_sphere.draw()

        # Calibrated Unit Sphere
        pts_centered = xyz - self.freeze_b
        pts_cal = (self.freeze_A @ pts_centered.T).T
        pts_cal_unit = pts_cal / np.linalg.norm(pts_cal, axis=1, keepdims=True)

        self.ax3d_cal.clear()
        draw_unit_sphere(self.ax3d_cal, r=1.0)
        for i, (c, lab) in enumerate(zip(colors, labels)):
            seg = pts_cal_unit[i * k:(i + 1) * k]
            if len(seg):
                self.ax3d_cal.scatter(*seg.T, c=c, s=4, label=lab, depthshade=False)
        self.ax3d_cal.set_title("Calibrated on Unit Sphere")
        self.ax3d_cal.set_box_aspect([1, 1, 1])
        self.ax3d_cal.legend()
        self.canvas3d_cal.draw()

        # 3D 动画
        elev = 20
        self.anim1 = FuncAnimation(self.fig3d, lambda i: self.ax3d.view_init(elev, i % 360), frames=360, interval=50)
        self.anim2 = FuncAnimation(self.fig_raw_sphere, lambda i: self.ax_raw_sphere.view_init(elev, i % 360), frames=360, interval=50)
        self.anim3 = FuncAnimation(self.fig3d_cal, lambda i: self.ax3d_cal.view_init(elev, i % 360), frames=360, interval=50)

        # XY 平面图（模态，不会闪退）
        dlg2d = QDialog(self.window)
        dlg2d.setWindowTitle("Calibrated XY Projection")
        dlg2d.resize(450, 450)
        layout = QVBoxLayout(dlg2d)
        fig2d, ax2d = plt.subplots()
        ax2d.set_aspect('equal')
        ax2d.scatter(pts_cal_unit[:, 0], pts_cal_unit[:, 1], s=4)
        circle = plt.Circle((0, 0), 1, color='r', fill=False, linestyle='--')
        ax2d.add_patch(circle)
        ax2d.set_title("Calibrated XY Plane Projection")
        canvas2d = FigureCanvas2D(fig2d)
        layout.addWidget(canvas2d)
        dlg2d.setLayout(layout)
        dlg2d.exec_()

    @pyqtSlot()
    def on_algo3d(self):
        fname, _ = QFileDialog.getOpenFileName(self.window, "Select CSV", "", "CSV (*.csv)")
        if not fname:
            return
        try:
            raw = np.loadtxt(fname, delimiter=',', ndmin=2)
            if raw.shape[1] not in (3, 6):
                raise ValueError("CSV must be N×3 or N×6")

            b, A = fit_ellipsoid_3d(raw)
            if b is None or A is None:
                QMessageBox.warning(self.window, "Error", "Algorithm failed")
                return

            pts_raw_unit = raw_to_unit(raw, b)
            pts_cal_unit = ellipsoid_to_sphere(raw, b, A)

            n = len(pts_raw_unit)
            k = n // 3
            colors = ['#ff0080', '#00e5ff', '#8000FF']
            sections = ['Level', 'Tilt', 'Stern']

            dlg = QDialog(self.window)
            dlg.setWindowTitle("Algorithm 3D View")
            dlg.resize(900, 600)

            fig = plt.figure(figsize=(9, 6))
            ax_raw = fig.add_subplot(121, projection='3d')
            ax_raw.set_title("Raw Unit Sphere (Hard-Iron Only)")
            ax_raw.set_box_aspect([1, 1, 1])
            draw_unit_sphere(ax_raw, r=1.0)
            for i, (c, lab) in enumerate(zip(colors, sections)):
                seg = pts_raw_unit[i*k:(i+1)*k]
                if len(seg):
                    ax_raw.scatter(*seg.T, c=c, s=4, label=lab, depthshade=False)
            ax_raw.legend()

            ax_cal = fig.add_subplot(122, projection='3d')
            ax_cal.set_title("Calibrated Unit Sphere")
            ax_cal.set_box_aspect([1, 1, 1])
            draw_unit_sphere(ax_cal, r=1.0)
            for i, (c, lab) in enumerate(zip(colors, sections)):
                seg = pts_cal_unit[i*k:(i+1)*k]
                if len(seg):
                    ax_cal.scatter(*seg.T, c=c, s=4, label=lab, depthshade=False)
            ax_cal.legend()

            canvas = FigureCanvas(fig)
            lay = QVBoxLayout(dlg)
            lay.addWidget(canvas)

            # 3D 动画
            from matplotlib.animation import FuncAnimation
            FuncAnimation(fig, lambda i: ax_raw.view_init(20, i % 360), frames=360, interval=50)
            FuncAnimation(fig, lambda i: ax_cal.view_init(20, i % 360), frames=360, interval=50)

            # XY 平面图（模态）
            dlg2d = QDialog(self.window)
            dlg2d.setWindowTitle("Calibrated XY Projection")
            dlg2d.resize(450, 450)
            layout = QVBoxLayout(dlg2d)
            fig2d, ax2d = plt.subplots()
            ax2d.set_aspect('equal')
            ax2d.scatter(pts_cal_unit[:, 0], pts_cal_unit[:, 1], s=4)
            circle = plt.Circle((0, 0), 1, color='r', fill=False, linestyle='--')
            ax2d.add_patch(circle)
            ax2d.set_title("Calibrated XY Plane Projection")
            canvas2d = FigureCanvas2D(fig2d)
            layout.addWidget(canvas2d)
            dlg2d.setLayout(layout)
            dlg2d.exec_()

            dlg.exec_()
        except Exception as e:
            QMessageBox.critical(self.window, "Error", str(e))

    def run(self):
        self.window.show()
        sys.exit(self.app.exec_())

if __name__ == "__main__":
    app = CalibrationApp()
    app.run()