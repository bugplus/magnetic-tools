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
# 3. CalibrationApp
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
        self.data_bridge.new_data[float, float, float].connect(
            self.handle_new_data, Qt.QueuedConnection)

        self.mag3d_data = []
        self.freeze_data = self.freeze_b = self.freeze_A = None
        self.timer = QTimer()
        self.timer.timeout.connect(self._update_3d_plot_safe)
        self.timer.start(200)

        # 按钮连接
        self.window.step0_btn.clicked.connect(self.on_step0_clicked)
        self.window.step1_btn.clicked.connect(self.start_step1)
        self.window.view3d_btn.clicked.connect(self.view_result_3d)

        # 新增按钮：算法 3D 视图
        self.window.algo3d_btn = QPushButton("算法 3D 视图")
        self.window.central_layout.addWidget(self.window.algo3d_btn)
        self.window.algo3d_btn.clicked.connect(self.on_algo3d)

    @pyqtSlot(float, float, float)
    def handle_new_data(self, mx, my, mz):
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
        self.window.set_status("3D Step1: level 30s...")
        self.thread = SerialThread(port, baud, self.data_bridge)
        self.thread.start()
        QTimer.singleShot(CALIBRATION_DURATION * 1000, self.thread.stop)
        QTimer.singleShot(CALIBRATION_DURATION * 1000,
                          lambda: self.window.enable_step1_btn(True))

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
        # ========= 保存原始数据 CSV =========
        csv_path = QFileDialog.getSaveFileName(
            self.window, "保存原始数据 CSV", "raw_mag.csv", "CSV (*.csv)")[0]
        if csv_path:
            np.savetxt(csv_path, np.array(self.freeze_data),
                       delimiter=',', fmt='%.6f')
        # ====================================
        self.window.show_result_dialog(c_code)
        self.window.enable_view3d_btn(True)
        self.window.set_status("3D Done")

    @pyqtSlot()
    def on_algo3d(self):
        """CSV -> Calibrate -> 3D view (same color & interaction as 3D-View)"""
        fname, _ = QFileDialog.getOpenFileName(
            self.window, "Select CSV", "", "CSV (*.csv)")
        if not fname:
            return
        try:
            raw = np.loadtxt(fname, delimiter=',', ndmin=2)
            if raw.shape[1] != 3:
                raise ValueError("CSV must be N×3")
            b, A = fit_ellipsoid_3d(raw)
            if b is None or A is None:
                QMessageBox.warning(self.window, "Error", "Algorithm failed")
                return
            cal = (A @ (raw - b).T).T

            # 与 3D-View 完全一致的分段颜色
            half = len(cal) // 2
            level_cal = cal[:half]
            tilt_cal  = cal[half:]

            # 独立弹窗
            dlg = QDialog(self.window)
            dlg.setWindowTitle("Algorithm 3D View")
            dlg.resize(800, 600)

            fig = plt.figure(figsize=(8, 6))
            ax  = fig.add_subplot(111, projection='3d')
            ax.set_box_aspect([1, 1, 1])

            # 与 3D-View 完全一致的配色
            if len(level_cal):
                ax.scatter(level_cal[:, 0], level_cal[:, 1], level_cal[:, 2],
                           c='#ff0080', s=4, label='Level', depthshade=True)
            if len(tilt_cal):
                ax.scatter(tilt_cal[:, 0], tilt_cal[:, 1], tilt_cal[:, 2],
                           c='#00e5ff', s=4, label='Tilt', depthshade=True)

            draw_unit_sphere(ax, r=1.0)
            ax.legend()
            ax.set_title("Algorithm Calibrated 3D View")

            canvas = FigureCanvas(fig)
            lay = QVBoxLayout(dlg)
            lay.addWidget(canvas)

            # ✅ 添加旋转动画
            from matplotlib.animation import FuncAnimation
            anim = FuncAnimation(fig, lambda i: ax.view_init(20, i % 360),
                                frames=360, interval=50, repeat=True)

            dlg.exec_()

        except Exception as e:
            QMessageBox.critical(self.window, "Error", str(e))


    def view_result_3d(self):
        if self.freeze_data is None:
            self.window.set_status("No frozen data. Run calibration first.")
            return
        xyz = np.array(self.freeze_data)
        half = len(xyz) // 2
        level_raw, tilt_raw = xyz[:half], xyz[half:]
        pts_centered = xyz - self.freeze_b
        pts_cal = (self.freeze_A @ pts_centered.T).T
        norms = np.linalg.norm(pts_cal, axis=1, keepdims=True)
        pts_cal_unit = pts_cal / norms
        level_cal, tilt_cal = pts_cal_unit[:half], pts_cal_unit[half:]

        self.ax3d.clear()
        if len(level_raw):
            self.ax3d.scatter(level_raw[:, 0], level_raw[:, 1], level_raw[:, 2],
                              c='#ff0080', s=8, label='Level', depthshade=True)
        if len(tilt_raw):
            self.ax3d.scatter(tilt_raw[:, 0], tilt_raw[:, 1], tilt_raw[:, 2],
                              c='#00e5ff', s=8, label='Tilt', depthshade=True)
        self.ax3d.set_title("Raw Mag Data: Level vs Tilt")
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