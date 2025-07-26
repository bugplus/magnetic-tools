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
from config import (
    CALIBRATION_DURATION, MIN_3D_POINTS,
    ANGLE_GATE_DEG, DIST_GATE_CM,
    UNIT_SPHERE_SCALE, DECIMAL_PRECISION
)

# -----------------------------
# 1. 线程安全串口读取（角度门控+稀疏化）
# -----------------------------
class DataBridge(QObject):
    new_data = pyqtSignal(float, float, float, float, float)

class SerialThread(QThread):
    def __init__(self, port, baud, bridge):
        super().__init__()
        self.bridge = bridge
        self.port, self.baud = port, baud
        self.running = True
        self.cache_mag = None
        self.cache_ang = None

    def run(self):
        # 1. 删除重复 import，提升启动速度
        try:
            with serial.Serial(self.port, self.baud, timeout=1) as ser:
                last_ang = None     # [pitch, roll, yaw]
                last_xyz = None     # [mx, my, mz]

                while self.running:
                    line = ser.readline().decode(errors='ignore')

                    # 2. 正则支持指数符号
                    m = re.search(r'mag_x=\s*([-\d\.eE+-]+),\s*mag_y=\s*([-\d\.eE+-]+),\s*mag_z=\s*([-\d\.eE+-]+)', line)
                    if m:
                        self.cache_mag = list(map(float, m.groups()))

                    a = re.search(r'pitch=\s*([-\d\.eE+-]+).*roll=\s*([-\d\.eE+-]+).*yaw=\s*([-\d\.eE+-]+)', line)
                    if a:
                        self.cache_ang = list(map(float, a.groups()))

                    if self.cache_mag and self.cache_ang:
                        mx, my, mz = -self.cache_mag[1], -self.cache_mag[0], self.cache_mag[2]
                        pitch, roll, yaw = self.cache_ang
                        xyz = np.array([mx, my, mz])
                        ang = np.array([pitch, roll, yaw])

                        # 角度门控：任一方向变化≥阈值即有效
                        if last_ang is None or np.any(np.abs(ang - last_ang) >= ANGLE_GATE_DEG):
                            last_ang = ang
                        else:
                            self.cache_mag = self.cache_ang = None
                            continue

                        # 磁力稀疏化
                        if last_xyz is None or np.linalg.norm(xyz - last_xyz) >= DIST_GATE_CM * 0.01:
                            last_xyz = xyz
                            self.bridge.new_data.emit(mx, my, mz, pitch, roll)

                        self.cache_mag = self.cache_ang = None
        except Exception as e:
            # 6. 串口异常弹窗
            QMessageBox.critical(None, "Serial Error", str(e))
    def stop(self):
        self.running = False

# -----------------------------
# 2. 核心函数：椭球拟合 + C 代码生成
# -----------------------------
def fit_ellipsoid_3d(points):
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
    # 4. 小数精度 6 → 8
    lines = [
        "/* 3D mag calibration (auto) */",
        f"const float HARD_IRON[3] = {{{bx:.{DECIMAL_PRECISION}f}f, {by:.{DECIMAL_PRECISION}f}f, {bz:.{DECIMAL_PRECISION}f}f}};",
        "const float SOFT_IRON[3][3] = {",
        f"  {{{A[0, 0]:.{DECIMAL_PRECISION}f}f, {A[0, 1]:.{DECIMAL_PRECISION}f}f, {A[0, 2]:.{DECIMAL_PRECISION}f}f}},",
        f"  {{{A[1, 0]:.{DECIMAL_PRECISION}f}f, {A[1, 1]:.{DECIMAL_PRECISION}f}f, {A[1, 2]:.{DECIMAL_PRECISION}f}f}},",
        f"  {{{A[2, 0]:.{DECIMAL_PRECISION}f}f, {A[2, 1]:.{DECIMAL_PRECISION}f}f, {A[2, 2]:.{DECIMAL_PRECISION}f}f}}",
        "};"
    ]
    return "\n".join(lines)


def ellipsoid_to_sphere(raw_xyz, b, A):
    """一键：原始磁向量 → 单位球向量"""
    centered = raw_xyz - b
    sphere   = (A @ centered.T).T
    return sphere / np.linalg.norm(sphere, axis=1, keepdims=True)

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

        # ---------- 3D 画布 ----------
        # 1. Raw 3D Mag（最上方）
        self.fig3d = plt.figure()
        self.canvas3d = FigureCanvas(self.fig3d)
        self.ax3d = self.fig3d.add_subplot(111, projection='3d')
        self.ax3d.set_title("Raw 3D Mag (μT)")
        self.window.central_layout.addWidget(self.canvas3d)

        # 2. Raw Unit Sphere（左下）
        self.fig_raw_sphere = plt.figure()
        self.canvas_raw_sphere = FigureCanvas(self.fig_raw_sphere)
        self.ax_raw_sphere = self.fig_raw_sphere.add_subplot(111, projection='3d')
        self.ax_raw_sphere.set_title("Raw Unit Sphere")

        # 3. Calibrated 3D Mag（右下）
        self.fig3d_cal = plt.figure()
        self.canvas3d_cal = FigureCanvas(self.fig3d_cal)
        self.ax3d_cal = self.fig3d_cal.add_subplot(111, projection='3d')
        self.ax3d_cal.set_title("Calibrated on Unit Sphere")

        # 水平布局：左 Raw Sphere，右 Calibrated
        from PyQt5.QtWidgets import QHBoxLayout
        hbox = QHBoxLayout()
        hbox.addWidget(self.canvas_raw_sphere)
        hbox.addWidget(self.canvas3d_cal)
        self.window.central_layout.addLayout(hbox)

        # ---------- 数据桥 & 定时器 ----------
        self.data_bridge = DataBridge()
        self.data_bridge.new_data[float, float, float, float, float].connect(
            self.handle_new_data, Qt.QueuedConnection)
        self.mag3d_data = []
        self.freeze_data = None
        self.freeze_b = None
        self.freeze_A = None
        self.timer = QTimer()
        self.timer.timeout.connect(self._update_3d_plot_safe)
        self.timer.start(200)

        # ---------- 按钮信号 ----------
        self.window.step0_btn.clicked.connect(lambda: self.start_step(0))
        self.window.step1_btn.clicked.connect(lambda: self.start_step(1))
        self.window.step2_btn.clicked.connect(lambda: self.start_step(2))
        self.window.view3d_btn.clicked.connect(self.view_result_3d)
        self.window.algo3d_btn.clicked.connect(self.on_algo3d)
        self.window.reset_btn.clicked.connect(self.reset_calibration)

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
            print(f"Plot error: {e}")

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

    # 7. 重置功能：清空数据，按钮复位，可立即重新校准
    def reset_calibration(self):
        self.mag3d_data.clear()
        self.freeze_data = None
        self.freeze_b = None
        self.freeze_A = None
        self.timer.start(200)
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
        xyz = np.array(self.freeze_data)
        self.ax3d.clear()
        self.ax3d.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], c='b', s=5)
        self.ax3d.set_title(f"Raw 3D Mag ({len(xyz)} pts)")
        self.canvas3d.draw()

        pts_cal_unit = ellipsoid_to_sphere(xyz, self.freeze_b, self.freeze_A)
        # 添加校准后数据保存功能
        cal_csv_path, _ = QFileDialog.getSaveFileName(
            self.window, "Save calibrated CSV", "calibrated_mag.csv", "CSV (*.csv)")
        if cal_csv_path:
            np.savetxt(cal_csv_path, pts_cal_unit, delimiter=',', fmt='%.8f')
            self.window.set_status(f"Calibrated data saved to {cal_csv_path}")
            
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
        csv_path, _ = QFileDialog.getSaveFileName(
            self.window, "Save raw CSV", "raw_mag.csv", "CSV (*.csv)")
        if csv_path:
            np.savetxt(csv_path, np.array(self.freeze_data),
                       delimiter=',', fmt='%.6f')
        self.window.show_result_dialog(c_code)
        self.window.enable_view3d_btn(True)
        self.window.set_status("3D Three-Step Done")
    def view_result_3d(self):
        if self.freeze_data is None:
            return

        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        from PyQt5.QtWidgets import QHBoxLayout

        xyz = np.array(self.freeze_data)
        n = len(xyz)
        k = n // 3

        # 分段颜色
        colors = ['#ff0080', '#00e5ff', '#8000FF']
        labels = ['Level', 'Tilt', 'Stern']

        # 原图（最上面）
        self.ax3d.clear()
        for i, (c, lab) in enumerate(zip(colors, labels)):
            seg = xyz[i * k : (i + 1) * k]
            if len(seg):
                self.ax3d.scatter(*seg.T, c=c, s=8, label=lab, depthshade=False)
        self.ax3d.set_title("Raw 3D Mag (μT)")
        self.ax3d.legend()
        self.ax3d.set_box_aspect([1, 1, 1])
        self.canvas3d.draw()

        # 单位球坐标
        raw_unit = xyz / np.linalg.norm(xyz, axis=1, keepdims=True)
        pts_centered = xyz - self.freeze_b
        pts_cal = (self.freeze_A @ pts_centered.T).T
        pts_cal_unit = pts_cal / np.linalg.norm(pts_cal, axis=1, keepdims=True)

        # 原单位球（左）
        self.ax_raw_sphere.clear()
        draw_unit_sphere(self.ax_raw_sphere, r=1.0)
        for i, (c, lab) in enumerate(zip(colors, labels)):
            seg = raw_unit[i * k : (i + 1) * k]
            if len(seg):
                self.ax_raw_sphere.scatter(*seg.T, c=c, s=4, label=lab, depthshade=False)
        self.ax_raw_sphere.set_title("Raw Unit Sphere")
        self.ax_raw_sphere.set_xlim([-1, 1])
        self.ax_raw_sphere.set_ylim([-1, 1])
        self.ax_raw_sphere.set_zlim([-1, 1])
        self.ax_raw_sphere.set_box_aspect([1, 1, 1])
        self.ax_raw_sphere.legend()
        self.canvas_raw_sphere.draw()

        # 校准单位球（右）
        self.ax3d_cal.clear()
        draw_unit_sphere(self.ax3d_cal, r=1.0)
        for i, (c, lab) in enumerate(zip(colors, labels)):
            seg = pts_cal_unit[i * k : (i + 1) * k]
            if len(seg):
                self.ax3d_cal.scatter(*seg.T, c=c, s=4, label=lab, depthshade=False)
        self.ax3d_cal.set_title("Calibrated on Unit Sphere")
        self.ax3d_cal.set_xlim([-1, 1])
        self.ax3d_cal.set_ylim([-1, 1])
        self.ax3d_cal.set_zlim([-1, 1])
        self.ax3d_cal.set_box_aspect([1, 1, 1])
        self.ax3d_cal.legend()
        self.canvas3d_cal.draw()

        # 动画
        elev = 20
        self.anim1 = FuncAnimation(self.fig3d, lambda i: self.ax3d.view_init(elev, i % 360), frames=360, interval=50)
        self.anim2 = FuncAnimation(self.fig_raw_sphere, lambda i: self.ax_raw_sphere.view_init(elev, i % 360), frames=360, interval=50)
        self.anim3 = FuncAnimation(self.fig3d_cal, lambda i: self.ax3d_cal.view_init(elev, i % 360), frames=360, interval=50)

    @pyqtSlot()
    def on_algo3d(self):
        """
        CSV → 3-D 椭球校准 → 单位球可视化（与 Calibrated 3D View 完全一致）
        """
        fname, _ = QFileDialog.getOpenFileName(self.window, "Select CSV", "", "CSV (*.csv)")
        if not fname:
            return
        try:
            raw = np.loadtxt(fname, delimiter=',', ndmin=2)
            if raw.shape[1] != 3:
                raise ValueError("CSV must be N×3")

            # 1. 椭球拟合
            b, A = fit_ellipsoid_3d(raw)
            if b is None or A is None:
                QMessageBox.warning(self.window, "Error", "Algorithm failed")
                return

            # 2. 校准
            pts_cal_unit = ellipsoid_to_sphere(raw, b, A)

            # 3. 分段颜色
            n = len(pts_cal_unit)
            k = n // 3
            level_cal = pts_cal_unit[:k]
            tilt_cal  = pts_cal_unit[k:2*k]
            stern_cal = pts_cal_unit[2*k:]

            # 4. 弹窗 + 3D
            dlg = QDialog(self.window)
            dlg.setWindowTitle("Algorithm 3D View")
            dlg.resize(800, 600)

            fig = plt.figure(figsize=(8, 6))
            ax  = fig.add_subplot(111, projection='3d')
            ax.set_box_aspect([1, 1, 1])

            # 单位球
            draw_unit_sphere(ax, r=1.0)

            # 三段散点
            if len(level_cal):
                ax.scatter(*level_cal.T, c='#ff0080', s=4, label='Level', depthshade=False)
            if len(tilt_cal):
                ax.scatter(*tilt_cal.T, c='#00e5ff', s=4, label='Tilt', depthshade=False)
            if len(stern_cal):
                ax.scatter(*stern_cal.T, c='#8000FF', s=4, label='Stern', depthshade=False)

            ax.legend()
            ax.set_title("Algorithm Calibrated 3D View")

            canvas = FigureCanvas(fig)
            lay = QVBoxLayout(dlg)
            lay.addWidget(canvas)

            # 动画引用防崩溃
            from matplotlib.animation import FuncAnimation
            dlg.anim = FuncAnimation(fig,
                                     lambda i: ax.view_init(20, i % 360),
                                     frames=360, interval=50, repeat=True)
            dlg.exec_()

        except Exception as e:
            QMessageBox.critical(self.window, "Error", str(e))

    def run(self):
        self.window.show()
        sys.exit(self.app.exec_())

if __name__ == "__main__":
    app = CalibrationApp()
    app.run()