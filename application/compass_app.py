
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
    UNIT_SPHERE_SCALE, DECIMAL_PRECISION,
    AUTO_YAW_RANGE_MIN, AUTO_PITCH_RANGE_MIN, AUTO_ROLL_RANGE_MIN,
    AUTO_STEP0_MIN_PTS, AUTO_STEP1_MIN_PTS, AUTO_STEP2_MIN_PTS,
    GRID_STEP_DEG, POINTS_PER_GRID

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
                    # Print every line to see what's coming in
                    # print(f"Serial input: {line.strip()}")
                    
                    m = re.search(r'mag_x=\s*([-\d\.eE+-]+),\s*mag_y=\s*([-\d\.eE+-]+),\s*mag_z=\s*([-\d\.eE+-]+)', line)
                    if m:
                        mx, my, mz = -float(m.group(2)), -float(m.group(1)), float(m.group(3))
                        # print(f"Magnetometer - X: {mx}, Y: {my}, Z: {mz}")
                        
                    a = re.search(r'\s*pitch=\s*([-\d\.eE+-]+)\s*,\s*roll=\s*([-\d\.eE+-]+)\s*,\s*yaw=\s*([-\d\.eE+-]+)', line)
                    if a:
                        pitch, roll, yaw = map(float, a.groups())
                        # print(f"Euler Angles - Pitch: {pitch:.2f}°, Roll: {roll:.2f}°, Yaw: {yaw:.2f}°")
                        
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
                            print(f"Data emitted - Pitch: {pitch:.2f}°, Roll: {roll:.2f}°, Yaw: {yaw:.2f}° | MX: {mx}, MY: {my}, MZ: {mz}")
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

    # 1. 中心化
    mean = np.mean(pts, axis=0)
    pts_centered = pts - mean

    # 2. 归一化尺度：防止病态
    scale = np.std(pts_centered, axis=0)
    pts_norm = pts_centered / scale

    # 3. 椭球拟合（10 参数）
    x, y, z = pts_norm.T
    D = np.column_stack([x*x, y*y, z*z, x*y, x*z, y*z, x, y, z, np.ones_like(x)])
    reg = 1e-3 * np.trace(D.T @ D) / D.shape[0]
    coeffs = np.linalg.solve(D.T @ D + reg * np.eye(10), D.T @ np.ones_like(x))

    Aq, Bq, Cq, Dq, Eq, Fq, G, H, I, J = coeffs
    Q = np.array([[Aq, Dq/2, Eq/2], [Dq/2, Bq, Fq/2], [Eq/2, Fq/2, Cq]])
    b_norm = -np.linalg.solve(Q, [G, H, I]) / 2

    # 4. 反归一化
    b = b_norm * scale + mean

    U, S, Vt = np.linalg.svd(Q)
    scale = 1.0 / np.sqrt(np.maximum(S, 1e-12))
    A_raw = Vt.T @ np.diag(scale) @ Vt
    det = np.linalg.det(A_raw)
    A_raw = A_raw * np.sign(det)
    A = A_raw / np.linalg.norm(A_raw, ord='fro')   # ← 关键：用 Frobenius 范数归一
    return b, A

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
    # ↓ 只加这一行，防病态，不影响外部
    safe_scale = max(np.abs(centered).max(), 1e-6)
    sphere = (centered / safe_scale) @ np.linalg.inv(A * safe_scale)
    norm = np.linalg.norm(sphere, axis=1, keepdims=True)
    norm = np.where(norm < 1e-6, 1, norm)
    sphere = sphere / norm
    return sphere
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
# ... existing code until class CalibrationApp starts ...

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

        # ====== 新增：记录每步数据的格子信息 ======
        self.step_grid_info = [[], [], []]  # 为每步存储格子信息

        # 按钮信号
        self.window.step0_btn.clicked.connect(lambda: self.start_step(0))
        self.window.step1_btn.clicked.connect(lambda: self.start_step(1))
        self.window.step2_btn.clicked.connect(lambda: self.start_step(2))
        self.window.view3d_btn.clicked.connect(self.view_result_3d)
        self.window.algo3d_btn.clicked.connect(self.on_algo3d)
        self.window.reset_btn.clicked.connect(self.reset_calibration)
        # ====== 新增：自动下一步控制变量 ======
        self.current_step = 0
        self.check_timer = None
        self.step_start_idx = [0, 0, 0]
                # ------------ 复用 config 的格点/角度/点数规则 -------------
        self.NEED_PER_GRID = {
            0: POINTS_PER_GRID,
            1: POINTS_PER_GRID,
            2: POINTS_PER_GRID
        }
        self.NEED_ANGLE_SPAN = {
            0: AUTO_YAW_RANGE_MIN,
            1: AUTO_PITCH_RANGE_MIN,
            2: AUTO_ROLL_RANGE_MIN
        }

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

    @pyqtSlot(float, float, float, float, float, float)
    def handle_new_data(self, mx, my, mz, pitch, roll, yaw):
        if self.freeze_data is None:
            angle_deg = yaw
            if angle_deg < 0:
                angle_deg += 360
            grid_index = int(angle_deg // GRID_STEP_DEG) % (360 // GRID_STEP_DEG)
            # ⭐ 直接带当前 step 编号
            self.mag3d_data.append([mx, my, mz, pitch, roll, yaw, grid_index, self.current_step])

    def start_step(self, step: int):
        """自动判断数据质量后进入下一步"""
        port = self.window.port_combo.currentText()
        baud = int(self.window.baud_combo.currentText())
        if "No" in port:
            QMessageBox.warning(self.window, "Error", "No port")
            return

        # 保存当前步骤号
        self.current_step = step

        # 记录本步骤起始索引
        self.step_start_idx[step] = len(self.mag3d_data)
        
        # 清空当前步骤的格子信息
        self.step_grid_info[step] = []

        if step == 0:
            # 仅 Step 0 清空全局数据
            self.mag3d_data.clear()
            self.freeze_data = None

        btn_list = [self.window.step0_btn, self.window.step1_btn, self.window.step2_btn]
        btn_list[step].setEnabled(False)

        # 启动串口线程
        self.thread = SerialThread(port, baud, self.data_bridge)
        self.thread.start()

        # 新增：QTimer 每秒检查一次数据质量
        self.check_timer = QTimer()
        self.check_timer.timeout.connect(lambda: self.check_step_completion(step))
        self.check_timer.start(1000)

        self.window.set_status(f"Step {step + 1} running... waiting for quality data")

    def check_step_completion(self, step: int):
        start = self.step_start_idx[step]
        data = np.array(self.mag3d_data[start:])

        # 基础健壮
        if data.ndim != 2 or data.shape[0] < 100 or data.shape[1] < 6:
            self.window.set_status(f"Step {step+1} 数据不足 ({len(data)})")
            return

        yaw   = np.where(data[:, 5] < 0, data[:, 5] + 360, data[:, 5])
        pitch = data[:, 3]
        roll  = data[:, 4]

        # 复用 config 变量
        need_per  = self.NEED_PER_GRID[step]
        need_span = self.NEED_ANGLE_SPAN[step]
        bins_yaw  = np.arange(0, AUTO_YAW_RANGE_MIN   + 1, GRID_STEP_DEG)
        bins_pr   = np.arange(-90, 91, GRID_STEP_DEG)

        cnt_yaw,   _ = np.histogram(yaw,   bins=bins_yaw)
        cnt_pitch, _ = np.histogram(pitch, bins=bins_pr)
        cnt_roll,  _ = np.histogram(roll,  bins=bins_pr)

        # 每轴满格判定
        yaw_ok   = (cnt_yaw   >= need_per).all()
        pitch_ok = (cnt_pitch >= need_per).all()
        roll_ok  = (cnt_roll  >= need_per).all()

        # 只检查当前步骤需要的轴
        if step == 0:   ok = yaw_ok
        elif step == 1: ok = pitch_ok
        else:           ok = roll_ok

        # 状态提示
        self.window.set_status(
            f"Step {step+1} 运行中... "
            f"yaw:{np.sum(cnt_yaw>=need_per)}/{len(bins_yaw)-1}  "
            f"pitch:{np.sum(cnt_pitch>=need_per)}/{len(bins_pr)-1}  "
            f"roll:{np.sum(cnt_roll>=need_per)}/{len(bins_pr)-1}"
        )

        # 满足 → 下一步
        if ok:
            self.check_timer.stop()
            self.thread.stop()

            btn_list = [self.window.step0_btn, self.window.step1_btn, self.window.step2_btn]
            if step < 2:
                btn_list[step + 1].setEnabled(True)
                self.window.set_status(f"Step {step + 1} ✓  准备 Step {step + 2}")
            else:
                # 只在 Step 2 真正结束
                self.freeze_data = list(self.mag3d_data)  # ← 先赋值
                self.finish_steps()
                
    def reset_calibration(self):
        if self.check_timer is not None:
            self.check_timer.stop()
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
        if self.check_timer is not None:
            self.check_timer.stop()
        if self.thread:
            self.thread.stop()
        if len(self.freeze_data) < MIN_3D_POINTS:
            self.window.set_status(f"Need ≥{MIN_3D_POINTS} points")
            return

        self.freeze_data = list(self.mag3d_data)
        self.freeze_b, self.freeze_A = fit_ellipsoid_3d(self.freeze_data)
        if self.freeze_b is None or self.freeze_A is None:
            self.window.set_status("3D Calibration failed")
            return

        # ---------- 1) 保存带 step_id 的 CSV ----------
       # 新：直接取采集时记录的 step_id（第8列）
        step_ids = np.array([row[-1] for row in self.freeze_data])
        save_arr = np.hstack([np.array(self.freeze_data), step_ids.reshape(-1, 1)])
        np.savetxt(os.path.join(CALIB_DIR, "raw_mag_with_orientation.csv"),
                   save_arr, delimiter=',', fmt='%.6f')

        # ---------- 2) 其余不变 ----------
        xyz = np.array(self.freeze_data)[:, :3]
        pts_cal_unit = ellipsoid_to_sphere(xyz, self.freeze_b, self.freeze_A)
        np.savetxt(os.path.join(CALIB_DIR, "calibrated_mag.csv"),
                   pts_cal_unit, delimiter=',', fmt='%.8f')
        c_code = generate_c_code_3d(self.freeze_b, np.linalg.inv(self.freeze_A))
        with open(os.path.join(CALIB_DIR, "mag_calibration.h"), 'w', encoding='utf-8') as f:
            f.write(c_code)

        self.timer.stop()
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

        # 直接使用下位机 yaw
        yaws = np.array(self.freeze_data)[:, 5]   # 第5列就是 yaw

        # 根据 yaw 角划分颜色
        color_map = {
            (0, 90): 'r',   # 第一象限
            (90, 180): 'g', # 第二象限
            (180, 270): 'b',# 第三象限
            (270, 360): 'm' # 第四象限
        }

        for (start, end), color in color_map.items():
            mask = ((yaws >= start) & (yaws < end)) | ((yaws + 360 >= start) & (yaws + 360 < end))
            ax2d.scatter(pts_cal_unit[mask, 0], pts_cal_unit[mask, 1], s=4, c=color, label=f'{start}-{end} deg')

        circle = plt.Circle((0, 0), 1, color='r', fill=False, linestyle='--')
        ax2d.add_patch(circle)
        ax2d.set_title("Calibrated XY Plane Projection")
        ax2d.legend()
        # 计算并显示三行指标
        distances = np.sqrt(pts_cal_unit[:, 0]**2 + pts_cal_unit[:, 1]**2)
        mean_dist = np.mean(distances)
        std_dist  = np.std(distances)
        circularity_error = std_dist / mean_dist if mean_dist != 0 else 0

        ax2d.text(0.02, 0.98,
                  f"Mean distance: {mean_dist:.4f}\n"
                  f"Std deviation: {std_dist:.4f}\n"
                  f"Circularity error: {circularity_error:.4f}",
                  transform=ax2d.transAxes,
                  verticalalignment='top',
                  fontsize=10,
                  bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

        canvas2d = FigureCanvas2D(fig2d)
        layout.addWidget(canvas2d)
        dlg2d.setLayout(layout)
        dlg2d.exec_()
        
       # 额外显示角度分布直方图
        self.show_step0_angle_distribution_histogram()
    @staticmethod
    def angle_color(yaw_vals):
        colors = np.full(yaw_vals.shape, 'gray', dtype=object)
        mask0 = (yaw_vals >= 0)   & (yaw_vals < 90)
        mask1 = (yaw_vals >= 90)  & (yaw_vals < 180)
        mask2 = (yaw_vals >= 180) & (yaw_vals < 270)
        mask3 = (yaw_vals >= 270) & (yaw_vals < 360)
        colors[mask0] = 'red'
        colors[mask1] = 'green'
        colors[mask2] = 'blue'
        colors[mask3] = 'orange'
        return colors
    def show_step0_angle_distribution_histogram(self):
        """显示第一步数据校准前后的XY图，按90°分段着色"""
        if self.freeze_data is None or self.freeze_b is None or self.freeze_A is None:
            return

        # 仅取第一步数据
        step0_data = np.array(self.freeze_data)[
            self.step_start_idx[0]:self.step_start_idx[1], :3
        ]
        mx_raw = step0_data[:, 0]
        my_raw = step0_data[:, 1]

        # 硬铁 + 软铁校准
        pts_centered = step0_data - self.freeze_b
        pts_cal = (self.freeze_A @ pts_centered.T).T
        norm = np.linalg.norm(pts_cal, axis=1, keepdims=True)
        norm = np.where(norm < 1e-6, 1, norm)
        pts_cal_unit = pts_cal / norm
        cal_mx = pts_cal_unit[:, 0]
        cal_my = pts_cal_unit[:, 1]

        # 直接用 step0 的 yaw 值着色
        step0_yaw = np.array(self.freeze_data)[
            self.step_start_idx[0]:self.step_start_idx[1], 5
        ]
        colors_raw = self.angle_color(step0_yaw)
        colors_cal = self.angle_color(step0_yaw)

        # 绘制
        dlg_xy = QDialog(self.window)
        dlg_xy.setWindowTitle("Step 0 XY Projection (90° Segments)")
        dlg_xy.resize(1200, 600)
        layout = QVBoxLayout(dlg_xy)

        fig_xy, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))

        # 原始
        ax1.scatter(mx_raw, my_raw, s=20, c=colors_raw, alpha=0.7)
        circle1 = plt.Circle((0, 0), 1, color='black', fill=False, ls='--', lw=1)
        ax1.add_patch(circle1)
        ax1.plot(0, 0, 'k+')
        ax1.axhline(0, color='black', linewidth=1.5)
        ax1.axvline(0, color='black', linewidth=1.5)
        ax1.set_aspect('equal')
        ax1.set_title("Step 0 Raw Data (90° Segments)")
        ax1.set_xlabel("Raw MX")
        ax1.set_ylabel("Raw MY")
        ax1.grid(True, alpha=0.3)

        # 校准后
        ax2.scatter(cal_mx, cal_my, s=20, c=colors_cal, alpha=0.7)
        circle2 = plt.Circle((0, 0), 1, color='black', fill=False, ls='--', lw=1)
        ax2.add_patch(circle2)
        ax2.plot(0, 0, 'k+')
        ax2.axhline(0, color='black', linewidth=1.5)
        ax2.axvline(0, color='black', linewidth=1.5)
        ax2.set_aspect('equal')
        ax2.set_title("Step 0 Calibrated Data (90° Segments)")
        ax2.set_xlabel("Calibrated MX")
        ax2.set_ylabel("Calibrated MY")
        ax2.grid(True, alpha=0.3)

        # 统计
        distances_raw = np.sqrt(mx_raw**2 + my_raw**2)
        distances_cal = np.sqrt(cal_mx**2 + cal_my**2)
        stats_txt = (
            f"Raw:  Mean={np.mean(distances_raw):.3f}, Std={np.std(distances_raw):.3f}\n"
            f"Cal:  Mean={np.mean(distances_cal):.3f}, Std={np.std(distances_cal):.3f}"
        )
        fig_xy.suptitle(stats_txt, fontsize=12)

        plt.tight_layout()
        canvas_xy = FigureCanvas2D(fig_xy)
        layout.addWidget(canvas_xy)
        dlg_xy.setLayout(layout)
        dlg_xy.exec_()


    @pyqtSlot()
    def on_algo3d(self):
        fname, _ = QFileDialog.getOpenFileName(
            self.window,
            "Select CSV (8-col, last = step_id)",
            "", "CSV (*.csv)")
        if not fname:
            return
        try:
            raw_all = np.loadtxt(fname, delimiter=',', ndmin=2)
            # 允许 7 或 8 列，兼容旧文件
            if raw_all.shape[1] not in (7, 8):
                raise ValueError("CSV must be N×7 or N×8 with step_id as last column")

            xyz     = raw_all[:, :3]
            step_id = raw_all[:, -1].astype(int)   # 最后一列总是 step_id

            # 整体椭球拟合
            b, A = fit_ellipsoid_3d(xyz)
            if b is None or A is None:
                QMessageBox.warning(self.window, "Error", "Algorithm failed")
                return

            pts_raw_unit = raw_to_unit(xyz, b)
            pts_cal_unit = ellipsoid_to_sphere(xyz, b, A)

            # ---------- 3D 总览 ----------
            dlg = QDialog(self.window)
            dlg.setWindowTitle("Algorithm 3D View")
            dlg.resize(900, 600)
            fig = plt.figure(figsize=(9, 6))
            ax_raw = fig.add_subplot(121, projection='3d')
            ax_cal = fig.add_subplot(122, projection='3d')
            for ax, title, pts in [(ax_raw, "Raw Unit Sphere", pts_raw_unit),
                                   (ax_cal, "Calibrated Unit Sphere", pts_cal_unit)]:
                draw_unit_sphere(ax, 1.0)
                for s, c, lab in [(0, '#ff0080', 'Step0'),
                                  (1, '#00e5ff', 'Step1'),
                                  (2, '#8000FF', 'Step2')]:
                    seg = pts[step_id == s]
                    if len(seg):
                        ax.scatter(*seg.T, c=c, s=4, label=lab, depthshade=False)
                ax.set_title(title)
                ax.set_box_aspect([1, 1, 1])
                ax.legend()
            canvas = FigureCanvas(fig)
            lay = QVBoxLayout(dlg)
            lay.addWidget(canvas)
            dlg.setLayout(lay)
            dlg.exec_()

            # ---------- Step0 XY 图（使用下位机 yaw 角着色） ----------
            mask0 = step_id == 0
            xy0_raw = xyz[mask0, :2]
            xy0_cal = pts_cal_unit[mask0, :2]

            # 取出 Step0 的下位机 yaw 角（CSV 第6列）
            yaw_step0 = raw_all[mask0, 5]          # 下位机原始 yaw
            yaw_step0 = np.where(yaw_step0 < 0, yaw_step0 + 360, yaw_step0)

            # 颜色映射：每90度一种颜色
            def color_of_yaw(y):
                if 0   <= y < 90:   return 'red'
                if 90  <= y < 180:  return 'green'
                if 180 <= y < 270:  return 'blue'
                return 'orange'

            colors = [color_of_yaw(y) for y in yaw_step0]

            dlg = QDialog(self.window)
            dlg.setWindowTitle("Step0 XY – Yaw Color")
            dlg.resize(900, 450)
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

            for ax, pts, title in [(ax1, xy0_raw, "Raw"), (ax2, xy0_cal, "Calibrated")]:
                for c, label in [('red',   '0-90°'),
                                 ('green', '90-180°'),
                                 ('blue',  '180-270°'),
                                 ('orange','270-360°')]:
                    mask = [clr == c for clr in colors]
                    ax.scatter(pts[mask, 0], pts[mask, 1], s=8, c=c, label=label)
                ax.add_patch(plt.Circle((0, 0), 1, ls='--', ec='r', fc='none'))
                ax.plot(0, 0, 'k+')
                ax.set_aspect('equal')
                ax.set_title(title)
                ax.legend()
                ax.grid(alpha=0.3)
                # 计算并显示三行指标
                distances = np.linalg.norm(xy0_cal, axis=1)
                mean_dist = np.mean(distances)
                std_dist = np.std(distances)
                circularity_error = std_dist / mean_dist if mean_dist != 0 else 0

                ax.text(0.02, 0.98,
                        f"Mean distance: {mean_dist:.4f}\n"
                        f"Std deviation: {std_dist:.4f}\n"
                        f"Circularity error: {circularity_error:.4f}",
                        transform=ax.transAxes,
                        verticalalignment='top',
                        fontsize=10,
                        family='monospace',
                        bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor='none', alpha=0.9))

            canvas = FigureCanvas(fig)
            lay = QVBoxLayout(dlg)
            lay.addWidget(canvas)
            dlg.setLayout(lay)
            dlg.exec_()

        except Exception as e:
            QMessageBox.critical(self.window, "Error", str(e))

    def show_step0_calibrated_xy_from_algo(self, b, A, raw_data):
        """显示算法处理后第一步数据的XY图"""
        # 获取第一步数据（前1/3的数据）
        n = len(raw_data)
        k = n // 3
        step0_data = raw_data[:k, :3]  # 只取前1/3的数据和前3列（mx, my, mz）
        
        # 应用校准算法到第一步数据
        pts_cal_unit = ellipsoid_to_sphere(step0_data, b, A)
        cal_mx = pts_cal_unit[:, 0]
        cal_my = pts_cal_unit[:, 1]
        
        # 计算角度用于着色
        yaws_step0 = raw_data[:k, 5]
        angles_deg = np.where(angles_deg < 0, angles_deg + 360, yaws_step0)
        
        # 创建新的对话框显示第一步校准后的XY图
        dlg_xy = QDialog(self.window)
        dlg_xy.setWindowTitle("Step 0 Calibrated XY Data")
        dlg_xy.resize(600, 500)
        layout = QVBoxLayout(dlg_xy)
        
        fig_xy, ax_xy = plt.subplots(figsize=(8, 8))
        ax_xy.set_aspect('equal')
        
        # 根据角度着色，每90度一个颜色
        colors_map = {
            (0, 90): 'red',
            (90, 180): 'green', 
            (180, 270): 'blue',
            (270, 360): 'orange'
        }
        
        for (start, end), color in colors_map.items():
            mask = ((yaws_step0 >= start) & (yaws_step0 < end)) | \
                   ((yaws_step0 + 360 >= start) & (yaws_step0 + 360 < end))
            ax_xy.scatter(cal_mx[mask], cal_my[mask], s=8, c=color, alpha=0.7, label=f'{start}°-{end}°')
        
        # 绘制参考单位圆
        circle = plt.Circle((0, 0), 1, color='red', fill=False, linestyle='--', linewidth=2)
        ax_xy.add_patch(circle)
        
        # 绘制原点
        ax_xy.plot(0, 0, 'k+', markersize=10, markeredgewidth=2)
        
        # 设置标题和标签
        ax_xy.set_title("Step 0 Data After Ellipsoid Calibration - XY Projection")
        ax_xy.set_xlabel("Calibrated MX")
        ax_xy.set_ylabel("Calibrated MY")
        ax_xy.grid(True, alpha=0.3)
        ax_xy.legend()
        # 计算并显示指标
        distances = np.sqrt(cal_mx**2 + cal_my**2)
        mean_dist = np.mean(distances)
        std_dist = np.std(distances)
        circularity_error = std_dist / mean_dist if mean_dist != 0 else 0

        ax_xy.text(0.02, 0.98,
                   f"Mean distance: {mean_dist:.4f}\n"
                   f"Std deviation: {std_dist:.4f}\n"
                   f"Circularity error: {circularity_error:.4f}",
                   transform=ax_xy.transAxes,
                   verticalalignment='top',
                   bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        # 计算并显示统计数据
        distances = np.sqrt(cal_mx**2 + cal_my**2)
        mean_dist = np.mean(distances)
        std_dist = np.std(distances)
        
        # 计算圆度误差（标准差/均值）
        circularity_error = std_dist / mean_dist if mean_dist != 0 else 0
        
        stats_text = f'Statistics:\n' \
                    f'  Mean distance: {mean_dist:.4f}\n' \
                    f'  Std deviation: {std_dist:.4f}\n' \
                    f'  Circularity error: {circularity_error:.4f}'
        ax_xy.text(0.02, 0.98, stats_text, transform=ax_xy.transAxes, 
                  verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

        canvas_xy = FigureCanvas2D(fig_xy)
        layout.addWidget(canvas_xy)
        dlg_xy.setLayout(layout)
        dlg_xy.exec_()
    def _show_step0_xy_offline(self, xy_raw, xy_cal):
        """离线 Step0 XY 图并打印三指标"""
        def metrics_str(xy):
            d = np.linalg.norm(xy, axis=1)
            mean, std = d.mean(), d.std()
            return f"Mean={mean:.4f}\nStd={std:.4f}\nCircErr={std/mean:.4f}"

        dlg = QDialog(self.window)
        dlg.setWindowTitle("Step0 XY – Offline")
        dlg.resize(900, 450)
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        for ax, pts, title, txt in [(ax1, xy_raw, "Raw", metrics_str(xy_raw)),
                                    (ax2, xy_cal, "Calibrated", metrics_str(xy_cal))]:
            ax.scatter(pts[:, 0], pts[:, 1], s=8, c='b')
            ax.add_patch(plt.Circle((0, 0), 1, ls='--', ec='r', fc='none'))
            ax.plot(0, 0, 'k+')
            ax.set_aspect('equal')
            ax.set_title(f"{title}\n{txt}")
            ax.grid(alpha=0.3)
        plt.tight_layout()
        lay = QVBoxLayout(dlg)
        lay.addWidget(FigureCanvas(fig))
        dlg.setLayout(lay)
        dlg.exec_()    
    def run(self):
        self.window.show()
        sys.exit(self.app.exec_())

if __name__ == "__main__":
    app = CalibrationApp()
    app.run()