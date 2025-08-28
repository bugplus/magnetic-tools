
# 磁力计三步校准上位机
# 作者：<bugplus>
# 版本：1.0
# 日期：2025-08-17
# 描述：磁力计三步校准上位机，用于校准磁力计，校准结果保存到文件中。
# 
#  水平数据校准：
#   1. 获取3D点云数据
#   2. 拟合3D椭球
#   3. 生成C代码
#   4. 保存C代码到文件
#   5. 运行C代码
#   6. 获取结果
#   7. 保存结果到文件
#   8. 显示结果
#   9. 循环以上步骤

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
from scipy.linalg import sqrtm   # 需要 SciPy
from config import (
    CALIBRATION_DURATION, MIN_3D_POINTS,
    ANGLE_GATE_DEG, DIST_GATE_CM,
    UNIT_SPHERE_SCALE, DECIMAL_PRECISION,
    AUTO_YAW_RANGE_MIN, AUTO_PITCH_RANGE_MIN, AUTO_ROLL_RANGE_MIN,
    AUTO_STEP0_MIN_PTS, AUTO_STEP1_MIN_PTS, AUTO_STEP2_MIN_PTS,
    GRID_STEP_DEG, POINTS_PER_GRID,STEP0_YAW_STEP_DEG ,STEP0_PPG          

)
import os
CALIB_DIR = os.path.join(os.path.dirname(__file__), "calibration_mag")
os.makedirs(CALIB_DIR, exist_ok=True)

# ------------------------------------------------------------------
# 通用正球校准：Step-0 2D圆心 + 全数据正球，主轴过圆心
# ------------------------------------------------------------------
def run_sphere_calibration_algorithm(xyz_all: np.ndarray,
                                     step_id: np.ndarray) -> tuple:
    from numpy.linalg import lstsq

    # 1. Step0 拟合正圆 → 圆心(cx,cy)
    mask0 = step_id == 0
    xy0 = xyz_all[mask0, :2]
    x, y = xy0[:, 0], xy0[:, 1]
    cx, cy, _ = lstsq(np.c_[2*x, 2*y, np.ones_like(x)],
                      x**2 + y**2, rcond=None)[0]

    # 2. 固定(cx,cy) 求 bz & 半径
    A_sphere = np.c_[-2*xyz_all[:, 2], np.ones(len(xyz_all))]
    rhs = (xyz_all[:, 0]-cx)**2 + (xyz_all[:, 1]-cy)**2 + xyz_all[:, 2]**2
    bz, r_sq = lstsq(A_sphere, rhs, rcond=None)[0]
    radius = np.sqrt(r_sq + bz**2)

    # 3. 生成参数
    bias = np.array([cx, cy, bz])
    A = np.eye(3) / radius
    pts_cal = (xyz_all - bias) @ A      # ← 关键修复
    return bias, A, pts_cal
def plot_standard_calibration_result(xyz_raw, xyz_cal, step_id, yaw_raw, parent=None):
    import matplotlib.pyplot as plt
    from PyQt5.QtWidgets import QDialog, QVBoxLayout
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
    from matplotlib.colors import LightSource

    def color90(y):
        return ['red', 'green', 'blue', 'orange'][int(y) // 90 % 4]

    # 只保留 Step-0 数据
    mask0 = step_id == 0
    xy_raw0 = xyz_raw[mask0, :2]
    yaw0 = yaw_raw[mask0]          # ← 关键：只对 Step-0 的 yaw
    c90 = [color90(y) for y in yaw0]

    # Step-0 圆心
    M = np.column_stack([2 * xy_raw0[:, 0],
                         2 * xy_raw0[:, 1],
                         np.ones_like(xy_raw0[:, 0])])
    cx, cy, _ = np.linalg.lstsq(M, xy_raw0[:, 0]**2 + xy_raw0[:, 1]**2, rcond=None)[0]

    fig = plt.figure(figsize=(18, 6))

    # 3D Raw
    ax1 = fig.add_subplot(131, projection='3d')
    u, v = np.linspace(0, 2 * np.pi, 60), np.linspace(0, np.pi, 30)
    x = np.outer(np.cos(u), np.sin(v))
    y = np.outer(np.sin(u), np.sin(v))
    z = np.outer(np.ones_like(u), np.cos(v))
    ls = LightSource(azdeg=45, altdeg=45)
    rgb = ls.shade(z, cmap=plt.cm.coolwarm, vert_exag=0.1, blend_mode='soft')
    ax1.plot_surface(x, y, z, facecolors=rgb, alpha=0.4, shade=True, antialiased=True)
    colors = ['#FF0000', '#00E5FF', '#8000FF']
    for s in (0, 1, 2):
        ax1.scatter(*xyz_raw[step_id == s].T, c=colors[s], s=12, label=f'Step-{s}')
    ax1.set_title('Raw 3D Mag')
    ax1.legend()
    ax1.set_box_aspect([1, 1, 1])

    # 3D Calibrated
    ax2 = fig.add_subplot(132, projection='3d')
    ax2.plot_surface(x, y, z, facecolors=rgb, alpha=0.4, shade=True, antialiased=True)
    for s in (0, 1, 2):
        ax2.scatter(*xyz_cal[step_id == s].T, c=colors[s], s=12, label=f'Step-{s}')
    ax2.set_title('Calibrated on Unit Sphere')
    ax2.legend()
    ax2.set_box_aspect([1, 1, 1])

    # Step-0 XY 90°分色（只画 Step-0）
    ax3 = fig.add_subplot(133)
    xy_cal0 = xyz_cal[mask0, :2]
    for col, (start, end) in [('red', (0, 90)), ('green', (90, 180)),
                              ('blue', (180, 270)), ('orange', (270, 360))]:
        mask = (yaw0 >= start) & (yaw0 < end)
        ax3.scatter(xy_raw0[mask, 0], xy_raw0[mask, 1],
                    s=8, c=col, alpha=0.7, label=f'{start}-{end}° raw')
        ax3.scatter(xy_cal0[mask, 0], xy_cal0[mask, 1],
                    marker='x', s=8, c=col, alpha=0.7, label=f'{start}-{end}° cal')
    ax3.scatter(cx, cy, marker='*', color='k', s=100,
                label=f'Center ({cx:.3f},{cy:.3f})')
    ax3.add_patch(plt.Circle((0, 0), 1, ls='--', ec='k', fc='none'))
    ax3.plot(0, 0, 'k+')
    ax3.set_aspect('equal')
    ax3.set_title('Step-0 XY – 90° bands')
    ax3.legend()
    ax3.grid(alpha=0.3)

    dlg = QDialog(parent)
    dlg.setWindowTitle("Sphere Calibration Result")
    dlg.resize(1400, 600)
    layout = QVBoxLayout(dlg)
    layout.addWidget(FigureCanvas(fig))
    dlg.setLayout(layout)
    dlg.exec_()
def step0_force_circle(step0_xyz: np.ndarray):
    """仅用 Step0 水平数据 → 强制拉圆 → 返回圆心(bx,by) 和 缩放因子"""
    xy = step0_xyz[:, :2]
    x, y = xy[:, 0], xy[:, 1]
    A = np.column_stack([2 * x, 2 * y, np.ones_like(x)])
    b = x**2 + y**2
    cx, cy, c = np.linalg.lstsq(A, b, rcond=None)[0]
    r = np.sqrt(cx**2 + cy**2 + c)
    return np.array([cx, cy]), 1.0 / r
# ---------------------------------------------------
# 新增：两步通用工具
# ---------------------------------------------------
def fit_circle_step0_only(step0_xyz: np.ndarray) -> np.ndarray:
    """仅用 Step-0 水平数据求圆心 (bx,by)"""
    xy = step0_xyz[:, :2]
    x, y = xy[:, 0], xy[:, 1]
    A = np.column_stack([2 * x, 2 * y, np.ones_like(x)])
    b = x ** 2 + y ** 2
    (bx, by, _), *_ = np.linalg.lstsq(A, b, rcond=None)
    return np.array([bx, by], dtype=float)


def fit_sphere_fixed_xy(all_xyz: np.ndarray, fixed_xy: np.ndarray):
    """固定 (bx,by) 用全部数据优化球心 z 与半径 r"""
    bx, by = fixed_xy
    x, y, z = all_xyz[:, 0], all_xyz[:, 1], all_xyz[:, 2]
    A2 = np.column_stack([-2 * z, np.ones_like(z)])
    rhs = (x - bx) ** 2 + (y - by) ** 2 + z ** 2
    (bz, r_sq), *_ = np.linalg.lstsq(A2, rhs, rcond=None)
    radius = np.sqrt(r_sq + bz ** 2)
    return np.array([bx, by, bz], dtype=float), radius
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
                        # todo test
                        # mx, my, mz = -float(m.group(2)), -float(m.group(1)), float(m.group(3))
                        mx, my, mz = float(m.group(1)), float(m.group(2)), float(m.group(3))
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
from scipy.linalg import sqrtm   # 放在文件顶部 import 区即可

def fit_ellipsoid_3d(points):
    """
    修正版 3D 椭球拟合
    返回 (b, A)：
        b : 硬铁偏移 (3,)
        A : 椭球->单位球的线性变换 (3,3)，已保证平均半径=1
    """
    pts = np.asarray(points, dtype=float)[:, :3]
    if pts.shape[0] < 10:
        return None, None

    # 1. 归一化防止病态
    mean = np.mean(pts, axis=0)
    centered = pts - mean
    scale = np.std(centered, axis=0)
    scale = np.where(scale == 0, 1.0, scale)
    pts_norm = centered / scale
    x, y, z = pts_norm.T

    # 2. 设计矩阵 & 圆度约束
    D = np.column_stack([x*x, y*y, z*z, x*y, x*z, y*z, x, y, z, np.ones_like(x)])
    C = np.array([[1.0, -1.0, 0, 0, 0, 0, 0, 0, 0, 0]])
    A_aug = np.vstack([D, C])
    b_aug = np.hstack([np.ones(pts.shape[0]), 0.0])
    reg = 1e-3 * np.trace(D.T @ D) / pts.shape[0]
    coeffs, *_ = np.linalg.lstsq(A_aug.T @ A_aug + reg * np.eye(10),
                                 A_aug.T @ b_aug, rcond=None)

    # 3. 提取 Q
    Aq, Bq, Cq, Dq, Eq, Fq, G, H, I, _ = coeffs
    Q = np.array([[Aq, Dq/2, Eq/2],
                  [Dq/2, Bq, Fq/2],
                  [Eq/2, Fq/2, Cq]])

    try:
        b_norm = -np.linalg.solve(Q, [G, H, I]) / 2
    except np.linalg.LinAlgError:
        return None, None

    # 4. 椭球->单位球矩阵 A = Q^{1/2}
    A_q = sqrtm(Q)
    if np.iscomplexobj(A_q):
        return None, None
    A_q *= np.sign(np.linalg.det(A_q))

    # 5. 强制平均半径=1（归一化坐标系）
    radii = np.linalg.norm(pts_norm @ A_q, axis=1)
    A_q /= np.mean(radii)

    # 6. 还原到原始尺度
    A = A_q / scale[None, :]
    b = b_norm * scale + mean
    return b, A


# ---------------------------------------
def generate_c_code_3d(b, A):
    if b is None or A is None:
        return "/* Error: Calibration failed */"

    bx, by, bz = b
    # ✅ 使用真实椭球拟合的 A 矩阵
    lines = [
        "/* 3D mag calibration (ellipsoid fit) */",
        f"const float HARD_IRON[3] = {{{bx:.6f}f, {by:.6f}f, {bz:.6f}f}};",
        "const float SOFT_IRON[3][3] = {"
    ]
    for row in A:
        line = "  {" + ", ".join(f"{v:.6f}f" for v in row) + "},"
        lines.append(line)
    lines.append("};")
    return "\n".join(lines)

def raw_to_unit(raw_xyz, b):
    centered = raw_xyz[:, :3] - b
    length = np.linalg.norm(centered, axis=1, keepdims=True)
    length[length == 0] = 1
    return centered / length

def ellipsoid_to_sphere(raw_xyz, b, A):
    """直接用新的 A，无需再处理病态 scale"""
    return (raw_xyz[:, :3] - b) @ A.T
def draw_unit_sphere(ax, r=1.0):
    u = np.linspace(0, 2 * np.pi, 60)
    v = np.linspace(0, np.pi, 30)
    x = r * np.outer(np.cos(u), np.sin(v))
    y = r * np.outer(np.sin(u), np.sin(v))
    z = r * np.outer(np.ones_like(u), np.cos(v))
    ls = LightSource(azdeg=45, altdeg=45)
    rgb = ls.shade(z, cmap=plt.cm.coolwarm, vert_exag=0.1, blend_mode='soft')
    ax.plot_surface(x, y, z, facecolors=rgb, alpha=0.4, shade=True, antialiased=True)



def fit_circle_2d(points_xy):
    """最小二乘拟合圆，返回圆心(cx,cy)"""
    x, y = points_xy[:, 0], points_xy[:, 1]
    A = np.column_stack([2 * x, 2 * y, np.ones_like(x)])
    b = x**2 + y**2
    (cx, cy, _), *_ = np.linalg.lstsq(A, b, rcond=None)
    return np.array([cx, cy], dtype=float)

def fit_sphere_fixed_xy(points_3d, fixed_xy):
    """固定球心 XY，只估 bz 与半径"""
    bx, by = fixed_xy
    x, y, z = points_3d[:, 0], points_3d[:, 1], points_3d[:, 2]
    A = np.column_stack([-2 * z, np.ones_like(z)])
    rhs = (x - bx)**2 + (y - by)**2 + z**2
    (bz, r_sq), *_ = np.linalg.lstsq(A, rhs, rcond=None)
    radius = np.sqrt(r_sq + bz**2)
    return np.array([bx, by, bz], dtype=float), radius
# -----------------------------
# 3. CalibrationApp
# -----------------------------
# ... existing code until class CalibrationApp starts ...

class CalibrationApp(QObject):
    """磁力计三步校准主应用"""

    # ================= 静态工具函数 =================
    @staticmethod
    def rotate_to_level(mx, my, mz, pitch, roll):
        """把磁矢量旋转回水平面，返回 (mx_level, my_level)"""
        pitch = np.radians(pitch)
        roll  = np.radians(roll)

        cx, sx = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(roll),  np.sin(roll)

        R_x = np.array([[1,  0,   0],
                        [0, cx,  sx],
                        [0,-sx,  cx]])

        R_y = np.array([[ cy, 0, -sy],
                        [  0, 1,   0],
                        [ sy, 0,  cy]])

        R = R_y @ R_x
        m_body = np.array([mx, my, mz])
        m_level = R @ m_body
        return float(m_level[0]), float(m_level[1])

    @staticmethod
    def fit_circle_2d(points_xy):
        """最小二乘拟合二维圆，返回 (cx, cy) 和 radius"""
        x, y = points_xy[:, 0], points_xy[:, 1]
        A = np.column_stack([2 * x, 2 * y, np.ones_like(x)])
        b = x**2 + y**2
        (cx, cy, c), *_ = np.linalg.lstsq(A, b, rcond=None)
        radius = np.sqrt(cx**2 + cy**2 + c)
        return np.array([cx, cy], dtype=float), radius

    @staticmethod
    def fit_sphere_fixed_xy(points_3d, fixed_xy):
        """固定 xy 球心，仅优化 bz 与半径"""
        bx, by = fixed_xy
        x, y, z = points_3d[:, 0], points_3d[:, 1], points_3d[:, 2]
        A = np.column_stack([-2 * z, np.ones_like(z)])
        rhs = (x - bx)**2 + (y - by)**2 + z**2
        (bz, r_sq), *_ = np.linalg.lstsq(A, rhs, rcond=None)
        radius = np.sqrt(r_sq + bz**2)
        return np.array([bx, by, bz], dtype=float), radius

    # ================ 构造/析构 ================
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
        # 只对 Step0 用 2° 步长，其余保持 10°
        if step == 0:
            bins_yaw = np.arange(0, 361, 2)   # 2° 一格，共 180 格
            need_per = 3                      # 每格至少 3 点
        else:
            bins_yaw = np.arange(0, AUTO_YAW_RANGE_MIN + 1, GRID_STEP_DEG)
            need_per = POINTS_PER_GRID
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


    @staticmethod
    def enforce_axis_through_circle(cx, cy, b, A):
        """
        把椭球硬铁 XY 强制拉到 (cx,cy)，Z 不变
        返回新 bias
        """
        b = b.copy()                       # 避免原地修改
        b[:2] = np.linalg.solve(A[:2, :2], [cx, cy]) + b[:2]
        return b
    
    def finish_steps(self):
        """仅采集数据并画标准图，算法由 Algo3D 固化"""
        if self.check_timer is not None:
            self.check_timer.stop()
        if hasattr(self, 'thread') and self.thread:
            self.thread.stop()

        raw = np.array(self.freeze_data)
        xyz_all = raw[:, :3]
        step_id = raw[:, -1].astype(int)
        yaw_all = raw[:, 5]

        # 保存原始数据
        os.makedirs(CALIB_DIR, exist_ok=True)
        np.savetxt(os.path.join(CALIB_DIR, "raw_mag_with_orientation.csv"),
                   raw, delimiter=',', fmt='%.6f')

        # 仅画图，不跑算法
        plot_standard_calibration_result(
            xyz_all, xyz_all, step_id, yaw_all, parent=self.window)
        self.window.set_status("Step data collected. Check the plot!")
    def view_result_3d(self):
        if self.freeze_data is None:
            return

        raw = np.array(self.freeze_data)
        xyz = raw[:, :3]
        step = raw[:, -1].astype(int)

        # 用新的校准参数
        if self.freeze_b is None or self.freeze_A is None:
            QMessageBox.warning(self.window, "Error",
                                "请先完成校准，再查看 3D 结果！")
            return

        scale = 1.0 / np.linalg.norm(self.freeze_A[0])
        pts_cal = (xyz - self.freeze_b) * scale

        yaw = raw[:, 5]
        yaw = np.where(yaw < 0, yaw + 360, yaw)

        # 90-degree coloring for Step-0 only
        def color90(y):
            if 0 <= y < 90:
                return 'red'
            elif 90 <= y < 180:
                return 'lime'
            elif 180 <= y < 270:
                return 'blue'
            else:
                return 'magenta'

        step_colors = {0: 'red', 1: 'lime', 2: 'blue'}

        fig = plt.figure(figsize=(18, 6))

        # 1) Raw 3D
        ax1 = fig.add_subplot(131, projection='3d')
        u = np.linspace(0, 2 * np.pi, 60)
        v = np.linspace(0, np.pi, 30)
        x = np.outer(np.cos(u), np.sin(v))
        y = np.outer(np.sin(u), np.sin(v))
        z = np.outer(np.ones_like(u), np.cos(v))
        ax1.plot_surface(x, y, z, color='skyblue', alpha=0.25, rstride=1, cstride=1)
        for s in (0, 1, 2):
            ax1.scatter(*xyz[step == s].T, c=step_colors[s], s=12, label=f'Step-{s}', depthshade=False)
        ax1.set_title('Raw Points')
        ax1.legend()
        ax1.set_box_aspect([1, 1, 1])

        # 2) Calibrated 3D
        ax2 = fig.add_subplot(132, projection='3d')
        ax2.plot_surface(x, y, z, color='skyblue', alpha=0.25, rstride=1, cstride=1)
        for s in (0, 1, 2):
            ax2.scatter(*pts_cal[step == s].T, c=step_colors[s], s=12, label=f'Step-{s}', depthshade=False)
        ax2.set_title('Calibrated on Unit Sphere')
        ax2.legend()
        ax2.set_box_aspect([1, 1, 1])

        # 3) Step-0 XY projection with 90-degree bands
        mask0 = step == 0
        xy_raw = xyz[mask0, :2]
        xy_cal = pts_cal[mask0, :2]
        yaw0 = yaw[mask0]
        c90 = [color90(y) for y in yaw0]

        ax3 = fig.add_subplot(133)
        ax3.scatter(*xy_raw.T, c=c90, s=8, alpha=0.8, label='Raw XY')
        ax3.scatter(*xy_cal.T, c=c90, s=8, alpha=0.8, label='Cal XY')
        ax3.add_patch(plt.Circle((0, 0), 1, ls='--', ec='k', fc='none'))
        ax3.plot(0, 0, 'k+')
        ax3.set_aspect('equal')
        ax3.legend()
        ax3.set_title('Step-0 XY (90-deg bands)')
        ax3.grid(alpha=0.3)

        dlg = QDialog(self.window)
        dlg.setWindowTitle('Step-0 90-degree View')
        dlg.resize(1400, 600)
        layout = QVBoxLayout(dlg)
        layout.addWidget(FigureCanvas(fig))
        dlg.setLayout(layout)
        dlg.exec_()
    @pyqtSlot()
    def on_algo3d(self):
        fname, _ = QFileDialog.getOpenFileName(self.window, "Select CSV", CALIB_DIR, "CSV (*.csv)")
        if not fname:
            return
        try:
            raw = np.loadtxt(fname, delimiter=',', ndmin=2)
            print(f"[DEBUG] 原始形状 {raw.shape}")
            print(f"[DEBUG] 原始前3行\n{raw[:3]}")

            if raw.shape[1] not in (8, 9):
                raise ValueError(f"CSV 必须是 8 或 9 列，当前 {raw.shape[1]} 列")

            # ✅ 正确提取数据
            xyz_all = raw[:, 0:3]
            step_id = raw[:, 7].astype(int)
            yaw_all = raw[:, 5]

            # ✅ 强制长度一致检查
            assert len(xyz_all) == len(step_id) == len(yaw_all), \
                f"行数不一致：xyz={len(xyz_all)}  step={len(step_id)}  yaw={len(yaw_all)}"

            # ✅ 运行校准算法
            bias, A, pts_cal = run_sphere_calibration_algorithm(xyz_all, step_id)

            # ✅ 显示标准图
            plot_standard_calibration_result(
                xyz_all, pts_cal, step_id, yaw_all, parent=self.window)

            # ✅ 显示 Step-0 校准后的 XY 图
            self.show_step0_calibrated_xy_from_algo(raw)

        except Exception as e:
            import traceback
            traceback.print_exc()
            QMessageBox.critical(self.window, "Algo3D Error", str(e))
    # ------------------------------------------------------------------
    # 新增：强制 Step0 XY 为正圆
    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # 新增：强制 Step0 XY 为正圆
    # ------------------------------------------------------------------
    def force_step0_xy_to_unit_circle(self, raw_data):
        """
        强制 Step0 数据 XY 投影为正圆，圆心 (0,0)，半径 1
        返回：cal_xy (N×2)
        """
        # 1. 取 Step0 数据
        mask = raw_data[:, 7] == 0
        xyz = raw_data[mask, :3]
        pitch = raw_data[mask, 3]
        roll  = raw_data[mask, 4]

        # 2. 旋转到水平面
        def rotate_to_level(mx, my, mz, pitch, roll):
            pitch = np.radians(pitch)
            roll  = np.radians(roll)
            cx, sx = np.cos(pitch), np.sin(pitch)
            cy, sy = np.cos(roll),  np.sin(roll)

            R_x = np.array([[1, 0,   0],
                            [0, cx,  sx],
                            [0,-sx,  cx]])
            R_y = np.array([[ cy, 0, -sy],
                            [  0, 1,   0],
                            [ sy, 0,  cy]])
            R = R_y @ R_x
            m_body = np.array([mx, my, mz])
            m_level = R @ m_body
            return float(m_level[0]), float(m_level[1])

        xy_level = np.array([
            rotate_to_level(mx, my, mz, p, r)
            for mx, my, mz, p, r in zip(xyz[:,0], xyz[:,1], xyz[:,2], pitch, roll)
        ])

        # 3. 强制圆心 (0,0)：先找圆心再平移
        x, y = xy_level[:,0], xy_level[:,1]
        A_mat = np.column_stack([2*x, 2*y, np.ones_like(x)])
        b_vec = x**2 + y**2
        (cx, cy, _), *_ = np.linalg.lstsq(A_mat, b_vec, rcond=None)
        xy_centered = xy_level - [cx, cy]

        # 4. 强制半径 1
        r_mean = np.linalg.norm(xy_centered, axis=1).mean()
        xy_unit = xy_centered / r_mean
        return xy_unit    
    def show_step0_calibrated_xy_from_algo(self, raw_data):
        """Step0 强制正圆：圆心(0,0) 半径 1"""
        # 1. 取出 Step0 数据
        mask = raw_data[:, 7] == 0
        xyz   = raw_data[mask, :3]
        pitch = raw_data[mask, 3]
        roll  = raw_data[mask, 4]

        # 2. 旋转到水平面
        def rotate(mx, my, mz, p, r):
            p, r = np.radians(p), np.radians(r)
            cx, sx, cy, sy = np.cos(p), np.sin(p), np.cos(r), np.sin(r)
            R = np.array([[cy,  sx*sy, -sy*cx],
                          [0,   cx,    sx],
                          [sy, -sx*cy,  cx]])
            return R @ [mx, my, mz]

        xy_level = np.array([
            rotate(mx, my, mz, p, r)[:2]
            for mx, my, mz, p, r in zip(xyz[:, 0], xyz[:, 1], xyz[:, 2], pitch, roll)
        ])

        # 3. 强制圆心(0,0) + 半径1
        cx, cy, _ = np.linalg.lstsq(np.c_[2*xy_level, np.ones(len(xy_level))],
                                    (xy_level**2).sum(1), rcond=None)[0]
        xy_centered = xy_level - [cx, cy]
        xy_unit = xy_centered / np.linalg.norm(xy_centered, axis=1, keepdims=True)

        # 4. 画图
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.scatter(xy_unit[:, 0], xy_unit[:, 1], s=8, c='blue')
        ax.add_patch(plt.Circle((0, 0), 1, ec='red', ls='--', fill=False))
        ax.plot(0, 0, 'k+')
        ax.set_aspect('equal')
        ax.set_title("Step0 XY - FORCED UNIT CIRCLE")
        ax.set_xlabel("Calibrated MX")
        ax.set_ylabel("Calibrated MY")
        ax.grid(alpha=0.3)

        dlg = QDialog(self.window)
        dlg.setWindowTitle("Step0 - FORCED UNIT CIRCLE")
        dlg.resize(600, 600)
        dlg.setLayout(QVBoxLayout(dlg))
        dlg.layout().addWidget(FigureCanvas(fig))
        dlg.exec_()
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