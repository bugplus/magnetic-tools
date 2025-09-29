
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
    GRID_STEP_DEG, POINTS_PER_GRID,STEP0_YAW_STEP_DEG ,STEP0_PPG,
    CIRCULARITY_ERROR_THRESHOLD          

)
import os
CALIB_DIR = os.path.join(os.path.dirname(__file__), "calibration_mag")
os.makedirs(CALIB_DIR, exist_ok=True)
from config import IMF_CLEAN_TH

from scipy.linalg import eigh

from config import ROBUST_N_SIGMA, ROBUST_MAX_ITER
# -------------------------------------------------------

# ------------------------------------------------------------------
# 通用正球校准：Step-0 2D圆心 + 全数据正球，主轴过圆心
# ------------------------------------------------------------------
def run_sphere_calibration_algorithm(xyz_all: np.ndarray,
                                     step_id: np.ndarray) -> tuple:
    from numpy.linalg import lstsq

    # 1. Step0 XY 圆心
    mask0 = step_id == 0
    xy0 = xyz_all[mask0, :2]
    cx, cy, _ = lstsq(np.c_[2*xy0[:,0], 2*xy0[:,1], np.ones(len(xy0))],
                      xy0[:,0]**2 + xy0[:,1]**2, rcond=None)[0]

    # 2. 固定(cx,cy) 求 bz
    A_z = np.c_[-2*xyz_all[:,2], np.ones(len(xyz_all))]
    rhs = (xyz_all[:,0]-cx)**2 + (xyz_all[:,1]-cy)**2 + xyz_all[:,2]**2
    bz, r_sq = lstsq(A_z, rhs, rcond=None)[0]
    radius = np.sqrt(r_sq + bz**2)

    # 3. 一步平移 + 缩放
    bias = np.array([cx, cy, bz])          # 球心
    A = np.eye(3) / radius                 # 整体缩放到半径 1
    pts_cal = (xyz_all - bias) @ A         # 先平移再缩放

    # 4. 确保 Step0 XY 圆心再平移到 0,0
    xy_cal0 = pts_cal[mask0, :2]
    cx_cal, cy_cal, _ = lstsq(np.c_[2*xy_cal0[:,0], 2*xy_cal0[:,1], np.ones(len(xy_cal0))],
                              xy_cal0[:,0]**2 + xy_cal0[:,1]**2, rcond=None)[0]
    # 二次平移仅作用于 XY
    pts_cal[:, :2] -= [cx_cal, cy_cal]

    # 5. 更新 bias
    bias[:2] += np.array([cx_cal, cy_cal]) * radius  # 把二次平移映射回原坐标
    return bias, A, pts_cal
def plot_standard_calibration_result(xyz_raw, xyz_cal, step_id, yaw_raw, parent=None):
    """
    两幅图：
    1. Raw 3D 与 Calibrated 3D 并排
    2. Step-0 XY Raw 与 Calibrated 并排（含圆心标注）
    """
    import matplotlib.pyplot as plt
    from PyQt5.QtWidgets import QDialog, QVBoxLayout
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
    from matplotlib.colors import LightSource

    colors = ['#FF3333',   # Step-0 鲜红
          '#33AA33',   # Step-1 翠绿
          '#3366FF']   # Step-2 亮蓝

    # --------------------------------------------------
    # 图1：3D 对比
    # --------------------------------------------------
    fig3d = plt.figure(figsize=(16, 8))

    # Raw 3D
    ax1 = fig3d.add_subplot(121, projection='3d')
    cx_raw, cy_raw, cz_raw = xyz_raw.mean(axis=0)
    u = np.linspace(0, 2 * np.pi, 60)
    v = np.linspace(0, np.pi, 30)
    x = cx_raw + np.outer(np.cos(u), np.sin(v))
    y = cy_raw + np.outer(np.sin(u), np.sin(v))
    z = cz_raw + np.outer(np.ones_like(u), np.cos(v))
    ls = LightSource(azdeg=45, altdeg=45)
    rgb = ls.shade(z, cmap=plt.cm.coolwarm, vert_exag=0.1, blend_mode='soft')
    ax1.plot_surface(x, y, z, facecolors=rgb, alpha=0.4, shade=True, antialiased=True)
    for s in (0, 1, 2):
        ax1.scatter(*xyz_raw[step_id == s].T, c=colors[s], s=12, label=f'Step-{s}')
    ax1.set_title('Raw 3D Mag')
    ax1.legend()
    ax1.set_box_aspect([1, 1, 1])

    # Calibrated 3D
    ax2 = fig3d.add_subplot(122, projection='3d')
    u = np.linspace(0, 2 * np.pi, 60)
    v = np.linspace(0, np.pi, 30)
    x = np.outer(np.cos(u), np.sin(v))
    y = np.outer(np.sin(u), np.sin(v))
    z = np.outer(np.ones_like(u), np.cos(v))
    ls = LightSource(azdeg=45, altdeg=45)
    rgb = ls.shade(z, cmap=plt.cm.coolwarm, vert_exag=0.1, blend_mode='soft')
    ax2.plot_surface(x, y, z, facecolors=rgb, alpha=0.3, shade=True, antialiased=True)
    radii = np.linalg.norm(xyz_cal, axis=1, keepdims=True)
    radii[radii == 0] = 1
    xyz_unit = xyz_cal / radii
    for s in (0, 1, 2):
        ax2.scatter(*xyz_unit[step_id == s].T, c=colors[s], s=12, label=f'Step-{s}')
    ax2.set_title('Calibrated on Unit Sphere (radius=1)')
    ax2.legend()
    ax2.set_box_aspect([1, 1, 1])

    dlg3d = QDialog(parent)
    dlg3d.setWindowTitle('3D Comparison')
    dlg3d.resize(1600, 800)
    QVBoxLayout(dlg3d).addWidget(FigureCanvas(fig3d))
    dlg3d.exec_()

    # --------------------------------------------------
    # 图2：Step-0 XY 对比（含圆心标注）
    # --------------------------------------------------
    mask0 = step_id == 0
    xy_raw0 = xyz_raw[mask0, :2]
    xy_cal0 = xyz_cal[mask0, :2]
    yaw0 = yaw_raw[mask0]
    yaw0 = np.where(yaw0 < 0, yaw0 + 360, yaw0)

    def color90(y):
        y = y % 360
        if 0 <= y < 90:   return 'red'
        if 90 <= y < 180: return 'lime'
        if 180 <= y < 270:return 'blue'
        return 'magenta'
    c90 = [color90(y) for y in yaw0]

    fig_xy = plt.figure(figsize=(12, 6))
    # 左：Raw
    ax_raw = fig_xy.add_subplot(121, aspect='equal')
    ax_raw.scatter(*xy_raw0.T, s=8, c=c90, alpha=0.8, label='Raw')
    ax_raw.add_patch(plt.Circle((0, 0), 1, ls='--', ec='k', fc='none'))
    cx_raw, cy_raw = xy_raw0.mean(axis=0)   # 圆心
    ax_raw.plot(cx_raw, cy_raw, 'r+', ms=10)
    ax_raw.text(cx_raw, cy_raw, f'({cx_raw:.2f}, {cy_raw:.2f})', color='r', fontsize=9)
    ax_raw.set_title('Step-0 XY Raw (90° colors)')
    ax_raw.legend()
    ax_raw.grid(alpha=0.3)

    # 右：Calibrated
    ax_cal = fig_xy.add_subplot(122, aspect='equal')
    ax_cal.scatter(*xy_cal0.T, s=8, c=c90, alpha=0.8, label='Calibrated')
    ax_cal.add_patch(plt.Circle((0, 0), 1, ls='--', ec='k', fc='none'))
    cx_cal, cy_cal = xy_cal0.mean(axis=0)   # 圆心
    ax_cal.plot(cx_cal, cy_cal, 'g+', ms=10)
    ax_cal.text(cx_cal, cy_cal, f'({cx_cal:.2f}, {cy_cal:.2f})', color='g', fontsize=9)
    ax_cal.set_title('Step-0 XY Calibrated (90° colors)')
    ax_cal.legend()
    ax_cal.grid(alpha=0.3)

    ax_cal.set_xlim(-1.2, 1.2)
    ax_cal.set_ylim(-1.2, 1.2)

    dlg_xy = QDialog(parent)
    dlg_xy.setWindowTitle('Step-0 XY Comparison')
    dlg_xy.resize(1200, 600)
    QVBoxLayout(dlg_xy).addWidget(FigureCanvas(fig_xy))
    dlg_xy.exec_()

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
    def project_to_horizontal(mx, my, mz, pitch, roll):
        """
        用地磁+姿态角把向量投影到水平面，返回 (mx_h, my_h)
        不用假设水平旋转，只要 pitch/roll 准，XY 就是圆
        """
        p = np.radians(pitch)
        r = np.radians(roll)

        cos_p = np.cos(p)
        sin_p = np.sin(p)
        cos_r = np.cos(r)
        sin_r = np.sin(r)

        # 水平面投影
        mx_h = mx * cos_r + mz * sin_r
        my_h = mx * sin_p * sin_r + my * cos_p - mz * sin_p * cos_r
        return float(mx_h), float(my_h)
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
        """三步数据采集完 → 自动3D椭球校准 → 画图+生成C代码"""
        if self.check_timer is not None:
            self.check_timer.stop()
        if hasattr(self, 'thread') and self.thread:
            self.thread.stop()

        raw = np.array(self.freeze_data)
        xyz_all = raw[:, :3]
        step_id = raw[:, -1].astype(int)
        yaw_all = raw[:, 5]

        os.makedirs(CALIB_DIR, exist_ok=True)
        np.savetxt(os.path.join(CALIB_DIR, "raw_mag_with_orientation.csv"),
                   raw, delimiter=',', fmt='%.6f')

        # 1. 真正做 3D 椭球校准
        bias, A, pts_cal = run_sphere_calibration_algorithm(xyz_all, step_id)

        # 2. 保存校准参数供后续查看
        self.freeze_b = bias
        self.freeze_A = A

        # 3. 画校准前后对比图
        plot_standard_calibration_result(xyz_all, pts_cal, step_id, yaw_all, parent=self.window)

        # 4. 生成 C 代码
        c_code = generate_c_code_3d(bias, A)
        c_path = os.path.join(CALIB_DIR, "mag_calib_3d.c")
        with open(c_path, 'w', encoding='utf-8') as f:
            f.write(c_code)

        self.window.set_status("3D 椭球校准完成，代码已保存至 mag_calib_3d.c")
    def view_result_3d(self):
        """
        一次性弹出三幅图：
        1) Raw 3D
        2) Calibrated 3D（单位球）
        3) Step-0 XY 投影，按 0-90-180-270 度分色
        """
        if self.freeze_data is None:
            return

        raw = np.array(self.freeze_data)
        xyz = raw[:, :3]
        step = raw[:, -1].astype(int)
        yaw = raw[:, 5]
        yaw = np.where(yaw < 0, yaw + 360, yaw)

        # 校准参数检查
        if self.freeze_b is None or self.freeze_A is None:
            QMessageBox.warning(self.window, "Error",
                                "请先完成校准，再查看 3D 结果！")
            return

        scale = 1.0 / np.linalg.norm(self.freeze_A[0])
        pts_cal = (xyz - self.freeze_b) * scale

        # ---------- 90 度分色函数 ----------
        def color90(y):
            if 0 <= y < 90:
                return 'red'
            elif 90 <= y < 180:
                return 'lime'
            elif 180 <= y < 270:
                return 'blue'
            else:
                return 'magenta'

        # ---------- 开始画图 ----------
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
            ax1.scatter(*xyz[step == s].T, c='b', s=12, label=f'Step-{s}', depthshade=False)
        ax1.set_title('Raw Points')
        ax1.legend()
        ax1.set_box_aspect([1, 1, 1])

        # 2) Calibrated 3D
        ax2 = fig.add_subplot(132, projection='3d')
        ax2.plot_surface(x, y, z, color='skyblue', alpha=0.25, rstride=1, cstride=1)
        ax2.set_xlim3d(-1.2, 1.2)
        ax2.set_ylim3d(-1.2, 1.2)
        ax2.set_zlim3d(-1.2, 1.2)

        for s in (0, 1, 2):
            ax2.scatter(*pts_cal[step == s].T, c='g', s=12, label=f'Step-{s}', depthshade=False)
        ax2.set_xlim(-1.2, 1.2)
        ax2.set_ylim(-1.2, 1.2)
        ax2.set_zlim(-1.2, 1.2)
        ax2.dist = 8
        ax2.set_box_aspect([1, 1, 1])
        ax2.set_title('Calibrated on Unit Sphere')
        ax2.legend()

        # 3) Step-0 XY 投影，90 度分色
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

        # ---------- 弹出窗口 ----------
        dlg = QDialog(self.window)
        dlg.setWindowTitle('3D Result – 90° Colored Step-0 XY')
        dlg.resize(1400, 600)
        lay = QVBoxLayout(dlg)
        lay.addWidget(FigureCanvas(fig))
        dlg.setLayout(lay)
        dlg.exec_()

    def _calc_step0_circularity_error(self, raw: np.ndarray) -> float:
        """计算Step-0数据的圆度误差（标准差/均值）"""
        mask = raw[:, 7] == 0  # Step-0数据
        if mask.sum() < 10:
            return 1.0  # 数据不足，返回最大误差
        
        # 提取Step-0的XY数据
        xy = raw[mask, :2]
        
        # 计算到原点的距离
        distances = np.linalg.norm(xy, axis=1)
        
        # 计算圆度误差（标准差/均值）
        if np.mean(distances) == 0:
            return 1.0
            
        return np.std(distances) / np.mean(distances)
        

    def _debug_plot_step0_xy(self, xy_data, title):
        """调试绘图函数（优化版）"""
        import matplotlib.pyplot as plt
        from PyQt5.QtWidgets import QDialog, QVBoxLayout
        from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
        
        fig, ax = plt.subplots(figsize=(6, 6))
        
        # 计算圆心
        center_x, center_y = np.mean(xy_data, axis=0)
        
        # 计算每个点相对于圆心的角度
        angles = np.arctan2(xy_data[:, 1] - center_y, xy_data[:, 0] - center_x) * 180 / np.pi
        angles = np.where(angles < 0, angles + 360, angles)  # 转换为0-360度
        
        # 定义每90度的颜色
        colors = []
        for angle in angles:
            if 0 <= angle < 90:
                colors.append('red')
            elif 90 <= angle < 180:
                colors.append('green')
            elif 180 <= angle < 270:
                colors.append('blue')
            else:
                colors.append('purple')
        
        # 绘制散点图
        ax.scatter(xy_data[:, 0], xy_data[:, 1], s=8, c=colors, alpha=0.7)
        
        # 绘制参考圆
        ax.add_patch(plt.Circle((0, 0), 1, ls='--', ec='r', fc='none'))
        
        # 标记圆心位置
        ax.plot(center_x, center_y, 'kx', markersize=10, markeredgewidth=2)
        
        ax.set_aspect('equal')
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        
        # 计算并显示圆度指标
        distances = np.linalg.norm(xy_data - [center_x, center_y], axis=1)
        circ_err = np.std(distances) / np.mean(distances) if np.mean(distances) > 0 else 1.0
        ax.text(0.05, 0.95, f"Circularity Error: {circ_err:.4f}", transform=ax.transAxes)
        ax.text(0.05, 0.90, f"Center: ({center_x:.3f}, {center_y:.3f})", transform=ax.transAxes)
        
        # 添加图例说明颜色对应的角度范围
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(facecolor='red', label='0°-90°'),
            Patch(facecolor='green', label='90°-180°'),
            Patch(facecolor='blue', label='180°-270°'),
            Patch(facecolor='purple', label='270°-360°'),
            plt.Line2D([0], [0], marker='x', color='k', label='Center', linestyle='None', markersize=10),
            plt.Line2D([0], [0], color='r', linestyle='--', label='Unit Circle')
        ]
        ax.legend(handles=legend_elements, loc='upper right')
        
        # 添加轴标签
        ax.set_xlabel("MX")
        ax.set_ylabel("MY")
        
        # 显示对话框
        dlg = QDialog(self.window)
        dlg.setWindowTitle(title)
        dlg.resize(600, 600)
        layout = QVBoxLayout()
        layout.addWidget(FigureCanvas(fig))
        dlg.setLayout(layout)
        dlg.exec_()    
    # ---------- 新增私有工具函数 ----------
    def advanced_circle_calibration(self, xy_data, target_error=0.05, max_iterations=10):
        """
        高级圆校准算法，目标圆度误差 < target_error
        使用迭代加权最小二乘法 + 异常值剔除
        """
        best_params = None
        best_error = float('inf')
        current_data = xy_data.copy()
        
        for iteration in range(max_iterations):
            # 1. 使用加权最小二乘法拟合圆
            cx, cy, radius = self.weighted_circle_fit(current_data)
            
            # 2. 计算每个点到圆心的距离和误差
            distances = np.linalg.norm(current_data - [cx, cy], axis=1)
            errors = np.abs(distances - radius)
            
            # 3. 计算当前圆度误差
            circ_err = np.std(distances) / np.mean(distances)
            print(f"Iteration {iteration+1}: Circularity error = {circ_err:.6f}")
            
            # 4. 检查是否达到目标
            if circ_err < target_error:
                print(f"达到目标圆度误差 {circ_err:.6f} < {target_error}")
                return cx, cy, radius, circ_err
            
            # 5. 更新最佳参数
            if circ_err < best_error:
                best_error = circ_err
                best_params = (cx, cy, radius)
            
            # 6. 剔除异常值（误差最大的5%）
            threshold = np.percentile(errors, 95)  # 95百分位
            mask = errors < threshold
            remaining_points = np.sum(mask)
            
            if remaining_points < len(current_data) * 0.7:  # 至少保留70%的点
                print(f"迭代 {iteration+1}: 剔除过多点({remaining_points}/{len(current_data)})，停止优化")
                break
                
            current_data = current_data[mask]
            
            # 7. 如果改进很小，提前终止
            if iteration > 0 and (best_error - circ_err) < 0.005:
                print(f"迭代 {iteration+1}: 改进很小({best_error-circ_err:.6f})，停止优化")
                break
        
        # 返回最佳结果
        if best_params:
            cx, cy, radius = best_params
            distances = np.linalg.norm(xy_data - [cx, cy], axis=1)
            circ_err = np.std(distances) / np.mean(distances)
            return cx, cy, radius, circ_err
        
        # 失败时返回原始数据的统计
        cx, cy = np.mean(xy_data, axis=0)
        distances = np.linalg.norm(xy_data - [cx, cy], axis=1)
        radius = np.mean(distances)
        circ_err = np.std(distances) / radius
        return cx, cy, radius, circ_err

    def weighted_circle_fit(self, xy_data):
        """加权最小二乘法圆拟合"""
        x, y = xy_data[:, 0], xy_data[:, 1]
        
        # 计算初始圆心和半径（未加权）
        cx0, cy0 = np.mean(xy_data, axis=0)
        distances0 = np.linalg.norm(xy_data - [cx0, cy0], axis=1)
        radius0 = np.mean(distances0)
        
        # 第一次拟合：计算权重（基于到初始圆的距离）
        errors0 = np.abs(distances0 - radius0)
        weights = 1.0 / (1.0 + errors0)  # 误差越小，权重越大
        
        # 加权最小二乘圆拟合
        A = np.column_stack([2*x, 2*y, np.ones_like(x)])
        b = x**2 + y**2
        
        # 应用权重
        W = np.diag(weights)
        A_weighted = W @ A
        b_weighted = W @ b
        
        # 求解
        try:
            cx, cy, c = np.linalg.lstsq(A_weighted, b_weighted, rcond=None)[0]
            radius = np.sqrt(cx**2 + cy**2 + c)
            return cx, cy, radius
        except:
            # 失败时返回简单平均值
            return cx0, cy0, radius0 
        
    def enhanced_forced_circle_calibration(self, xyz_all, step_id):
        """
        增强型强制单位圆校准
        使用高级圆校准算法优化Step-0 XY数据
        """
        # 1. 标准3D椭球校准
        bias, A, _ = run_sphere_calibration_algorithm(xyz_all, step_id)
        
        if bias is None or A is None:
            print("Error: Initial 3D ellipsoid fitting failed")
            return None, None, None
        
        # 2. 全体数据先校准一次
        xyz_cal = (xyz_all - bias) @ A.T
        
        # 3. 提取Step-0 XY数据
        mask0 = step_id == 0
        xy_cal = xyz_cal[mask0, :2]
        
        # 4. 使用高级圆校准算法
        cx, cy, radius, circ_err = self.advanced_circle_calibration(xy_cal, target_error=0.05)
        print(f"高级圆校准结果: 圆心=({cx:.6f}, {cy:.6f}), 半径={radius:.6f}, 圆度误差={circ_err:.6f}")
        
        # 5. 构造最终参数
        # 创建缩放矩阵（只缩放XY平面）
        S = np.eye(3)
        S[0, 0] = 1.0 / radius
        S[1, 1] = 1.0 / radius
        
        # 计算新的变换矩阵
        final_A = S @ A
        
        # 计算新的硬铁偏移
        xy_offset = np.array([cx, cy, 0])
        bias_adjustment = xy_offset @ np.linalg.inv(A).T
        final_bias = bias + bias_adjustment
        
        # 6. 最终点云
        pts_final = (xyz_all - final_bias) @ final_A.T
        
        return final_bias, final_A, pts_final
    # ===================================================================
    #  终极鲁棒圆校准：干扰大一次性压成正圆，四象限半径差 < 0.01
    # ===================================================================
    def robust_final_circle_calib(self, xyz_all, step_id, yaw_all):
        """返回 (bias, A, pts_cal) 一步到位"""
        # 1. 先标准 3D 椭球
        bias, A, _ = run_sphere_calibration_algorithm(xyz_all, step_id)
        xyz_cal = (xyz_all - bias) @ A.T

        # 2. 取出 Step-0 水平面数据
        mask0 = step_id == 0
        xy = xyz_cal[mask0, :2]
        yaw0 = yaw_all[mask0]

        # 3. 终极鲁棒圆校准
        xy_final = self._robust_circle_core(xy, yaw0, target_circ=0.01)

        # 4. 构造修正量（只动 XY，Z 不动）
        cx, cy = xy.mean(0) - xy_final.mean(0)          # 圆心平移
        scale = np.mean(np.linalg.norm(xy_final, axis=1))  # 应该≈1
        S = np.diag([1.0/scale, 1.0/scale, 1.0])
        A_final = S @ A
        bias_final = bias + np.array([cx, cy, 0])

        # 5. 全点云应用
        pts_final = (xyz_all - bias_final) @ A_final.T
        return bias_final, A_final, pts_final

    # ---------------- 核心：鲁棒圆 ----------------
    def _robust_circle_core(self, xy, yaw, target_circ=0.01):
        """
        硬拉正圆：标准差→1 + 飞点剃掉 + 半径→1
        """

        # 0. 剃飞点（先干净）
        d0 = np.linalg.norm(xy, axis=1)
        mask = d0 < d0.mean() + ROBUST_N_SIGMA * d0.std()
        xy, yaw = xy[mask], yaw[mask]

        # 1. 去相关旋转
        cov = np.cov(xy.T)
        eigval, eigvec = eigh(cov)
        xy_rot = (xy - xy.mean(axis=0)) @ eigvec

        # 2. 长短轴分别压到 1（硬尺子）
        long, short = np.sqrt(eigval)
        xy_rot[:, 0] /= long
        xy_rot[:, 1] /= short

        # 3. 圆心归零
        xy_rot -= xy_rot.mean(axis=0)

        # 4. 标准差再压 1（视觉正圆）
        std_x, std_y = np.std(xy_rot, axis=0)
        if std_x > 0 and std_y > 0:
            xy_rot[:, 0] /= std_x
            xy_rot[:, 1] /= std_y

        # 5. 最终半径 = 1
        xy_rot /= np.mean(np.linalg.norm(xy_rot, axis=1))

        return xy_rot @ eigvec.T + xy.mean(axis=0)
    
    def robust_final_circle_calib_enhanced(self, xyz_all, step_id, yaw_all):
        """
        强化版终极鲁棒圆校准 - 专注于强制椭圆转正圆
        确保不会返回空数据
        """
        # 1. 取出 Step-0 数据
        mask0 = step_id == 0
        xy_raw = xyz_all[mask0, :2]  # 只取XY
        
        # 2. 计算原始数据的椭圆参数
        # 计算中心
        center_x, center_y = np.mean(xy_raw, axis=0)
        
        # 中心化数据
        xy_centered = xy_raw - [center_x, center_y]
        
        # 计算协方差矩阵
        cov = np.cov(xy_centered.T)
        
        # 计算特征值和特征向量
        eigvals, eigvecs = np.linalg.eigh(cov)
        
        # 计算椭圆的长短轴
        a = np.sqrt(eigvals[1])  # 长轴
        b = np.sqrt(eigvals[0])  # 短轴
        
        # 计算旋转角度
        angle = np.arctan2(eigvecs[1, 1], eigvecs[0, 1])
        
        # 3. 构造变换矩阵，将椭圆转换为正圆
        # 旋转矩阵（将椭圆旋转到与坐标轴对齐）
        R = np.array([[np.cos(-angle), -np.sin(-angle)],
                    [np.sin(-angle), np.cos(-angle)]])
        
        # 缩放矩阵（将长短轴缩放为相同长度）
        S = np.array([[1.0/a, 0.0],
                    [0.0, 1.0/b]])
        
        # 组合变换：先旋转，再缩放，再旋转回去
        T = R.T @ S @ R
        
        # 4. 应用变换到原始数据
        xy_transformed = xy_centered @ T
        
        # 5. 计算最终缩放因子，确保半径为1
        radii = np.linalg.norm(xy_transformed, axis=1)
        mean_radius = np.mean(radii)
        if mean_radius > 0:
            final_scale = 1.0 / mean_radius
        else:
            final_scale = 1.0
        
        # 6. 构造3D变换矩阵和偏移向量
        A_3d = np.eye(3)
        A_3d[0:2, 0:2] = T * final_scale  # 应用XY平面的变换
        
        bias = np.zeros(3)
        bias[0] = center_x
        bias[1] = center_y
        
        # 7. 应用变换到所有点
        pts_cal = (xyz_all - bias) @ A_3d.T
        
        return bias, A_3d, pts_cal

    def _hyper_robust_circle_enforcement(self, xy_data, yaw_data):
        """
        超强制的正圆变换 - 专注于强制椭圆转正圆
        不考虑数据质量，只关注将椭圆转换为正圆
        """
        # 1. 计算椭圆的主轴和旋转角度
        if len(xy_data) < 2:
            return xy_data
            
        # 计算协方差矩阵
        cov = np.cov(xy_data.T)
        
        # 计算特征值和特征向量
        eigvals, eigvecs = np.linalg.eigh(cov)
        
        # 计算椭圆的长短轴
        a = np.sqrt(eigvals[1])  # 长轴
        b = np.sqrt(eigvals[0])  # 短轴
        
        # 计算旋转角度
        angle = np.arctan2(eigvecs[1, 1], eigvecs[0, 1])
        
        # 2. 构造逆变换矩阵，将椭圆转换为正圆
        # 先旋转回正，然后缩放，再旋转回去
        R = np.array([[np.cos(angle), -np.sin(angle)],
                    [np.sin(angle), np.cos(angle)]])
        
        S = np.array([[1.0/a, 0.0],
                    [0.0, 1.0/b]])
        
        # 逆变换矩阵
        T = R @ S @ R.T
        
        # 3. 应用变换
        center = np.mean(xy_data, axis=0)
        centered = xy_data - center
        transformed = centered @ T
        
        # 4. 确保半径为1
        radii = np.linalg.norm(transformed, axis=1)
        mean_radius = np.mean(radii)
        if mean_radius > 0:
            transformed /= mean_radius
        
        # 5. 返回变换后的数据（保持中心不变）
        return transformed + center
    
    def _imf(self, xy, xyz):
        """IMF < 0.05 干净；> 0.05 上狠活"""
        cov = np.cov(xy.T)
        eig = np.linalg.eigvalsh(cov)
        flat = np.sqrt(eig.max() / eig.min()) - 1.0
        var3 = np.cov(xyz.T).trace()
        k = 1e-4                                     # 经验常数
        return float(flat + k * var3)

    @pyqtSlot()
    def on_algo3d(self):
        """校准算法入口（优化版）"""
        fname, _ = QFileDialog.getOpenFileName(
            self.window, "Select CSV", CALIB_DIR, "CSV (*.csv)")
        if not fname:
            return

        try:
            raw = np.loadtxt(fname, delimiter=',', ndmin=2)
            if raw.shape[1] not in (8, 9):
                raise ValueError(f"CSV must be 8 or 9 columns, got {raw.shape[1]}")

            xyz_all = raw[:, :3]
            step_id = raw[:, 7].astype(int)
            yaw_all = raw[:, 5]

            # 1. 计算圆度误差
            circ_err = self._calc_step0_circularity_error(raw)
            print(f"圆度误差: {circ_err:.6f}")

            # 2. 显示校准前的数据
            mask0 = step_id == 0
            xy_raw_step0 = xyz_all[mask0, :2]
            self._debug_plot_step0_xy(xy_raw_step0, f"Step0 XY Raw (CircErr: {circ_err:.6f})")

            # 3. 按阈值选择校准方法
            # 计算干扰强度指标 IMF
            imf_val = self._imf(xy_raw_step0, xyz_all[mask0])
            print(f"IMF={imf_val:.3f}")

            if imf_val < IMF_CLEAN_TH:# 干净环境
                print("小干扰，标准3D椭球校准")
                bias, A, pts_cal = run_sphere_calibration_algorithm(xyz_all, step_id)
            else:
                print("大干扰，终极鲁棒圆校准")
                bias, A, pts_cal = self.robust_final_circle_calib_enhanced(xyz_all, step_id, yaw_all)
                
                # 添加调试输出
                print(f"校准结果: bias={bias}, A={A}")
                print(f"校准后数据形状: {pts_cal.shape if pts_cal is not None else 'None'}")
                if pts_cal is not None and len(pts_cal) > 0:
                    print(f"校准后数据范围: X[{pts_cal[:,0].min():.3f}, {pts_cal[:,0].max():.3f}], "
                        f"Y[{pts_cal[:,1].min():.3f}, {pts_cal[:,1].max():.3f}], "
                        f"Z[{pts_cal[:,2].min():.3f}, {pts_cal[:,2].max():.3f}]")

            if bias is None or A is None:
                QMessageBox.warning(self.window, "校准失败", "校准算法失败，请检查数据质量")
                return

            # 4. 显示校准后的数据
            xy_cal_step0 = pts_cal[mask0, :2]
            self._debug_plot_step0_xy(xy_cal_step0, "Step0 XY Calibrated")

            # 5. 绘制校准结果
            plot_standard_calibration_result(xyz_all, pts_cal, step_id, yaw_all, parent=self.window)

            # 6. 生成C代码
            c_code = generate_c_code_3d(bias, A)
            c_path = os.path.join(CALIB_DIR, "mag_calib_3d.c")
            with open(c_path, 'w', encoding='utf-8') as f:
                f.write(c_code)
                
            QMessageBox.information(self.window, "完成", "校准完成，代码已保存")

        except Exception as e:
            import traceback
            traceback.print_exc()
            QMessageBox.critical(self.window, "Algo3D Error", str(e))
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
        def project_to_horizontal(mx, my, mz, pitch, roll):
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
            project_to_horizontal(mx, my, mz, p, r)
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

    def _force_2d_circle_calibration(self, raw: np.ndarray):
        """椭圆→正圆→圆心(0,0)→半径1→Z不动"""
        mask = raw[:, 7] == 0
        xyz0  = raw[mask, :3]
        pitch = raw[mask, 3]
        roll  = raw[mask, 4]

        # 1. 投影到水平面
        xy = np.array([self.project_to_horizontal(mx, my, mz, p, r)
                    for mx, my, mz, p, r in zip(
                        xyz0[:, 0], xyz0[:, 1], xyz0[:, 2], pitch, roll)])

        # 2. 椭圆→正圆：去相关+等比例
        cov = np.cov(xy.T)
        eigval, eigvec = np.linalg.eigh(cov)
        eigval = np.maximum(eigval, 1e-6)
        xy_rot = xy @ eigvec
        r_mean = np.sqrt(eigval.mean())
        if r_mean == 0:
            r_mean = 1.0
        xy_circ = xy_rot / r_mean @ eigvec.T

        # 3. 移到 (0,0)
        cx, cy = xy_circ.mean(axis=0)
        xy_unit = xy_circ - [cx, cy]

        # 4. 3D 只改 XY，Z 不动
        bias = np.array([cx, cy, 0.0])
        S = np.diag([1.0/r_mean, 1.0/r_mean, 1.0])
        A_xy = eigvec @ S[:2, :2] @ eigvec.T
        A = np.eye(3)
        A[:2, :2] = A_xy
        pts_cal = (xyz0 - bias) @ A

        return bias, A, pts_cal
    
    def _plot_step0_forced_circle(self, raw: np.ndarray, pts_cal: np.ndarray):
        """Step-0 XY 四色 90° 分区，RAW + Cal 同一角度同色"""
        mask = raw[:, 7] == 0
        xyz0  = raw[mask, :3]
        pitch = raw[mask, 3]
        roll  = raw[mask, 4]
        yaw0  = raw[mask, 5]          # 角度在第 6 列
        yaw0  = np.where(yaw0 < 0, yaw0 + 360, yaw0)

        # 旋转到水平面
        xy_raw = np.array([self.project_to_horizontal(mx, my, mz, p, r)
                        for mx, my, mz, p, r in zip(
                            xyz0[:, 0], xyz0[:, 1], xyz0[:, 2], pitch, roll)])
        xy_cal = pts_cal[:, :2]

        # 四色函数
        def color90(y):
            y = y % 360
            if 0 <= y < 90:   return 'red'
            if 90 <= y < 180: return 'lime'
            if 180 <= y < 270:return 'blue'
            return 'magenta'

        c90 = [color90(y) for y in yaw0]   # 只算一次

        # ---- 画图 ----
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.scatter(xy_raw[:, 0], xy_raw[:, 1], c=c90, s=12, alpha=0.8,
                edgecolors='k', linewidths=0.3, label='RAW')
        ax.scatter(xy_cal[:, 0], xy_cal[:, 1], c=c90, s=12, alpha=0.8,
                edgecolors='k', linewidths=0.3, label='Cal')

        ax.add_patch(plt.Circle((0, 0), 1, ls='--', ec='k', fc='none'))
        ax.plot(0, 0, 'k+', ms=8)
        ax.set_aspect('equal')
        ax.legend()
        ax.grid(alpha=0.3)
        ax.set_title('Step-0 XY – 90° Colors (RAW & Cal)')

        dlg = QDialog(self.window)
        dlg.setWindowTitle("Step-0 XY – 90° Four-Color")
        dlg.resize(650, 650)
        lay = QVBoxLayout(dlg)
        lay.addWidget(FigureCanvas(fig))
        dlg.setLayout(lay)
        dlg.exec_()
    
    def _safe_calibration_2d_fallback(self, raw: np.ndarray):
        """仅用 Step-0 数据 → 旋转水平 → 强制正圆（圆心 0,0 半径 1）"""
        mask = raw[:, 7] == 0
        xyz0 = raw[mask, :3]
        pitch = raw[mask, 3]
        roll = raw[mask, 4]

        # 1. 旋转到水平面
        xy_level = np.array([self.project_to_horizontal(mx, my, mz, p, r)
                             for mx, my, mz, p, r in zip(
                                 xyz0[:, 0], xyz0[:, 1], xyz0[:, 2], pitch, roll)])

        # 2. 强制圆心 (0,0)
        x, y = xy_level[:, 0], xy_level[:, 1]
        A_mat = np.c_[2*x, 2*y, np.ones_like(x)]
        b_vec = x**2 + y**2
        cx, cy, _ = np.linalg.lstsq(A_mat, b_vec, rcond=None)[0]
        xy_centered = xy_level - [cx, cy]

        # 3. 强制半径 1
        r_mean = np.linalg.norm(xy_centered, axis=1).mean()
        xy_unit = xy_centered / r_mean

        # 4. 构造 3D 校准结果：只修正 XY，Z 保持原样
        bias = np.array([cx, cy, 0.0])          # 硬铁偏移
        A = np.eye(3) / r_mean                  # 各向同性缩放
        pts_cal = (xyz0 - bias) @ A             # 先平移再缩放 → 正圆

        return bias, A, pts_cal

    def _plot_step0_fallback_result(self, raw: np.ndarray, pts_cal: np.ndarray):
        """Step-0 XY 四色 90° 分区，RAW + Cal 同一角度同色"""
        # 1. 先取 Step-0 子集
        mask0 = raw[:, 7] == 0
        raw0  = raw[mask0]
        xyz0  = raw0[:, :3]
        pitch = raw0[:, 3]
        roll  = raw0[:, 4]
        yaw0  = raw0[:, 5]
        yaw0  = np.where(yaw0 < 0, yaw0 + 360, yaw0)
        # pts0  = pts_cal[mask0]          # ← 关键：用同样 mask 取校准结果
        pts0 = pts_cal

        # 2. RAW：原始投影
        xy_raw = np.array([self.project_to_horizontal(mx, my, mz, p, r)
                        for mx, my, mz, p, r in zip(
                            xyz0[:, 0], xyz0[:, 1], xyz0[:, 2], pitch, roll)])

        # 3. CAL：直接拿校准后的 XY（已正圆+圆心0+半径1）
        xy_cal = pts0[:, :2]

        # 4. 四色分区
        def color90(y):
            y = y % 360
            return ['red', 'lime', 'blue', 'magenta'][int(y // 90)]
        c90 = [color90(y) for y in yaw0]

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
        for lb, col in [('0-90°', 'red'), ('90-180°', 'lime'),
                        ('180-270°', 'blue'), ('270-360°', 'magenta')]:
            m = [(lb == '0-90°'   and 0   <= y < 90)  or
                (lb == '90-180°' and 90  <= y < 180) or
                (lb == '180-270°'and 180 <= y < 270) or
                (lb == '270-360°'and 270 <= y < 360) for y in yaw0]
            ax1.scatter(xy_raw[m, 0], xy_raw[m, 1], c=col, s=8, alpha=0.8, label=lb)
            ax2.scatter(xy_cal[m, 0], xy_cal[m, 1], c=col, s=8, alpha=0.8, marker='x', label=lb)

        for ax, title in [(ax1, 'Raw'), (ax2, 'Calibrated')]:
            ax.add_patch(plt.Circle((0, 0), 1, ls='--', ec='k', fc='none'))
            ax.plot(0, 0, 'k+', ms=8)
            ax.set_aspect('equal')
            ax.legend(title='Yaw range')
            ax.grid(alpha=0.3)
            ax.set_title(f'Step-0 XY {title} (90° colors)')

        dlg = QDialog(self.window)
        dlg.setWindowTitle("Step-0 XY – 90° Colors (RAW & Cal)")
        dlg.resize(1200, 600)
        lay = QVBoxLayout(dlg)
        lay.addWidget(FigureCanvas(fig))
        dlg.setLayout(lay)
        dlg.exec_()

    def run_3d_plus_step0_forced_circle(self, xyz_all, step_id):
        """
        强制单位圆校准（优化版）
        保持函数名不变，优化实现逻辑
        """
        # 1. 标准3D椭球校准
        bias, A, _ = run_sphere_calibration_algorithm(xyz_all, step_id)
        
        if bias is None or A is None:
            print("Error: Initial 3D ellipsoid fitting failed")
            return None, None, None
        
        # 2. 全体数据先校准一次
        xyz_cal = (xyz_all - bias) @ A.T
        
        # 3. 提取Step-0 XY数据
        mask0 = step_id == 0
        xy_cal = xyz_cal[mask0, :2]
        
        # 4. 使用简单有效的圆拟合
        x, y = xy_cal[:, 0], xy_cal[:, 1]
        A_mat = np.column_stack([2*x, 2*y, np.ones_like(x)])
        b_vec = x**2 + y**2
        
        try:
            cx, cy, c = np.linalg.lstsq(A_mat, b_vec, rcond=None)[0]
            radius = np.sqrt(cx**2 + cy**2 + c)
        except:
            # 失败时使用简单平均值
            cx, cy = np.mean(xy_cal, axis=0)
            distances = np.linalg.norm(xy_cal - [cx, cy], axis=1)
            radius = np.mean(distances)
        
        # 5. 构造最终参数
        S = np.eye(3)
        S[0, 0] = 1.0 / radius
        S[1, 1] = 1.0 / radius
        
        final_A = S @ A
        xy_offset = np.array([cx, cy, 0])
        bias_adjustment = xy_offset @ np.linalg.inv(A).T
        final_bias = bias + bias_adjustment
        
        # 6. 最终点云
        pts_final = (xyz_all - final_bias) @ final_A.T
        
        print(f"强制单位圆校准: 圆心=({cx:.3f}, {cy:.3f}), 半径={radius:.3f}")
        
        return final_bias, final_A, pts_final

if __name__ == "__main__":
    app = CalibrationApp()
    app.run()