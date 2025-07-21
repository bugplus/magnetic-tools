import sys
import serial
import threading
import time
import numpy as np
import re
from PyQt5.QtWidgets import QApplication
from compass_ui import CompassMainWindow

import matplotlib.pyplot as plt
from config import MAG_AXIS_MAP_A, MAG_AXIS_MAP_B, SENSOR_TYPE  # 新增导入

CALIBRATION_DURATION = 30  # seconds

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

def fit_circle_least_squares(xy):
    x = xy[:, 0]
    y = xy[:, 1]
    A = np.c_[2*x, 2*y, np.ones(x.shape)]
    b = x**2 + y**2
    c, resid, rank, s = np.linalg.lstsq(A, b, rcond=None)
    center_x, center_y = c[0], c[1]
    radius = np.sqrt(c[2] + center_x**2 + center_y**2)
    return np.array([center_x, center_y]), radius

def calibrate_magnetometer_standard(xy):
    # 1. Soft iron: scale y to match x radius (about origin)
    radius_x = (np.max(xy[:, 0]) - np.min(xy[:, 0])) / 2
    radius_y = (np.max(xy[:, 1]) - np.min(xy[:, 1])) / 2
    scale = radius_x / radius_y if radius_y != 0 else 1.0
    soft_calibrated = xy.copy()
    soft_calibrated[:, 1] *= scale

    # 2. Hard iron: fit circle center after scaling, then shift to origin
    center, _ = fit_circle_least_squares(soft_calibrated)
    hard_soft_calibrated = soft_calibrated - center

    return soft_calibrated, hard_soft_calibrated, center, scale

def calibrate_magnetometer_least_squares(xy):
    center, _ = fit_circle_least_squares(xy)
    shifted = xy - center
    radius_x = (np.max(shifted[:, 0]) - np.min(shifted[:, 0])) / 2
    radius_y = (np.max(shifted[:, 1]) - np.min(shifted[:, 1])) / 2
    scale = radius_x / radius_y if radius_y != 0 else 1.0
    calibrated = shifted.copy()
    calibrated[:, 1] *= scale
    return shifted, calibrated, center, scale

def generate_c_code(center, scale):
    cx, cy = center[0], center[1]
    c_code = f"""
/**
 * 磁力计校准算法 - 2×2 对角软铁矩阵版
 * 与原 SOFT_IRON_SCALE 完全等价，保留所有接口与示例
 */

#include <math.h>
#include <stdio.h>

// ---------- 校准参数 ----------
#define HARD_IRON_OFFSET_X {cx:.6f}f
#define HARD_IRON_OFFSET_Y {cy:.6f}f

// 2×2 对角软铁矩阵（与原 scale 等价）
static const float SOFT_IRON_MATRIX[2][2] = {{
    {{1.0f, 0.0f}},
    {{0.0f, {scale:.6f}f}}
}};

// ---------- 数据结构 ----------
typedef enum {{
    MAPPING_TYPE_1 = 1,  // X→Y, Y→X, Z→Z
    MAPPING_TYPE_2 = 2,  // X→X, Y→Y, Z→Z
    MAPPING_TYPE_3 = 3   // X→-X, Y→Y, Z→Z
}} mapping_type_t;

typedef struct {{
    float mag_x, mag_y, mag_z;
    float pitch, roll, yaw;
    mapping_type_t mapping;
}} tilt_input_t;

typedef struct {{
    float mx_comp, my_comp;   // 倾斜补偿后水平磁场
    float mx_cal, my_cal;     // 软铁/硬铁校准后
}} tilt_output_t;

// ---------- 坐标映射 ----------
void map_coordinates(float mag_x, float mag_y, float mag_z,
                     mapping_type_t mapping,
                     float *mx_out, float *my_out, float *mz_out) {{
    switch(mapping) {{
        case MAPPING_TYPE_1:
            *mx_out = mag_y;
            *my_out = mag_x;
            *mz_out = mag_z;
            break;
        case MAPPING_TYPE_2:
            *mx_out = mag_x;
            *my_out = mag_y;
            *mz_out = mag_z;
            break;
        case MAPPING_TYPE_3:
            *mx_out = -mag_x;
            *my_out = mag_y;
            *mz_out = mag_z;
            break;
        default:
            *mx_out = mag_x;
            *my_out = mag_y;
            *mz_out = mag_z;
            break;
    }}
}}

// ---------- 倾斜补偿 ----------
void tilt_compensation(tilt_input_t *input, tilt_output_t *output) {{
    float mx, my, mz;
    map_coordinates(input->mag_x, input->mag_y, input->mag_z,
                    input->mapping, &mx, &my, &mz);

    float pitch_rad = input->pitch * M_PI / 180.0f;
    float roll_rad  = input->roll  * M_PI / 180.0f;

    float cos_p = cosf(pitch_rad);
    float sin_p = sinf(pitch_rad);
    float cos_r = cosf(roll_rad);
    float sin_r = sinf(roll_rad);

    output->mx_comp = mx * cos_p + mz * sin_p;
    output->my_comp = mx * sin_p * sin_r + my * cos_r - mz * cos_p * sin_r;
}}

// ---------- 软铁/硬铁校准 ----------
void calibrate_magnetometer(float *mx_comp, float *my_comp) {{
    // 1. 硬铁
    float x = *mx_comp - HARD_IRON_OFFSET_X;
    float y = *my_comp - HARD_IRON_OFFSET_Y;

    // 2. 软铁（矩阵形式，等价 y *= scale）
    *mx_comp = SOFT_IRON_MATRIX[0][0] * x + SOFT_IRON_MATRIX[0][1] * y;
    *my_comp = SOFT_IRON_MATRIX[1][0] * x + SOFT_IRON_MATRIX[1][1] * y;
}}

// ---------- 完整处理 ----------
void process_magnetometer_data(tilt_input_t *input, tilt_output_t *output) {{
    tilt_compensation(input, output);
    calibrate_magnetometer(&output->mx_comp, &output->my_comp);
    output->mx_cal = output->mx_comp;
    output->my_cal = output->my_comp;
}}

// ---------- 使用示例 ----------
void example_usage(void) {{
    tilt_input_t input = {{
        .mag_x = 247.0f,
        .mag_y = 137.0f,
        .mag_z = -57.0f,
        .pitch = 62.46f,
        .roll  = -1.01f,
        .yaw   = -90.37f,
        .mapping = MAPPING_TYPE_1
    }};
    tilt_output_t output;

    process_magnetometer_data(&input, &output);
    float heading = atan2f(output.my_cal, output.mx_cal) * 180.0f / M_PI;
    if (heading < 0) heading += 360.0f;

    printf("倾斜补偿: mx=%.2f, my=%.2f\\n", output.mx_comp, output.my_comp);
    printf("校准结果: mx=%.2f, my=%.2f\\n", output.mx_cal,  output.my_cal);
    printf("航向角: %.1f°\\n", heading);
}}
"""
    return c_code

def plot_two(raw_data, calibrated, center, scale):
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 10))
    
    # 第一个图：真正的原始椭圆数据（倾斜补偿前）
    ax1.set_title("Original Ellipse Data (Before Tilt Compensation)")
    ax1.axis('equal')
    if raw_data is not None and len(raw_data) > 0:
        ax1.plot(raw_data[:, 0], raw_data[:, 1], 'r.', alpha=0.7, label='Original Data')
    ax1.plot(0, 0, 'k+', markersize=10, mew=2, label='Origin')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # 第二个图：倾斜补偿后数据
    ax2.set_title(f"Final Calibrated Data\nSoft Iron Scale: {scale:.4f}")
    ax2.axis('equal')
    if calibrated is not None and len(calibrated) > 0:
        ax2.plot(calibrated[:, 0], calibrated[:, 1], 'g.', alpha=0.7, label='Calibrated Data')
    ax2.plot(0, 0, 'k+', markersize=10, mew=2, label='Origin')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # 第三个图：校准参数
    ax3.set_title("Calibration Parameters")
    ax3.axis('off')
    param_text = f"""
Calibration Parameters:
Hard Iron Offset X: {center[0]:.2f}
Hard Iron Offset Y: {center[1]:.2f}
Soft Iron Scale: {scale:.4f}

Data Statistics:
Original Data Points: {len(raw_data) if raw_data is not None else 0}
Calibrated Data Points: {len(calibrated) if calibrated is not None else 0}

Coordinate Mapping: MAPPING_TYPE_1 (X->Y, Y->X, Z->Z)
Tilt Compensation: Enabled
Soft Iron Calibration: Enabled
Hard Iron Calibration: Enabled
"""
    ax3.text(0.1, 0.9, param_text, transform=ax3.transAxes, fontsize=10, 
             verticalalignment='top', fontfamily='monospace')
    
    # 第四个图：校准效果对比
    ax4.set_title("Calibration Effect Comparison")
    if raw_data is not None and calibrated is not None and len(raw_data) > 0 and len(calibrated) > 0:
        # 计算半径分布
        raw_radius = np.sqrt(raw_data[:, 0]**2 + raw_data[:, 1]**2)
        cal_radius = np.sqrt(calibrated[:, 0]**2 + calibrated[:, 1]**2)
        
        ax4.hist(raw_radius, bins=20, alpha=0.7, label='Before Calibration', color='blue')
        ax4.hist(cal_radius, bins=20, alpha=0.7, label='After Calibration', color='green')
        ax4.set_xlabel('Radius')
        ax4.set_ylabel('Frequency')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        # 添加统计信息
        raw_std = np.std(raw_radius)
        cal_std = np.std(cal_radius)
        improvement = (raw_std - cal_std) / raw_std * 100
        ax4.text(0.02, 0.98, f'Before STD: {raw_std:.2f}\nAfter STD: {cal_std:.2f}\nImprovement: {improvement:.1f}%', 
                transform=ax4.transAxes, verticalalignment='top', 
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    plt.show()

class CalibrationApp:
    def __init__(self):
        self.app = QApplication(sys.argv)
        self.window = CompassMainWindow()
        self.window.start_calibration_signal.connect(self.start_calibration)
        self.window.view_result_signal.connect(self.view_result)
        self.serial_thread = None
        self.raw_mag_data = []  # [mx, my, mz, roll, pitch, yaw]
        self.calibrated = False
        self.calibration_params = None
        self.fig1 = None
        self.fig2 = None
        self.ax_original = None
        self.ax_projected = None
        self.ax_soft_iron = None
        self.ax_final = None
        self.original_plot = None
        self.projected_plot = None
        self._pending_mag = None  # 用于暂存mag行
        self._pending_euler = None # 用于暂存欧拉角
        self.last_xy = None     # 缓存上一次画过的点

    def start_calibration(self, port, baudrate):
        try:
            if self.fig1 is not None:
                plt.close(self.fig1)
            if self.fig2 is not None:
                plt.close(self.fig2)
        except Exception:
            pass
        self.fig1 = None
        self.fig2 = None
        self.ax_original = None
        self.ax_projected = None
        self.ax_soft_iron = None
        self.ax_final = None
        self.original_plot = None
        self.projected_plot = None

        self.raw_mag_data.clear()
        self.calibrated = False
        self.calibration_params = None
        self._pending_euler = None  # 清空待处理的欧拉角数据
        self.window.clear_all_canvases()
        self.window.set_status("Calibrating...")
        self.serial_thread = SerialReader(port, baudrate, self.on_serial_data)
        self.serial_thread.start()
        self.calibration_start_time = time.time()
        # 创建两个画布：第一个显示原始数据和投影数据，第二个显示校准过程
        self.fig1, (self.ax_original, self.ax_projected) = plt.subplots(1, 2, figsize=(12, 5))
        self.fig2, (self.ax_soft_iron, self.ax_final) = plt.subplots(1, 2, figsize=(12, 5))
        
        # 第一个画布：原始数据和倾斜补偿数据
        self.ax_original.set_title("Original Data (mx, my)")
        self.ax_projected.set_title("Tilt Compensated Data")
        self.ax_original.axis('equal')
        self.ax_projected.axis('equal')
        
        # 第二个画布：校准过程
        self.ax_soft_iron.set_title("Hard Iron Calibrated")
        self.ax_final.set_title("Final Calibrated (Hard + Soft Iron)")
        self.ax_soft_iron.axis('equal')
        self.ax_final.axis('equal')
        
        # 实时绘图对象
        self.original_plot, = self.ax_original.plot([], [], 'r.', alpha=0.7)
        self.projected_plot, = self.ax_projected.plot([], [], 'b.', alpha=0.7)
        
        plt.tight_layout()
        self.fig1.show()
        self.fig1.canvas.manager.window.move(50, 50)
        self.fig2.show()
        self.fig2.canvas.manager.window.move(50, 600)
        threading.Thread(target=self._calibration_timer, daemon=True).start()

    def _calibration_timer(self):
        while time.time() - self.calibration_start_time < CALIBRATION_DURATION:
            time.sleep(0.1)
        if self.serial_thread:
            self.serial_thread.stop()
        self.perform_calibration()

    def extract_mag(self, mag_line):
        return [float(x) for x in re.findall(r'-?\d+\.?\d*', mag_line)]

    def map_mag_to_imu(self, mx, my, mz):
        mag = np.stack([mx, my, mz], axis=-1)
        mat = MAG_AXIS_MAP_A if SENSOR_TYPE == "A" else MAG_AXIS_MAP_B  # 仅改这一行
        mag_imu = mag @ np.array(mat).T
        return mag_imu[..., 0], mag_imu[..., 1], mag_imu[..., 2]

    def on_serial_data(self, line):
        try:
            # 适配你的数据格式：simples行、pitch行、mag_x行
            if not line.strip():
                return
            if line.startswith('simples:'):
                # 解析simples行：simples:62.463959,-1.012322,-90.374313,27.030861
                vals = re.findall(r'simples:([\-\d\.]+),([\-\d\.]+),([\-\d\.]+),([\-\d\.]+)', line)
                if vals:
                    pitch, roll, yaw, testyaw = map(float, vals[0])
                    self._pending_euler = (pitch, roll, yaw)
            elif line.startswith('pitch='):
                # 解析pitch行：pitch= 62.463959, roll= -1.012322,yaw= -90.374313, testyaw= 27.030861
                vals = re.findall(r'pitch=\s*([\-\d\.]+),\s*roll=\s*([\-\d\.]+),\s*yaw=\s*([\-\d\.]+)', line)
                if vals:
                    pitch, roll, yaw = map(float, vals[0])
                    self._pending_euler = (pitch, roll, yaw)
            elif line.startswith('mag_x='):
                # 解析mag_x行：mag_x=247, mag_y=137, mag_z=-57
                vals = re.findall(r'mag_x=\s*([\-\d\.]+),\s*mag_y=\s*([\-\d\.]+),\s*mag_z=\s*([\-\d\.]+)', line)
                if vals and self._pending_euler is not None:
                    mx, my, mz = map(float, vals[0])
                    pitch, roll, yaw = self._pending_euler
                    # 坐标系映射
                    mx, my, mz = self.map_mag_to_imu(np.array([mx]), np.array([my]), np.array([mz]))
                    mx, my, mz = mx[0], my[0], mz[0]
                    self.raw_mag_data.append([mx, my, mz, roll, pitch, yaw])
                    self._pending_euler = None
                    print(f"Data paired: mag=({mx:.1f}, {my:.1f}, {mz:.1f}), euler=({roll:.1f}, {pitch:.1f}, {yaw:.1f})")
                arr = np.array(self.raw_mag_data)
                if arr.shape[0] > 0 and self.original_plot is not None and self.projected_plot is not None:
                    # 显示原始数据（红色）
                    mx, my, mz, roll, pitch, yaw = arr.T
                    self.original_plot.set_data(mx, my)
                    
                    # 计算并显示倾斜补偿后的数据（蓝色）
                    roll_rad = np.radians(roll)
                    pitch_rad = np.radians(pitch)
                    mxh, myh = [], []
                    for i in range(len(mx)):
                        # 使用标准的倾斜补偿公式
                        cos_p = np.cos(pitch_rad[i])
                        sin_p = np.sin(pitch_rad[i])
                        cos_r = np.cos(roll_rad[i])
                        sin_r = np.sin(roll_rad[i])
                        
                        # 标准倾斜补偿公式（使用旋转矩阵）
                        # 先绕X轴旋转（pitch），再绕Y轴旋转（roll）
                        mx_comp = mx[i] * cos_p + mz[i] * sin_p
                        my_comp = mx[i] * sin_p * sin_r + my[i] * cos_r - mz[i] * cos_p * sin_r
                        
                        mxh.append(mx_comp)
                        myh.append(my_comp)
                    xy = np.stack([mxh, myh], axis=1)
                    if self.last_xy is None or not np.array_equal(self.last_xy, xy):
                        self.last_xy = xy
                        self.projected_plot.set_data(xy[:, 0], xy[:, 1])
                    self.projected_plot.set_data(xy[:, 0], xy[:, 1])
                    
                    # 显示统计信息
                    print(f"Total points: {len(arr)}")
                    print(f"Roll range: {roll.min():.1f}° to {roll.max():.1f}°")
                    print(f"Pitch range: {pitch.min():.1f}° to {pitch.max():.1f}°")
                    
                    # 调试信息：显示一些数据点的对比
                    if len(mx) > 10:
                        print(f"Sample data point 0:")
                        print(f"  Raw: mx={mx[0]:.1f}, my={my[0]:.1f}, mz={mz[0]:.1f}")
                        print(f"  Euler: roll={roll[0]:.1f}°, pitch={pitch[0]:.1f}°, yaw={yaw[0]:.1f}°")
                    
                    # 更新坐标轴 - 确保能看到形状差异
                    if self.ax_original is not None:
                        self.ax_original.relim()
                        self.ax_original.autoscale_view()
                        self.ax_original.set_aspect('equal', adjustable='box')
                    if self.ax_projected is not None:
                        self.ax_projected.relim()
                        self.ax_projected.autoscale_view()
                        self.ax_projected.set_aspect('equal', adjustable='box')
                    
                    # 刷新画布
                    if self.fig1 is not None:
                        self.fig1.canvas.draw_idle()
                        self.fig1.canvas.flush_events()
                        self.fig1.canvas.manager.window.move(50, 50)
        except Exception as e:
            print(f"Serial data parsing error: {e}")
            print(f"Problematic line: {line.strip()}")

    def perform_calibration(self):
        self.window.set_status("Calibration finished. Processing data...")
        if len(self.raw_mag_data) < 10:
            self.window.set_status("Not enough data for calibration.")
            self.window.start_btn.setEnabled(True)
            self.window.port_combo.setEnabled(True)
            self.window.baud_combo.setEnabled(False)
            self.window.refresh_btn.setEnabled(True)
            return

        arr = np.array(self.raw_mag_data)
        mx, my, mz, roll, pitch, yaw = arr.T

        # 1. 倾斜补偿（二维）
        roll_rad = np.radians(roll)
        pitch_rad = np.radians(pitch)
        xy = []
        for i in range(len(mx)):
            cos_p = np.cos(pitch_rad[i])
            sin_p = np.sin(pitch_rad[i])
            cos_r = np.cos(roll_rad[i])
            sin_r = np.sin(roll_rad[i])
            mx_comp = mx[i] * cos_p + mz[i] * sin_p
            my_comp = mx[i] * sin_p * sin_r + my[i] * cos_r - mz[i] * cos_p * sin_r
            xy.append([mx_comp, my_comp])
        xy = np.array(xy)

        # 2. 硬铁校准：去中心
        center_hard, _ = fit_circle_least_squares(xy)
        hard_calibrated = xy - center_hard

        # 3. 软铁校准：单轴缩放 → 对角矩阵（与原算法等价）
        radius_x = (np.max(hard_calibrated[:, 0]) - np.min(hard_calibrated[:, 0])) / 2
        radius_y = (np.max(hard_calibrated[:, 1]) - np.min(hard_calibrated[:, 1])) / 2
        scale = radius_x / radius_y if radius_y != 0 else 1.0

        # 构造对角矩阵（与原单轴缩放完全等价）
        soft_matrix = np.array([[1.0, 0.0],
                                [0.0, scale]])
        soft_inv = soft_matrix  # 对角矩阵逆矩阵即取倒数

        soft_calibrated = (soft_inv @ hard_calibrated.T).T
        final_calibrated = soft_calibrated

        # 4. 离群过滤
        dist = np.linalg.norm(final_calibrated, axis=1)
        median_dist = np.median(dist)
        std_dist = np.std(dist)
        mask = np.abs(dist - median_dist) < 3 * std_dist
        filtered_final_calibrated = final_calibrated[mask]

        # 5. 绘图
        if self.ax_projected is not None:
            self.ax_projected.cla()
            self.ax_projected.set_title("Tilt Compensated Data")
            self.ax_projected.axis('equal')
            self.ax_projected.plot(xy[:, 0], xy[:, 1], 'b.', alpha=0.7)
            self.ax_projected.plot(center_hard[0], center_hard[1], 'ro', label='Center')
            self.ax_projected.plot(0, 0, 'g+', markersize=10)
            self.ax_projected.legend()

        if self.ax_soft_iron is not None:
            self.ax_soft_iron.cla()
            self.ax_soft_iron.set_title("Hard Iron Calibrated")
            self.ax_soft_iron.axis('equal')
            self.ax_soft_iron.plot(hard_calibrated[:, 0], hard_calibrated[:, 1], 'r.', alpha=0.7)
            self.ax_soft_iron.plot(0, 0, 'k+', markersize=10, mew=2)

        if self.ax_final is not None:
            self.ax_final.cla()
            self.ax_final.set_title("Final Calibrated (Hard + Soft Iron)")
            self.ax_final.axis('equal')
            self.ax_final.plot(filtered_final_calibrated[:, 0], filtered_final_calibrated[:, 1], 'g.', alpha=0.7)
            self.ax_final.plot(0, 0, 'k+', markersize=10, mew=2)

        # 6. 统一坐标轴
        R = max(np.abs(hard_calibrated).max(), np.abs(filtered_final_calibrated).max())
        for ax in (self.ax_original, self.ax_projected, self.ax_soft_iron, self.ax_final):
            if ax is not None:
                ax.set_xlim(-R, R)
                ax.set_ylim(-R, R)
                ax.set_aspect('equal', adjustable='box')

        if self.fig1 is not None:
            self.fig1.canvas.draw_idle()
            self.fig1.canvas.flush_events()
        if self.fig2 is not None:
            self.fig2.canvas.draw_idle()
            self.fig2.canvas.flush_events()
            self.fig2.canvas.manager.window.move(50, 600)

        # 7. 保存参数
        self.calibration_params = (center_hard, soft_inv,
                                   np.column_stack([mx, my]),
                                   filtered_final_calibrated)

        self.window.set_status("Calibration finished. Click View Result to see results.")
        self.window.enable_view_btn(True)
        self.window.start_btn.setEnabled(True)
        self.window.port_combo.setEnabled(True)
        self.window.baud_combo.setEnabled(True)
        self.window.refresh_btn.setEnabled(True)
    def view_result(self):
        if not self.calibration_params:
            self.window.set_status("Please finish calibration first.")
            return
        
        center, scale, original_data, calibrated = self.calibration_params
        
        # 生成C代码
        c_code = generate_c_code(center, scale)
        
        # 显示详细结果
        print("\n" + "="*50)
        print("Magnetometer Calibration Results")
        print("="*50)
        print(f"Hard Iron Offset: X={center[0]:.2f}, Y={center[1]:.2f}")
        print(f"Soft Iron Scale: {scale:.4f}")
        print(f"Original Data Points: {len(original_data)}")
        print(f"Calibrated Data Points: {len(calibrated)}")
        
        # 计算校准效果
        if len(original_data) > 0 and len(calibrated) > 0:
            raw_radius = np.sqrt(original_data[:, 0]**2 + original_data[:, 1]**2)
            cal_radius = np.sqrt(calibrated[:, 0]**2 + calibrated[:, 1]**2)
            raw_std = np.std(raw_radius)
            cal_std = np.std(cal_radius)
            improvement = (raw_std - cal_std) / raw_std * 100
            print(f"Original Data STD: {raw_std:.2f}")
            print(f"Calibrated Data STD: {cal_std:.2f}")
            print(f"Improvement: {improvement:.1f}%")
        
        print("="*50)
        
        # 显示图形：原始椭圆数据 vs 校准后数据
        plot_two(original_data, calibrated, center, scale)
        
        # 显示结果对话框
        self.window.show_result_dialog(c_code)

    def run(self):
        self.window.show()
        sys.exit(self.app.exec_())