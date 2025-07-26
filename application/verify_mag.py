#!/usr/bin/env python3
"""
磁力计校准系统验证工具
用法：
    1. 验证原始数据质量：python verify_mag.py raw raw_mag.csv
    2. 验证校准数据质量：python verify_mag.py cal calibrated_mag.csv
    3. 完整流程验证：python verify_mag.py full raw_mag.csv
"""

import numpy as np
import sys
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def fit_ellipsoid_3d(points):
    """椭球拟合算法（与compass_app.py相同）"""
    pts = np.asarray(points, dtype=float)
    if pts.shape[0] < 10:
        raise ValueError("点数不足 10")

    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    D = np.column_stack([x*x, y*y, z*z, x*y, x*z, y*z, x, y, z, np.ones_like(x)])

    # Tikhonov正则化
    lam = 1e-6 * np.trace(D.T @ D) / D.shape[1]
    DTD = D.T @ D + lam * np.eye(10)
    DTy = D.T @ np.ones_like(x)
    
    try:
        coeffs = np.linalg.solve(DTD, DTy)
    except np.linalg.LinAlgError:
        raise RuntimeError("正则化后仍无法求解")

    Aq, Bq, Cq, Dq, Eq, Fq, G, H, I, J = coeffs
    Q = np.array([
        [Aq, Dq/2, Eq/2],
        [Dq/2, Bq, Fq/2],
        [Eq/2, Fq/2, Cq]
    ])
    
    eig_vals, eig_vecs = np.linalg.eigh(Q)
    eig_vals = np.maximum(eig_vals, 1e-6)
    if np.any(eig_vals <= 0):
        raise RuntimeError("强制正定后仍失败")

    b = -np.linalg.solve(Q, [G, H, I]) / 2
    scale = np.sqrt(1.0 / eig_vals)
    A_cal = eig_vecs @ np.diag(scale) @ eig_vecs.T
    A_cal = np.linalg.inv(A_cal)
    return b, A_cal

def ellipsoid_to_sphere(raw_xyz, b, A):
    """原始磁向量 → 单位球向量（与app算法相同）"""
    centered = raw_xyz - b
    sphere = (A @ centered.T).T
    return sphere / np.linalg.norm(sphere, axis=1, keepdims=True)

def verify_raw_data(csv_path):
    """验证原始数据质量"""
    raw = np.loadtxt(csv_path, delimiter=',', ndmin=2)
    if raw.ndim != 2 or raw.shape[1] != 3:
        raise ValueError("CSV必须为N×3格式")
    
    n = len(raw)
    print(f"\n=== 原始数据验证 [文件: {os.path.basename(csv_path)}] ===")
    print(f"数据点数: {n}")
    
    # 基本统计
    norms = np.linalg.norm(raw, axis=1)
    print(f"模长范围: {np.min(norms):.2f} - {np.max(norms):.2f}")
    print(f"均值: {np.mean(raw, axis=0)}")
    
    # 椭球拟合质量
    try:
        b, A = fit_ellipsoid_3d(raw)
        print(f"拟合偏移量: {b}")
        print(f"拟合转换矩阵条件数: {np.linalg.cond(A):.2f}")
        return b, A
    except Exception as e:
        print(f"❌ 椭球拟合失败: {str(e)}")
        return None, None

def verify_calibrated_data(csv_path):
    """验证校准后数据质量"""
    calibrated = np.loadtxt(csv_path, delimiter=',')
    if calibrated.ndim != 2 or calibrated.shape[1] != 3:
        raise ValueError("CSV必须为N×3格式")
    
    print(f"\n=== 校准数据验证 [文件: {os.path.basename(csv_path)}] ===")
    
    # 计算指标
    norms = np.linalg.norm(calibrated, axis=1)
    rad_err = np.max(np.abs(norms - 1.0))
    min_cos = np.min(np.sum(calibrated * calibrated, axis=1))
    
    print(f"半径误差 (最大绝对误差): {rad_err:.2e}")
    print(f"最小方向余弦: {min_cos:.6f}")
    
    # 质量评级
    RAD_THRESHOLD = 1e-4
    COS_THRESHOLD = 0.9999
    
    if rad_err < RAD_THRESHOLD and min_cos > COS_THRESHOLD:
        print("✅ 校准数据验证通过")
        return True
    else:
        issues = []
        if rad_err >= RAD_THRESHOLD:
            issues.append(f"半径误差超标: {rad_err:.2e} > {RAD_THRESHOLD}")
        if min_cos <= COS_THRESHOLD:
            issues.append(f"方向余弦不足: {min_cos:.6f} <= {COS_THRESHOLD}")
        print("❌ " + ", ".join(issues))
        return False

def plot_3d_data(data, title, filename, is_calibrated=False):
    """生成3D数据可视化图表"""
    fig = plt.figure(figsize=(10, 8))
    
    if is_calibrated:
        ax = fig.add_subplot(111, projection='3d')
        
        # 绘制单位球
        u = np.linspace(0, 2 * np.pi, 30)
        v = np.linspace(0, np.pi, 30)
        x = np.outer(np.cos(u), np.sin(v))
        y = np.outer(np.sin(u), np.sin(v))
        z = np.outer(np.ones_like(u), np.cos(v))
        ax.plot_wireframe(x, y, z, color='gray', alpha=0.1)
        
        # 绘制校准点
        norms = np.linalg.norm(data, axis=1)
        ax.scatter(
            data[:,0], data[:,1], data[:,2], 
            c=norms,
            cmap='viridis', s=20, alpha=0.7
        )
        
        ax.set_title(title)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_box_aspect([1,1,1])
    else:
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(data[:,0], data[:,1], data[:,2], s=5)
        ax.set_title(title)
        ax.set_box_aspect([1,1,1])
    
    plt.tight_layout()
    plt.savefig(filename)
    print(f"可视化图表已保存至: {filename}")
    plt.close()

def full_verification(raw_path):
    """执行完整验证流程"""
    # 验证原始数据
    print("\n" + "="*40)
    print("开始完整验证流程")
    print("="*40)
    
    try:
        raw = np.loadtxt(raw_path, delimiter=',', ndmin=2)
        if raw.ndim != 2 or raw.shape[1] != 3:
            raise ValueError("原始数据格式错误")
        
        # 绘制原始数据3D图
        plot_3d_data(raw, "原始磁力数据", raw_path.replace('.csv', '_raw.png'))
        
        # 执行拟合
        b, A = verify_raw_data(raw_path)
        if b is None or A is None:
            return False
        
        # 执行校准转换
        calibrated = ellipsoid_to_sphere(raw, b, A)
        
        # 保存校准后数据
        cal_path = raw_path.replace('raw_', 'cal_').replace('.csv', '_calibrated.csv')
        np.savetxt(cal_path, calibrated, delimiter=',', fmt='%.8f')
        print(f"\n校准后数据已保存至: {cal_path}")
        
        # 绘制校准后数据3D图
        plot_3d_data(calibrated, "校准后磁力数据", raw_path.replace('.csv', '_cal.png'), True)
        
        # 验证校准数据
        cal_result = verify_calibrated_data(cal_path)
        
        print("\n" + "="*40)
        if cal_result:
            print("✅ 完整验证通过! 所有指标符合要求")
        else:
            print("❌ 完整验证失败! 存在不符合项")
        print("="*40)
        
        return cal_result
    except Exception as e:
        print(f"❌ 验证过程出错: {str(e)}")
        return False

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("磁力计校准验证工具")
        print("="*40)
        print("用法:")
        print("  验证原始数据质量: python verify_mag.py raw raw_mag.csv")
        print("  验证校准数据质量: python verify_mag.py cal calibrated_mag.csv")
        print("  执行完整验证流程: python verify_mag.py full raw_mag.csv")
        sys.exit(1)
    
    mode = sys.argv[1].lower()
    csv_path = sys.argv[2]
    
    if not os.path.isfile(csv_path):
        print(f"错误: 文件不存在 {csv_path}")
        sys.exit(1)
    
    try:
        if mode == "raw":
            verify_raw_data(csv_path)
            plot_3d_data(np.loadtxt(csv_path, delimiter=','), "原始磁力数据", csv_path.replace('.csv', '.png'))
        
        elif mode == "cal":
            result = verify_calibrated_data(csv_path)
            plot_3d_data(np.loadtxt(csv_path, delimiter=','), "校准后磁力数据", csv_path.replace('.csv', '.png'), True)
            sys.exit(0 if result else 1)
        
        elif mode == "full":
            result = full_verification(csv_path)
            sys.exit(0 if result else 1)
        
        else:
            print(f"错误: 未知模式 '{mode}'")
            sys.exit(1)
        
    except Exception as e:
        print(f"❌ 运行失败: {str(e)}")
        sys.exit(1)