#!/usr/bin/env python3
"""
一键验证地磁椭球校准正确性
用法：
    python verify_cal.py raw_mag.csv
"""

import numpy as np
import sys
import os

# ---------------------------------------
# 1. 椭球拟合（带行列式符号修正）
# ---------------------------------------
def fit_ellipsoid_3d(points):
    pts = np.asarray(points, dtype=float)
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    D = np.column_stack([x*x, y*y, z*z, x*y, x*z, y*z, x, y, z, np.ones_like(x)])
    lam = 1e-6 * np.trace(D.T @ D) / D.shape[1]
    coeffs = np.linalg.solve(D.T @ D + lam * np.eye(10), D.T @ np.ones_like(x))
    Aq, Bq, Cq, Dq, Eq, Fq, G, H, I, _ = coeffs
    Q = np.array([[Aq, Dq/2, Eq/2], [Dq/2, Bq, Fq/2], [Eq/2, Fq/2, Cq]])
    eig_vals, eig_vecs = np.linalg.eigh(Q)
    eig_vals = np.maximum(eig_vals, 1e-6)
    b = -np.linalg.solve(Q, [G, H, I]) / 2
    scale = np.sqrt(1.0 / eig_vals)
    A = np.linalg.inv(eig_vecs @ np.diag(scale) @ eig_vecs.T)
    # 强制行列式为正
    if np.linalg.det(A) < 0:
        A = -A
    return b, A

def ellipsoid_to_sphere(raw_xyz, b, A):
    centered = raw_xyz - b
    sphere = (A @ centered.T).T
    return sphere / np.linalg.norm(sphere, axis=1, keepdims=True)

# ---------------------------------------
# 2. 验证
# ---------------------------------------
def verify(csv_path):
    raw = np.loadtxt(csv_path, delimiter=',')
    if raw.ndim != 2 or raw.shape[1] != 3:
        raise ValueError("CSV 须为 N×3 磁向量")

    b, A = fit_ellipsoid_3d(raw)
    unit = ellipsoid_to_sphere(raw, b, A)

    rad_err = np.max(np.abs(np.linalg.norm(unit, axis=1) - 1.0))
    print(f"半径误差: {rad_err:.2e}")

    ok = rad_err < 1e-4          # 只保留这一条
    print("✅ 校准算法正确" if ok else "❌ 校准算法有误")
    return ok

# ---------------------------------------
# 3. 入口
# ---------------------------------------
if __name__ == "__main__":
    if len(sys.argv) != 2 or not os.path.isfile(sys.argv[1]):
        print("用法: python verify_cal.py raw_mag.csv")
        sys.exit(1)
    try:
        verify(sys.argv[1])
    except Exception as e:
        print("❌ 运行失败:", e)