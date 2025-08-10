#!/usr/bin/env python3
"""
Magnetometer Calibration Validation Tool
Usage:
    1. Validate raw data: python verify_mag.py raw raw_mag.csv
    2. Validate calibrated data: python verify_mag.py cal calibrated_mag.csv
    3. Full validation process: python verify_mag.py full raw_mag.csv
"""

import numpy as np
import sys
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def fit_ellipsoid_3d(points):
    """Ellipsoid fitting algorithm (same as compass_app.py)"""
    pts = np.asarray(points, dtype=float)
    # Handle 6-column data (mx, my, mz, pitch, roll, yaw) by using only first 3 columns
    if pts.ndim == 2 and pts.shape[1] == 6:
        pts = pts[:, :3]
    elif pts.ndim == 2 and pts.shape[1] != 3:
        raise ValueError("Data must be in N×3 or N×6 format")
        
    if pts.shape[0] < 10:
        raise ValueError("Insufficient points (min 10 required)")

    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    D = np.column_stack([x*x, y*y, z*z, x*y, x*z, y*z, x, y, z, np.ones_like(x)])

    # Tikhonov regularization
    lam = 1e-6 * np.trace(D.T @ D) / D.shape[1]
    DTD = D.T @ D + lam * np.eye(10)
    DTy = D.T @ np.ones_like(x)
    
    try:
        coeffs = np.linalg.solve(DTD, DTy)
    except np.linalg.LinAlgError:
        raise RuntimeError("Failed to solve after regularization")

    Aq, Bq, Cq, Dq, Eq, Fq, G, H, I, J = coeffs
    Q = np.array([
        [Aq, Dq/2, Eq/2],
        [Dq/2, Bq, Fq/2],
        [Eq/2, Fq/2, Cq]
    ])
    
    eig_vals, eig_vecs = np.linalg.eigh(Q)
    eig_vals = np.maximum(eig_vals, 1e-6)
    if np.any(eig_vals <= 0):
        raise RuntimeError("Failed to make matrix positive definite")

    b = -np.linalg.solve(Q, [G, H, I]) / 2
    scale = np.sqrt(1.0 / eig_vals)
    A_cal = eig_vecs @ np.diag(scale) @ eig_vecs.T
    A_cal = np.linalg.inv(A_cal)
    return b, A_cal

def ellipsoid_to_sphere(raw_xyz, b, A):
    """Raw magnetic vector → unit sphere vector (same as app algorithm)"""
    # Handle 6-column data (mx, my, mz, pitch, roll, yaw) by using only first 3 columns
    if raw_xyz.ndim == 2 and raw_xyz.shape[1] == 6:
        raw_xyz = raw_xyz[:, :3]
    elif raw_xyz.ndim == 2 and raw_xyz.shape[1] != 3:
        raise ValueError("Data must be in N×3 or N×6 format")
        
    centered = raw_xyz - b
    sphere = (A @ centered.T).T
    return sphere / np.linalg.norm(sphere, axis=1, keepdims=True)

def verify_raw_data(csv_path):
    """Validate raw data quality"""
    raw = np.loadtxt(csv_path, delimiter=',', ndmin=2)
    # Handle 6-column data (mx, my, mz, pitch, roll, yaw) by using only first 3 columns
    if raw.ndim == 2 and raw.shape[1] == 6:
        mag_data = raw[:, :3]
    elif raw.ndim == 2 and raw.shape[1] == 3:
        mag_data = raw
    else:
        raise ValueError("CSV must be in N×3 or N×6 format")
    
    n = len(mag_data)
    print(f"\n=== Raw Data Validation [File: {os.path.basename(csv_path)}] ===")
    print(f"Data points: {n}")
    
    # Basic statistics
    norms = np.linalg.norm(mag_data, axis=1)
    print(f"Norm range: {np.min(norms):.2f} - {np.max(norms):.2f}")
    print(f"Mean: {np.mean(mag_data, axis=0)}")
    
    # Ellipsoid fitting quality
    try:
        b, A = fit_ellipsoid_3d(raw)
        print(f"Fitted offset: {b}")
        print(f"Transformation matrix condition number: {np.linalg.cond(A):.2f}")
        return b, A
    except Exception as e:
        print(f"❌❌ Ellipsoid fitting failed: {str(e)}")
        return None, None

def verify_calibrated_data(csv_path):
    """Validate calibrated data quality"""
    calibrated = np.loadtxt(csv_path, delimiter=',')
    if calibrated.ndim != 2 or calibrated.shape[1] != 3:
        raise ValueError("CSV must be in N×3 format")
    
    print(f"\n=== Calibrated Data Validation [File: {os.path.basename(csv_path)}] ===")
    
    # Calculate metrics
    norms = np.linalg.norm(calibrated, axis=1)
    rad_err = np.max(np.abs(norms - 1.0))
    min_cos = np.min(np.sum(calibrated * calibrated, axis=1))
    
    print(f"Radius error (max abs error): {rad_err:.2e}")
    print(f"Min direction cosine: {min_cos:.6f}")
    
    # Quality rating
    RAD_THRESHOLD = 1e-4
    COS_THRESHOLD = 0.9999
    
    if rad_err < RAD_THRESHOLD and min_cos > COS_THRESHOLD:
        print("✅ Calibrated data validation passed")
        return True
    else:
        issues = []
        if rad_err >= RAD_THRESHOLD:
            issues.append(f"Radius error exceeds threshold: {rad_err:.2e} > {RAD_THRESHOLD}")
        if min_cos <= COS_THRESHOLD:
            issues.append(f"Direction cosine too low: {min_cos:.6f} <= {COS_THRESHOLD}")
        print("❌❌ " + ", ".join(issues))
        return False

def plot_3d_data(data, title, filename, is_calibrated=False):
    """Generate 3D data visualization"""
    # Handle 6-column data (mx, my, mz, pitch, roll, yaw) by using only first 3 columns
    if data.ndim == 2 and data.shape[1] == 6:
        data = data[:, :3]
    elif data.ndim == 2 and data.shape[1] != 3:
        raise ValueError("Data must be in N×3 or N×6 format")
        
    fig = plt.figure(figsize=(10, 8))
    
    if is_calibrated:
        ax = fig.add_subplot(111, projection='3d')
        
        # Draw unit sphere
        u = np.linspace(0, 2 * np.pi, 30)
        v = np.linspace(0, np.pi, 30)
        x = np.outer(np.cos(u), np.sin(v))
        y = np.outer(np.sin(u), np.sin(v))
        z = np.outer(np.ones_like(u), np.cos(v))
        ax.plot_wireframe(x, y, z, color='gray', alpha=0.1)
        
        # Plot calibrated points
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
    print(f"Visualization saved to: {filename}")
    plt.close()

def full_verification(raw_path):
    """Execute full validation process"""
    # Validate raw data
    print("\n" + "="*40)
    print("Starting Full Validation Process")
    print("="*40)
    
    try:
        raw = np.loadtxt(raw_path, delimiter=',', ndmin=2)
        # Handle 6-column data (mx, my, mz, pitch, roll, yaw) by using only first 3 columns
        if raw.ndim == 2 and raw.shape[1] == 6:
            mag_data = raw[:, :3]
        elif raw.ndim == 2 and raw.shape[1] == 3:
            mag_data = raw
        else:
            raise ValueError("Raw data format error - must be N×3 or N×6")
        
        # Plot raw data 3D visualization
        plot_3d_data(mag_data, "Raw Magnetic Data", raw_path.replace('.csv', '_raw.png'))
        
        # Perform fitting
        b, A = verify_raw_data(raw_path)
        if b is None or A is None:
            return False
        
        # Perform calibration transformation
        calibrated = ellipsoid_to_sphere(raw, b, A)
        
        # Save calibrated data
        cal_path = raw_path.replace('raw_', 'cal_').replace('.csv', '_calibrated.csv') 
        # If original file was raw_mag_with_orientation.csv, make it cal_mag_with_orientation.csv
        if "with_orientation" in raw_path:
            cal_path = raw_path.replace('raw_mag_with_orientation.csv', 'cal_mag_with_orientation.csv')
        else:
            cal_path = raw_path.replace('raw_mag.csv', 'cal_mag.csv')
            
        np.savetxt(cal_path, calibrated, delimiter=',', fmt='%.8f')
        print(f"\nCalibrated data saved to: {cal_path}")
        
        # Plot calibrated data 3D visualization
        plot_3d_data(calibrated, "Calibrated Magnetic Data", raw_path.replace('.csv', '_cal.png'), True)
        
        # Validate calibrated data
        cal_result = verify_calibrated_data(cal_path)
        
        print("\n" + "="*40)
        if cal_result:
            print("✅ Full validation passed! All metrics meet requirements")
        else:
            print("❌❌ Full validation failed! Some metrics do not meet requirements")
        print("="*40)
        
        return cal_result
    except Exception as e:
        print(f"❌❌ Validation process error: {str(e)}")
        return False

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Magnetometer Calibration Validation Tool")
        print("="*40)
        print("Usage:")
        print("  Validate raw data: python verify_mag.py raw raw_mag.csv")
        print("  Validate calibrated data: python verify_mag.py cal calibrated_mag.csv")
        print("  Full validation process: python verify_mag.py full raw_mag.csv")
        sys.exit(1)
    
    mode = sys.argv[1].lower()
    csv_path = sys.argv[2]
    
    if not os.path.isfile(csv_path):
        print(f"Error: File not found {csv_path}")
        sys.exit(1)
    
    try:
        if mode == "raw":
            raw = np.loadtxt(csv_path, delimiter=',', ndmin=2)
            # Handle 6-column data (mx, my, mz, pitch, roll, yaw) by using only first 3 columns for plotting
            if raw.ndim == 2 and raw.shape[1] == 6:
                mag_data = raw[:, :3]
            elif raw.ndim == 2 and raw.shape[1] == 3:
                mag_data = raw
            else:
                raise ValueError("Data must be in N×3 or N×6 format")
                
            verify_raw_data(csv_path)
            plot_3d_data(mag_data, "Raw Magnetic Data", csv_path.replace('.csv', '.png'))
        
        elif mode == "cal":
            result = verify_calibrated_data(csv_path)
            calibrated = np.loadtxt(csv_path, delimiter=',')
            plot_3d_data(calibrated, "Calibrated Magnetic Data", csv_path.replace('.csv', '.png'), True)
            sys.exit(0 if result else 1)
        
        elif mode == "full":
            result = full_verification(csv_path)
            sys.exit(0 if result else 1)
        
        else:
            print(f"Error: Unknown mode '{mode}'")
            sys.exit(1)
        
    except Exception as e:
        print(f"❌❌ Runtime error: {str(e)}")
        sys.exit(1)