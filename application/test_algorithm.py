import numpy as np
import matplotlib.pyplot as plt

# 1. 读取数据
try:
    data = np.loadtxt('水平数据.csv', delimiter=',', skiprows=1)
except Exception:
    data = np.loadtxt('水平数据.csv', delimiter=',')  # 无表头

mx, my, mz, pitch, roll = data.T

# 2. 倾角补偿
def tilt_compensate(mx, my, mz, pitch, roll):
    mx_h = mx * np.cos(pitch) + mz * np.sin(pitch)
    my_h = mx * np.sin(roll) * np.sin(pitch) + my * np.cos(roll) - mz * np.sin(roll) * np.cos(pitch)
    return mx_h, my_h

mx_h, my_h = tilt_compensate(mx, my, mz, pitch, roll)
xy = np.stack([mx_h, my_h], axis=1)

print("倾角补偿后数据样例：")
print(xy[:5])

# 3. 椭圆拟合
def fit_circle_least_squares(xy):
    x = xy[:, 0]
    y = xy[:, 1]
    A = np.c_[2*x, 2*y, np.ones(x.shape)]
    b = x**2 + y**2
    c, resid, rank, s = np.linalg.lstsq(A, b, rcond=None)
    center_x, center_y = c[0], c[1]
    radius = np.sqrt(c[2] + center_x**2 + center_y**2)
    return np.array([center_x, center_y]), radius

def calibrate_2d_ellipse(xy):
    radius_x = (np.max(xy[:, 0]) - np.min(xy[:, 0])) / 2
    radius_y = (np.max(xy[:, 1]) - np.min(xy[:, 1])) / 2
    scale = radius_x / radius_y if radius_y != 0 else 1.0
    soft_calibrated = xy.copy()
    soft_calibrated[:, 1] *= scale
    center, _ = fit_circle_least_squares(soft_calibrated)
    calibrated = soft_calibrated - center
    return calibrated, center, scale

calibrated, center, scale = calibrate_2d_ellipse(xy)
print("拟合中心:", center, "y轴缩放比:", scale)

# 4. 分组模拟两次采集
n = len(mx_h)
cal1 = calibrated[:n//2]
cal2 = calibrated[n//2:]

def heading_array(mx, my):
    headings = np.degrees(np.arctan2(-my, mx))
    headings[headings < 0] += 360
    return headings

heading1 = heading_array(cal1[:,0], cal1[:,1])
heading2 = heading_array(cal2[:,0], cal2[:,1])
mean1 = np.mean(heading1)
mean2 = np.mean(heading2)
std1 = np.std(heading1)
std2 = np.std(heading2)
heading_drift = abs(mean1 - mean2)

print('水平均值:', mean1, 'std:', std1)
print('30度均值:', mean2, 'std:', std2)
print('heading_drift:', heading_drift)

# 5. 可视化
plt.figure(figsize=(10,4))
plt.subplot(1,2,1)
plt.title('Tilt Compensated')
plt.scatter(mx_h, my_h, s=10, label='Tilt Compensated')
plt.axis('equal')
plt.legend()
plt.subplot(1,2,2)
plt.title('Calibrated')
plt.scatter(calibrated[:,0], calibrated[:,1], s=10, label='Calibrated')
plt.axis('equal')
plt.legend()
plt.show()