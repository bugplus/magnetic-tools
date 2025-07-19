# config.py

# Calibration duration in seconds
CALIBRATION_DURATION = 60

# 磁力计到IMU坐标轴的映射关系
# 例如: imu_x = mag_map[0][0]*mag_x + mag_map[0][1]*mag_y + mag_map[0][2]*mag_z
# 磁力计到IMU坐标轴的映射关系
# 尝试新的映射：X→Y, Y→X, Z→Z
MAG_AXIS_MAP = [
       [ 0,  1,  0],  # imu_x = mag_y
       [ 1,  0,  0],  # imu_y = mag_x
       [ 0,  0,  1],  # imu_z = mag_z
   ]