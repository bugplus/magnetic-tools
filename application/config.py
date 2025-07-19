# config.py

# Calibration duration in seconds
CALIBRATION_DURATION = 60

# 磁力计到IMU坐标轴的映射关系
# 例如: imu_x = mag_map[0][0]*mag_x + mag_map[0][1]*mag_y + mag_map[0][2]*mag_z
MAG_AXIS_MAP = [
       [ 0, -1,  0],
       [-1,  0,  0],
       [ 0,  0,  1],
   ]