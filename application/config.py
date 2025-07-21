# config.py
# config.py

# 原有配置全部保留
CALIBRATION_DURATION = 60

# 原有映射矩阵重命名为 A 板专用
MAG_AXIS_MAP_A = [
    [0, 1, 0],
    [1, 0, 0],
    [0, 0, 1],
]

# 新增 B 板映射矩阵（顺时针90°补偿）
MAG_AXIS_MAP_B = [
    [ 1,  0, 0],  # imu_x =  my
    [ 0, -1, 0],  # imu_y = -mx
    [ 0,  0, 1],  # imu_z =  mz
]

# 新增选择开关（默认用 A 板）
SENSOR_TYPE = "B"  # 改为 "B" 即适配 B 板