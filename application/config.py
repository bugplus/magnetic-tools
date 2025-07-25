# config.py
# ---------------- 采样 & 门控 ----------------
CALIBRATION_DURATION = 30          # 每步采集时长（秒）
MIN_3D_POINTS        = 100         # 每步最少点数
ANGLE_GATE_DEG       = 2.0         # 角度变化门控阈值（度）
DIST_GATE_CM         = 1.0         # 磁力空间稀疏阈值（厘米）

# ---------------- 坐标系 ----------------
MAG_AXIS_MAP_A = [
    [0, 1, 0],
    [1, 0, 0],
    [0, 0, 1],
]

MAG_AXIS_MAP_B = [
    [ 1,  0, 0],
    [ 0, -1, 0],
    [ 0,  0, 1],
]

SENSOR_TYPE = "B"