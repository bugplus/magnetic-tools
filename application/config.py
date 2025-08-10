# config.py
# ---------------- 采样 & 门控 ----------------
CALIBRATION_DURATION = 60          # 每步采集时长（秒）
MIN_3D_POINTS        = 200         # 每步最少点数
ANGLE_GATE_DEG       = 0.0         # 角度变化门控阈值（度）
DIST_GATE_CM         = 0.0         # 磁力空间稀疏阈值（厘米）
UNIT_SPHERE_SCALE = 1.02
DECIMAL_PRECISION = 8

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