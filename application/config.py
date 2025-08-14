# config.py
# ---------------- 采样 & 门控 ----------------
CALIBRATION_DURATION = 60          # 每步采集时长（秒）
MIN_3D_POINTS        = 200         # 每步最少点数
ANGLE_GATE_DEG       = 0.0         # 角度变化门控阈值（度）
DIST_GATE_CM         = 0.0         # 磁力空间稀疏阈值（厘米）
UNIT_SPHERE_SCALE = 1.02
DECIMAL_PRECISION = 8


# ---------------- 自动下一步阈值 ----------------
AUTO_YAW_RANGE_MIN   = 360   # 航向角最小覆盖范围（度）
AUTO_PITCH_RANGE_MIN = 30    # 俯仰角最小覆盖范围（度）
AUTO_ROLL_RANGE_MIN  = 30    # 横滚角最小覆盖范围（度）

# 每步最小点数（与 MIN_3D_POINTS 区分，可独立设置）
AUTO_STEP0_MIN_PTS = 500
AUTO_STEP1_MIN_PTS = 200
AUTO_STEP2_MIN_PTS = 100
# ---------------- 新自动下一步规则 ----------------
GRID_STEP_DEG   = 10   # 每 10° 一格
POINTS_PER_GRID = 10   # 每格至少 10 点
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