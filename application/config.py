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
# config.py 追加
STEP0_YAW_STEP_DEG = 5          # Step0 水平每 2° 一格
STEP0_PPG          = 3          # Step0 每格 3 点


# ---------------- 干扰强度指标 IMF 阈值 ----------------
# IMF < IMF_CLEAN_TH  → 干净环境（标准3D椭球）
# IMF ≥ IMF_CLEAN_TH  → 大干扰（终极鲁棒圆）
IMF_CLEAN_TH = 0.05          # 干净/干扰分界

# ---------------- 圆度误差阈值（旧） ----------------
# 仅保留给旧代码兼容，可逐步废弃
CIRCULARITY_ERROR_THRESHOLD = 0.16
# 自动算法切换阈值：Step-0 圆度误差 > 该值走 2D 保底
CIRCULARITY_ERROR_THRESHOLD = 0.16  # 30%

# ---------------- 鲁棒圆校准一次性参数 ----------------
ROBUST_N_SIGMA      = 2.0     # 剃飞点：> nσ 直接扔
ROBUST_MAX_ITER     = 50      # 迭代次数，给足
ROBUST_HARD_STRETCH = True    # True=强制标准差一致（视觉正圆）

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