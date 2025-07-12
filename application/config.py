# config.py
PORT = 'COM4'               # 默认串口号
BAUD_RATE = 115200          # 波特率
TIMEOUT = 1                 # 串口超时时间
UPDATE_INTERVAL = 50        # 更新间隔（毫秒）
MAX_POINTS = 600            # 最大数据点数
CALIBRATION_DURATION = 30   # 采集持续时间（秒）
TOLERANCE_THRESHOLD = 1     # 点的容忍阈值，单位：像素
# 添加以下参数
REALTIME_UPDATE_INTERVAL = 100  # 实时更新间隔(毫秒)
MAX_REALTIME_POINTS = 100       # 实时显示的最大点数