# 文件名: config.py
# 说明: 包含程序的全局配置参数

# 串口通信参数
PORT = 'COM4'               # 默认串口号
BAUD_RATE = 115200          # 波特率
TIMEOUT = 1                 # 串口超时时间（秒）

# 数据更新和校准参数
UPDATE_INTERVAL = 50        # 更新间隔（毫秒）
MAX_POINTS = 600            # 最大数据点数
CALIBRATION_DURATION = 30   # 采集持续时间（秒）
TOLERANCE_THRESHOLD = 1     # 点的容忍阈值，单位：像素

# 图形界面参数
WINDOW_WIDTH = 400          # 窗口宽度
WINDOW_HEIGHT = 200         # 窗口高度
BAUD_RATE_OPTIONS = ["9600", "19200", "38400", "57600", "115200"]  # 波特率选项
DEFAULT_BAUD_RATE = "115200"  # 默认波特率