# compass_plot.py
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

class CompassPlot:
    def __init__(self):
        # 创建三个画布
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(1, 3, figsize=(18, 6))

        # 图1：原始数据
        self.line1, = self.ax1.plot([], [], 'r.', markersize=3)
        self.ax1.set_title("Raw Magnetometer X-Y Data\n(Collecting for Calibration)")
        self.ax1.set_xlabel("mag_x")
        self.ax1.set_ylabel("mag_y")
        self.ax1.axhline(0, color='black', lw=0.5)
        self.ax1.axvline(0, color='black', lw=0.5)
        self.ax1.grid(True)
        self.ax1.axis('equal')
        self.ax1.set_xlim(-300, 300)
        self.ax1.set_ylim(-300, 300)

        # 图2：缩放后
        self.line2, = self.ax2.plot([], [], 'g.', markersize=3)
        self.ax2.set_title("After Scaling Only\n(Scale: x=?, y=?)")
        self.ax2.set_xlabel("mag_x (scaled)")
        self.ax2.set_ylabel("mag_y (scaled)")
        self.ax2.axhline(0, color='black', lw=0.5)
        self.ax2.axvline(0, color='black', lw=0.5)
        self.ax2.grid(True)
        self.ax2.axis('equal')

        # 图3：校准后
        self.line3, = self.ax3.plot([], [], 'b.', markersize=3)
        self.ax3.set_title("Fully Calibrated\n(Offset: (0.0, 0.0), Scale: x=1.000, y=1.000)")
        self.ax3.set_xlabel("mag_x (calibrated)")
        self.ax3.set_ylabel("mag_y (calibrated)")
        self.ax3.axhline(0, color='black', lw=0.5)
        self.ax3.axvline(0, color='black', lw=0.5)
        self.ax3.grid(True)
        self.ax3.axis('equal')

        # 启动动画
        self.ani = FuncAnimation(self.fig, self.update, frames=None,
                                 interval=50, blit=False, cache_frame_data=False)
        plt.tight_layout()
        plt.show()

    def update(self, frame, raw_data=None, calibrated_data=None):
        if raw_data:
            xs = np.array([x[0] for x in raw_data])
            ys = np.array([y[1] for y in raw_data])
            self.line1.set_data(xs, ys)

            x_min, x_max = min(xs), max(xs)
            y_min, y_max = min(ys), max(ys)
            margin = 50
            self.ax1.set_xlim(x_min - margin, x_max + margin)
            self.ax1.set_ylim(y_min - margin, y_max + margin)

        if calibrated_data:
            calibrated_xs, calibrated_ys = calibrated_data
            self.line3.set_data(calibrated_xs, calibrated_ys)

        return self.line1, self.line2, self.line3