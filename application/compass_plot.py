# compass_plot.py

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from config import MAX_POINTS, CALIBRATION_DURATION, UPDATE_INTERVAL
import time

class CompassPlot:
    def __init__(self, core):
        self.core = core
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(1, 3, figsize=(18, 6))

        # Initialize plot elements
        self.line1, = self.ax1.plot([], [], 'r.', markersize=3)
        self.line2, = self.ax2.plot([], [], 'g.', markersize=3)
        self.line3, = self.ax3.plot([], [], 'b.', markersize=3)
        self.center_point1, = self.ax1.plot([], [], 'ko', markersize=5)  # Center point for raw data
        self.center_point2, = self.ax2.plot([], [], 'ko', markersize=5)  # Center point for scaled data
        self.center_point3, = self.ax3.plot([], [], 'ko', markersize=5)  # Center point for calibrated data

        self.init_axes()

        # Start animation
        self.ani = FuncAnimation(
            self.fig,
            self.update,
            frames=None,
            interval=UPDATE_INTERVAL,
            blit=False,
            cache_frame_data=False
        )

        plt.tight_layout()
        plt.show()

    def init_axes(self):
        # Raw Data plot
        self.ax1.set_title("Raw Magnetometer X-Y Data\n(Collecting for Calibration)")
        self.ax1.set_xlabel("mag_x")
        self.ax1.set_ylabel("mag_y")
        self.ax1.axhline(0, color='black', lw=0.5)
        self.ax1.axvline(0, color='black', lw=0.5)
        self.ax1.grid(True)
        self.ax1.axis('equal')
        self.ax1.set_xlim(-300, 300)
        self.ax1.set_ylim(-300, 300)

        # Scaled Data plot
        self.ax2.set_title("After Scaling Only\n(Scale: x=?, y=?)")
        self.ax2.set_xlabel("mag_x (scaled)")
        self.ax2.set_ylabel("mag_y (scaled)")
        self.ax2.axhline(0, color='black', lw=0.5)
        self.ax2.axvline(0, color='black', lw=0.5)
        self.ax2.grid(True)
        self.ax2.axis('equal')

        # Fully Calibrated Data plot
        self.ax3.set_title("Fully Calibrated\n(Offset: (0.0, 0.0), Scale: x=1.000, y=1.000)")
        self.ax3.set_xlabel("mag_x (calibrated)")
        self.ax3.set_ylabel("mag_y (calibrated)")
        self.ax3.axhline(0, color='black', lw=0.5)
        self.ax3.axvline(0, color='black', lw=0.5)
        self.ax3.grid(True)
        self.ax3.axis('equal')

    def update(self, frame):
        current_time = time.time()

        if self.core.ser and self.core.ser.is_open and self.core.data_collection_started and not self.core.calibration_done:
            while self.core.ser.in_waiting:
                line_str = self.core.ser.readline().decode('utf-8', errors='replace').strip()
                try:
                    data = line_str.split(',')
                    if len(data) >= 2:
                        x = int(data[0].split('=')[1])
                        y = int(data[1].split('=')[1])
                        self.core.raw_data.append((x, y))
                        if len(self.core.raw_data) > MAX_POINTS:
                            self.core.raw_data.pop(0)

                        if self.core.data_collection_started and (current_time - self.core.start_time <= CALIBRATION_DURATION):
                            print(f"Received Data: mag_x={x}, mag_y={y}")

                        if not self.core.data_collection_started:
                            self.core.start_time = current_time
                            self.core.data_collection_started = True

                except Exception as e:
                    print(f"[ERROR] 数据解析失败: {e}")
                    continue

        if len(self.core.raw_data) >= 2 and not self.core.calibration_done:
            xs = np.array([x[0] for x in self.core.raw_data])
            ys = np.array([y[1] for y in self.core.raw_data])
            self.line1.set_data(xs, ys)

            x_min, x_max = min(xs), max(xs)
            y_min, y_max = min(ys), max(ys)
            margin = 50
            self.ax1.set_xlim(x_min - margin, x_max + margin)
            self.ax1.set_ylim(y_min - margin, y_max + margin)

        if self.core.data_collection_started and not self.core.calibration_done and (current_time - self.core.start_time > CALIBRATION_DURATION):
            if len(self.core.raw_data) >= 6:
                xs = np.array([x[0] for x in self.core.raw_data])
                ys = np.array([y[1] for y in self.core.raw_data])

                x_min, x_max = min(xs), max(xs)
                y_min, y_max = min(ys), max(ys)
                margin = 50
                self.core.x_range_final = (x_min - margin, x_max + margin)
                self.core.y_range_final = (y_min - margin, y_max + margin)

                x_range = x_max - x_min
                y_range = y_max - y_min

                if x_range >= y_range:
                    self.core.scale_x = 1.0
                    self.core.scale_y = x_range / y_range
                else:
                    self.core.scale_x = y_range / x_range
                    self.core.scale_y = 1.0

                scaled_xs = xs * self.core.scale_x
                scaled_ys = ys * self.core.scale_y

                self.line2.set_data(scaled_xs, scaled_ys)
                self.ax2.set_title(f"After Scaling Only\n(Scale: x={self.core.scale_x:.3f}, y={self.core.scale_y:.3f})")

                # Calculate and plot center point for scaled data
                self.core.center_x = (max(scaled_xs) + min(scaled_xs)) / 2
                self.core.center_y = (max(scaled_ys) + min(scaled_ys)) / 2
                self.center_point2.set_data([self.core.center_x], [self.core.center_y])

                calibrated_xs = scaled_xs - self.core.center_x
                calibrated_ys = scaled_ys - self.core.center_y

                self.line3.set_data(calibrated_xs, calibrated_ys)
                self.ax3.set_title(f"Fully Calibrated\n(Offset: ({self.core.center_x:.1f}, {self.core.center_y:.1f}), "
                                    f"Scale: x={self.core.scale_x:.3f}, y={self.core.scale_y:.3f})")

                # Plot center point for calibrated data
                self.center_point3.set_data([0], [0])  # Center point for calibrated data is (0,0)

                # Calculate and plot center point for raw data
                raw_center_x = (x_max + x_min) / 2
                raw_center_y = (y_max + y_min) / 2
                self.center_point1.set_data([raw_center_x], [raw_center_y])

                for ax in [self.ax1, self.ax2, self.ax3]:
                    ax.set_xlim(self.core.x_range_final)
                    ax.set_ylim(self.core.y_range_final)

            self.core.calibration_done = True
            print("[INFO] 校准完成，已切换至校准图")

        return self.line1, self.line2, self.line3, self.center_point1, self.center_point2, self.center_point3