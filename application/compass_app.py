"""
File: compass_app.py
Purpose: Core compass application class
Description: Full support for split-output sensor data
"""
import math
import numpy as np
import matplotlib.pyplot as plt
import serial
import time
import queue
import threading
import re
from config import config

class CompassApp:
    """Optimized for split-output sensor data devices"""
    def __init__(self, port, baud_rate):
        self.port = port
        self.baud_rate = baud_rate
        self.ser = None
        self.data_collection_active = False
        self.serial_thread_active = False
        self.serial_thread = None
        self.data_queue = queue.Queue(maxsize=200)
        
        # Data storage
        self.raw_data = []
        self.compensated_data = []
        self.xs, self.ys = [], []
        
        # Data buffers
        self.current_mag = {}   # {"mag_x": -101, "mag_y": 116, ...}
        self.current_att = {}   # {"pitch": -0.32, "roll": 0.13, ...}
        
        # Calibration
        self.scale_x = 1.0
        self.scale_y = 1.0
        self.center_x = 0.0
        self.center_y = 0.0
        self.calibration_complete = False
        
        # Plotting
        self.fig = None
        self.ax1, self.ax2, self.ax3 = None, None, None
        self.scatter1, self.scatter2, self.scatter3 = None, None, None

    # === SERIAL COMMUNICATION ===
    def connect_serial(self):
        """Connect to serial port"""
        try:
            # Close existing connection
            if self.ser and self.ser.is_open:
                self.disconnect_serial()
                
            # Open new connection
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=0.1,
                write_timeout=0.1,
                inter_byte_timeout=0.01
            )
            
            # Start reading thread
            self.serial_thread_active = True
            self.serial_thread = threading.Thread(target=self.serial_reading_thread)
            self.serial_thread.daemon = True
            self.serial_thread.start()
            
            print(f"Connected to {self.port} @ {self.baud_rate} baud")
            return True
        except Exception as e:
            print(f"Serial connection failed: {e}")
            return False
            
    def disconnect_serial(self):
        """Disconnect from serial port"""
        try:
            # Stop thread first
            self.serial_thread_active = False
            
            # Close serial port
            if self.ser and self.ser.is_open:
                self.ser.close()
                print("Serial port disconnected")
            
            # Cleanup
            self.ser = None
            self.clear_buffers()
            return True
        except Exception as e:
            print(f"Disconnect error: {e}")
            return False
            
    def clear_buffers(self):
        """Reset data buffers"""
        self.data_queue = queue.Queue(maxsize=200)
        self.current_mag = {}
        self.current_att = {}
    
    def serial_reading_thread(self):
        """Thread for continuous serial reading"""
        buffer = ""
        while self.serial_thread_active:
            try:
                # Skip if no connection
                if not self.ser or not self.ser.is_open:
                    time.sleep(0.1)
                    continue
                    
                # Read available data
                data = self.ser.read(self.ser.in_waiting).decode('ascii', errors='ignore')
                if data:
                    buffer += data
                    
                    # Process all complete lines
                    while '\n' in buffer:
                        line_end = buffer.index('\n')
                        line = buffer[:line_end].strip()
                        buffer = buffer[line_end+1:]
                        
                        # Add to processing queue
                        if line:
                            self.data_queue.put(line)
                else:
                    time.sleep(0.01)  # Reduce CPU usage
                    
            except Exception as e:
                print(f"Serial thread error: {e}")
                buffer = ""
                time.sleep(0.1)
    
    # === DATA PROCESSING ===
    def start_data_collection(self):
        """Prepare for new data collection"""
        self.raw_data = []
        self.compensated_data = []
        self.xs, self.ys = [], []
        self.clear_buffers()
        self.data_collection_active = True
        print("Data collection started")
        return True
        
    def stop_data_collection(self):
        """Stop data collection"""
        self.data_collection_active = False
        print("Data collection stopped")
        return True
        
    def collect_data(self):
        """Process sensor data from queue"""
        if not self.data_collection_active:
            return False
            
        # Process available lines
        processed = False
        while not self.data_queue.empty():
            line = self.data_queue.get_nowait()
            self.process_data_line(line)
            
            # Check if we have complete data set
            if self.check_complete_dataset():
                self.calculate_datapoint()
                processed = True
                
        return processed
    
    def process_data_line(self, line):
        """Process a single data line"""
        try:
            # Split and trim all pairs
            pairs = [pair.strip() for pair in line.split(',')]
            
            # Process each key-value pair
            for pair in pairs:
                if '=' not in pair:
                    continue
                    
                key, value = pair.split('=', 1)
                key = key.strip().lower()
                if not key:
                    continue
                    
                # Convert value to float (when possible)
                try:
                    num_value = float(value)
                except ValueError:
                    continue
                    
                # Store in appropriate buffer
                if key in ['mag_x', 'mag_y', 'mag_z']:
                    self.current_mag[key] = num_value
                elif key in ['pitch', 'roll', 'raw']:
                    self.current_att[key] = num_value
                    
        except Exception as e:
            print(f"Data processing error: {e}")
    
    def check_complete_dataset(self):
        """Check if we have all required fields"""
        mag_ok = all(key in self.current_mag for key in ['mag_x', 'mag_y'])
        att_ok = all(key in self.current_att for key in ['pitch', 'roll'])
        return mag_ok and att_ok
    
    def calculate_datapoint(self):
        """Calculate tilt-compensated datapoint"""
        mag_x = self.current_mag['mag_x']
        mag_y = self.current_mag['mag_y']
        mag_z = self.current_mag.get('mag_z', 0.0)
        pitch = self.current_att['pitch']
        roll = self.current_att['roll']
        
        try:
            # Apply tilt compensation
            x_comp, y_comp = self.tilt_compensation(mag_x, mag_y, mag_z, pitch, roll)
            
            # Store data
            self.raw_data.append((mag_x, mag_y, mag_z, pitch, roll))
            self.compensated_data.append((x_comp, y_comp))
            self.xs.append(x_comp)
            self.ys.append(y_comp)
            
            print(f"Collected: X={x_comp:.1f}, Y={y_comp:.1f}")
            
            # Update plots
            if self.fig:
                self.update_plots()
                
            # Clear buffers for next set
            self.current_mag = {}
            self.current_att = {}
            
            return True
        except Exception as e:
            print(f"Calculation error: {e}")
            return False
        
    def tilt_compensation(self, mag_x, mag_y, mag_z, pitch, roll):
        """Tilt compensation: Project magnetometer data to horizontal plane"""
        pitch_rad = math.radians(pitch)
        roll_rad = math.radians(roll)
        
        cos_pitch = math.cos(pitch_rad)
        sin_pitch = math.sin(pitch_rad)
        cos_roll = math.cos(roll_rad)
        sin_roll = math.sin(roll_rad)
        
        mag_x_comp = mag_x * cos_pitch + mag_z * sin_pitch
        mag_y_comp = mag_x * sin_roll * sin_pitch + mag_y * cos_roll - mag_z * sin_roll * cos_pitch
        
        return mag_x_comp, mag_y_comp
    
    # === PLOTTING ===
    def init_plots(self):
        """Initialize the three-plot display"""
        plt.ion()  # Enable interactive mode
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(1, 3, figsize=(18, 6))
        plt.subplots_adjust(wspace=0.3)
        
        # Figure 1: Raw data
        self.scatter1 = self.ax1.scatter([], [], c=config.COLORS["raw"], s=15, alpha=0.6)
        self.ax1.set_title(config.PLOT_TITLES["raw_data"])
        self.ax1.set_xlabel("Mag X")
        self.ax1.set_ylabel("Mag Y")
        self.ax1.grid(True)
        self.ax1.set_xlim(-1000, 1000)
        self.ax1.set_ylim(-1000, 1000)
        
        # Figure 2: Scaled data
        self.scatter2 = self.ax2.scatter([], [], c=config.COLORS["scaled"], s=15, alpha=0.6)
        self.ax2.set_title(config.PLOT_TITLES["scaled_data"])
        self.ax2.set_xlabel("Scaled X")
        self.ax2.set_ylabel("Scaled Y")
        self.ax2.grid(True)
        self.ax2.set_xlim(-1000, 1000)
        self.ax2.set_ylim(-1000, 1000)
        
        # Figure 3: Calibrated data
        self.scatter3 = self.ax3.scatter([], [], c=config.COLORS["calibrated"], s=15, alpha=0.6)
        self.ax3.set_title(config.PLOT_TITLES["calibrated_data"])
        self.ax3.set_xlabel("Calibrated X")
        self.ax3.set_ylabel("Calibrated Y")
        self.ax3.grid(True)
        self.ax3.axhline(0, color='black', linewidth=0.8)
        self.ax3.axvline(0, color='black', linewidth=0.8)
        self.ax3.set_xlim(-500, 500)
        self.ax3.set_ylim(-500, 500)
        
        plt.tight_layout()
        self.fig.canvas.draw()
        plt.show(block=False)
        print("Plots initialized")
        
    def update_plots(self):
        """Update plots with latest data"""
        if not self.fig or not self.xs:
            return
            
        try:
            # Recent points only
            display_count = min(100, len(self.xs))
            xs = self.xs[-display_count:]
            ys = self.ys[-display_count:]
            
            # Update raw plot
            self.scatter1.set_offsets(np.column_stack([xs, ys]))
            
            # Update scaled plot
            scaled_x = [x * self.scale_x for x in xs]
            scaled_y = [y * self.scale_y for y in ys]
            self.scatter2.set_offsets(np.column_stack([scaled_x, scaled_y]))
            
            # Update calibrated plot
            cal_x = [x * self.scale_x - self.center_x for x in xs]
            cal_y = [y * self.scale_y - self.center_y for y in ys]
            self.scatter3.set_offsets(np.column_stack([cal_x, cal_y]))
            
            # Redraw
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(0.001)
        except Exception as e:
            print(f"Plot update error: {e}")
    
    # === CALIBRATION ===
    def calibrate_data(self):
        """Perform calibration algorithm"""
        if len(self.compensated_data) < 20:
            print("Calibration failed: insufficient data")
            return False
            
        try:
            # Calculate center
            xs, ys = zip(*self.compensated_data)
            self.center_x = sum(xs) / len(xs)
            self.center_y = sum(ys) / len(ys)
            
            # Calculate scale factors
            x_range = max(xs) - min(xs)
            y_range = max(ys) - min(ys)
            avg_range = (x_range + y_range) / 2
            
            self.scale_x = avg_range / x_range if x_range > 0 else 1.0
            self.scale_y = avg_range / y_range if y_range > 0 else 1.0
            
            print(f"Calibration complete: center=({self.center_x:.1f}, {self.center_y:.1f}), "
                  f"scale=({self.scale_x:.3f}, {self.scale_y:.3f})")
            
            self.calibration_complete = True
            return True
        except Exception as e:
            print(f"Calibration error: {e}")
            return False
    
    # === REAL-TIME PROCESSING ===
    def realtime_calibration(self, mag_x, mag_y, mag_z, pitch, roll):
        """Apply calibration to new data point"""
        try:
            # Apply tilt compensation
            x_comp, y_comp = self.tilt_compensation(mag_x, mag_y, mag_z, pitch, roll)
            
            # Apply calibration parameters
            return (
                x_comp * self.scale_x - self.center_x,
                y_comp * self.scale_y - self.center_y
            )
        except:
            return 0.0, 0.0