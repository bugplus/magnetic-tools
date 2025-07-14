"""
File: compass_ui.py
Purpose: Compass calibration system GUI
Description: Implements the main interface with serial port selection and calibration control
"""
import sys
import time
import serial
import serial.tools.list_ports
import numpy as np
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                            QPushButton, QLabel, QComboBox, QMessageBox, 
                            QListWidget, QGroupBox, QSizePolicy)
from PyQt5.QtCore import QTimer
from config import config
from compass_app import CompassApp

class CompassUI(QWidget):
    """Magnetic Compass Calibration System GUI"""
    def __init__(self):
        super().__init__()
        self.app_instance = None
        self.is_calibrated = False
        self.realtime_active = False
        self.realtime_paused = False
        self.realtime_data = []
        self.init_ui()
    
    def init_ui(self):
        """Initialize user interface"""
        self.setWindowTitle('Magnetic Compass Calibration System')
        self.setGeometry(100, 100, 900, 700)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # Main layout
        main_layout = QVBoxLayout()
        main_layout.setSpacing(10)
        
        # Serial port selection area
        port_group = QGroupBox("Serial Port Settings")
        port_layout = QHBoxLayout()
        port_layout.setSpacing(10)
        
        self.port_label = QLabel("Port:")
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(150)
        self.refresh_ports()
        
        self.baud_label = QLabel("Baud Rate:")
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(['9600', '19200', '38400', '57600', '115200'])
        self.baud_combo.setCurrentText('115200')
        self.baud_combo.setMinimumWidth(100)
        
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.connect_serial)
        self.connect_button.setFixedWidth(100)
        
        self.refresh_button = QPushButton("Refresh")
        self.refresh_button.clicked.connect(self.refresh_ports)
        self.refresh_button.setFixedWidth(100)
        
        port_layout.addWidget(self.port_label)
        port_layout.addWidget(self.port_combo)
        port_layout.addWidget(self.baud_label)
        port_layout.addWidget(self.baud_combo)
        port_layout.addWidget(self.connect_button)
        port_layout.addWidget(self.refresh_button)
        port_group.setLayout(port_layout)
        
        # Data list
        self.data_list = QListWidget()
        self.data_list.setMinimumHeight(150)
        
        # Button area
        button_group = QGroupBox("Control Panel")
        button_layout = QHBoxLayout()
        
        self.calibration_button = QPushButton("Start Calibration")
        self.calibration_button.clicked.connect(self.start_calibration)
        self.calibration_button.setFixedWidth(150)
        
        self.results_button = QPushButton("Show Results")
        self.results_button.clicked.connect(self.show_algorithm_results)
        self.results_button.setFixedWidth(120)
        
        self.save_data_button = QPushButton("Save Data")
        self.save_data_button.setFixedWidth(100)
        
        self.load_data_button = QPushButton("Load Data")
        self.load_data_button.setFixedWidth(100)
        
        self.algorithm_button = QPushButton("Analyze Algorithm")
        self.algorithm_button.setFixedWidth(150)
        
        button_layout.addWidget(self.calibration_button)
        button_layout.addWidget(self.results_button)
        button_layout.addWidget(self.save_data_button)
        button_layout.addWidget(self.load_data_button)
        button_layout.addWidget(self.algorithm_button)
        button_group.setLayout(button_layout)
        
        # Real-time control area
        realtime_group = QGroupBox("Real-time Mode")
        realtime_layout = QHBoxLayout()
        
        self.run_button = QPushButton("Run Real-time")
        self.run_button.clicked.connect(self.start_realtime)
        self.run_button.setFixedWidth(120)
        
        self.pause_button = QPushButton("Pause")
        self.pause_button.clicked.connect(self.toggle_pause)
        self.pause_button.setFixedWidth(80)
        
        self.stop_button = QPushButton("Stop")
        self.stop_button.clicked.connect(self.stop_realtime)
        self.stop_button.setFixedWidth(80)
        
        self.realtime_status = QLabel("Real-time: Stopped")
        
        realtime_layout.addWidget(self.run_button)
        realtime_layout.addWidget(self.pause_button)
        realtime_layout.addWidget(self.stop_button)
        realtime_layout.addWidget(self.realtime_status)
        realtime_group.setLayout(realtime_layout)
        
        # Status area
        status_layout = QHBoxLayout()
        self.status_label = QLabel("Status: Disconnected")
        status_layout.addWidget(self.status_label)
        
        # Add to main layout
        main_layout.addWidget(port_group)
        main_layout.addWidget(self.data_list)
        main_layout.addWidget(button_group)
        main_layout.addWidget(realtime_group)
        main_layout.addLayout(status_layout)
        
        # Set main layout
        self.setLayout(main_layout)
    
    def refresh_ports(self):
        """Refresh available serial ports"""
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        available_ports = [port.device for port in ports]
        if available_ports:
            self.port_combo.addItems(available_ports)
        else:
            self.port_combo.addItem("No ports available")
    
    def connect_serial(self):
        """Connect to serial port"""
        selected_port = self.port_combo.currentText()
        if selected_port == "No ports available":
            QMessageBox.warning(self, "Port Error", "No available serial ports")
            return False
            
        baud_rate = int(self.baud_combo.currentText())
        
        # Clean up previous connection
        if self.app_instance:
            self.app_instance.disconnect_serial()
            self.app_instance = None
            time.sleep(0.5)  # Short delay
        
        # Create new instance
        self.app_instance = CompassApp(selected_port, baud_rate)
        
        # Attempt connection
        if self.app_instance.connect_serial():
            self.status_label.setText(f"Status: Connected to {selected_port}@{baud_rate} baud")
            return True
        else:
            self.status_label.setText("Status: Connection failed")
            return False
    
    def start_calibration(self):
        """Start calibration process"""
        if not self.app_instance:
            QMessageBox.critical(self, "Error", "Compass application not initialized")
            return False
            
        # Connect serial if needed
        if not hasattr(self.app_instance, 'ser') or not self.app_instance.ser or not self.app_instance.ser.is_open:
            if not self.connect_serial():
                QMessageBox.critical(self, "Connection Error", "Failed to connect to serial port")
                return False
        
        # Initialize plots
        try:
            self.app_instance.init_plots()
        except Exception as e:
            QMessageBox.critical(self, "Plot Error", f"Failed to initialize plots: {e}")
            return False
        
        # Start data collection
        if not self.app_instance.start_data_collection():
            QMessageBox.critical(self, "Data Error", "Failed to start data collection")
            return False
        
        # Clear data list
        self.data_list.clear()
        
        # Setup timers
        self.calibration_time_remaining = config.CALIBRATION_DURATION
        self.calibration_timer = QTimer(self)
        self.calibration_timer.timeout.connect(self.update_calibration_timer)
        self.calibration_timer.start(1000)  # Update every second
        
        self.data_collection_timer = QTimer(self)
        self.data_collection_timer.timeout.connect(self.collect_data_point)
        self.data_collection_timer.start(config.UPDATE_INTERVAL)
        
        # Update UI
        self.calibration_button.setEnabled(False)
        self.status_label.setText(f"Status: Calibrating... {self.calibration_time_remaining} seconds remaining")
        
        return True
    
    def collect_data_point(self):
        """Collect a data point and update UI"""
        if not self.app_instance or not self.app_instance.data_collection_active:
            return
            
        try:
            # Collect data from queue
            if self.app_instance.collect_data():
                # Update list with latest point
                point_idx = len(self.app_instance.xs)
                if point_idx > 0:
                    x_val = self.app_instance.xs[-1]
                    y_val = self.app_instance.ys[-1]
                    self.data_list.addItem(f"Point {point_idx}: X={x_val:.2f}, Y={y_val:.2f}")
                    self.data_list.scrollToBottom()
                    
                # Process events to keep UI responsive
                QApplication.processEvents()
        except Exception as e:
            print(f"Data collection error in UI: {e}")
    
    def update_calibration_timer(self):
        """Update calibration timer and handle completion"""
        if self.calibration_time_remaining > 0:
            self.calibration_time_remaining -= 1
            self.status_label.setText(f"Status: Calibrating... {self.calibration_time_remaining} seconds remaining")
        else:
            # Stop timers
            self.calibration_timer.stop()
            self.data_collection_timer.stop()
            
            # Stop data collection
            self.app_instance.stop_data_collection()
            
            # Attempt calibration
            calibration_success = False
            if len(self.app_instance.raw_data) >= 10:
                calibration_success = self.app_instance.calibrate_data()
            
            # Reset calibration button
            self.calibration_button.setEnabled(True)
            
            if calibration_success:
                self.is_calibrated = True
                self.status_label.setText("Status: Calibration complete")
                QMessageBox.information(self, "Success", "Calibration completed successfully")
            else:
                self.status_label.setText("Status: Calibration failed")
                QMessageBox.critical(self, "Error", "Calibration failed due to insufficient data or processing error")
    
    def show_algorithm_results(self):
        """Display calibrated data vs raw data"""
        if not self.is_calibrated or not self.app_instance or not self.app_instance.raw_data:
            QMessageBox.warning(self, "Data Error", "Please complete calibration first")
            return
            
        # Get calibration parameters
        scale_x = self.app_instance.scale_x
        scale_y = self.app_instance.scale_y
        center_x = self.app_instance.center_x
        center_y = self.app_instance.center_y
        raw_data = self.app_instance.raw_data
        
        # Process each data point through calibration
        calibrated_points = []
        for point in raw_data:
            if len(point) != 5:  # Skip invalid entries
                continue
            mag_x, mag_y, mag_z, pitch, roll = point
            
            # Apply calibration
            cal_x, cal_y = self.app_instance.realtime_calibration(mag_x, mag_y, mag_z, pitch, roll)
            calibrated_points.append((cal_x, cal_y))
        
        # Prepare plots
        plt.figure(figsize=(14, 6))
        plt.suptitle("Calibration Results Comparison")
        
        # Raw data plot
        plt.subplot(1, 2, 1)
        xs = [p[0] for p in raw_data]
        ys = [p[1] for p in raw_data]
        plt.scatter(xs, ys, c=config.COLORS["raw"], s=20, alpha=0.6)
        plt.title("Raw Magnetic Data")
        plt.xlabel("Mag X")
        plt.ylabel("Mag Y")
        plt.grid(True)
        
        # Calibrated data plot
        plt.subplot(1, 2, 2)
        if calibrated_points:
            xs_cal, ys_cal = zip(*calibrated_points)
            plt.scatter(xs_cal, ys_cal, c=config.COLORS["calibrated"], s=20, alpha=0.6)
            plt.scatter([0], [0], c='black', marker='x', s=80, label="Center")
            plt.title("Calibrated Data")
            plt.xlabel("Calibrated X")
            plt.ylabel("Calibrated Y")
            plt.grid(True)
            plt.axhline(0, color='black', linewidth=0.5)
            plt.axvline(0, color='black', linewidth=0.5)
            plt.axis('equal')
            plt.legend()
        
        plt.tight_layout()
        plt.show()
    
    def start_realtime(self):
        """Start real-time calibration display"""
        if not self.is_calibrated:
            QMessageBox.warning(self, "Calibration Required", "Please complete calibration first")
            return
            
        # Skip if already running
        if self.realtime_active:
            return
            
        # Verify connection
        if not self.app_instance or not self.app_instance.ser or not self.app_instance.ser.is_open:
            if not self.connect_serial():
                return
                
        # Setup real-time plot
        self.realtime_fig, self.realtime_ax = plt.subplots(figsize=(8, 8))
        self.realtime_ax.set_title("Real-time Calibrated Data")
        self.realtime_ax.set_xlabel("Calibrated X")
        self.realtime_ax.set_ylabel("Calibrated Y")
        self.realtime_ax.grid(True)
        self.realtime_ax.axhline(0, color='black', linewidth=0.8)
        self.realtime_ax.axvline(0, color='black', linewidth=0.8)
        self.realtime_scatter = self.realtime_ax.scatter([], [], c=config.COLORS["calibrated"], s=30, alpha=0.7)
        self.realtime_ax.set_xlim(-200, 200)
        self.realtime_ax.set_ylim(-200, 200)
        plt.ion()
        plt.show(block=False)
        
        # Start real-time data collection
        self.realtime_data = []
        self.realtime_active = True
        self.realtime_paused = False
        self.realtime_status.setText("Real-time: Running")
        
        # Setup timer
        self.realtime_timer = QTimer(self)
        self.realtime_timer.timeout.connect(self.update_realtime_plot)
        self.realtime_timer.start(config.REALTIME_UPDATE_INTERVAL)
    
    def update_realtime_plot(self):
        """Update the real-time plot"""
        if not self.realtime_active or self.realtime_paused:
            return
            
        if not self.app_instance or not self.app_instance.ser or not self.app_instance.ser.is_open:
            self.stop_realtime()
            return
            
        # Collect all available data points
        points_collected = 0
        while self.app_instance.data_queue.qsize() > 0 and points_collected < 20:
            # Process the data line
            if self.app_instance.collect_data():
                points_collected += 1
                # Get last point
                if self.app_instance.xs:
                    x = self.app_instance.xs[-1]
                    y = self.app_instance.ys[-1]
                    
                    # Apply calibration parameters
                    x_scaled = x * self.app_instance.scale_x
                    y_scaled = y * self.app_instance.scale_y
                    x_cal = x_scaled - self.app_instance.center_x
                    y_cal = y_scaled - self.app_instance.center_y
                    self.realtime_data.append((x_cal, y_cal))
        
        # Keep within point limit
        if len(self.realtime_data) > config.MAX_REALTIME_POINTS:
            self.realtime_data = self.realtime_data[-config.MAX_REALTIME_POINTS:]
            
        # Update plot if we have data
        if self.realtime_data:
            xs, ys = zip(*self.realtime_data)
            self.realtime_scatter.set_offsets(np.column_stack([xs, ys]))
            self.realtime_fig.canvas.draw_idle()
            self.realtime_fig.canvas.flush_events()
    
    def toggle_pause(self):
        """Pause/resume real-time display"""
        if not self.realtime_active:
            return
            
        self.realtime_paused = not self.realtime_paused
        if self.realtime_paused:
            self.realtime_status.setText("Real-time: Paused")
            self.pause_button.setText("Resume")
        else:
            self.realtime_status.setText("Real-time: Running")
            self.pause_button.setText("Pause")
    
    def stop_realtime(self):
        """Stop real-time mode"""
        if not self.realtime_active:
            return
            
        if hasattr(self, 'realtime_timer') and self.realtime_timer.isActive():
            self.realtime_timer.stop()
            
        self.realtime_active = False
        self.realtime_paused = False
        self.realtime_status.setText("Real-time: Stopped")
        self.pause_button.setText("Pause")
        
        if hasattr(self, 'realtime_fig'):
            plt.close(self.realtime_fig)