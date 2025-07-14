"""
File: config.py
Purpose: System configuration parameters
Description: Contains all key configuration parameters for the system
"""

class Config:
    # Serial port configuration
    PORT = 'COM3'
    BAUD_RATE = 115200
    TIMEOUT = 1
    
    # Data collection configuration
    UPDATE_INTERVAL = 100        # Update interval (ms)
    MAX_POINTS = 300             # Maximum data points
    CALIBRATION_DURATION = 30    # Calibration duration (seconds)
    
    # Real-time mode configuration
    REALTIME_UPDATE_INTERVAL = 100  # Real-time update interval (ms)
    MAX_REALTIME_POINTS = 100       # Maximum points for real-time display
    
    # Unified color scheme
    COLORS = {
        "raw": "#FF0000",       # Red - raw data
        "scaled": "#0000FF",    # Blue - scaled data
        "calibrated": "#00FF00" # Green - calibrated data
    }
    
    # Plot configuration
    PLOT_TITLES = {
        "raw_data": "Raw Data (Fig1)",
        "scaled_data": "Scaled Data (Fig2)",
        "calibrated_data": "Calibrated Data (Fig3)",
        "analysis_raw": "Raw Data",
        "analysis_calibrated": "Calibrated Data",
        "realtime_raw": "Realtime Raw Data",
        "realtime_calibrated": "Realtime Calibrated Data"
    }
    
    # Axis range configuration
    AXIS_LIMITS = {
        "raw": (-1000, 1000),     # Raw data axis range
        "calibrated": (-500, 500)  # Calibrated data axis range
    }

# Global configuration instance
config = Config()