# Magnetometer Compass Viewer with Calibration GUI

A PyQt5-based GUI application for visualizing and calibrating magnetometer data from a serial port.

This software reads raw magnetic field data from a sensor, plots it in real-time, and performs automatic calibration to center and scale the data after a fixed duration.

## 🧩 Features

- ✅ Real-time plotting of X-Y magnetometer data
- ✅ Auto-calibration after configurable time (default: 50s)
- ✅ Three matplotlib subplots:
  - Raw Data
  - Scaled Data
  - Fully Calibrated Data
- ✅ Serial port & baud rate selection via PyQt5 UI
- ✅ Start/Stop control for data collection
- ✅ Easy configuration via config.py

## 📁 File Structure

compassProject/
├── config.py              # Configuration parameters
├── compass_core.py        # Core logic: serial handling & calibration
├── compass_plot.py        # Plotting logic using matplotlib
├── compass_ui.py          # PyQt5 UI interface
├── main.py                # Program entry point
└── README.md              # This file

## ⚙️ Requirements

Install dependencies via pip:

pip install pyserial numpy matplotlib PyQt5

## ▶️ How to Run

1. Connect your magnetometer device via USB/UART.
2. Clone the repository:

git clone https://github.com/yourname/your-repo-name.git
cd your-repo-name

3. Install dependencies (if not already):

pip install -r requirements.txt

💡 If you don't have a `requirements.txt`, create one easily:

pip freeze > requirements.txt

4. Run the app:

python main.py

5. In the GUI:
   - Select the correct COM port
   - Choose the baud rate
   - Click "Start" to begin collecting data
   - After calibration time, the system will auto-stop and display calibrated results

## 💡 Notes

- Ensure the selected serial port is available and not used by another program.
- The sensor should output data in the format like:
  mag_x=123, mag_y=456
- You can adjust behavior in config.py:
  - PORT: Default serial port
  - BAUD_RATE: Communication speed
  - CALIBRATION_DURATION: Time (seconds) before calibration
  - MAX_POINTS: Max number of data points stored

## 🤝 Contributing

Contributions are welcome! Whether you want to:
- Add new features (e.g., save plots, export data)
- Improve the UI
- Fix bugs or enhance documentation

Feel free to open an issue or submit a pull request!

## 📬 Contact

If you have any questions or need help integrating this into your project, feel free to reach out at:

📧 Email: your_email@example.com  
🐙 GitHub: https://github.com/yourname
