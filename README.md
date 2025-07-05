# Compass Viewer with PyQt5 and Matplotlib

这是一个基于 PyQt5 和 Matplotlib 的指南针数据可视化工具。它通过串口接收磁场数据，并实时绘制原始数据、校准后的数据以及最终的校准结果。

## 功能特点
- 实时接收串口数据，绘制磁场数据的 X-Y 图。
- 自动校准功能，将椭圆数据校准为圆形。
- 支持用户选择串口和波特率。
- 数据动态更新，支持长时间运行。

## 项目结构
- `main.py`：程序的入口点，负责启动应用程序。
- `compass_ui.py`：包含 `CompassUI` 类，负责用户界面的创建和控制。
- `compass_app.py`：包含 `CompassApp` 类，负责串口通信和数据处理。
- `utils.py`：（可选）用于存放通用工具函数。

## 环境依赖
- Python 3.8+
- PyQt5
- Matplotlib
- NumPy
- PySerial

## 安装依赖
在项目根目录下运行以下命令以安装所有依赖：
```bash
pip install pyqt5 matplotlib numpy pyserial