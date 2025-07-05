
### 第三次输出（完整文件）
```markdown
# Compass Viewer Project

## 项目简介
这是一个基于 Python 的指南针数据可视化项目，通过串口接收磁力计数据，并实时绘制数据图表。项目包含串口通信、数据校准和图形界面等功能。

## 功能特点
- **串口通信**：支持多种波特率，自动检测可用串口。
- **数据校准**：自动校准磁力计数据，支持缩放和偏移校正。
- **实时绘图**：实时绘制原始数据、校准数据和完全校准后的数据。
- **用户界面**：基于 PyQt5 的图形界面，支持启动/停止数据采集。

## 项目结构
CompassViewer/
├── compass_app.py       # 负责串口通信、数据处理和绘图
├── compass_ui.py        # 负责创建用户界面和控制程序启动/停止
├── config.py            # 配置文件，包含全局参数
├── main.py              # 程序入口点
└── README.md            # 项目说明文档
复制

## 安装与运行

### 环境依赖
确保你的系统已安装以下依赖：
- Python 3.7 或更高版本
- PyQt5
- Matplotlib
- NumPy
- PySerial

你可以通过以下命令安装所有依赖：
```bash
pip install PyQt5 matplotlib numpy pyserial


