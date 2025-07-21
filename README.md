
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

## 使用方法

1. 连接磁力计设备并确保其正常工作。
2. （可选）在 [config.py](file://f:\bugplus\python-prj\Magnetic-tools\magnetic-tools\application\config.py) 中修改默认串口号。
3. 运行 [main.py](file://f:\bugplus\python-prj\Magnetic-tools\magnetic-tools\application\main.py) 启动应用程序。
4. 在界面中选择串口号和波特率。
5. 点击 **"Start"** 开始采集数据。
6. 数据采集结束后，点击 **"Stop"** 停止采集并执行校准。

# 配置文件说明

配置文件 `config.py` 包含以下全局参数：

| 参数名称            | 说明                           | 默认值   |
|---------------------|--------------------------------|----------|
| `PORT`              | 默认串口号（例如 COM4 ）       | `COM4`   |
| `BAUD_RATE`         | 默认波特率（例如 115200 ）      | `115200` |
| `TIMEOUT`           | 串口超时时间（秒）             | `1`      |
| `UPDATE_INTERVAL`   | 数据更新间隔（毫秒）           | `50`     |
| `MAX_POINTS`        | 最大数据点数                   | `600`    |
| `CALIBRATION_DURATION` | 数据采集持续时间（秒）       | `30`     |
| `TOLERANCE_THRESHOLD` | 数据点的容忍阈值（单位：像素） | `1`      |

## 联系方式
如果您在使用过程中遇到任何问题，或者有任何建议和反馈，欢迎通过以下方式联系项目维护者：

- **电子邮件**：[bugplus@163.com](mailto:example@example.com)
- **GitHub Issues**：[项目GitHub页面](https://github.com/bugplus/CompassViewer/issues) 提交问题
- **社交媒体**：[@yourusername](https://twitter.com/bugplus)（如果适用）

## 许可
本项目采用 MIT License 进行许可，这是一项非常宽松的开源许可证。MIT License 允许您自由地使用、复制、修改、合并、发布、分发、再许可和/或销售软件的副本，并且允许他人这样做，但必须满足以下条件：

1. **署名**：您必须给予适当的署名，提供许可证链接，并指出是否进行了修改。您可以以任何合理的方式进行署名，但不得以任何方式暗示许可人认可您或您的使用。

2. **免责声明**：许可人不提供任何形式的明示或暗示的担保，包括但不限于适销性、特定用途适用性和非侵权性的担保。在任何情况下，即使被告知可能发生此类损害的可能性，许可人也不对任何直接的、间接的、偶然的、惩罚性的或后果性的损害负责。

有关 MIT License 的更多详细信息，请参阅 [LICENSE 文件](LICENSE) 或访问 [opensource.org](https://opensource.org/licenses/MIT) 获取更多信息。
