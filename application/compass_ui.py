from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QPushButton, QLabel,
    QComboBox, QHBoxLayout, QMessageBox, QFileDialog, QDialog, QTextEdit
)
from PyQt5.QtCore import pyqtSignal
import serial.tools.list_ports

class ResultDialog(QDialog):
    def __init__(self, c_code, parent=None):
        super().__init__(parent)
        self.setWindowTitle("C语言校准代码")
        layout = QVBoxLayout(self)
        code_label = QLabel("自动生成C语言校准代码：")
        layout.addWidget(code_label)
        code_box = QTextEdit()
        code_box.setReadOnly(True)
        code_box.setPlainText(c_code)
        layout.addWidget(code_box)
        save_btn = QPushButton("保存C代码")
        save_btn.clicked.connect(lambda: self.save_code(c_code))
        layout.addWidget(save_btn)

    def save_code(self, c_code):
        fname, _ = QFileDialog.getSaveFileName(self, "保存C代码", "calibration.c", "C Files (*.c)")
        if fname:
            with open(fname, "w", encoding="utf-8") as f:
                f.write(c_code)
            QMessageBox.information(self, "保存成功", f"已保存到 {fname}")

class CompassMainWindow(QMainWindow):
    start_calibration_signal = pyqtSignal(str, int)
    view_result_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setWindowTitle("磁罗盘校准工具")
        self.resize(600, 200)
        self.central = QWidget()
        self.setCentralWidget(self.central)
        main_layout = QVBoxLayout(self.central)

        # 串口选择区
        port_layout = QHBoxLayout()
        self.port_combo = QComboBox()
        self.refresh_ports()
        port_layout.addWidget(QLabel("串口号:"))
        port_layout.addWidget(self.port_combo)
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "115200"])
        self.baud_combo.setCurrentText("115200")
        port_layout.addWidget(QLabel("波特率:"))
        port_layout.addWidget(self.baud_combo)
        self.refresh_btn = QPushButton("刷新串口")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        port_layout.addWidget(self.refresh_btn)
        main_layout.addLayout(port_layout)

        # 控制按钮
        btn_layout = QHBoxLayout()
        self.start_btn = QPushButton("Start Calibration")
        self.start_btn.clicked.connect(self.on_start)
        btn_layout.addWidget(self.start_btn)
        self.view_btn = QPushButton("View Result")
        self.view_btn.clicked.connect(self.on_view_result)
        self.view_btn.setEnabled(False)
        btn_layout.addWidget(self.view_btn)
        main_layout.addLayout(btn_layout)

        # 状态栏
        self.status_label = QLabel("请连接设备并选择串口")
        main_layout.addWidget(self.status_label)

    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.port_combo.addItem(p.device)
        if not ports:
            self.port_combo.addItem("无可用串口")

    def on_start(self):
        port = self.port_combo.currentText()
        baud = int(self.baud_combo.currentText())
        if "无可用串口" in port:
            QMessageBox.warning(self, "错误", "未检测到串口")
            return
        self.start_btn.setEnabled(False)
        self.port_combo.setEnabled(False)
        self.baud_combo.setEnabled(False)
        self.refresh_btn.setEnabled(False)
        self.status_label.setText("校准中...")
        self.view_btn.setEnabled(False)
        self.start_calibration_signal.emit(port, baud)

    def on_view_result(self):
        self.view_result_signal.emit()

    def set_status(self, text):
        self.status_label.setText(text)

    def clear_all_canvases(self):
        pass  # 兼容旧接口，无实际操作

    def update_all_canvases(self, raw_data, shifted, calibrated):
        pass  # 兼容旧接口，无实际操作

    def show_result_dialog(self, c_code):
        dlg = ResultDialog(c_code, self)
        dlg.exec_()

    def enable_view_btn(self, enable=True):
        self.view_btn.setEnabled(enable)