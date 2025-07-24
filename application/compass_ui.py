from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QPushButton, QLabel,
    QComboBox, QHBoxLayout, QMessageBox, QFileDialog, QDialog, QTextEdit
)
from PyQt5.QtCore import pyqtSignal
import serial.tools.list_ports  # 确保导入 serial 模块
from PyQt5.QtCore import pyqtSlot  # ✅ 确保导入


class ResultDialog(QDialog):
    def __init__(self, c_code, parent=None):
        super().__init__(parent)
        self.setWindowTitle("C Calibration Code")
        layout = QVBoxLayout(self)

        code_label = QLabel("Copy & paste into your firmware:")
        layout.addWidget(code_label)

        self.code_box = QTextEdit()  # ✅ 加上 self.
        self.code_box.setReadOnly(True)
        self.code_box.setPlainText(c_code)
        layout.addWidget(self.code_box)

        save_btn = QPushButton("Save .c")
        save_btn.clicked.connect(self.save_code)
        layout.addWidget(save_btn)

    def save_code(self):
        fname, _ = QFileDialog.getSaveFileName(self, "Save C", "calibration.c", "C Files (*.c)")
        if fname:
            try:
                with open(fname, "w", encoding="utf-8") as f:
                    f.write(self.code_box.toPlainText())  # ✅ 现在不会报错了
                QMessageBox.information(self, "Saved", f"Saved to {fname}")
            except Exception as e:
                QMessageBox.critical(self, "Save Failed", f"Failed to save file:\n{str(e)}")

class CompassMainWindow(QMainWindow):
    start_calibration_signal = pyqtSignal(str, int)
    view_result_signal = pyqtSignal()
    step0_done = pyqtSignal()
    step1_done = pyqtSignal()
    view3d_done = pyqtSignal()
    show_result_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Magnetic Compass Calibrator")
        self.resize(600, 200)
        self.central = QWidget()
        self.setCentralWidget(self.central)

        # 初始化 central_layout
        self.central_layout = QVBoxLayout(self.central)

        # Port & baud
        port_layout = QHBoxLayout()
        self.port_combo = QComboBox()
        self.refresh_ports()
        port_layout.addWidget(QLabel("Port:"))
        port_layout.addWidget(self.port_combo)
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "115200"])
        self.baud_combo.setCurrentText("115200")
        port_layout.addWidget(QLabel("Baud:"))
        port_layout.addWidget(self.baud_combo)
        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        port_layout.addWidget(self.refresh_btn)
        self.central_layout.addLayout(port_layout)

        # Buttons
        btn_layout = QHBoxLayout()
        self.start_btn = QPushButton("Start Calibration")
        self.start_btn.clicked.connect(self.on_start)
        btn_layout.addWidget(self.start_btn)
        self.view_btn = QPushButton("View Result")
        self.view_btn.clicked.connect(self.on_view_result)
        self.view_btn.setEnabled(False)
        btn_layout.addWidget(self.view_btn)
        # 新增按钮
        self.algo3d_btn = QPushButton("算法 3D 视图")
        btn_layout.addWidget(self.algo3d_btn)

        # 3D buttons
        self.step0_btn = QPushButton("3D-Step1 Level")
        self.step1_btn = QPushButton("3D-Step2 NoseUp")
        self.step2_btn = QPushButton("3D-Step3 SternUp")   # ←新增
        self.view3d_btn = QPushButton("3D-View")

        btn_layout.addWidget(self.step0_btn)
        btn_layout.addWidget(self.step1_btn)
        btn_layout.addWidget(self.step2_btn)   # ←新增
        btn_layout.addWidget(self.view3d_btn)

        self.step1_btn.setEnabled(False)
        self.step2_btn.setEnabled(False)       # ←新增
        self.view3d_btn.setEnabled(False)

        self.central_layout.addLayout(btn_layout)

        # Status
        self.status_label = QLabel("Select port and start")
        self.central_layout.addWidget(self.status_label)

    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.port_combo.addItem(p.device)
        if not ports:
            self.port_combo.addItem("No ports")

    def on_start(self):
        port = self.port_combo.currentText()
        baud = int(self.baud_combo.currentText())
        if "No ports" in port:
            QMessageBox.warning(self, "Error", "No port")
            return
        self.start_btn.setEnabled(False)
        self.port_combo.setEnabled(False)
        self.baud_combo.setEnabled(False)
        self.refresh_btn.setEnabled(False)
        self.view_btn.setEnabled(False)
        self.start_calibration_signal.emit(port, baud)

    def on_step0(self):
        port = self.port_combo.currentText()
        baud = int(self.baud_combo.currentText())
        if "No ports" in port:
            QMessageBox.warning(self, "Error", "No port")
            return
        self.step0_btn.setEnabled(False)
        self.start_calibration_signal.emit(port, baud)

    def on_step1(self):
        self.step1_btn.setEnabled(False)
        self.step1_done.emit()

    def on_view_result(self):
        self.view_result_signal.emit()

    def on_view3d(self):
        self.view3d_done.emit()

    def set_status(self, text):
        self.status_label.setText(text)


    def enable_view_btn(self, enable=True):
        self.view_btn.setEnabled(enable)

    def enable_step1_btn(self, enable=True):
        self.step1_btn.setEnabled(enable)

    def enable_view3d_btn(self, enable=True):
        self.view3d_btn.setEnabled(enable)

    @pyqtSlot(str)  # ✅ 加上这个装饰器
    def show_result_dialog(self, c_code):
        dlg = ResultDialog(c_code, self)
        dlg.exec_()