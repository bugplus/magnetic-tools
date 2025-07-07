# main.py
# 说明: 程序的入口点，负责启动 CompassUI 界面。

import sys
from PyQt5.QtWidgets import QApplication
from compass_ui import CompassUI

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = CompassUI()
    window.resize(400, 200)
    window.show()
    sys.exit(app.exec_())