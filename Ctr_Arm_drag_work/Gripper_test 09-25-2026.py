import os
import sys
from datetime import datetime
from typing import List
import serial.tools.list_ports

from point_io import PointAction, read_point_program_csv, save_point_program_csv
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from robot_worker import RobotWorker
from trajectory_io import save_trajectory_csv
from PyQt5.QtWidgets import (
    QApplication,
    QFileDialog,
    QComboBox,
    QDoubleSpinBox,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

# IN PROGRESS - NOT READY
class TestingGrippers():
    def __init__(self):
        self.worker = RobotWorker
        self.worker.request_connect(self,"COM3")
        self.worker.request_capture_point(self,3,4,5,2)
        self.worker._enable_gripper_once()
        self.worker._ensure_gripper_torque_pos_mode_once(self)
        self.worker._send_gripper_control(self, 3,2)
        self.worker.request_control_gripper(self, 2, 3, .01)

    def scan_serial_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        if not ports:
            self.port_combo.addItem("未检测到串口", None)
            self.log("未检测到可用串口")
            return

        for port in ports:
            self.port_combo.addItem(f"{port.device} - {port.description}", port.device)
        self.log(f"扫描到 {len(ports)} 个串口")
    
def main():
    app = QApplication(sys.argv)
    window = TestingGrippers()
    window.show()
    sys.exit(app.exec_())
    
    
    if __name__ == "__main__":
        main()