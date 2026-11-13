import os
import sys
import csv
import cv2
import time
from datetime import datetime
from typing import List
import torch

import serial.tools.list_ports
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QBrush, QColor, QFont, QPixmap
from PyQt5.QtWidgets import (
    QAbstractScrollArea,
    QApplication,
    QFileDialog,
    QComboBox,
    QDoubleSpinBox,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QGridLayout,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from point_io import PointAction, read_point_program_csv, save_point_program_csv
from robot_worker import RobotWorker
from trajectory_io import save_trajectory_csv
from camera import CameraWork, check_camera_quietly

ARM_DOF = 6
TOTAL_DOF = 7

ZH_TO_EN = {
    "拖动示教控制器 - 夹爪力位混合版": "Drag Teaching Controller - Gripper Force-Position Mode",
    "机械臂拖动示教系统：J1~J6 + 夹爪G反馈/力位混合控制": "Robot Arm Program: J1-J6 + Gripper G Feedback / F-Pos Control",
    "连续轨迹示教": "Continuous Trajectory",
    "点位示教": "Point Teaching",
    "串口连接": "Serial Connection",
    "串口:": "Port:",
    "刷新": "Refresh",
    "连接": "Connect",
    "连接中...": "Connecting...",
    "断开": "Disconnect",
    "断开中...": "Disconnecting...",
    "关闭中...": "Closing...",
    "设置0点": "Set 0",
    "设置0点中...": "Setting 0...",
    "回到初始位置": "Return to 0",
    "返回初始位置中...": "Returning 0...",
    "使能": "Enable",
    "使能中...": "Enabling...",
    "失能": "Disable",
    "失能中...": "Disabling...",
    "未连接": "Disconnected",
    "连续轨迹记录：自动记录 J1~J6 + 夹爪G反馈": "Continuous Recording: Automatically Record J1-J6 + Gripper G Feedback",
    "文件名:": "File name:",
    "保存路径:": "Save path:",
    "浏览": "Browse",
    "记录频率:": "Record rate:",
    "重力补偿:": "Gravity compensation:",
    "开始连续拖动示教": "Start Continuous Drag Teaching",
    "停止连续记录": "Stop Continuous Recording",
    "状态: 就绪": "Status: Ready",
    "连续轨迹播放：读取CSV并按时间戳慢放/原速播放，CSV中的G作为第7轴播放": "Continuous Playback: Read CSV and play by timestamp; G is played as the gripper target",
    "轨迹文件:": "Trajectory file:",
    "选择": "Select",
    "播放速度:": "Playback speed:",
    "播放连续轨迹": "Play Continuous Trajectory",
    "停止连续播放": "Stop Continuous Playback",
    "状态: 无轨迹": "Status: No trajectory",
    "状态: 已选择轨迹": "Status: Trajectory selected",
    "说明：夹爪位置G从 get_current_joint_angles() 读取；夹爪动作不走 set_joint_angles，而是调用 MotorControl.control_pos_force(Motor7, G, 夹爪速度, 夹爪力度)。示教时夹爪力位混合命令参数为0。": "Note: Gripper position G is read from feedback. Gripper motion does not use set_joint_angles; it uses RobotCtrl.control_pos_force(gripper, G, gripper speed, gripper force). During teaching, gripper force-position command parameters are 0.",
    "点位示教：开启重力补偿，把机械臂和夹爪拖到位置后点击“记录当前点位”": "Point Teaching: Enable gravity compensation, drag the arm and gripper to a pose, then click Capture Current Point",
    "默认最大轴速度:": "Default max joint speed:",
    "默认间隔:": "Default pause:",
    "默认夹爪速度:": "Default gripper speed:",
    "夹爪力度:": "Gripper force:",
    "开始点位示教/重力补偿": "Start Point Teaching / Gravity Compensation",
    "停止点位示教/保持当前位置": "Stop Point Teaching / Hold Current Pose",
    "记录当前点位": "Capture Current Point",
    "播放点位程序": "Play Point Program",
    "停止点位播放": "Stop Point Playback",
    "播放: 未开始": "Playback: Not started",
    "点位程序编辑：J1~J6、夹爪G、六轴最大速度、间隔、夹爪速度、夹爪力度均可直接修改": "Point Program Editor: J1-J6, gripper G, arm max speed, pause, gripper speed, and gripper force are editable",
    "导入点位CSV": "Import Point CSV",
    "保存点位CSV": "Save Point CSV",
    "删除选中": "Delete Selected",
    "上移": "Move Up",
    "下移": "Move Down",
    "清空": "Clear",
    "序号": "No.",
    "G夹爪": "G\nGripper",
    "六轴最大速度(rad/s)": "Arm Max Speed\n(rad/s)",
    "间隔(s)": "Pause\n(s)",
    "夹爪速度": "Gripper\nSpeed",
    "夹爪力度": "Gripper\nForce",
    "备注": "Note",
    "说明：同步速度只针对J1~J6。六轴最大速度是机械臂关节速度上限，程序按J1~J6角度差自动分配速度；夹爪G不参与同步，播放时通过control_pos_force按表格里的夹爪速度和力度单独执行，默认速度5。蓝色行表示当前动作。": "Note: Synchronization applies only to J1-J6. Arm max speed is the speed limit for the arm joints; the program automatically distributes speeds according to J1-J6 angle deltas. Gripper G is not synchronized with the arm; it is executed separately through control_pos_force using the gripper speed and force in the table. The blue row indicates the current action.",
    "实时关节角度": "Real-Time Joint Angles",
    "日志": "Log",
    "错误": "Error",
    "提示": "Notice",
    "确认": "Confirm",
    "确定": "OK",
    "请选择有效的串口": "Please select a valid serial port",
    "请先停止当前示教或播放任务": "Please stop the current teaching or playback task first",
    "请先连接机械臂": "Please connect the robot arm first",
    "请先使能机械臂": "Please enable the robot arm first",
    "请先停止其他示教或播放任务": "Please stop other teaching or playback tasks first",
    "请先选择轨迹文件": "Please select a trajectory file first",
    "播放或连续记录中不能记录点位": "Cannot capture a point during playback or continuous recording",
    "确定清空所有点位吗？": "Clear all points?",
    "没有点位可保存": "No points to save",
    "点位程序为空": "Point program is empty",
    "选择保存目录": "Select Save Directory",
    "选择连续轨迹CSV": "Select Continuous Trajectory CSV",
    "保存点位程序": "Save Point Program",
    "导入点位程序": "Import Point Program",
    "机械臂错误": "Robot Error",
    "取消": "Cancel",
    "确定将当前位置设置为0点吗？\n设置后不可逆。": "Set the current pose as the zero point?\nThis cannot be undone.",
    "0点设置完成": "Zero point set",
    "状态: 准备中...": "Status: Preparing...",
    "状态: 正在停止...": "Status: Stopping...",
    "状态: 切换到MIT重力补偿模式...": "Status: Switching to MIT gravity compensation mode...",
    "状态: 切换到位置速度模式...": "Status: Switching to position-velocity mode...",
    "状态: 记录中...": "Status: Recording...",
    "状态: 播放中...": "Status: Playing...",
    "状态: 已停止": "Status: Stopped",
    "状态: 播放完成": "Status: Playback complete",
    "状态: 点位示教中，可拖动机械臂和夹爪并记录点位": "Status: Point teaching. Drag the arm and gripper, then capture points",
    "状态: 点位示教已停止": "Status: Point teaching stopped",
    "状态: 点位播放中...": "Status: Point playback running...",
    "状态: 点位播放完成": "Status: Point playback complete",
    "相机传感器": "Camera Sensor",
    "AI 物体与颜色检测器": "AI Object and Color Detector",
    "关闭摄像头": "Camera Off",
    "打开摄像头": "Camera On",
    "开启摄像头检测": "Camera detection started",
}

EN_TO_ZH = {v: k for k, v in ZH_TO_EN.items()}

POINT_HEADERS_ZH = [
    "序号", "J1", "J2", "J3", "J4", "J5", "J6", "G夹爪",
    "六轴最大速度(rad/s)", "间隔(s)", "夹爪速度", "夹爪力度", "备注"
]

POINT_HEADERS_EN = [ZH_TO_EN.get(x, x) for x in POINT_HEADERS_ZH]

class DragTeachWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.is_connected = False
        self.is_path_recording = False
        self.is_path_playing = False
        self.is_point_teaching = False
        self.is_point_playing = False
        self.current_point_highlight = -1
        self.lang = "zh"
        self._closing_after_home = False
        self._allow_close = False
        self.connected_port = ""
        self.conn_status_raw = "未连接"
        self.path_record_status_raw = "状态: 就绪"
        self.path_play_status_raw = "状态: 无轨迹"
        self.point_teach_status_raw = "状态: 就绪"
        self.point_play_status_raw = "播放: 未开始"
        self.is_setting_zero = False
        self.arm_enabled = False
        self.is_toggling_enable = False
        self.enable_target_enabled = False
        self.is_returning_home = False

        self.worker = RobotWorker()
        self.worker.log_msg.connect(self.log)
        self.worker.error_msg.connect(self.on_worker_error)
        self.worker.connection_changed.connect(self.on_connection_changed)
        self.worker.angles_updated.connect(self.on_angles_updated)
        self.worker.enable_state_changed.connect(self.on_enable_state_changed)
        self.worker.enable_toggle_finished.connect(self.on_enable_toggle_finished)
        self.worker.zero_set_finished.connect(self.on_zero_set_finished)
        self.worker.home_return_finished.connect(self.on_home_return_finished)

        self.worker.record_status.connect(self.path_record_status_update)
        self.worker.record_finished.connect(self.on_path_record_finished)
        self.worker.traj_play_status.connect(self.path_play_status_update)
        self.worker.traj_play_finished.connect(self.on_path_play_finished)

        self.worker.point_teach_status.connect(self.point_teach_status_update)
        self.worker.point_captured.connect(self.on_point_captured)
        self.worker.point_play_status.connect(self.point_play_status_update)
        self.worker.point_action_started.connect(self.highlight_point_row)
        self.worker.point_play_finished.connect(self.on_point_play_finished)
        self.worker.finished.connect(self.on_worker_thread_finished)
        self.worker.start()

        # ==================== Camera state ====================

        # One worker and one preview per active camera.
        self.camera_workers = {}
        self.camera_preview_labels = {}
        self.latest_camera_images = {}

        # Keep this for compatibility with existing single-camera code.
        self.latest_camera_image = None

        # Point-teaching camera recording.
        self.point_recording_dir = None
        self.point_recording_csv = None
        self.point_recording_file = None
        self.point_recording_writer = None
        self.point_capture_count = 0

        # Continuous trajectory camera recording.
        self.CT_session_dir = None
        self.CT_camera_images_dir = None

        self.CT_camera_timer = QTimer(self)
        self.CT_camera_timer.timeout.connect(self.capture_CT_camera_frame)
        self.CT_camera_interval = 250  # 4 FPS per active camera
        
        self.init_ui()
        self.scan_serial_ports() 

    # ==================== UI 构建 ====================
    def init_ui(self):
        self.setWindowTitle("拖动示教控制器 - 夹爪力位混合版")
        self.resize(1180, 820)
        self.setMinimumSize(900, 650)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout(central_widget)

        self.title_label = QLabel("机械臂拖动示教系统：J1~J6 + 夹爪G反馈/力位混合控制")
        title = self.title_label
        title.setFont(QFont("Arial", 15, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title)

        main_layout.addWidget(self.build_connection_group())
        
        self.tabs = QTabWidget()
        self.tabs.addTab(self.build_path_tab(), "连续轨迹示教")
        self.tabs.addTab(self.build_point_tab(), "点位示教")
        self.tabs.addTab(self.build_camera_tab(), "相机传感器") 
        main_layout.addWidget(self.tabs, 1)

        main_layout.addWidget(self.build_angle_group())
        main_layout.addWidget(self.build_log_group())

    def build_connection_group(self):
        group = QGroupBox("串口连接")
        layout = QHBoxLayout(group)

        layout.addWidget(QLabel("串口:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(260)
        layout.addWidget(self.port_combo)

        self.refresh_btn = QPushButton("刷新")
        self.refresh_btn.clicked.connect(self.scan_serial_ports)
        self.refresh_btn.setMaximumWidth(70)
        layout.addWidget(self.refresh_btn)

        self.connect_btn = QPushButton("连接")
        self.connect_btn.clicked.connect(self.toggle_connection)
        self.connect_btn.setMinimumWidth(90)
        self.connect_btn.setStyleSheet("background-color: #4CAF50; color: white;")
        layout.addWidget(self.connect_btn)

        self.language_btn = QPushButton("English")
        self.language_btn.clicked.connect(self.toggle_language)
        self.language_btn.setMinimumWidth(90)
        layout.addWidget(self.language_btn)

        self.set_zero_btn = QPushButton("设置0点")
        self.set_zero_btn.clicked.connect(self.confirm_set_zero)
        self.set_zero_btn.setMinimumWidth(90)
        self.set_zero_btn.setEnabled(False)
        layout.addWidget(self.set_zero_btn)

        # Return to the saved connection pose
        self.home_btn = QPushButton("回到初始位置")
        self.home_btn.clicked.connect(self.return_to_home)
        self.home_btn.setMinimumWidth(110)
        self.home_btn.setEnabled(False)
        layout.addWidget(self.home_btn)

        self.enable_btn = QPushButton("使能")
        self.enable_btn.clicked.connect(self.toggle_arm_enable)
        self.enable_btn.setMinimumWidth(90)
        self.enable_btn.setEnabled(False)
        layout.addWidget(self.enable_btn)

        layout.addStretch()
        self.conn_status = QLabel("未连接")
        self.conn_status.setStyleSheet("color: gray;")
        layout.addWidget(self.conn_status)
        return group

    def build_path_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        record_group = QGroupBox("连续轨迹记录：自动记录 J1~J6 + 夹爪G反馈")
        record_layout = QVBoxLayout(record_group)

        name_layout = QHBoxLayout()
        name_layout.addWidget(QLabel("文件名:"))
        self.path_filename_edit = QLineEdit()
        self.path_filename_edit.setText(f"trajectory_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
        name_layout.addWidget(self.path_filename_edit)
        record_layout.addLayout(name_layout)

        save_layout = QHBoxLayout()
        save_layout.addWidget(QLabel("保存路径:"))
        self.path_save_dir_edit = QLineEdit()
        self.path_save_dir_edit.setText(self.get_default_path())
        save_layout.addWidget(self.path_save_dir_edit)
        self.path_browse_btn = QPushButton("浏览")
        self.path_browse_btn.clicked.connect(self.browse_path_save_dir)
        save_layout.addWidget(self.path_browse_btn)
        record_layout.addLayout(save_layout)

        param_layout = QHBoxLayout()
        param_layout.addWidget(QLabel("记录频率:"))
        self.path_record_hz_spin = QSpinBox()
        self.path_record_hz_spin.setRange(10, 200)
        self.path_record_hz_spin.setValue(100)
        self.path_record_hz_spin.setSuffix(" Hz")
        self.path_record_hz_spin.setMaximumWidth(90)
        param_layout.addWidget(self.path_record_hz_spin)

        param_layout.addWidget(QLabel("重力补偿:"))
        self.path_gravity_hz_spin = QSpinBox()
        self.path_gravity_hz_spin.setRange(20, 500)
        self.path_gravity_hz_spin.setValue(100)
        self.path_gravity_hz_spin.setSuffix(" Hz")
        self.path_gravity_hz_spin.setMaximumWidth(90)
        param_layout.addWidget(self.path_gravity_hz_spin)
        param_layout.addStretch()
        record_layout.addLayout(param_layout)

        self.path_record_btn = QPushButton("开始连续拖动示教")
        self.path_record_btn.setMinimumHeight(42)
        self.path_record_btn.clicked.connect(self.toggle_path_record)
        record_layout.addWidget(self.path_record_btn)

        self.path_record_status = QLabel("状态: 就绪")
        record_layout.addWidget(self.path_record_status)
        layout.addWidget(record_group)

        play_group = QGroupBox("连续轨迹播放：读取CSV并按时间戳慢放/原速播放，CSV中的G作为第7轴播放")
        play_layout = QVBoxLayout(play_group)

        file_layout = QHBoxLayout()
        file_layout.addWidget(QLabel("轨迹文件:"))
        self.path_play_file_edit = QLineEdit()
        self.path_play_file_edit.setReadOnly(True)
        file_layout.addWidget(self.path_play_file_edit)
        self.path_select_file_btn = QPushButton("选择")
        self.path_select_file_btn.clicked.connect(self.select_path_file)
        file_layout.addWidget(self.path_select_file_btn)
        play_layout.addLayout(file_layout)

        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("播放速度:"))
        self.path_play_speed_spin = QDoubleSpinBox()
        self.path_play_speed_spin.setRange(0.1, 1.0)
        self.path_play_speed_spin.setSingleStep(0.1)
        self.path_play_speed_spin.setDecimals(1)
        self.path_play_speed_spin.setValue(1.0)
        self.path_play_speed_spin.setSuffix(" x")
        self.path_play_speed_spin.setMaximumWidth(90)
        speed_layout.addWidget(self.path_play_speed_spin)
        speed_layout.addStretch()
        play_layout.addLayout(speed_layout)

        self.path_play_btn = QPushButton("播放连续轨迹")
        self.path_play_btn.setMinimumHeight(40)
        self.path_play_btn.clicked.connect(self.toggle_path_play)
        self.path_play_btn.setEnabled(False)
        play_layout.addWidget(self.path_play_btn)

        self.path_play_status = QLabel("状态: 无轨迹")
        play_layout.addWidget(self.path_play_status)
        layout.addWidget(play_group)

        hint = QLabel(
            "说明：夹爪位置G从 get_current_joint_angles() 读取；夹爪动作不走 set_joint_angles，而是调用 MotorControl.control_pos_force(Motor7, G, 夹爪速度, 夹爪力度)。示教时夹爪力位混合命令参数为0。"
        )
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #555;")
        layout.addWidget(hint)
        layout.addStretch()
        return tab

    def build_point_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        teach_group = QGroupBox("点位示教：开启重力补偿，把机械臂和夹爪拖到位置后点击“记录当前点位”")
        teach_layout = QVBoxLayout(teach_group)

        param_layout = QHBoxLayout()
        param_layout.addWidget(QLabel("重力补偿:"))
        self.point_gravity_hz_spin = QSpinBox()
        self.point_gravity_hz_spin.setRange(20, 500)
        self.point_gravity_hz_spin.setValue(100)
        self.point_gravity_hz_spin.setSuffix(" Hz")
        self.point_gravity_hz_spin.setMaximumWidth(90)
        param_layout.addWidget(self.point_gravity_hz_spin)

        param_layout.addSpacing(15)
        param_layout.addWidget(QLabel("默认最大轴速度:"))
        self.point_default_speed_spin = QDoubleSpinBox()
        self.point_default_speed_spin.setRange(0.01, 3.0)
        self.point_default_speed_spin.setSingleStep(0.05)
        self.point_default_speed_spin.setDecimals(2)
        self.point_default_speed_spin.setValue(1.00)
        self.point_default_speed_spin.setSuffix(" rad/s")
        self.point_default_speed_spin.setMaximumWidth(130)
        param_layout.addWidget(self.point_default_speed_spin)

        param_layout.addWidget(QLabel("默认间隔:"))
        self.point_default_pause_spin = QDoubleSpinBox()
        self.point_default_pause_spin.setRange(0.0, 999.0)
        self.point_default_pause_spin.setSingleStep(0.1)
        self.point_default_pause_spin.setDecimals(2)
        self.point_default_pause_spin.setValue(0.50)
        self.point_default_pause_spin.setSuffix(" s")
        self.point_default_pause_spin.setMaximumWidth(100)
        param_layout.addWidget(self.point_default_pause_spin)

        param_layout.addWidget(QLabel("默认夹爪速度:"))
        self.point_default_gripper_speed_spin = QDoubleSpinBox()
        self.point_default_gripper_speed_spin.setRange(0.01, 20.0)
        self.point_default_gripper_speed_spin.setSingleStep(0.5)
        self.point_default_gripper_speed_spin.setDecimals(2)
        self.point_default_gripper_speed_spin.setValue(5.0)
        self.point_default_gripper_speed_spin.setSuffix(" rad/s")
        self.point_default_gripper_speed_spin.setMaximumWidth(120)
        param_layout.addWidget(self.point_default_gripper_speed_spin)

        param_layout.addWidget(QLabel("夹爪力度:"))
        self.point_default_force_spin = QDoubleSpinBox()
        self.point_default_force_spin.setRange(0.0, 1.0)
        self.point_default_force_spin.setSingleStep(0.05)
        self.point_default_force_spin.setDecimals(2)
        self.point_default_force_spin.setValue(0.10)
        self.point_default_force_spin.setMaximumWidth(90)
        param_layout.addWidget(self.point_default_force_spin)
        param_layout.addStretch()
        teach_layout.addLayout(param_layout)

        button_layout = QHBoxLayout()
        self.point_teach_btn = QPushButton("开始点位示教/重力补偿")
        self.point_teach_btn.clicked.connect(self.toggle_point_teach)
        button_layout.addWidget(self.point_teach_btn)

        self.capture_point_btn = QPushButton("记录当前点位")
        self.capture_point_btn.clicked.connect(self.capture_current_point)
        button_layout.addWidget(self.capture_point_btn)

        self.point_play_btn = QPushButton("播放点位程序")
        self.point_play_btn.clicked.connect(self.toggle_point_play)
        button_layout.addWidget(self.point_play_btn)
        teach_layout.addLayout(button_layout)

        self.point_teach_status = QLabel("状态: 就绪")
        teach_layout.addWidget(self.point_teach_status)
        self.point_play_status = QLabel("播放: 未开始")
        teach_layout.addWidget(self.point_play_status)
        layout.addWidget(teach_group)

        table_group = QGroupBox("点位程序编辑：J1~J6、夹爪G、六轴最大速度、间隔、夹爪速度、夹爪力度均可直接修改")
        table_layout = QVBoxLayout(table_group)

        file_buttons = QHBoxLayout()
        self.point_load_btn = QPushButton("导入点位CSV")
        self.point_load_btn.clicked.connect(self.load_point_program)
        file_buttons.addWidget(self.point_load_btn)

        self.point_save_btn = QPushButton("保存点位CSV")
        self.point_save_btn.clicked.connect(self.save_point_program)
        file_buttons.addWidget(self.point_save_btn)

        self.point_delete_btn = QPushButton("删除选中")
        self.point_delete_btn.clicked.connect(self.delete_selected_points)
        file_buttons.addWidget(self.point_delete_btn)

        self.point_up_btn = QPushButton("上移")
        self.point_up_btn.clicked.connect(lambda: self.move_selected_point(-1))
        file_buttons.addWidget(self.point_up_btn)

        self.point_down_btn = QPushButton("下移")
        self.point_down_btn.clicked.connect(lambda: self.move_selected_point(1))
        file_buttons.addWidget(self.point_down_btn)

        self.point_clear_btn = QPushButton("清空")
        self.point_clear_btn.clicked.connect(self.clear_points)
        file_buttons.addWidget(self.point_clear_btn)
        file_buttons.addStretch()
        table_layout.addLayout(file_buttons)

        self.point_table = QTableWidget(0, 13)
        self.point_table.setHorizontalHeader(WrappedHeaderView(Qt.Horizontal, self.point_table))
        #self.point_table.setWordWrap(True)
        self.point_table.setHorizontalHeaderLabels(POINT_HEADERS_EN if self.lang == "en" else POINT_HEADERS_ZH)

        self.point_column_weights = [
            55, 55, 55, 55, 55, 55, 55,
            100, 190, 80, 120, 120, 80
        ]   

        header = self.point_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Interactive)

        header.setMinimumSectionSize(50)
        header.setDefaultAlignment(Qt.AlignCenter)
        header.setTextElideMode(Qt.ElideNone)
        header.setStretchLastSection(False)

        default_widths = [
            60,  # No.
            60,  # J1
            60,  # J2
            60,  # J3
            60,  # J4
            60,  # J5
            60,  # J6
            100, # Gripper G
            175, # Arm Max Speed
            85,  # Pause
            115, # Gripper Speed
            115, # Gripper Force
            100, # Note
        ]

        for col, width in enumerate(default_widths):
            self.point_table.setColumnWidth(col, width)

        # Let the table scroll horizontally if the window is too narrow.
        self.point_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.point_table.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)

        # Give the multi-line column labels enough vertical space.
        header.setMinimumHeight(48)
        header.setDefaultSectionSize(60)

        self.point_table.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.point_table.setMinimumHeight(200)
        self.point_table.setSizeAdjustPolicy(QAbstractScrollArea.AdjustIgnored)

        self.point_table.verticalHeader().setVisible(False)
        self.point_table.setAlternatingRowColors(True)
        self.point_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.point_table.setSelectionMode(QTableWidget.SingleSelection)

        table_layout.addWidget(self.point_table, 1)
        layout.addWidget(table_group, 1)

        hint = QLabel(
            "说明：同步速度只针对J1~J6。六轴最大速度是机械臂关节速度上限，程序按J1~J6角度差自动分配速度；夹爪G不参与同步，播放时通过control_pos_force按表格里的夹爪速度和力度单独执行，默认速度5。蓝色行表示当前动作。"
        )
        hint.setWordWrap(True)
        hint.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Minimum)
        hint.setStyleSheet("color: #555;")
        layout.addWidget(hint, 0)

        QTimer.singleShot(0, self._resize_point_columns)

        return tab

    def build_angle_group(self):
        group = QGroupBox("实时关节角度")
        layout = QVBoxLayout(group)
        self.angle_text = QLabel("J1: 0.0000  J2: 0.0000  J3: 0.0000  J4: 0.0000  J5: 0.0000  J6: 0.0000  G: 0.0000")
        self.angle_text.setAlignment(Qt.AlignCenter)
        self.angle_text.setFont(QFont("Consolas", 10))
        layout.addWidget(self.angle_text)
        return group

    def build_log_group(self):
        group = QGroupBox("日志")
        layout = QVBoxLayout(group)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(120)
        layout.addWidget(self.log_text)
        return group
    
    def build_camera_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        group = QGroupBox("AI Object and Color Detector")
        group_layout = QVBoxLayout(group)

        # ==================== Camera mode ====================
        camera_layout = QHBoxLayout()

        camera_layout.addWidget(QLabel("Camera mode:"))

        self.camera_mode_selector = QComboBox()
        self.camera_mode_selector.addItem("Single Camera", "single")
        self.camera_mode_selector.addItem("Multiple Cameras", "multiple")
        self.camera_mode_selector.currentIndexChanged.connect(self.on_camera_mode_changed)
        camera_layout.addWidget(self.camera_mode_selector)

        camera_layout.addSpacing(15)

        camera_layout.addWidget(QLabel("Select camera:"))

        # **NEW: Single-camera dropdown**
        self.camera_selector = QComboBox()
        self.camera_selector.setMinimumWidth(150)
        camera_layout.addWidget(self.camera_selector)

        self.refresh_camera_btn = QPushButton("Refresh Cameras")
        self.refresh_camera_btn.clicked.connect(self.scan_cameras)
        camera_layout.addWidget(self.refresh_camera_btn)

        camera_layout.addStretch()
        group_layout.addLayout(camera_layout)

        # ==================== Multiple-camera selection ====================

        self.camera_multi_selector = QListWidget()
        self.camera_multi_selector.setSelectionMode(QListWidget.MultiSelection)
        self.camera_multi_selector.setMaximumHeight(90)

        group_layout.addWidget(self.camera_multi_selector)

        # ==================== Camera preview ====================

        self.camera_preview_container = QWidget()
        self.camera_preview_layout = QGridLayout(self.camera_preview_container)

        self.camera_preview_scroll = QScrollArea()
        self.camera_preview_scroll.setWidgetResizable(True)
        self.camera_preview_scroll.setWidget(self.camera_preview_container)

        # **The preview now gets the majority of the available space.**
        group_layout.addWidget(self.camera_preview_scroll, 1)

        # ==================== Start/stop cameras ====================

        self.toggle_camera_btn = QPushButton("Open Camera(s)")
        self.toggle_camera_btn.setMinimumHeight(40)
        self.toggle_camera_btn.clicked.connect(self.toggle_camera_detector)

        group_layout.addWidget(self.toggle_camera_btn)

        layout.addWidget(group, 1)

        self.scan_cameras()
        self.on_camera_mode_changed()

        return tab
    # ==================== 基础功能 ====================

    def ui(self, zh_text: str) -> str:
        if self.lang == "en":
            return ZH_TO_EN.get(zh_text, zh_text)
        return EN_TO_ZH.get(zh_text, zh_text)

    def ui_status(self, text: str) -> str:
        if self.lang == "zh":
            return text
        if text in ZH_TO_EN:
            return ZH_TO_EN[text]
        if text.startswith("已连接 "):
            return "Connected " + text.replace("已连接 ", "", 1)
        if text.startswith("状态: 已记录 ") and text.endswith(" 点"):
            n = text.replace("状态: 已记录 ", "", 1).replace(" 点", "")
            return f"Status: recorded {n} points"
        if text.startswith("状态: 正在执行第 "):
            # 状态: 正在执行第 1/5 个动作，预计运动 2.00s
            try:
                rest = text.replace("状态: 正在执行第 ", "", 1)
                count_part, time_part = rest.split(" 个动作，预计运动 ", 1)
                return f"Status: executing action {count_part}, estimated motion {time_part}"
            except Exception:
                return text
        return text

    def toggle_language(self):
        self.lang = "en" if self.lang == "zh" else "zh"
        self.apply_language()

    def apply_language(self):
        self.setWindowTitle(self.ui("拖动示教控制器 - 夹爪力位混合版"))
        self.title_label.setText(self.ui("机械臂拖动示教系统：J1~J6 + 夹爪G反馈/力位混合控制"))
        self.language_btn.setText("English" if self.lang == "zh" else "中文")
        self.tabs.setTabText(0, self.ui("连续轨迹示教"))
        self.tabs.setTabText(1, self.ui("点位示教"))
        self.tabs.setTabText(2, self.ui("相机传感器"))
        self.point_table.setHorizontalHeaderLabels(POINT_HEADERS_EN if self.lang == "en" else POINT_HEADERS_ZH)

        forward = ZH_TO_EN if self.lang == "en" else EN_TO_ZH
        for widget in self.findChildren((QLabel, QPushButton, QGroupBox)):
            if widget is self.angle_text:
                continue

            if isinstance(widget, QGroupBox):
                current = widget.title()
                if current in forward:
                    widget.setTitle(forward[current])
            else:
                current = widget.text()
                if current in forward:
                    widget.setText(forward[current])

        self.refresh_dynamic_ui_texts()

    def refresh_dynamic_ui_texts(self):
        if self.is_connected:
            self.conn_status_raw = f"已连接 {self.connected_port}"
        else:
            self.conn_status_raw = "未连接"
        self.conn_status.setText(self.ui_status(self.conn_status_raw))

        if self.connect_btn.isEnabled():
            self.connect_btn.setText(self.ui("断开") if self.is_connected else self.ui("连接"))

        self.set_zero_btn.setText(self.ui("设置0点中...") if self.is_setting_zero else self.ui("设置0点"))
        self.set_zero_btn.setEnabled(self.is_connected and not self.is_setting_zero and not self.is_toggling_enable)

        if self.is_toggling_enable:
            enable_text = "使能中..." if self.enable_target_enabled else "失能中..."
        else:
            enable_text = "失能" if self.arm_enabled else "使能"
        self.enable_btn.setText(self.ui(enable_text))
        self.enable_btn.setEnabled(self.is_connected and not self.is_setting_zero and not self.is_toggling_enable)

        self.path_record_btn.setText(self.ui("停止连续记录") if self.is_path_recording else self.ui("开始连续拖动示教"))
        self.path_play_btn.setText(self.ui("停止连续播放") if self.is_path_playing else self.ui("播放连续轨迹"))
        self.point_teach_btn.setText(self.ui("停止点位示教/保持当前位置") if self.is_point_teaching else self.ui("开始点位示教/重力补偿"))
        self.point_play_btn.setText(self.ui("停止点位播放") if self.is_point_playing else self.ui("播放点位程序"))

        self.path_record_status.setText(self.ui_status(self.path_record_status_raw))
        self.path_play_status.setText(self.ui_status(self.path_play_status_raw))
        self.point_teach_status.setText(self.ui_status(self.point_teach_status_raw))
        self.point_play_status.setText(self.ui_status(self.point_play_status_raw))

        if hasattr(self, "home_btn"):
            self.home_btn.setText(self.ui("返回初始位置中...") if self.is_returning_home else self.ui("回到初始位置"))

    def warn(self, title_zh: str, message_zh: str):
        QMessageBox.warning(self, self.ui(title_zh), self.ui(message_zh))

    def warn_text(self, title_zh: str, message: str):
        QMessageBox.warning(self, self.ui(title_zh), message)

    def get_default_path(self):
        path = os.path.join(os.path.expanduser("~"), "Yang-Lab-Robot-Detection-test", "Ctr_Arm_drag_work", "robot_arm_gripper_code", "Trajectory Points", "RobotTrajectories")
        os.makedirs(path, exist_ok=True)
        return path

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

    def toggle_connection(self):
        if not self.is_connected:
            port = self.port_combo.currentData()
            if not port:
                self.warn("错误", "请选择有效的串口")
                return
            self.connect_btn.setEnabled(False)
            self.connect_btn.setText(self.ui("连接中..."))
            self.set_zero_btn.setEnabled(False)
            self.enable_btn.setEnabled(False)
            self.worker.request_connect(port)
        else:
            if self.is_any_running():
                self.warn("提示", "请先停止当前示教或播放任务")
                return
            self.connect_btn.setEnabled(False)
            self.set_zero_btn.setEnabled(False)
            self.enable_btn.setEnabled(False)
            self.connect_btn.setText(self.ui("断开中..."))
            self.conn_status_raw = "断开中..."
            self.conn_status.setText(self.ui_status(self.conn_status_raw))
            self.conn_status.setStyleSheet("color: orange;")
            self.worker.request_disconnect()

    def on_connection_changed(self, connected, port):
        self.is_connected = connected
        self.arm_enabled = connected
        self.is_toggling_enable = False
        self.connect_btn.setEnabled(True)
        if connected:
            self.connect_btn.setText(self.ui("断开"))
            self.connect_btn.setStyleSheet("background-color: #f44336; color: white;")
            self.port_combo.setEnabled(False)
            self.refresh_btn.setEnabled(False)
            self.connected_port = port
            self.conn_status_raw = f"已连接 {port}"
            self.conn_status.setText(self.ui_status(self.conn_status_raw))
            self.conn_status.setStyleSheet("color: green;")
            self.set_zero_btn.setEnabled(not self.is_setting_zero)
            self.enable_btn.setText(self.ui("失能"))
            self.enable_btn.setEnabled(True)
            self.home_btn.setEnabled(self.arm_enabled)
        else:
            self.is_path_recording = False
            self.is_path_playing = False
            self.is_point_teaching = False
            self.is_point_playing = False
            self.arm_enabled = False
            self.connect_btn.setText(self.ui("连接"))
            self.connect_btn.setStyleSheet("background-color: #4CAF50; color: white;")
            self.port_combo.setEnabled(True)
            self.refresh_btn.setEnabled(True)
            self.connected_port = ""
            self.conn_status_raw = "未连接"
            self.conn_status.setText(self.ui_status(self.conn_status_raw))
            self.conn_status.setStyleSheet("color: gray;")
            self.set_zero_btn.setEnabled(False)
            self.enable_btn.setText(self.ui("使能"))
            self.enable_btn.setEnabled(False)
            self.reset_all_buttons_after_stop()
            self.home_btn.setEnabled(False)

    def is_any_running(self):
        return self.is_path_recording or self.is_path_playing or self.is_point_teaching or self.is_point_playing

    def confirm_set_zero(self):
        if not self.is_connected:
            self.warn("错误", "请先连接机械臂")
            return
        if self.is_any_running():
            self.warn("提示", "请先停止当前示教或播放任务")
            return
        if self.is_setting_zero or self.is_toggling_enable:
            return

        box = QMessageBox(self)
        box.setIcon(QMessageBox.Warning)
        box.setWindowTitle(self.ui("确认"))
        box.setText(self.ui("确定将当前位置设置为0点吗？\n设置后不可逆。"))
        ok_btn = box.addButton(self.ui("确定"), QMessageBox.AcceptRole)
        cancel_btn = box.addButton(self.ui("取消"), QMessageBox.RejectRole)
        box.setDefaultButton(cancel_btn)
        box.exec_()
        if box.clickedButton() is not ok_btn:
            return

        self.is_setting_zero = True
        self.set_zero_btn.setEnabled(False)
        self.set_zero_btn.setText(self.ui("设置0点中..."))
        self.enable_btn.setEnabled(False)
        self.connect_btn.setEnabled(False)
        self.tabs.setEnabled(False)
        self.worker.request_set_zero()

    def return_to_home(self):
        if not self.is_connected:
            self.warn("错误", "请先连接机械臂")
            return

        if not self.arm_enabled:
            self.warn("错误", "请先使能机械臂")
            return

        if self.is_any_running():
            self.warn("提示", "请先停止当前示教或播放任务")
            return

        if self.is_setting_zero or self.is_toggling_enable or self.is_returning_home:
            return

        self.is_returning_home = True

        self.home_btn.setEnabled(False)
        self.home_btn.setText(self.ui("返回初始位置中..."))

        self.set_zero_btn.setEnabled(False)
        self.enable_btn.setEnabled(False)
        self.connect_btn.setEnabled(False)
        self.tabs.setEnabled(False)

        self.worker.request_return_home()

    def on_home_return_finished(self, ok, message):
        # Return Home operation has finished.
        self.is_returning_home = False

        # Restore the interface.
        self.tabs.setEnabled(True)
        self.connect_btn.setEnabled(True)

        # Restore button labels and availability.
        self.home_btn.setText(self.ui("回到初始位置"))
        self.home_btn.setEnabled(self.is_connected and self.arm_enabled)

        self.set_zero_btn.setText(self.ui("设置0点"))
        self.set_zero_btn.setEnabled(self.is_connected and not self.is_setting_zero and not self.is_toggling_enable)

        self.enable_btn.setEnabled(self.is_connected and not self.is_toggling_enable)

        # Display an error only if the operation failed.
        if not ok:
            self.warn_text("机械臂错误", message)

        self.log(message)

    def on_zero_set_finished(self, ok, message):
        self.is_setting_zero = False
        self.tabs.setEnabled(True)
        self.connect_btn.setEnabled(True)
        self.set_zero_btn.setText(self.ui("设置0点"))
        self.set_zero_btn.setEnabled(self.is_connected)
        self.enable_btn.setEnabled(self.is_connected and not self.is_toggling_enable)
        if not ok:
            self.warn_text("机械臂错误", message)

    def toggle_arm_enable(self):
        if not self.is_connected:
            self.warn("错误", "请先连接机械臂")
            return
        if self.is_any_running():
            self.warn("提示", "请先停止当前示教或播放任务")
            return
        if self.is_setting_zero or self.is_toggling_enable:
            return

        target_enabled = not self.arm_enabled
        self.is_toggling_enable = True
        self.enable_target_enabled = target_enabled
        self.enable_btn.setEnabled(False)
        self.enable_btn.setText(self.ui("使能中..." if target_enabled else "失能中..."))
        self.set_zero_btn.setEnabled(False)
        self.connect_btn.setEnabled(False)
        self.tabs.setEnabled(False)
        self.worker.request_set_enabled(target_enabled)

    def on_enable_state_changed(self, enabled):
        self.arm_enabled = bool(enabled)
        if not self.is_toggling_enable:
            self.enable_btn.setText(self.ui("失能" if self.arm_enabled else "使能"))
            self.enable_btn.setEnabled(self.is_connected and not self.is_setting_zero)

    def on_enable_toggle_finished(self, ok, enabled, message):
        self.is_toggling_enable = False
        self.arm_enabled = bool(enabled)
        self.tabs.setEnabled(True)
        self.connect_btn.setEnabled(True)
        self.set_zero_btn.setEnabled(self.is_connected and not self.is_setting_zero)
        self.enable_btn.setText(self.ui("失能" if self.arm_enabled else "使能"))
        self.enable_btn.setEnabled(self.is_connected)
        if not ok:
            self.warn_text("机械臂错误", message)

    def other_task_running(self, current: str):
        flags = {
            "path_record": self.is_path_recording,
            "path_play": self.is_path_playing,
            "point_teach": self.is_point_teaching,
            "point_play": self.is_point_playing,
        }
        return any(v for k, v in flags.items() if k != current)

    def reset_all_buttons_after_stop(self):
        self.path_record_btn.setEnabled(True)
        self.path_record_btn.setText(self.ui("开始连续拖动示教"))
        self.path_play_btn.setEnabled(bool(self.path_play_file_edit.text()))
        self.path_play_btn.setText(self.ui("播放连续轨迹"))
        self.path_select_file_btn.setEnabled(True)
        self.set_path_record_controls_enabled(True)

        self.point_teach_btn.setEnabled(True)
        self.point_teach_btn.setText(self.ui("开始点位示教/重力补偿"))
        self.point_play_btn.setEnabled(True)
        self.point_play_btn.setText(self.ui("播放点位程序"))
        self.set_point_edit_controls_enabled(True)
        if hasattr(self, "set_zero_btn"):
            self.set_zero_btn.setEnabled(self.is_connected and not self.is_setting_zero and not self.is_toggling_enable)
        if hasattr(self, "enable_btn"):
            self.enable_btn.setEnabled(self.is_connected and not self.is_setting_zero and not self.is_toggling_enable)
        self.highlight_point_row(-1)

    def _resize_point_columns(self):
        if not hasattr(self, "point_table"):
            return

        self.point_table.horizontalHeader().setStretchLastSection(True)

        table = self.point_table
        available_width = table.viewport().width()

        if available_width <= 0:
            return

        weights = self.point_column_weights
        total_weight = sum(weights)

        # Distribute the available width according to the column weights.
        widths = [
            int(available_width * weight / total_weight)
            for weight in weights
        ]

        # Account for rounding so the widths add up exactly.
        widths[-1] += available_width - sum(widths)

        for col, width in enumerate(widths):
            table.setColumnWidth(col, max(width, 1))

    def resizeEvent(self, event):
        super().resizeEvent(event)

        if hasattr(self, "point_table"):
            QTimer.singleShot(0, self._resize_point_columns)

    # ==================== 连续轨迹示教 ====================
    def browse_path_save_dir(self):
        path = QFileDialog.getExistingDirectory(self, self.ui("选择保存目录"), self.path_save_dir_edit.text())
        if path:
            self.path_save_dir_edit.setText(path)

    def build_path_save_filepath(self):
        filename = self.path_filename_edit.text().strip()
        if not filename:
            filename = f"trajectory_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        if not filename.lower().endswith(".csv"):
            filename += ".csv"
        save_dir = self.path_save_dir_edit.text().strip() or self.get_default_path()
        return os.path.join(save_dir, filename)

    def set_path_record_controls_enabled(self, enabled):
        self.path_filename_edit.setEnabled(enabled)
        self.path_browse_btn.setEnabled(enabled)
        self.path_record_hz_spin.setEnabled(enabled)
        self.path_gravity_hz_spin.setEnabled(enabled)

    def toggle_path_record(self):
        if not self.is_connected:
            self.warn("错误", "请先连接机械臂")
            return
        if not self.arm_enabled:
            self.warn("错误", "请先使能机械臂")
            return
        if self.other_task_running("path_record"):
            self.warn("错误", "请先停止其他示教或播放任务")
            return

        if not self.is_path_recording:
            self.is_path_recording = True
            self.start_CT_camera_recording()
            self.path_record_btn.setText(self.ui("停止连续记录"))
            self.set_path_record_controls_enabled(False)
            self.path_record_status_update("状态: 准备中...")
            self.worker.request_start_path_record(self.path_record_hz_spin.value(), self.path_gravity_hz_spin.value())
        else:
            self.path_record_btn.setEnabled(False)
            self.path_record_status_update("状态: 正在停止...")
            self.worker.request_stop_path_record()

    def on_path_record_finished(self, rows):
        count = len(rows)

        if count > 0:
            session_dir = self.CT_session_dir or self.get_default_path()
            filename = self.path_filename_edit.text().strip()

            if not filename:
                filename = f"trajectory_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            if not filename.lower().endswith(".csv"):
                filename += ".csv"

            filepath = os.path.join(session_dir, filename)

            try:
                save_trajectory_csv(filepath, rows)
                self.log(f"连续轨迹已保存: {filepath}")
            except Exception as e:
                self.warn_text("错误", f"保存失败:\n{e}")
                self.log(f"保存失败: {e}")

        # Stop Camera Recording
        self.stop_CT_camera_recording()

        self.is_path_recording = False
        self.path_record_btn.setEnabled(True)
        self.path_record_btn.setText(self.ui("开始连续拖动示教"))
        self.set_path_record_controls_enabled(True)
        self.path_record_status_update(f"状态: 已记录 {count} 点")

    def path_record_status_update(self, text):
        self.path_record_status_raw = text
        self.path_record_status.setText(self.ui_status(text))
        if "记录中" in text:
            self.path_record_status.setStyleSheet("color: red;")
        elif "失败" in text:
            self.path_record_status.setStyleSheet("color: red;")
        elif "准备" in text or "切换" in text or "停止" in text:
            self.path_record_status.setStyleSheet("color: orange;")
        else:
            self.path_record_status.setStyleSheet("color: green;")

    # ==================== 连续轨迹播放 ====================
    def select_path_file(self):
        path, _ = QFileDialog.getOpenFileName(self, self.ui("选择连续轨迹CSV"), self.get_default_path(), "CSV Files (*.csv)")
        if path:
            self.path_play_file_edit.setText(path)
            self.path_play_btn.setEnabled(True)
            self.path_play_status_update("状态: 已选择轨迹")
            self.log(f"已选择连续轨迹: {path}")

    def toggle_path_play(self):
        if not self.is_connected:
            self.warn("错误", "请先连接机械臂")
            return
        if not self.arm_enabled:
            self.warn("错误", "请先使能机械臂")
            return
        if self.other_task_running("path_play"):
            self.warn("错误", "请先停止其他示教或播放任务")
            return

        if not self.is_path_playing:
            filepath = self.path_play_file_edit.text().strip()
            if not filepath:
                self.warn("错误", "请先选择轨迹文件")
                return
            self.is_path_playing = True
            self.path_play_btn.setText(self.ui("停止连续播放"))
            self.path_select_file_btn.setEnabled(False)
            self.path_play_status_update("状态: 准备中...")
            self.worker.request_start_path_play(filepath, self.path_play_speed_spin.value())
        else:
            self.path_play_btn.setEnabled(False)
            self.path_play_status_update("状态: 正在停止...")
            self.worker.request_stop_path_play()

    def on_path_play_finished(self):
        self.is_path_playing = False
        self.path_play_btn.setEnabled(True)
        self.path_play_btn.setText(self.ui("播放连续轨迹"))
        self.path_select_file_btn.setEnabled(True)

    def path_play_status_update(self, text):
        self.path_play_status_raw = text
        self.path_play_status.setText(self.ui_status(text))
        if "播放中" in text:
            self.path_play_status.setStyleSheet("color: red;")
        elif "失败" in text:
            self.path_play_status.setStyleSheet("color: red;")
        elif "准备" in text or "切换" in text or "停止" in text:
            self.path_play_status.setStyleSheet("color: orange;")
        else:
            self.path_play_status.setStyleSheet("color: black;")

    # ==================== 点位示教 ====================
    def toggle_point_teach(self):
        if not self.is_connected:
            self.warn("错误", "请先连接机械臂")
            return
        if not self.arm_enabled:
            self.warn("错误", "请先使能机械臂")
            return
        if self.other_task_running("point_teach"):
            self.warn("错误", "请先停止其他示教或播放任务")
            return

        if not self.is_point_teaching:
            self.start_point_camera_recording()
            self.is_point_teaching = True
            self.point_teach_btn.setText(self.ui("停止点位示教/保持当前位置"))
            self.point_teach_status_update("状态: 准备中...")
            self.worker.request_start_point_teach(self.point_gravity_hz_spin.value())
        else:
            self.point_teach_btn.setEnabled(False)
            self.point_teach_status_update("状态: 正在停止...")
            self.worker.request_stop_point_teach()

    def point_teach_status_update(self, text):
        self.point_teach_status_raw = text
        self.point_teach_status.setText(self.ui_status(text))
        if "示教中" in text:
            self.point_teach_status.setStyleSheet("color: red;")
        elif "失败" in text:
            self.point_teach_status.setStyleSheet("color: red;")
        elif "准备" in text or "切换" in text or "停止" in text:
            self.point_teach_status.setStyleSheet("color: orange;")
        else:
            self.point_teach_status.setStyleSheet("color: green;")

        if "已停止" in text:
            self.is_point_teaching = False
            self.point_teach_btn.setEnabled(True)
            self.point_teach_btn.setText(self.ui("开始点位示教/重力补偿"))
            self.stop_point_camera_recording()

    def capture_current_point(self):
        if not self.is_connected:
            self.warn("错误", "请先连接机械臂")
            return
        if self.is_point_playing or self.is_path_playing or self.is_path_recording:
            self.warn("错误", "播放或连续记录中不能记录点位")
            return
        self.worker.request_capture_point(
            self.point_default_speed_spin.value(),
            self.point_default_pause_spin.value(),
            self.point_default_gripper_speed_spin.value(),
            self.point_default_force_spin.value(),
        )

    def on_point_captured(self, q, speed, pause, gripper_speed, gripper_force_limit):
        if len(q) < TOTAL_DOF:
            q = list(q[:ARM_DOF]) + [0.0]
        self.save_point_camera_reference(q)
        self.append_point_row(PointAction(q=list(q[:TOTAL_DOF]), speed=speed, pause=pause, gripper_speed=gripper_speed, gripper_force_limit=gripper_force_limit, note=""))

    def append_point_row(self, action: PointAction):
        row = self.point_table.rowCount()
        self.point_table.insertRow(row)
        self.set_point_row(row, action)
        self.renumber_points()
        self.point_table.selectRow(row)

    def set_point_row(self, row: int, action: PointAction):
        q = list(action.q)
        if len(q) < TOTAL_DOF:
            q = q[:ARM_DOF] + [0.0]
        values = [
            str(row + 1),
            *[f"{v:.6f}" for v in q[:TOTAL_DOF]],
            f"{float(action.speed):.3f}",
            f"{float(action.pause):.3f}",
            f"{float(action.gripper_speed):.3f}",
            f"{float(action.gripper_force_limit):.3f}",
            action.note,
        ]
        for col, value in enumerate(values):
            item = QTableWidgetItem(value)
            item.setTextAlignment(Qt.AlignCenter if col != 12 else Qt.AlignVCenter | Qt.AlignLeft)
            if col == 0:
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)
            self.point_table.setItem(row, col, item)

    def renumber_points(self):
        for row in range(self.point_table.rowCount()):
            item = self.point_table.item(row, 0)
            if item is None:
                item = QTableWidgetItem()
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                self.point_table.setItem(row, 0, item)
            item.setText(str(row + 1))
            item.setTextAlignment(Qt.AlignCenter)

    def get_point_actions_from_table(self) -> List[PointAction]:
        actions: List[PointAction] = []
        for row in range(self.point_table.rowCount()):
            try:
                q = [float(self.cell_text(row, c)) for c in range(1, 8)]
                speed = float(self.cell_text(row, 8))
                pause = float(self.cell_text(row, 9))
                gripper_speed = float(self.cell_text(row, 10))
                gripper_force_limit = float(self.cell_text(row, 11))
                note = self.cell_text(row, 12)
            except ValueError as e:
                raise ValueError(f"第 {row + 1} 行存在非数字内容，请检查角度、六轴速度、间隔、夹爪速度和夹爪力度") from e
            if speed <= 0:
                raise ValueError(f"第 {row + 1} 行最大速度必须大于0")
            if pause < 0:
                raise ValueError(f"第 {row + 1} 行间隔不能小于0")
            if gripper_speed <= 0:
                raise ValueError(f"第 {row + 1} 行夹爪速度必须大于0")
            if not (0.0 <= gripper_force_limit <= 1.0):
                raise ValueError(f"第 {row + 1} 行夹爪力度建议在 0.0 到 1.0 之间")
            actions.append(PointAction(q=q, speed=speed, pause=pause, gripper_speed=gripper_speed, gripper_force_limit=gripper_force_limit, note=note))
        return actions

    def cell_text(self, row: int, col: int) -> str:
        item = self.point_table.item(row, col)
        return "" if item is None else item.text().strip()

    def delete_selected_points(self):
        rows = sorted({idx.row() for idx in self.point_table.selectedIndexes()}, reverse=True)
        if not rows:
            return
        for row in rows:
            self.point_table.removeRow(row)
        self.renumber_points()
        self.highlight_point_row(-1)

    def move_selected_point(self, direction: int):
        selected = self.point_table.selectedIndexes()
        if not selected:
            return
        row = selected[0].row()
        new_row = row + direction
        if new_row < 0 or new_row >= self.point_table.rowCount():
            return
        actions = self.get_point_actions_from_table()
        actions[row], actions[new_row] = actions[new_row], actions[row]
        self.reload_point_table(actions)
        self.point_table.selectRow(new_row)

    def clear_points(self):
        if self.point_table.rowCount() == 0:
            return
        reply = QMessageBox.question(self, self.ui("确认"), self.ui("确定清空所有点位吗？"))
        if reply == QMessageBox.Yes:
            self.point_table.setRowCount(0)
            self.highlight_point_row(-1)

    def move_gripper_x(self, direction: int):
        selected = self.point_table.selectedIndexes()
        if not selected:
            return
        row = selected[0].row()
        new_row = row + direction
        if new_row < 0 or new_row >= self.point_table.rowCount():
            return
        actions = self.get_point_actions_from_table()
        actions[row], actions[new_row] = actions[new_row], actions[row]
        self.reload_point_table(actions)
        self.point_table.selectRow(new_row)   

    def reload_point_table(self, actions: List[PointAction]):
        self.point_table.setRowCount(0)
        for action in actions:
            self.append_point_row(action)
        self.highlight_point_row(-1)

    def save_point_program(self):
        try:
            actions = self.get_point_actions_from_table()
        except Exception as e:
            self.warn_text("错误", str(e))
            return
        if not actions:
            self.warn("提示", "没有点位可保存")
            return

        default_name = f"points_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        filepath, _ = QFileDialog.getSaveFileName(self, self.ui("保存点位程序"), os.path.join(self.get_default_path(), default_name), "CSV Files (*.csv)")
        if not filepath:
            return
        if not filepath.lower().endswith(".csv"):
            filepath += ".csv"
        try:
            save_point_program_csv(filepath, actions)
            self.log(f"点位程序已保存: {filepath}")
        except Exception as e:
            self.warn_text("错误", f"保存失败:\n{e}")

    def load_point_program(self):
        filepath, _ = QFileDialog.getOpenFileName(self, self.ui("导入点位程序"), self.get_default_path(), "CSV Files (*.csv)")
        if not filepath:
            return
        try:
            actions = read_point_program_csv(filepath)
            self.reload_point_table(actions)
            self.log(f"点位程序已导入: {filepath}")
        except Exception as e:
            self.warn_text("错误", f"导入失败:\n{e}")

    def set_point_edit_controls_enabled(self, enabled: bool):
        self.point_table.setEnabled(enabled)
        self.point_load_btn.setEnabled(enabled)
        self.point_save_btn.setEnabled(enabled)
        self.point_delete_btn.setEnabled(enabled)
        self.point_up_btn.setEnabled(enabled)
        self.point_down_btn.setEnabled(enabled)
        self.point_clear_btn.setEnabled(enabled)
        self.capture_point_btn.setEnabled(enabled)
        self.point_default_gripper_speed_spin.setEnabled(enabled)
        self.point_default_force_spin.setEnabled(enabled)
        self.point_teach_btn.setEnabled(enabled or self.is_point_teaching)

    # ==================== 点位播放 ====================
    def toggle_point_play(self):
        if not self.is_connected:
            self.warn("错误", "请先连接机械臂")
            return
        if not self.arm_enabled:
            self.warn("错误", "请先使能机械臂")
            return
        if self.other_task_running("point_play"):
            self.warn("错误", "请先停止其他示教或播放任务")
            return

        if not self.is_point_playing:
            try:
                actions = self.get_point_actions_from_table()
            except Exception as e:
                self.warn_text("错误", str(e))
                return
            if not actions:
                self.warn("错误", "点位程序为空")
                return
            self.is_point_playing = True
            self.point_play_btn.setText(self.ui("停止点位播放"))
            self.set_point_edit_controls_enabled(False)
            self.point_play_status_update("状态: 准备中...")
            self.highlight_point_row(-1)
            self.worker.request_start_point_play(actions)
        else:
            self.point_play_btn.setEnabled(False)
            self.point_play_status_update("状态: 正在停止...")
            self.worker.request_stop_point_play()

    def point_play_status_update(self, text):
        self.point_play_status_raw = text
        self.point_play_status.setText(self.ui_status(text))
        if "正在执行" in text or "播放中" in text:
            self.point_play_status.setStyleSheet("color: red;")
        elif "失败" in text:
            self.point_play_status.setStyleSheet("color: red;")
        elif "准备" in text or "切换" in text or "停止" in text:
            self.point_play_status.setStyleSheet("color: orange;")
        else:
            self.point_play_status.setStyleSheet("color: black;")

    def highlight_point_row(self, row_index: int):
        self.current_point_highlight = row_index
        normal_brush = QBrush()
        blue_brush = QBrush(QColor(80, 160, 255))
        for r in range(self.point_table.rowCount()):
            for c in range(self.point_table.columnCount()):
                item = self.point_table.item(r, c)
                if item is None:
                    continue
                item.setBackground(blue_brush if r == row_index else normal_brush)
        if 0 <= row_index < self.point_table.rowCount():
            self.point_table.scrollToItem(self.point_table.item(row_index, 0))
            self.point_table.selectRow(row_index)

    def on_point_play_finished(self):
        self.is_point_playing = False
        self.point_play_btn.setEnabled(True)
        self.point_play_btn.setText(self.ui("播放点位程序"))
        self.set_point_edit_controls_enabled(True)
        self.highlight_point_row(-1)

    # ==================== 相机检测器 ====================

    def on_camera_mode_changed(self):
        """Switch between single-camera dropdown and multiple-camera list."""

        if self.camera_workers:
            return

        single_mode = self.camera_mode_selector.currentData() == "single"

        # Show the dropdown for single-camera mode.
        self.camera_selector.setVisible(single_mode)

        # Show the selection list for multiple-camera mode.
        self.camera_multi_selector.setVisible(not single_mode)

    def get_selected_camera_indices(self):
        """Return selected camera indices based on the current mode."""

        if self.camera_mode_selector.currentData() == "single":

            # Get the selected camera from the dropdown.
            camera_index = self.camera_selector.currentData()

            if camera_index is None:
                return []

            return [int(camera_index)]

        else:

            # Get all selected cameras from the multiple-camera list.
            return [int(item.data(Qt.UserRole)) for item in self.camera_multi_selector.selectedItems()]

    def toggle_camera_detector(self):
        # If cameras are already running, stop all of them.
        if self.camera_workers:
            self.stop_all_cameras()
            return

        camera_indices = self.get_selected_camera_indices()

        if not camera_indices:
            self.warn("错误", "请选择至少一个摄像头")
            return

        # Single-camera mode should only start one camera.
        if self.camera_mode_selector.currentData() == "single":
            camera_indices = camera_indices[:1]

        self.camera_mode_selector.setEnabled(False)
        self.camera_selector.setEnabled(False)
        self.camera_multi_selector.setEnabled(False)
        self.refresh_camera_btn.setEnabled(False)

        self.clear_camera_previews()

        for camera_index in camera_indices:
            self.start_camera_worker(camera_index)

        self.toggle_camera_btn.setText("Close Camera(s)")

    def start_camera_worker(self, camera_index):
        """Create and start one independent worker for a camera."""

        try:
            worker = CameraWork(camera_index=camera_index)

            worker.frame_ready.connect(lambda image, index=camera_index: self.update_camera_frame(index, image))

            worker.log_msg.connect(self.log)

            worker.finished.connect(lambda index=camera_index: self.on_camera_finished(index))

            self.camera_workers[camera_index] = worker
            self.latest_camera_images[camera_index] = None

            # Create a separate preview for this camera.
            preview = QLabel(f"Camera {camera_index}: Waiting for image...")
            preview.setAlignment(Qt.AlignCenter)
            preview.setStyleSheet("background-color: black; color: white;")

            # **NEW: Larger preview in single-camera mode.**
            if self.camera_mode_selector.currentData() == "single":
                preview.setMinimumSize(640, 480)
                preview.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
                column_count = 1

            else:
                preview.setMinimumSize(320, 240)
                preview.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
                column_count = 2

            self.camera_preview_labels[camera_index] = preview

            # Arrange previews based on the selected mode.
            position = len(self.camera_preview_labels) - 1
            row = position // column_count
            column = position % column_count

            self.camera_preview_layout.addWidget(preview, row, column)

            worker.start()

            self.log(f"Started camera {camera_index}")

        except Exception as e:
            self.log(f"Failed to start camera {camera_index}: {e}")

    def update_camera_frame(self, camera_index, qt_image):
        """Update the latest frame and the corresponding preview."""

        self.latest_camera_images[camera_index] = qt_image.copy()

        # Preserve compatibility with existing single-camera references.
        if self.latest_camera_image is None or camera_index == min(self.latest_camera_images):
            self.latest_camera_image = qt_image.copy()

        preview = self.camera_preview_labels.get(camera_index)

        if preview is not None:
            preview.setPixmap(QPixmap.fromImage(qt_image).scaled(preview.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def on_camera_finished(self, camera_index):
        """Handle a camera worker that has finished."""

        worker = self.camera_workers.get(camera_index)

        if worker is not None and not worker.isRunning():
            self.camera_workers.pop(camera_index, None)

        self.latest_camera_images.pop(camera_index, None)
        self.camera_preview_labels.pop(camera_index, None)

        if not self.camera_workers:
            self.latest_camera_image = None

            self.camera_mode_selector.setEnabled(True)
            self.camera_selector.setEnabled(True)
            self.refresh_camera_btn.setEnabled(True)
            self.camera_multi_selector.setEnabled(True)
            self.toggle_camera_btn.setText("Open Camera(s)")

            self.log("All cameras stopped.")

    def stop_all_cameras(self):
        """Stop every active camera worker."""

        workers = list(self.camera_workers.items())

        for camera_index, worker in workers:
            worker.stop()

            if worker.isRunning():
                worker.wait()

            self.camera_workers.pop(camera_index, None)

        self.latest_camera_images.clear()
        self.latest_camera_image = None

        self.clear_camera_previews()

        self.camera_mode_selector.setEnabled(True)
        self.camera_selector.setEnabled(True)
        self.camera_multi_selector.setEnabled(True)
        self.refresh_camera_btn.setEnabled(True)

        self.toggle_camera_btn.setText("Open Camera(s)")

        self.log("All cameras stopped.")

    def clear_camera_previews(self):
        """Remove all camera preview widgets."""

        while self.camera_preview_layout.count():
            item = self.camera_preview_layout.takeAt(0)
            widget = item.widget()

            if widget is not None:
                widget.deleteLater()

        self.camera_preview_labels.clear()

    def scan_cameras(self):
        """Detect available camera indices."""

        if self.camera_workers:
            self.log("Stop all cameras before refreshing the camera list.")
            return

        # Clear previous camera selections.
        self.camera_selector.clear()
        self.camera_multi_selector.clear()

        available_cameras = []

        for index in range(10):
            if check_camera_quietly(index):
                available_cameras.append(index)

        # Populate both selection controls.
        for index in available_cameras:

            # Single-camera dropdown.
            self.camera_selector.addItem(f"Camera {index}", index)

            # Multiple-camera list.
            item = QListWidgetItem(f"Camera {index}")
            item.setData(Qt.UserRole, index)
            self.camera_multi_selector.addItem(item)

        # Automatically select the first camera.
        if self.camera_selector.count() > 0:
            self.camera_selector.setCurrentIndex(0)

        if self.camera_multi_selector.count() > 0:
            self.camera_multi_selector.item(0).setSelected(True)

        self.on_camera_mode_changed()

        if not available_cameras:
            self.log("No available cameras detected.")
        else:
            self.log(f"Detected cameras: {available_cameras}")

    def start_point_camera_recording(self):
        """Create a point-teaching session for all active cameras."""

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        session_dir = os.path.join(self.get_default_path(), f"point_session_{timestamp}")
        os.makedirs(session_dir, exist_ok=True)

        self.point_recording_dir = session_dir
        self.point_recording_csv = os.path.join(session_dir, "camera_references.csv")

        self.point_recording_file = open(self.point_recording_csv, "w", newline="", encoding="utf-8")

        self.point_recording_writer = csv.writer(self.point_recording_file)

        self.point_recording_writer.writerow(["point_number", "timestamp", "camera_index", "image_file", "J1", "J2", "J3", "J4", "J5", "J6", "G"])

        self.point_capture_count = 0

        # Create a separate image folder for every active camera.
        for camera_index in self.camera_workers:
            os.makedirs(os.path.join(session_dir, f"camera_{camera_index}"), exist_ok=True)

        self.log(f"Point camera recording folder: {session_dir}")

    def stop_point_camera_recording(self):
            """Flush and close the point-teaching camera reference CSV safely."""

            recording_file = self.point_recording_file
            try:
                if recording_file is not None and not recording_file.closed:
                    recording_file.flush()
                    recording_file.close()
            except Exception as e:
                self.log(f"Error closing point camera recording: {e}")
            finally:
                self.point_recording_file = None
                self.point_recording_writer = None
                self.log("Point camera recording stopped.")

    def save_point_camera_reference(self, q):
        """Save one snapshot from every active camera for a captured point."""

        if self.point_recording_writer is None:
            return

        if not self.latest_camera_images:
            self.log("No camera frames available for this point.")
            return

        self.point_capture_count += 1
        point_number = self.point_capture_count

        timestamp = datetime.now().isoformat(timespec="milliseconds")

        angles = list(q[:7])

        while len(angles) < 7:
            angles.append("")

        saved_count = 0

        for camera_index, qt_image in list(self.latest_camera_images.items()):
            if qt_image is None:
                continue

            camera_folder = os.path.join(self.point_recording_dir, f"camera_{camera_index}")

            os.makedirs(camera_folder, exist_ok=True)

            filename = f"point_{point_number:03d}.png"
            image_path = os.path.join(camera_folder, filename)

            if not qt_image.save(image_path, "PNG"):
                self.log(f"Failed to save camera {camera_index}: {image_path}")
                continue

            # Store a relative path so the session folder can be moved.
            relative_path = os.path.relpath(image_path, self.point_recording_dir)

            self.point_recording_writer.writerow([point_number, timestamp, camera_index, relative_path, *angles])

            saved_count += 1

        self.point_recording_file.flush()

        self.log(f"Saved point {point_number} from {saved_count} camera(s).")

    def start_CT_camera_recording(self):
        """Start recording frames from every active camera."""

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        session_dir = os.path.join(self.get_default_path(), f"trajectory_session_{timestamp}")
        os.makedirs(session_dir, exist_ok=True)

        self.CT_session_dir = session_dir
        self.point_recording_dir = session_dir

        self.CT_camera_images_dir = os.path.join(session_dir, "camera_images")
        os.makedirs(self.CT_camera_images_dir, exist_ok=True)

        # Create a folder for each active camera.
        for camera_index in self.camera_workers:
            os.makedirs(os.path.join(self.CT_camera_images_dir, f"camera_{camera_index}"), exist_ok=True)

        self.point_recording_csv = os.path.join(session_dir, "camera_references.csv")

        self.point_recording_file = open(self.point_recording_csv, "w",newline="", encoding="utf-8")

        self.point_recording_writer = csv.writer(self.point_recording_file)

        self.point_recording_writer.writerow(["elapsed_time_s", "camera_index", "image_file"])

        # Each camera has its own frame counter.
        self.CT_camera_frame_counts = {camera_index: 0 for camera_index in self.camera_workers}

        self.CT_camera_start_time = time.perf_counter()

        self.CT_camera_timer.start(self.CT_camera_interval)

        self.log(f"Camera recording folder: {session_dir}")

    def stop_CT_camera_recording(self):
        """Stop trajectory frame capture and close its CSV safely."""

        if self.CT_camera_timer.isActive():
            self.CT_camera_timer.stop()

        recording_file = self.point_recording_file
        try:
            if recording_file is not None and not recording_file.closed:
                recording_file.flush()
                recording_file.close()
        except Exception as e:
            self.log(f"Error closing trajectory camera recording: {e}")
        finally:
            self.point_recording_file = None
            self.point_recording_writer = None
            self.CT_camera_frame_counts = {}
            self.log("Trajectory camera recording stopped.")
    
    def capture_CT_camera_frame(self):
        """Save the latest frame from every active camera."""

        if self.point_recording_writer is None:
            return

        elapsed_time = (time.perf_counter() - self.CT_camera_start_time)

        saved_count = 0

        for camera_index, qt_image in list(self.latest_camera_images.items()):
            if qt_image is None:
                continue

            camera_folder = os.path.join(self.CT_camera_images_dir, f"camera_{camera_index}")
            os.makedirs(camera_folder, exist_ok=True)

            frame_count = (self.CT_camera_frame_counts.get(camera_index, 0) + 1)

            filename = f"frame_{frame_count:06d}.png"
            image_path = os.path.join(camera_folder, filename)

            if not qt_image.save(image_path, "PNG"):
                self.log(f"Failed to save camera {camera_index}: {image_path}")
                continue

            self.CT_camera_frame_counts[camera_index] = frame_count

            relative_path = os.path.relpath(image_path, self.CT_session_dir)

            self.point_recording_writer.writerow([f"{elapsed_time:.4f}", camera_index, relative_path])

            saved_count += 1

        if saved_count:
            self.point_recording_file.flush()

    # ==================== 公共回调 ====================
    def on_angles_updated(self, q):
        if len(q) < ARM_DOF:
            return
        g_text = f"  G: {q[6]:.4f}" if len(q) >= TOTAL_DOF else "  G: 无反馈"
        self.angle_text.setText(f"J1: {q[0]:.4f}  J2: {q[1]:.4f}  J3: {q[2]:.4f}  " f"J4: {q[3]:.4f}  J5: {q[4]:.4f}  J6: {q[5]:.4f}" + g_text)

    def on_worker_error(self, text):
        self.warn_text("机械臂错误", text)
        self.is_path_recording = False
        self.is_path_playing = False
        self.is_point_teaching = False
        self.is_point_playing = False
        self.is_setting_zero = False
        self.is_toggling_enable = False
        self.tabs.setEnabled(True)
        self.connect_btn.setEnabled(True)
        self.set_zero_btn.setText(self.ui("设置0点"))
        self.set_zero_btn.setEnabled(self.is_connected)
        self.enable_btn.setText(self.ui("失能" if self.arm_enabled else "使能"))
        self.enable_btn.setEnabled(self.is_connected)
        self.reset_all_buttons_after_stop()

    def log(self, msg):
        ts = datetime.now().strftime("%H:%M:%S")
        log_message = f"[{ts}] {msg}"

        # Check whether the log widget has been created yet
        if hasattr(self, "log_text") and self.log_text is not None:
            self.log_text.append(log_message)
        else:
            print(log_message)

    def closeEvent(self, event):
        if self._allow_close:
            event.accept()
            return

        event.ignore()
        if self._closing_after_home:
            return

        self._closing_after_home = True
        self.log("关闭程序：正在回零并失能，请勿断电。")
        self.conn_status_raw = "关闭中..."
        self.conn_status.setText(self.ui_status(self.conn_status_raw))
        self.conn_status.setStyleSheet("color: orange;")
        self.connect_btn.setEnabled(False)
        self.refresh_btn.setEnabled(False)
        self.language_btn.setEnabled(False)
        self.set_zero_btn.setEnabled(False)
        self.enable_btn.setEnabled(False)
        self.tabs.setEnabled(False)
        self.stop_all_cameras()
        self.worker.request_shutdown()

    def on_worker_thread_finished(self):
        if self._closing_after_home:
            self._allow_close = True
            self.close()

class WrappedHeaderView(QHeaderView):
    def __init__(self, orientation, parent=None):
        super().__init__(orientation, parent)
        self.setDefaultAlignment(Qt.AlignCenter)
        self.setMinimumHeight(48)

    def paintSection(self, painter, rect, logicalIndex):
        if not rect.isValid():
            return

        painter.save()
        painter.fillRect(rect, self.palette().button())

        model = self.model()
        text = model.headerData(
            logicalIndex, self.orientation(), Qt.DisplayRole
        )

        from PyQt5.QtCore import QRect
        from PyQt5.QtGui import QFontMetrics

        text_rect = rect.adjusted(3, 2, -3, -2)
        painter.setPen(self.palette().buttonText().color())
        painter.setFont(self.font())

        painter.drawText(
            text_rect,
            Qt.AlignCenter | Qt.TextWordWrap,
            str(text or "")
        )
        painter.restore()

def main():
    app = QApplication(sys.argv)

    window = DragTeachWindow()
    window.show()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
