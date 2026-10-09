import os
import sys
from datetime import datetime
from typing import List

import serial.tools.list_ports
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QBrush, QColor, QFont
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

from point_io import PointAction, read_point_program_csv, save_point_program_csv
from robot_worker import RobotWorker
from trajectory_io import save_trajectory_csv


ARM_DOF = 6
TOTAL_DOF = 7


ZH_TO_EN = {
    "拖动示教控制器 - 夹爪力位混合版": "Drag Teaching Controller - Gripper Force-Position Mode",
    "机械臂拖动示教系统：J1~J6 + 夹爪G反馈/力位混合控制": "Robot Drag Teaching System: J1-J6 + Gripper G Feedback / Force-Position Control",
    "连续轨迹示教": "Continuous Trajectory",
    "点位示教": "Point Teaching",
    "串口连接": "Serial Connection",
    "串口:": "Port:",
    "刷新": "Refresh",
    "连接": "Connect",
    "连接中...": "Connecting...",
    "断开": "Disconnect",
    "断开中，正在回零...": "Disconnecting, returning home...",
    "关闭中，正在回零并失能...": "Closing, returning home and disabling...",
    "设置0点": "Set Zero",
    "设置0点中...": "Setting zero...",
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
    "G夹爪": "G Gripper",
    "六轴最大速度(rad/s)": "Arm Max Speed (rad/s)",
    "间隔(s)": "Pause (s)",
    "夹爪速度": "Gripper Speed",
    "夹爪力度": "Gripper Force",
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

        self.worker = RobotWorker()
        self.worker.log_msg.connect(self.log)
        self.worker.error_msg.connect(self.on_worker_error)
        self.worker.connection_changed.connect(self.on_connection_changed)
        self.worker.angles_updated.connect(self.on_angles_updated)
        self.worker.enable_state_changed.connect(self.on_enable_state_changed)
        self.worker.enable_toggle_finished.connect(self.on_enable_toggle_finished)
        self.worker.zero_set_finished.connect(self.on_zero_set_finished)

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

        self.init_ui()
        self.scan_serial_ports()

    # ==================== UI 构建 ====================
    def init_ui(self):
        self.setWindowTitle("拖动示教控制器 - 夹爪力位混合版")
        self.setGeometry(180, 80, 1180, 820)

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
        main_layout.addWidget(self.tabs)

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
        self.point_table.setHorizontalHeaderLabels(POINT_HEADERS_ZH)
        self.point_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.point_table.verticalHeader().setVisible(False)
        self.point_table.setAlternatingRowColors(True)
        self.point_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.point_table.setSelectionMode(QTableWidget.SingleSelection)
        table_layout.addWidget(self.point_table)

        hint = QLabel(
            "说明：同步速度只针对J1~J6。六轴最大速度是机械臂关节速度上限，程序按J1~J6角度差自动分配速度；夹爪G不参与同步，播放时通过control_pos_force按表格里的夹爪速度和力度单独执行，默认速度5。蓝色行表示当前动作。"
        )
        hint.setStyleSheet("color: #555;")
        table_layout.addWidget(hint)
        layout.addWidget(table_group)
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

    def warn(self, title_zh: str, message_zh: str):
        QMessageBox.warning(self, self.ui(title_zh), self.ui(message_zh))

    def warn_text(self, title_zh: str, message: str):
        QMessageBox.warning(self, self.ui(title_zh), message)

    def get_default_path(self):
        path = os.path.join(os.path.expanduser("~"), "Documents", "RobotTrajectories")
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
            self.connect_btn.setText(self.ui("断开中，正在回零..."))
            self.conn_status_raw = "断开中，正在回零..."
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
            filepath = self.build_path_save_filepath()
            try:
                save_trajectory_csv(filepath, rows)
                self.log(f"连续轨迹已保存: {filepath}")
            except Exception as e:
                self.warn_text("错误", f"保存失败:\n{e}")
                self.log(f"保存失败: {e}")

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

    # ==================== 公共回调 ====================
    def on_angles_updated(self, q):
        if len(q) < ARM_DOF:
            return
        g_text = f"  G: {q[6]:.4f}" if len(q) >= TOTAL_DOF else "  G: 无反馈"
        self.angle_text.setText(
            f"J1: {q[0]:.4f}  J2: {q[1]:.4f}  J3: {q[2]:.4f}  "
            f"J4: {q[3]:.4f}  J5: {q[4]:.4f}  J6: {q[5]:.4f}" + g_text
        )

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
        self.log_text.append(f"[{ts}] {msg}")

    def closeEvent(self, event):
        if self._allow_close:
            event.accept()
            return

        event.ignore()
        if self._closing_after_home:
            return

        self._closing_after_home = True
        self.log("关闭程序：正在回零并失能，请勿断电。")
        self.conn_status_raw = "关闭中，正在回零并失能..."
        self.conn_status.setText(self.ui_status(self.conn_status_raw))
        self.conn_status.setStyleSheet("color: orange;")
        self.connect_btn.setEnabled(False)
        self.refresh_btn.setEnabled(False)
        self.language_btn.setEnabled(False)
        self.set_zero_btn.setEnabled(False)
        self.enable_btn.setEnabled(False)
        self.tabs.setEnabled(False)
        self.worker.request_shutdown()

    def on_worker_thread_finished(self):
        if self._closing_after_home:
            self._allow_close = True
            self.close()


def main():
    app = QApplication(sys.argv)
    window = DragTeachWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
