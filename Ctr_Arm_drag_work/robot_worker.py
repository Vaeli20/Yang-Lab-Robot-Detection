import inspect
import importlib
import queue
import time
import traceback
from typing import List, Optional, Tuple, Any

from PyQt5.QtCore import QThread, pyqtSignal

from filters import VelocityFilter
from trajectory_io import read_trajectory_csv, TrajectoryPoint
from point_io import PointAction


ARM_DOF = 6
TOTAL_DOF = 7  # J1~J6 + G(夹爪反馈位置)


class RobotWorker(QThread):
    """
    机器人 IO 工作线程。

    设计要点：
    - RobotController 只在本线程中创建和访问，避免 UI 线程、定时器、播放线程抢同一串口/CAN。
    - J1~J6 使用机械臂位置/速度/MIT模式接口。
    - 夹爪不是第7轴 set_joint_angles 控制，而是独立调用：
          self.arm.RobotCtrl.control_pos_force(self.arm.gripper/Motor7, position, velocity, force_limit)
    - 夹爪模式不参与 MIT / 位置速度模式切换；示教阶段夹爪力位混合命令参数给 0。
    - 夹爪位置 G 仍然从 get_current_joint_angles() 的第7个反馈读取；如果该函数只返回6维，UI会明确提示无夹爪反馈。
    """

    log_msg = pyqtSignal(str)
    error_msg = pyqtSignal(str)

    connection_changed = pyqtSignal(bool, str)
    angles_updated = pyqtSignal(list)

    record_status = pyqtSignal(str)
    record_finished = pyqtSignal(list)

    traj_play_status = pyqtSignal(str)
    traj_play_finished = pyqtSignal()

    point_teach_status = pyqtSignal(str)
    point_captured = pyqtSignal(list, float, float, float, float)

    point_play_status = pyqtSignal(str)
    point_action_started = pyqtSignal(int)
    point_play_finished = pyqtSignal()

    enable_state_changed = pyqtSignal(bool)
    enable_toggle_finished = pyqtSignal(bool, bool, str)
    zero_set_finished = pyqtSignal(bool, str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._cmd_q = queue.Queue()
        self._running = True

        self.arm = None
        self.connected = False
        self.enabled = False
        self.current_mode: Optional[str] = None
        self.state = "idle"  # idle / path_record / path_play / point_teach / point_play / set_zero / set_enabled

        self.ui_period = 0.10
        self.next_ui_time = 0.0

        # 连续拖动示教
        self.record_rows: List[List[float]] = []
        self.record_t0 = 0.0
        self.record_period = 0.01
        self.gravity_period = 0.01
        self.next_record_time = 0.0
        self.next_gravity_time = 0.0
        self.prev_q: Optional[List[float]] = None
        self.prev_sample_time: Optional[float] = None
        self.velocity_filters = [VelocityFilter() for _ in range(TOTAL_DOF)]

        # 连续轨迹播放
        self.trajectory: List[TrajectoryPoint] = []
        self.play_speed = 1.0
        self.play_t0 = 0.0
        self.play_index = 0
        self.last_sent_play_index = -1
        self.path_gripper_velocity = 5.0
        self.path_gripper_force_limit = 0.1

        # 点位示教
        self.point_gravity_period = 0.01
        self.next_point_gravity_time = 0.0

        # 点位播放
        self.point_program: List[PointAction] = []
        self.point_index = -1
        self.point_next_time = 0.0
        self.point_last_q: Optional[List[float]] = None
        self.point_min_motion_time = 0.05
        self.point_speed_min = 0.01
        self.point_speed_max = 3.0
        self.gripper_speed_min = 0.0
        self.gripper_speed_max = 20.0
        self.axis_delta_eps = 1e-5

        # 断开/关闭前自动回零参数。
        # 只回 J1~J6；夹爪仍保持力位混合模式，不强制夹爪回零，避免夹持物体时误动作。
        self.home_target = [0.0] * ARM_DOF
        self.home_speed = 1.0          # rad/s，回零时最大关节速度
        self.home_speed_min = 0.05     # rad/s，角度很小时给一个最低速度
        self.home_tolerance = 0.03     # rad，判断到达零位的角度误差
        self.home_timeout = 12.0       # s，回零最长等待时间
        self.home_resend_period = 0.25 # s，回零过程中周期性重发目标

        # 模式切换参数：底层电机偶尔漏收时可改为3。
        self.mode_repeat = 2
        self.mode_settle_s = 0.015

        # 夹爪适配
        self._gripper_motor = None
        self._last_gripper_zero_time = 0.0
        self.gripper_zero_period = 0.05
        self._warned_gripper_motor = False
        self._warned_gripper_control = False
        self._warned_gripper_feedback = False
        self._warned_mode_fallback = False
        self._last_gripper_read_error = ""

    # ==================== UI线程调用：只入队 ====================
    def request_connect(self, port: str):
        self._cmd_q.put(("connect", port))

    def request_disconnect(self):
        self._cmd_q.put(("disconnect",))

    def request_shutdown(self):
        self._cmd_q.put(("shutdown",))

    def request_start_path_record(self, record_hz: int, gravity_hz: int):
        self._cmd_q.put(("start_path_record", int(record_hz), int(gravity_hz)))

    def request_stop_path_record(self):
        self._cmd_q.put(("stop_path_record",))

    def request_start_path_play(self, filepath: str, speed: float):
        self._cmd_q.put(("start_path_play", filepath, float(speed)))

    def request_stop_path_play(self):
        self._cmd_q.put(("stop_path_play",))

    def request_start_point_teach(self, gravity_hz: int):
        self._cmd_q.put(("start_point_teach", int(gravity_hz)))

    def request_stop_point_teach(self):
        self._cmd_q.put(("stop_point_teach",))

    def request_capture_point(self, default_speed: float, default_pause: float, gripper_speed: float, gripper_force_limit: float):
        self._cmd_q.put(("capture_point", float(default_speed), float(default_pause), float(gripper_speed), float(gripper_force_limit)))

    def request_start_point_play(self, actions: List[PointAction]):
        self._cmd_q.put(("start_point_play", actions))

    def request_stop_point_play(self):
        self._cmd_q.put(("stop_point_play",))

    def request_set_zero(self):
        self._cmd_q.put(("set_zero",))

    def request_set_enabled(self, enabled: bool):
        self._cmd_q.put(("set_enabled", bool(enabled)))

    # ==================== 主循环 ====================
    def run(self):
        while self._running:
            self._drain_commands()
            now = time.perf_counter()

            try:
                if self.connected:
                    if self.state == "path_record":
                        self._path_record_loop(now)
                    elif self.state == "path_play":
                        self._path_play_loop(now)
                    elif self.state == "point_teach":
                        self._point_teach_loop(now)
                    elif self.state == "point_play":
                        self._point_play_loop(now)
                    else:
                        self._idle_loop(now)
            except Exception as e:
                self.log_msg.emit(f"运行错误: {e}")

            time.sleep(0.001)

        self._disconnect(close_only=False)

    def _drain_commands(self):
        while True:
            try:
                cmd = self._cmd_q.get_nowait()
            except queue.Empty:
                return

            try:
                self._handle_command(cmd)
            except Exception as e:
                self.log_msg.emit(traceback.format_exc())
                self.error_msg.emit(str(e))

    def _handle_command(self, cmd):
        name = cmd[0]
        if name == "connect":
            self._connect(cmd[1])
        elif name == "disconnect":
            self._disconnect(close_only=False)
        elif name == "shutdown":
            self._running = False
        elif name == "start_path_record":
            self._start_path_record(record_hz=cmd[1], gravity_hz=cmd[2])
        elif name == "stop_path_record":
            self._stop_path_record()
        elif name == "start_path_play":
            self._start_path_play(filepath=cmd[1], speed=cmd[2])
        elif name == "stop_path_play":
            self._stop_path_play(user_stop=True)
        elif name == "start_point_teach":
            self._start_point_teach(gravity_hz=cmd[1])
        elif name == "stop_point_teach":
            self._stop_point_teach()
        elif name == "capture_point":
            self._capture_point(default_speed=cmd[1], default_pause=cmd[2], gripper_speed=cmd[3], gripper_force_limit=cmd[4])
        elif name == "start_point_play":
            self._start_point_play(actions=cmd[1])
        elif name == "stop_point_play":
            self._stop_point_play(user_stop=True)
        elif name == "set_zero":
            self._handle_set_zero_command()
        elif name == "set_enabled":
            self._handle_set_enabled_command(bool(cmd[1]))

    def _require_idle_or(self, allowed_state: str):
        if self.state not in ("idle", allowed_state):
            raise RuntimeError(f"当前正在执行 {self.state}，请先停止当前任务")

    def _handle_set_zero_command(self):
        try:
            self._set_zero_current_pose()
        except Exception as e:
            self.log_msg.emit(f"设置0点失败: {e}")
            self.zero_set_finished.emit(False, str(e))
        else:
            self.zero_set_finished.emit(True, "0点设置完成")

    def _handle_set_enabled_command(self, target_enabled: bool):
        action = "使能" if target_enabled else "失能"
        try:
            self._set_motor_enabled(target_enabled)
        except Exception as e:
            self.log_msg.emit(f"{action}失败: {e}")
            self.enable_state_changed.emit(bool(self.enabled))
            self.enable_toggle_finished.emit(False, bool(self.enabled), str(e))
        else:
            message = "机械臂已使能" if self.enabled else "机械臂已失能"
            self.log_msg.emit(message)
            self.enable_state_changed.emit(bool(self.enabled))
            self.enable_toggle_finished.emit(True, bool(self.enabled), message)

    def _set_motor_enabled(self, target_enabled: bool):
        if self.arm is None or not self.connected:
            raise RuntimeError("请先连接机械臂")
        if self.state != "idle":
            raise RuntimeError(f"当前正在执行 {self.state}，请先停止当前任务")

        self.state = "set_enabled"
        try:
            if target_enabled:
                if not self.enabled:
                    self.log_msg.emit("正在使能电机...")
                    self._enable_arm_joints_once()
                self._enable_gripper_once()
                self._ensure_gripper_torque_pos_mode_once()
                self._hold_current_position()
            else:
                if not self.enabled:
                    self.current_mode = None
                    self.enable_state_changed.emit(False)
                    return
                self.log_msg.emit("正在失能电机...")
                self._disable_all_motors()
        finally:
            if self.state == "set_enabled":
                self.state = "idle"

    def _set_zero_current_pose(self):
        if self.arm is None or not self.connected:
            raise RuntimeError("请先连接机械臂")
        if self.state != "idle":
            raise RuntimeError(f"当前正在执行 {self.state}，请先停止当前任务")

        self.state = "set_zero"
        set_zero_started = False
        was_enabled = bool(self.enabled)
        ok = False
        try:
            q_before = self._safe_get_angles()
            if q_before is not None and len(q_before) >= ARM_DOF:
                self.log_msg.emit(f"设置0点前反馈：J1~J6 = {[round(float(x), 4) for x in q_before[:ARM_DOF]]}")

            set_zero = getattr(self.arm, "set_zero", None)
            if not callable(set_zero):
                raise RuntimeError("ArmDriver中没有可用的 set_zero() 接口")

            self.log_msg.emit("正在将当前位置设置为0点，请勿移动机械臂或断电...")
            set_zero_started = True
            ok = bool(set_zero())
        finally:
            if set_zero_started and self.arm is not None and self.connected:
                self.enabled = False
                self.current_mode = None
                if was_enabled:
                    try:
                        self._enable_arm_joints_once()
                        self._enable_gripper_once()
                        self._ensure_gripper_torque_pos_mode_once()
                        self._hold_current_position()
                    except Exception as e:
                        self.log_msg.emit(f"设置0点后恢复保持失败: {e}")
                self.enable_state_changed.emit(bool(self.enabled))
            if self.state == "set_zero":
                self.state = "idle"

        q_after = self._safe_get_angles()
        if q_after is not None and len(q_after) >= ARM_DOF:
            self.angles_updated.emit(self._pad_q_to_match(q_after))
            self.log_msg.emit(f"设置0点后反馈：J1~J6 = {[round(float(x), 4) for x in q_after[:ARM_DOF]]}")

        if not ok:
            raise RuntimeError("0点设置失败：电机反馈未确认归零，请检查机械臂状态后重试")
        self.log_msg.emit("0点设置完成")

    # ==================== 连接 / 模式 ====================
    def _connect(self, port: str):
        if self.connected:
            self._disconnect(close_only=False)

        self.log_msg.emit(f"正在连接 {port} ...")
        from ArmDriver import RobotController

        self.arm = RobotController(port=port, type="Grivity_arm")
        self.connected = True
        self.enabled = False
        self.current_mode = None
        self.state = "idle"

        self._gripper_motor = None
        self._warned_gripper_motor = False
        self._warned_gripper_control = False
        self._warned_gripper_feedback = False
        self._warned_mode_fallback = False
        self._last_gripper_read_error = ""

        self._prepare_gripper_motor()
        self._enable_arm_joints_once()
        self._enable_gripper_once()
        self._ensure_gripper_torque_pos_mode_once()

        q_raw = None
        try:
            q_raw = list(self.arm.get_current_joint_angles())
            self.log_msg.emit(f"关节反馈维度诊断：get_current_joint_angles() 返回 {len(q_raw)} 维：{[round(float(x), 4) for x in q_raw]}")
        except Exception as e:
            self.log_msg.emit(f"首次读取关节角失败: {e}")

        q = self._safe_get_angles(raw=q_raw)
        if q is not None:
            self.angles_updated.emit(q)
            if len(q) >= TOTAL_DOF:
                self.log_msg.emit("检测到7维反馈：J1~J6 + 夹爪G，拖动示教会自动记录夹爪位置。")
            else:
                self.log_msg.emit("警告：当前反馈只有6维。请确认 ArmDriver.get_current_joint_angles() 末尾是否返回夹爪角度G。")

        self.connection_changed.emit(True, port)
        self.log_msg.emit("机械臂连接成功")

    def _disconnect(self, close_only: bool):
        """
        断开连接。
        close_only=True：只关闭串口，不做回零/失能。
        close_only=False：先停止当前任务，再J1~J6回零，最后失能并关闭串口。
        """
        if self.arm is not None and self.connected and not close_only:
            self._stop_active_task_before_disconnect()
            self._return_home_before_disable()

        self.state = "idle"
        self.trajectory = []
        self.point_program = []

        if self.arm is not None:
            try:
                if self.enabled and not close_only:
                    self.log_msg.emit("正在失能电机...")
                    self.arm.disable()
                    time.sleep(0.02)
            except Exception as e:
                self.log_msg.emit(f"disable 失败: {e}")

            try:
                self.arm.close_serial()
            except Exception as e:
                self.log_msg.emit(f"关闭串口失败: {e}")

        self.arm = None
        self.connected = False
        self.enabled = False
        self.current_mode = None
        self._gripper_motor = None
        self.enable_state_changed.emit(False)
        self.connection_changed.emit(False, "")
        self.log_msg.emit("已断开连接")

    def _stop_active_task_before_disconnect(self):
        """断开/关闭前先把当前示教或播放任务收尾，避免边播放边回零。"""
        try:
            if self.state == "path_record":
                self._stop_path_record()
            elif self.state == "path_play":
                self._stop_path_play(user_stop=True)
            elif self.state == "point_teach":
                self._stop_point_teach()
            elif self.state == "point_play":
                self._stop_point_play(user_stop=True)
        except Exception as e:
            self.log_msg.emit(f"停止当前任务失败，继续执行回零断开: {e}")
            self.state = "idle"

    def _calc_sync_speeds_6dof(self, q_from: List[float], q_target: List[float], max_speed: float) -> Tuple[List[float], float]:
        """按最大关节速度计算J1~J6同步到达速度。"""
        q_from = [float(x) for x in list(q_from)[:ARM_DOF]]
        q_target = [float(x) for x in list(q_target)[:ARM_DOF]]
        max_speed = max(self.home_speed_min, float(max_speed))
        deltas = [abs(q_target[i] - q_from[i]) for i in range(ARM_DOF)]
        max_delta = max(deltas) if deltas else 0.0
        if max_delta < self.axis_delta_eps:
            return [self.home_speed_min] * ARM_DOF, 0.0
        motion_time = max(0.2, max_delta / max_speed)
        speeds = []
        for d in deltas:
            if d < self.axis_delta_eps:
                speeds.append(self.home_speed_min)
            else:
                speeds.append(min(max(d / motion_time, self.home_speed_min), max_speed))
        return speeds, motion_time

    def _return_home_before_disable(self):
        """断开/关闭前，先让J1~J6回到零位，再允许失能。夹爪不参与回零。"""
        if self.arm is None or not self.connected or not self.enabled:
            return

        try:
            q_now = self._safe_get_angles()
            if q_now is None or len(q_now) < ARM_DOF:
                self.log_msg.emit("回零跳过：无法读取当前J1~J6角度")
                return

            q6_now = [float(x) for x in q_now[:ARM_DOF]]
            target = list(self.home_target)
            speeds, estimated_time = self._calc_sync_speeds_6dof(q6_now, target, self.home_speed)

            self.log_msg.emit(
                f"断开/关闭前自动回零：目标 J1~J6 = {[round(x, 3) for x in target]}，"
                f"速度 = {[round(v, 3) for v in speeds]}"
            )

            self._ensure_mode("pos_vel")
            self._set_arm_joint_angles(target, v=speeds)

            t0 = time.perf_counter()
            deadline = t0 + min(self.home_timeout, max(estimated_time + 1.0, 1.0))
            next_resend = t0 + self.home_resend_period
            stable_count = 0

            while time.perf_counter() < deadline:
                now = time.perf_counter()
                q_check = self._safe_get_angles()
                if q_check is not None and len(q_check) >= ARM_DOF:
                    err = max(abs(float(q_check[i]) - target[i]) for i in range(ARM_DOF))
                    self.angles_updated.emit(self._pad_q_to_match(q_check))
                    if err <= self.home_tolerance:
                        stable_count += 1
                        if stable_count >= 3:
                            break
                    else:
                        stable_count = 0

                if now >= next_resend:
                    self._set_arm_joint_angles(target, v=speeds)
                    next_resend = now + self.home_resend_period

                time.sleep(0.02)

            self._set_arm_joint_angles(target, v=[self.home_speed_min] * ARM_DOF)
            time.sleep(0.15)
            self.log_msg.emit("J1~J6 已执行回零，准备失能。")
        except Exception as e:
            self.log_msg.emit(f"回零失败，将继续失能断开: {e}")

    def _enable_arm_joints_once(self):
        """只启用 J1~J6；夹爪由 _enable_gripper_once 单独启用。"""
        if self.enabled or self.arm is None:
            return

        joints = list(getattr(self.arm, "joints", []) or [])
        ctrl = self._get_motor_control()
        if joints and ctrl is not None and hasattr(ctrl, "enable"):
            try:
                for motor in joints[:ARM_DOF]:
                    ctrl.enable(motor)
                    time.sleep(0.003)
                self.enabled = True
                self.enable_state_changed.emit(True)
                time.sleep(0.02)
                self.log_msg.emit("J1~J6 已启用")
                return
            except Exception as e:
                self.log_msg.emit(f"按 self.arm.joints 启用J1~J6失败，尝试 arm.enable(): {e}")

        try:
            self.arm.enable()
            self.enabled = True
            self.enable_state_changed.emit(True)
            time.sleep(0.02)
            self.log_msg.emit("提示：使用 arm.enable() 启用电机。")
        except Exception:
            all_motors = list(getattr(self.arm, "joints", []) or [])
            ctrl = self._get_motor_control()
            if ctrl is None:
                raise
            for motor in all_motors[:ARM_DOF]:
                ctrl.enable(motor)
                time.sleep(0.003)
            self.enabled = True
            self.enable_state_changed.emit(True)
            time.sleep(0.02)

    def _enable_gripper_once(self):
        """单独启用夹爪电机，不通过 arm.enable()，避免影响J1~J6。"""
        if self.arm is None:
            return
        motor = self._gripper_motor or self._prepare_gripper_motor()
        ctrl = self._get_motor_control()
        if motor is None or ctrl is None:
            return
        try:
            if hasattr(ctrl, "enable"):
                ctrl.enable(motor)
                time.sleep(0.01)
            refresh = getattr(ctrl, "refresh_motor_status", None)
            if callable(refresh):
                refresh(motor)
            self.log_msg.emit("夹爪电机已单独使能")
        except Exception as e:
            self.log_msg.emit(f"夹爪使能失败: {e}")

    def _disable_all_motors(self):
        if self.arm is None:
            raise RuntimeError("机械臂未连接")

        disable = getattr(self.arm, "disable", None)
        if callable(disable):
            ok = bool(disable())
            if not ok:
                raise RuntimeError("电机失能反馈未确认，请检查机械臂状态")
        else:
            ctrl = self._get_motor_control()
            if ctrl is None or not hasattr(ctrl, "disable"):
                raise RuntimeError("ArmDriver中没有可用的 disable() 接口")

            for motor in list(getattr(self.arm, "joints", []) or [])[:ARM_DOF]:
                ctrl.disable(motor)
                time.sleep(0.003)

            gripper = self._gripper_motor or self._prepare_gripper_motor()
            if gripper is not None:
                ctrl.disable(gripper)
                time.sleep(0.003)

        self.enabled = False
        self.current_mode = None
        self.enable_state_changed.emit(False)
        time.sleep(0.02)

    def _get_armdriver_symbol(self, name: str):
        """从 ArmDriver 或 DM_CAN 命名空间获取 Control_Type / DM_variable 等符号。"""
        for module_name in ("ArmDriver", "DM_CAN"):
            try:
                module = importlib.import_module(module_name)
                value = getattr(module, name, None)
                if value is not None:
                    return value
            except Exception:
                pass
        return None

    def _get_control_type_value(self, mode: str):
        control_type = self._get_armdriver_symbol("Control_Type")
        if control_type is None:
            return None
        if mode == "mit":
            return getattr(control_type, "MIT", None)
        if mode == "pos_vel":
            return getattr(control_type, "POS_VEL", None)
        if mode in ("torque_pos", "force_pos"):
            return getattr(control_type, "Torque_Pos", None)
        return None

    def _get_motor_control(self):
        """兼容你的 ArmDriver：控制对象实际叫 RobotCtrl，不是 MotorControl。"""
        if self.arm is None:
            return None
        return (
            getattr(self.arm, "RobotCtrl", None)
            or getattr(self.arm, "MotorControl", None)
            or getattr(self.arm, "motor_control", None)
        )

    def _set_single_motor_mode(self, motor, mode: str) -> bool:
        """只设置单个电机模式，用于J1~J6，避免调用会切换夹爪的 set_mit_mode/set_pos_vel_mode。"""
        mode_value = self._get_control_type_value(mode)
        if mode_value is None:
            return False

        set_motor_mode = getattr(self.arm, "set_motor_mode", None)
        if callable(set_motor_mode):
            return bool(set_motor_mode(motor, mode_value))

        ctrl = self._get_motor_control()
        dm_variable = self._get_armdriver_symbol("DM_variable")
        if ctrl is None or dm_variable is None:
            return False
        ctrl_mode = getattr(dm_variable, "CTRL_MODE", None)
        if ctrl_mode is None:
            return False

        change = getattr(ctrl, "change_motor_param", None)
        refresh = getattr(ctrl, "refresh_motor_status", None)
        read = getattr(ctrl, "read_motor_param", None)
        if not callable(change):
            return False
        try:
            change(motor, ctrl_mode, mode_value)
            if callable(refresh):
                refresh(motor)
            if callable(read):
                return read(motor, ctrl_mode) == mode_value
            return True
        except Exception:
            return False

    def _ensure_gripper_torque_pos_mode_once(self):
        """夹爪初始化时确保一次力位混合模式；之后J1~J6模式切换不再碰夹爪。"""
        if self.arm is None:
            return
        motor = self._gripper_motor or self._prepare_gripper_motor()
        if motor is None:
            return
        ok = self._set_single_motor_mode(motor, "torque_pos")
        if ok:
            self.log_msg.emit("夹爪模式：已设置/保持为 Torque_Pos 力位混合模式")
        else:
            self.log_msg.emit("提示：未能自动确认夹爪 Torque_Pos 模式；若夹爪不动作，请检查 ArmDriver 中 Motor7/gripper 的控制模式")

    def _ensure_mode(self, mode: str):
        if self.arm is None:
            raise RuntimeError("机械臂未连接")

        self._enable_arm_joints_once()
        if self.current_mode == mode:
            return

        for _ in range(self.mode_repeat):
            self._switch_arm_mode_only(mode)
            time.sleep(self.mode_settle_s)

        self.current_mode = mode

    def _switch_arm_mode_only(self, mode: str):
        """
        尽量只切换 J1~J6 的模式，不碰夹爪 Motor7。
        如果你的 ArmDriver 里 set_mit_mode()/set_pos_vel_mode() 内部遍历了 gripper/Motor7，
        请在 ArmDriver 中把它改成只遍历 self.joints。
        """
        if mode not in ("mit", "pos_vel"):
            raise ValueError(f"未知模式: {mode}")

        # 优先调用用户可能提供的“只切机械臂”方法。
        candidates = []
        if mode == "mit":
            candidates = ["set_arm_mit_mode", "set_joints_mit_mode", "set_mit_mode_for_joints", "set_mit_mode_joints_only"]
        else:
            candidates = ["set_arm_pos_vel_mode", "set_joints_pos_vel_mode", "set_pos_vel_mode_for_joints", "set_pos_vel_mode_joints_only"]

        for name in candidates:
            func = getattr(self.arm, name, None)
            if callable(func):
                self._call_mode_func(func)
                return

        # 优先用 ArmDriver.set_motor_mode 对 J1~J6 逐个设置，绝不调用会切换夹爪的 set_mit_mode/set_pos_vel_mode。
        joints = list(getattr(self.arm, "joints", []) or [])[:ARM_DOF]
        if joints:
            ok_all = True
            for i, joint in enumerate(joints, start=1):
                ok = self._set_single_motor_mode(joint, mode)
                if not ok:
                    ok_all = False
                    self.log_msg.emit(f"J{i} 单独切换 {mode} 模式失败")
                    break
            if ok_all:
                return

        # 最后才回退到原有接口；你的ArmDriver里这两个函数会切夹爪，所以通常不应走到这里。
        fallback = getattr(self.arm, "set_mit_mode" if mode == "mit" else "set_pos_vel_mode", None)
        if callable(fallback):
            if not self._warned_mode_fallback:
                self._warned_mode_fallback = True
                self.log_msg.emit("警告：未能逐个切换J1~J6，暂用 ArmDriver 原 set_mit_mode/set_pos_vel_mode；该函数可能会切换夹爪模式。")
            self._call_mode_func(fallback)
            # 回退后再把夹爪恢复为力位混合模式。
            self._ensure_gripper_torque_pos_mode_once()
            return

        raise RuntimeError(f"ArmDriver中没有可用的 {mode} 模式切换函数")

    def _call_mode_func(self, func):
        try:
            sig = inspect.signature(func)
            # 绑定方法不包含self。若需要一个参数，优先传J1~J6电机列表。
            if len(sig.parameters) >= 1:
                func(list(getattr(self.arm, "joints", []) or [])[:ARM_DOF])
            else:
                func()
        except (TypeError, ValueError):
            # 某些C扩展/动态函数拿不到签名，先尝试无参，再尝试传joints。
            try:
                func()
            except TypeError:
                func(list(getattr(self.arm, "joints", []) or [])[:ARM_DOF])

    def _hold_current_position(self):
        if self.arm is None or not self.connected:
            return
        try:
            q = self._safe_get_angles()
            self._ensure_mode("pos_vel")
            if q is not None and len(q) >= ARM_DOF:
                self._set_arm_joint_angles(q[:ARM_DOF], v=[0.05] * ARM_DOF)
                self.angles_updated.emit(q)
        except Exception as e:
            self.log_msg.emit(f"当前位置保持失败: {e}")

    # ==================== 夹爪适配 ====================
    def _prepare_gripper_motor(self):
        """查找或尝试创建 Motor7。优先使用 ArmDriver 中已定义的 self.gripper / self.Motor7。"""
        if self.arm is None:
            return None

        for name in ("Motor7", "motor7", "gripper", "Gripper", "gripper_motor"):
            motor = getattr(self.arm, name, None)
            if motor is not None:
                self._gripper_motor = motor
                self.log_msg.emit(f"夹爪电机对象：使用 self.arm.{name}")
                return motor

        # 如果没有 Motor7，则尝试用 Motor1 的同类构造一个 Motor7。
        sample = None
        for name in ("Motor1", "motor1"):
            sample = getattr(self.arm, name, None)
            if sample is not None:
                break
        if sample is None:
            joints = list(getattr(self.arm, "joints", []) or [])
            if joints:
                sample = joints[0]

        if sample is not None:
            cls = type(sample)
            for args in ((0x07, 0x17), (7, 0x17), (0x07,), (7,)):
                try:
                    motor = cls(*args)
                    setattr(self.arm, "Motor7", motor)
                    self._gripper_motor = motor
                    self.log_msg.emit(f"夹爪电机对象：已用 {cls.__name__}{args} 创建 self.arm.Motor7")
                    return motor
                except Exception:
                    pass

        if not self._warned_gripper_motor:
            self._warned_gripper_motor = True
            self.log_msg.emit("警告：没有找到 self.arm.Motor7，也无法自动创建。请在 ArmDriver.RobotController 中定义 Motor7，ID为 0x07,0x17。")
        return None

    def _send_gripper_control(self, physical_position: float, velocity: float, force_limit: float, *, suppress_errors: bool = False) -> bool:
        """调用夹爪力位混合：RobotCtrl.control_pos_force(gripper, pos, vel, force)。"""
        if self.arm is None:
            return False

        motor = self._gripper_motor or self._prepare_gripper_motor()
        ctrl = self._get_motor_control()
        func = getattr(ctrl, "control_pos_force", None) if ctrl is not None else None

        if motor is None or not callable(func):
            if not self._warned_gripper_control and not suppress_errors:
                self._warned_gripper_control = True
                self.log_msg.emit("警告：无法调用夹爪 control_pos_force。已检查 self.arm.RobotCtrl / self.arm.MotorControl 和 self.arm.gripper/Motor7。")
            return False

        try:
            func(motor, float(physical_position), float(velocity), float(force_limit))
            return True
        except Exception as e:
            if not suppress_errors:
                self.log_msg.emit(f"夹爪control_pos_force失败: {e}")
            return False

    def _send_gripper_teach_zero(self, now: Optional[float] = None):
        """示教/拖动阶段保持夹爪力位混合模式命令为0参数。"""
        now = time.perf_counter() if now is None else now
        if now - self._last_gripper_zero_time < self.gripper_zero_period:
            return
        self._last_gripper_zero_time = now
        self._send_gripper_control(0.0, 0.0, 0.0, suppress_errors=True)

    def _read_gripper_position_fallback(self) -> Optional[float]:
        """
        当 get_current_joint_angles() 只返回6维时，尝试从常见夹爪读取接口补第7维。
        如果你的驱动没有这些接口，建议直接在 get_current_joint_angles() 末尾返回夹爪位置。
        """
        if self.arm is None:
            return None

        for name in ("get_current_gripper_angles", "get_current_gripper_angle", "get_gripper_position", "read_gripper_position", "get_gripper_angle"):
            func = getattr(self.arm, name, None)
            if callable(func):
                try:
                    return float(func())
                except Exception as e:
                    self._last_gripper_read_error = f"{name}: {e}"

        motor = self._gripper_motor or self._prepare_gripper_motor()
        if motor is None:
            return None

        # 刷新状态后从电机对象字段读。
        for ctrl_name in ("MotorControl", "RobotCtrl"):
            ctrl = getattr(self.arm, ctrl_name, None)
            if ctrl is None:
                continue
            refresh = getattr(ctrl, "refresh_motor_status", None)
            if callable(refresh):
                try:
                    refresh(motor)
                    break
                except Exception:
                    pass

        for attr in ("position", "pos", "q", "angle", "physical_position", "Position", "Pos"):
            if hasattr(motor, attr):
                try:
                    return float(getattr(motor, attr))
                except Exception:
                    pass

        for name in ("get_position", "getPosition", "read_position", "readPosition"):
            func = getattr(motor, name, None)
            if callable(func):
                try:
                    return float(func())
                except Exception:
                    pass

        return None

    # ==================== 关节角兼容工具 ====================
    def _safe_get_angles(self, raw: Optional[List[float]] = None) -> Optional[List[float]]:
        try:
            q = list(raw) if raw is not None else list(self.arm.get_current_joint_angles())
            if len(q) >= TOTAL_DOF:
                return [float(x) for x in q[:TOTAL_DOF]]
            if len(q) >= ARM_DOF:
                g = self._read_gripper_position_fallback()
                if g is not None:
                    return [float(x) for x in q[:ARM_DOF]] + [float(g)]
                if not self._warned_gripper_feedback:
                    self._warned_gripper_feedback = True
                    extra = f"；尝试补读失败：{self._last_gripper_read_error}" if self._last_gripper_read_error else ""
                    self.log_msg.emit("提示：当前只能读到J1~J6，读不到夹爪G。实时显示G不会变化" + extra)
                return [float(x) for x in q[:ARM_DOF]]
            return [float(x) for x in q]
        except Exception as e:
            self.log_msg.emit(f"读取关节角失败: {e}")
            return None

    def _pad_q_to_match(self, q: List[float], reference: Optional[List[float]] = None) -> List[float]:
        q = list(q)
        if len(q) >= TOTAL_DOF:
            return [float(x) for x in q[:TOTAL_DOF]]
        if len(q) >= ARM_DOF:
            g = 0.0
            if reference is not None and len(reference) >= TOTAL_DOF:
                g = float(reference[6])
            return [float(x) for x in q[:ARM_DOF]] + [g]
        return [float(x) for x in q]

    def _set_arm_joint_angles(self, q6: List[float], v: Optional[List[float]] = None):
        """只发送J1~J6，不把夹爪塞进 set_joint_angles。"""
        if self.arm is None:
            raise RuntimeError("机械臂未连接")

        q6 = [float(x) for x in list(q6)[:ARM_DOF]]
        if len(q6) < ARM_DOF:
            raise RuntimeError("机械臂目标角度不足6维")
        v6 = [float(x) for x in list(v)[:ARM_DOF]] if v is not None else None

        if v6 is None:
            self.arm.set_joint_angles(q6)
        else:
            self.arm.set_joint_angles(q6, v=v6)

    # ==================== 连续拖动示教 ====================
    def _start_path_record(self, record_hz: int, gravity_hz: int):
        if not self.connected:
            raise RuntimeError("请先连接机械臂")
        self._require_idle_or("path_record")

        record_hz = max(10, min(200, record_hz))
        gravity_hz = max(20, min(500, gravity_hz))

        self.record_status.emit("状态: 切换到MIT重力补偿模式...")
        self._ensure_mode("mit")
        self._send_gripper_teach_zero()

        self.record_rows = []
        self.record_period = 1.0 / record_hz
        self.gravity_period = 1.0 / gravity_hz
        self.record_t0 = time.perf_counter()
        self.next_record_time = self.record_t0
        self.next_gravity_time = self.record_t0
        self.prev_q = None
        self.prev_sample_time = None
        self.velocity_filters = [VelocityFilter() for _ in range(TOTAL_DOF)]

        self.state = "path_record"
        self.record_status.emit("状态: 记录中...")
        self.log_msg.emit(f"开始连续拖动示教：记录 {record_hz} Hz，重力补偿 {gravity_hz} Hz。夹爪力位混合命令参数为0，夹爪位置从反馈G记录。")

    def _path_record_loop(self, now: float):
        if now >= self.next_gravity_time:
            try:
                self.arm.gravity_compensation()
                self._send_gripper_teach_zero(now)
            finally:
                self.next_gravity_time += self.gravity_period
                if self.next_gravity_time < now - self.gravity_period:
                    self.next_gravity_time = now + self.gravity_period

        if now >= self.next_record_time:
            q_read = self._safe_get_angles()
            if q_read is not None and len(q_read) >= ARM_DOF:
                q = self._pad_q_to_match(q_read, self.prev_q)
                t = now - self.record_t0

                if self.prev_q is not None and self.prev_sample_time is not None:
                    dt = max(1e-6, now - self.prev_sample_time)
                    raw_dq = [(q[i] - self.prev_q[i]) / dt for i in range(len(q))]
                else:
                    raw_dq = [0.0] * len(q)

                if len(q) < TOTAL_DOF:
                    q = q[:ARM_DOF] + [0.0]
                    raw_dq = raw_dq[:ARM_DOF] + [0.0]

                dq = [self.velocity_filters[i].filter(raw_dq[i]) for i in range(TOTAL_DOF)]
                self.record_rows.append([t] + q[:TOTAL_DOF] + dq[:TOTAL_DOF])

                self.prev_q = q[:TOTAL_DOF]
                self.prev_sample_time = now

                if now >= self.next_ui_time:
                    self.angles_updated.emit(q[:TOTAL_DOF])
                    self.next_ui_time = now + self.ui_period

            self.next_record_time += self.record_period
            if self.next_record_time < now - self.record_period:
                self.next_record_time = now + self.record_period

    def _stop_path_record(self):
        if self.state != "path_record":
            return
        self.state = "idle"
        rows = list(self.record_rows)
        self.record_status.emit(f"状态: 已记录 {len(rows)} 点")
        self.record_finished.emit(rows)
        self.log_msg.emit(f"连续拖动示教停止，共 {len(rows)} 点")
        self._hold_current_position()

    # ==================== 连续轨迹播放 ====================
    def _start_path_play(self, filepath: str, speed: float):
        if not self.connected:
            raise RuntimeError("请先连接机械臂")
        self._require_idle_or("path_play")

        points = read_trajectory_csv(filepath)
        if not points:
            raise RuntimeError("轨迹文件为空或格式不正确")

        current_q = self._safe_get_angles()
        for p in points:
            p.q = self._pad_q_to_match(p.q, current_q)
            if p.dq is not None:
                if len(p.dq) < TOTAL_DOF:
                    p.dq = list(p.dq[:ARM_DOF]) + [0.0]
                else:
                    p.dq = list(p.dq[:TOTAL_DOF])

        self.traj_play_status.emit("状态: 切换到位置速度模式...")
        self._ensure_mode("pos_vel")

        self.trajectory = points
        self.play_speed = float(max(0.1, min(1.0, speed)))
        self.play_t0 = time.perf_counter()
        self.play_index = 0
        self.last_sent_play_index = -1
        self.state = "path_play"

        self.traj_play_status.emit("状态: 播放中...")
        self.log_msg.emit(f"开始连续轨迹播放：{len(points)} 点，速度 {self.play_speed:g}x。J1~J6走set_joint_angles，夹爪G走control_pos_force。")

    def _path_play_loop(self, now: float):
        if not self.trajectory:
            self._stop_path_play(user_stop=False)
            return

        elapsed = (now - self.play_t0) * self.play_speed
        target_index = self.play_index
        while target_index + 1 < len(self.trajectory) and self.trajectory[target_index + 1].t <= elapsed:
            target_index += 1

        if target_index != self.last_sent_play_index:
            p = self.trajectory[target_index]
            v6 = self._path_play_velocity_for_point(target_index)
            self._set_arm_joint_angles(p.q[:ARM_DOF], v=v6)
            if len(p.q) >= TOTAL_DOF:
                self._send_gripper_control(p.q[6], self.path_gripper_velocity, self.path_gripper_force_limit)
            self.last_sent_play_index = target_index
            self.play_index = target_index

            if now >= self.next_ui_time:
                self.angles_updated.emit(list(p.q[:TOTAL_DOF]))
                self.next_ui_time = now + self.ui_period

        if elapsed >= self.trajectory[-1].t:
            final = self.trajectory[-1]
            self._set_arm_joint_angles(final.q[:ARM_DOF], v=[0.05] * ARM_DOF)
            if len(final.q) >= TOTAL_DOF:
                self._send_gripper_control(final.q[6], self.path_gripper_velocity, self.path_gripper_force_limit)
            self._stop_path_play(user_stop=False)

    def _path_play_velocity_for_point(self, index: int) -> List[float]:
        p = self.trajectory[index]
        if p.dq is not None:
            return [min(max(abs(v) * self.play_speed, 0.02), 2.0) for v in p.dq[:ARM_DOF]]

        if index + 1 < len(self.trajectory):
            p2 = self.trajectory[index + 1]
            dt = max(1e-3, (p2.t - p.t) / self.play_speed)
            return [min(max(abs(p2.q[i] - p.q[i]) / dt, 0.02), 2.0) for i in range(ARM_DOF)]

        return [0.05] * ARM_DOF

    def _stop_path_play(self, user_stop: bool):
        if self.state != "path_play":
            return
        self.state = "idle"
        self.traj_play_status.emit("状态: 已停止" if user_stop else "状态: 播放完成")
        self.log_msg.emit("连续轨迹播放已停止" if user_stop else "连续轨迹播放完成")
        self.traj_play_finished.emit()
        self._hold_current_position()

    # ==================== 点位示教 ====================
    def _start_point_teach(self, gravity_hz: int):
        if not self.connected:
            raise RuntimeError("请先连接机械臂")
        self._require_idle_or("point_teach")

        gravity_hz = max(20, min(500, gravity_hz))
        self.point_teach_status.emit("状态: 切换到MIT重力补偿模式...")
        self._ensure_mode("mit")
        self._send_gripper_teach_zero()

        now = time.perf_counter()
        self.point_gravity_period = 1.0 / gravity_hz
        self.next_point_gravity_time = now
        self.state = "point_teach"

        self.point_teach_status.emit("状态: 点位示教中，可拖动机械臂和夹爪并记录点位")
        self.log_msg.emit(f"开始点位示教：重力补偿 {gravity_hz} Hz。夹爪力位混合命令参数为0，夹爪位置从反馈G记录。")

    def _point_teach_loop(self, now: float):
        if now >= self.next_point_gravity_time:
            try:
                self.arm.gravity_compensation()
                self._send_gripper_teach_zero(now)
            finally:
                self.next_point_gravity_time += self.point_gravity_period
                if self.next_point_gravity_time < now - self.point_gravity_period:
                    self.next_point_gravity_time = now + self.point_gravity_period

        if now >= self.next_ui_time:
            q = self._safe_get_angles()
            if q is not None and len(q) >= ARM_DOF:
                self.angles_updated.emit(self._pad_q_to_match(q))
            self.next_ui_time = now + self.ui_period

    def _capture_point(self, default_speed: float, default_pause: float, gripper_speed: float, gripper_force_limit: float):
        if not self.connected:
            raise RuntimeError("请先连接机械臂")

        q = self._safe_get_angles()
        if q is None or len(q) < ARM_DOF:
            raise RuntimeError("读取当前关节角失败，无法记录点位")

        q = self._pad_q_to_match(q)
        default_speed = min(max(float(default_speed), self.point_speed_min), self.point_speed_max)
        default_pause = max(0.0, float(default_pause))
        gripper_speed = min(max(float(gripper_speed), self.gripper_speed_min), self.gripper_speed_max)
        gripper_force_limit = max(0.0, min(1.0, float(gripper_force_limit)))
        self.point_captured.emit(q[:TOTAL_DOF], default_speed, default_pause, gripper_speed, gripper_force_limit)
        self.log_msg.emit("已记录当前点位：J1~J6 + 夹爪G")

    def _stop_point_teach(self):
        if self.state != "point_teach":
            return
        self.state = "idle"
        self.point_teach_status.emit("状态: 点位示教已停止")
        self.log_msg.emit("点位示教已停止")
        self._hold_current_position()

    # ==================== 点位播放 ====================
    def _start_point_play(self, actions: List[PointAction]):
        if not self.connected:
            raise RuntimeError("请先连接机械臂")
        self._require_idle_or("point_play")
        if not actions:
            raise RuntimeError("点位程序为空")

        current_q = self._safe_get_angles()
        clean_actions: List[PointAction] = []
        for i, a in enumerate(actions, start=1):
            if len(a.q) < ARM_DOF:
                raise RuntimeError(f"第 {i} 行点位角度不足6个")
            q = self._pad_q_to_match(a.q, current_q)
            speed = min(max(float(a.speed), self.point_speed_min), self.point_speed_max)
            pause = max(0.0, float(a.pause))
            gripper_speed = min(max(float(a.gripper_speed), self.gripper_speed_min), self.gripper_speed_max)
            gripper_force_limit = max(0.0, min(1.0, float(a.gripper_force_limit)))
            clean_actions.append(PointAction(q=q[:TOTAL_DOF], speed=speed, pause=pause, gripper_speed=gripper_speed, gripper_force_limit=gripper_force_limit, note=a.note))

        self.point_play_status.emit("状态: 切换到位置速度模式...")
        self._ensure_mode("pos_vel")

        self.point_program = clean_actions
        self.point_index = -1
        self.point_next_time = time.perf_counter()
        self.point_last_q = self._pad_q_to_match(self._safe_get_angles() or clean_actions[0].q)
        self.state = "point_play"

        self.point_play_status.emit("状态: 点位播放中...")
        self.log_msg.emit(f"开始点位播放：{len(clean_actions)} 个动作。J1~J6同步速度分配，夹爪G独立control_pos_force。")

    def _calc_arm_sync_speeds(self, q_from: List[float], q_target: List[float], max_arm_speed: float) -> Tuple[List[float], float]:
        """只对 J1~J6 做同步速度分配，夹爪不参与。"""
        q_from = self._pad_q_to_match(q_from)
        q_target = self._pad_q_to_match(q_target, q_from)
        max_arm_speed = min(max(float(max_arm_speed), self.point_speed_min), self.point_speed_max)

        deltas = [abs(q_target[i] - q_from[i]) for i in range(ARM_DOF)]
        max_delta = max(deltas)

        if max_delta < self.axis_delta_eps:
            return [self.point_speed_min] * ARM_DOF, self.point_min_motion_time

        motion_time = max(self.point_min_motion_time, max_delta / max_arm_speed)
        speeds: List[float] = []
        for d in deltas:
            if d < self.axis_delta_eps:
                speeds.append(self.point_speed_min)
            else:
                speeds.append(min(max(d / motion_time, self.point_speed_min), max_arm_speed))
        return speeds, motion_time

    def _point_play_loop(self, now: float):
        if not self.point_program:
            self._stop_point_play(user_stop=False)
            return

        if now < self.point_next_time:
            if now >= self.next_ui_time:
                q = self._safe_get_angles()
                if q is not None:
                    self.angles_updated.emit(self._pad_q_to_match(q))
                self.next_ui_time = now + self.ui_period
            return

        next_index = self.point_index + 1
        if next_index >= len(self.point_program):
            self._stop_point_play(user_stop=False)
            return

        action = self.point_program[next_index]
        q_target = list(action.q[:TOTAL_DOF])
        pause = max(0.0, float(action.pause))

        if self.point_last_q is None:
            self.point_last_q = self._pad_q_to_match(self._safe_get_angles() or q_target)
        q_from = self.point_last_q or q_target

        arm_speeds, arm_motion_time = self._calc_arm_sync_speeds(q_from, q_target, action.speed)

        gripper_delta = abs(q_target[6] - q_from[6]) if len(q_target) >= TOTAL_DOF and len(q_from) >= TOTAL_DOF else 0.0
        gripper_speed = min(max(float(action.gripper_speed), self.gripper_speed_min), self.gripper_speed_max)
        gripper_motion_time = gripper_delta / gripper_speed if gripper_speed > 1e-9 and gripper_delta >= self.axis_delta_eps else 0.0

        estimated_motion_time = max(arm_motion_time, gripper_motion_time, self.point_min_motion_time)

        self._set_arm_joint_angles(q_target[:ARM_DOF], v=arm_speeds)
        self._send_gripper_control(q_target[6], gripper_speed, action.gripper_force_limit)

        self.point_index = next_index
        self.point_last_q = q_target
        self.point_next_time = now + estimated_motion_time + pause

        self.point_action_started.emit(next_index)
        self.point_play_status.emit(
            f"状态: 正在执行第 {next_index + 1}/{len(self.point_program)} 个动作，预计运动 {estimated_motion_time:.2f}s"
        )
        self.angles_updated.emit(q_target)
        self.log_msg.emit(
            f"点位 {next_index + 1}: arm_max_v={action.speed:.3f}, "
            f"arm_v={[round(v, 3) for v in arm_speeds]}, "
            f"G={q_target[6]:.3f}, G_v={gripper_speed:.3f}, force={action.gripper_force_limit:.2f}"
        )

    def _stop_point_play(self, user_stop: bool):
        if self.state != "point_play":
            return
        self.state = "idle"
        self.point_play_status.emit("状态: 已停止" if user_stop else "状态: 点位播放完成")
        self.log_msg.emit("点位播放已停止" if user_stop else "点位播放完成")
        self.point_action_started.emit(-1)
        self.point_play_finished.emit()
        self._hold_current_position()

    # ==================== 空闲刷新 ====================
    def _idle_loop(self, now: float):
        if now < self.next_ui_time:
            return
        q = self._safe_get_angles()
        if q is not None and len(q) >= ARM_DOF:
            self.angles_updated.emit(self._pad_q_to_match(q))
        self.next_ui_time = now + self.ui_period
