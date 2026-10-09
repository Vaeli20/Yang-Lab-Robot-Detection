import csv
import os
from dataclasses import dataclass
from typing import List


ARM_DOF = 6
TOTAL_DOF = 7  # J1~J6 + G(夹爪)


@dataclass
class PointAction:
    q: List[float]                    # 7维：J1~J6 + G
    speed: float = 0.30               # rad/s，只作为J1~J6机械臂关节的最大速度
    pause: float = 0.50               # s，到达该点后等待时间
    gripper_speed: float = 5.0        # 夹爪单独速度，不参与J1~J6同步计算
    gripper_force_limit: float = 0.1  # 如果驱动没有力限接口，只保存，不发送
    note: str = ""


POINT_HEADER = [
    "index",
    "j1", "j2", "j3", "j4", "j5", "j6", "g",
    "max_arm_speed_rad_s",
    "pause_s",
    "gripper_speed_rad_s",
    "gripper_force_limit",
    "note",
]


def _pad_q(q: List[float]) -> List[float]:
    q = list(q)
    if len(q) >= TOTAL_DOF:
        return q[:TOTAL_DOF]
    if len(q) >= ARM_DOF:
        return q[:ARM_DOF] + [0.0]
    return q + [0.0] * (TOTAL_DOF - len(q))


def save_point_program_csv(filepath: str, actions: List[PointAction]) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
    with open(filepath, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)
        writer.writerow(POINT_HEADER)
        for i, a in enumerate(actions, start=1):
            q = _pad_q(a.q)
            writer.writerow(
                [i]
                + [f"{v:.8f}" for v in q[:TOTAL_DOF]]
                + [
                    f"{float(a.speed):.4f}",
                    f"{float(a.pause):.4f}",
                    f"{float(a.gripper_speed):.4f}",
                    f"{float(a.gripper_force_limit):.4f}",
                    a.note,
                ]
            )


def read_point_program_csv(filepath: str) -> List[PointAction]:
    """
    支持新版点位CSV，也兼容旧版：
    - 新版：index + J1~J6 + G + max_arm_speed + pause + gripper_speed + gripper_force + note
    - 上一版：index + J1~J6 + G + max_speed + pause + gripper_force + note，夹爪速度默认5.0
    - 更早错误版：index + J1~J6 + max_speed + pause + gripper_position + gripper_velocity + gripper_force + note
      兼容时把 gripper_position 作为 G，把 gripper_velocity 作为 gripper_speed。
    - 最旧版：index + J1~J6 + speed + pause + note，G默认0，夹爪速度默认5.0。
    """
    actions: List[PointAction] = []
    with open(filepath, "r", newline="", encoding="utf-8-sig") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            return actions

        header_lower = [h.strip().lower() for h in header]
        has_g = "g" in header_lower or "夹爪" in "".join(header_lower)
        has_gripper_speed = "gripper_speed_rad_s" in header_lower or "gripper_speed" in header_lower
        old_cmd_gripper = "gripper_position" in header_lower

        for line_no, row in enumerate(reader, start=2):
            if not row:
                continue
            try:
                if has_g and has_gripper_speed and len(row) >= 12:
                    # 新版：index, J1..J6, G, max_arm_speed, pause, gripper_speed, force, note
                    q = [float(row[i]) for i in range(1, 8)]
                    speed = float(row[8])
                    pause = float(row[9])
                    gripper_speed = float(row[10])
                    gripper_force_limit = float(row[11])
                    note = row[12] if len(row) >= 13 else ""
                elif has_g and len(row) >= 11:
                    # 上一版：index, J1..J6, G, max_speed, pause, force, note
                    q = [float(row[i]) for i in range(1, 8)]
                    speed = float(row[8])
                    pause = float(row[9])
                    gripper_speed = 5.0
                    gripper_force_limit = float(row[10])
                    note = row[11] if len(row) >= 12 else ""
                elif old_cmd_gripper and len(row) >= 12:
                    # 错误夹爪命令版：index, J1..J6, speed, pause, gripper_position, gripper_velocity, force, note
                    q6 = [float(row[i]) for i in range(1, 7)]
                    speed = float(row[7])
                    pause = float(row[8])
                    g = float(row[9])
                    gripper_speed = float(row[10])
                    gripper_force_limit = float(row[11])
                    note = row[12] if len(row) >= 13 else ""
                    q = q6 + [g]
                elif len(row) >= 9:
                    # 最旧版：index, J1..J6, speed, pause, note
                    q6 = [float(row[i]) for i in range(1, 7)]
                    speed = float(row[7])
                    pause = float(row[8])
                    gripper_speed = 5.0
                    gripper_force_limit = 0.1
                    note = row[9] if len(row) >= 10 else ""
                    q = q6 + [0.0]
                else:
                    raise ValueError("列数不足")

                actions.append(
                    PointAction(
                        q=_pad_q(q),
                        speed=speed,
                        pause=pause,
                        gripper_speed=gripper_speed,
                        gripper_force_limit=gripper_force_limit,
                        note=note,
                    )
                )
            except ValueError as e:
                raise ValueError(f"点位CSV第 {line_no} 行格式错误: {row}") from e

    return actions
