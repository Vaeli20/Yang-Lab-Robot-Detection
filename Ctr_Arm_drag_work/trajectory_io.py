import csv
import os
from dataclasses import dataclass
from typing import List, Optional


ARM_DOF = 6
TOTAL_DOF = 7  # J1~J6 + G(夹爪)


@dataclass
class TrajectoryPoint:
    t: float
    q: List[float]          # 新版优先为7维：J1~J6 + G
    dq: Optional[List[float]] = None


TRAJECTORY_HEADER = [
    "timestamp",
    "j1", "j2", "j3", "j4", "j5", "j6", "g",
    "v1", "v2", "v3", "v4", "v5", "v6", "vg",
]


OLD_WITH_CMD_GRIPPER_HEADER_HINTS = {"gripper_position", "gripper_velocity", "gripper_force_limit"}


def _to_float_list(row: List[str], start: int, count: int) -> List[float]:
    return [float(row[start + i]) for i in range(count)]


def read_trajectory_csv(filepath: str) -> List[TrajectoryPoint]:
    """
    支持格式：
    1. 新格式：timestamp + J1~J6 + G + V1~V6 + VG，共15列。
    2. 旧格式：timestamp + J1~J6 + V1~V6，共13列。
    3. 更旧格式：timestamp + J1~J6，共7列。
    4. 上一版错误夹爪格式：timestamp + J1~J6 + V1~V6 + gripper_position + ...，
       会把 gripper_position 作为 G 兼容读取。
    """
    points: List[TrajectoryPoint] = []

    with open(filepath, "r", newline="", encoding="utf-8-sig") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            return points

        header_lower = [h.strip().lower() for h in header]
        old_cmd_gripper = any(h in OLD_WITH_CMD_GRIPPER_HEADER_HINTS for h in header_lower)

        for line_no, row in enumerate(reader, start=2):
            if not row or len(row) < 7:
                continue
            try:
                t = float(row[0])

                if len(row) >= 15 and not old_cmd_gripper:
                    # 新版：t + 7位置 + 7速度
                    q = _to_float_list(row, 1, TOTAL_DOF)
                    dq = _to_float_list(row, 8, TOTAL_DOF)
                elif len(row) >= 16 and old_cmd_gripper:
                    # 上一版：t + 6位置 + 6速度 + gripper_position/velocity/force
                    q6 = _to_float_list(row, 1, ARM_DOF)
                    dq6 = _to_float_list(row, 7, ARM_DOF)
                    g = float(row[13])
                    q = q6 + [g]
                    dq = dq6 + [0.05]
                elif len(row) >= 13:
                    q = _to_float_list(row, 1, ARM_DOF)
                    dq = _to_float_list(row, 7, ARM_DOF)
                else:
                    q = _to_float_list(row, 1, ARM_DOF)
                    dq = None

                points.append(TrajectoryPoint(t=t, q=q, dq=dq))
            except ValueError as e:
                raise ValueError(f"CSV 第 {line_no} 行格式错误: {row}") from e

    if not points:
        return points

    t0 = points[0].t
    for p in points:
        p.t = max(0.0, p.t - t0)

    return points


def save_trajectory_csv(filepath: str, rows: List[List[float]]) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
    with open(filepath, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)
        writer.writerow(TRAJECTORY_HEADER)
        writer.writerows(rows)
