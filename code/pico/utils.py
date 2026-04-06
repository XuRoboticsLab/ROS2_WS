# ─────────────────────────────────────────────
# XR 位姿 → Twist 增量
# ─────────────────────────────────────────────
#
#  每个手柄对应一个 ArmConverter 实例。
#  calibrate() 在 Grip 单击时调用，记录初始位姿。
#  compute_twist() 在 Grip 按住时每帧调用，返回相对初始的增量。
#
#  输出的 Twist 含义与 panthera_ros 的 shared_state 一致：
#    linear  → 位移增量 (m)，已映射到机械臂坐标系
#    angular → 旋转增量 (rad)，已映射到机械臂坐标系，以 rx/ry/rz 分量表示
# ─────────────────────────────────────────────

import numpy as np
from scipy.spatial.transform import Rotation

import time


from config import XR_TO_ROBOT_POS, XR_TO_ROBOT_ROT, TRANSLATION_SCALE, ROTATION_SCALE, MAX_DELTA_POS, GRIP_THRESHOLD, DOUBLE_TAP_WINDOW


def _quat_to_matrix(qx, qy, qz, qw) -> np.ndarray:
    norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    if norm < 1e-10:
        return np.eye(3)
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
    return np.array([
        [1 - 2*(qy*qy + qz*qz),   2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw),   1 - 2*(qx*qx + qz*qz),   2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),       2*(qy*qz + qx*qw),   1 - 2*(qx*qx + qy*qy)],
    ])


def _pose7d_to_pos_rot(pose_7d):
    pos = np.array(pose_7d[:3], dtype=float)
    rot = _quat_to_matrix(pose_7d[3], pose_7d[4], pose_7d[5], pose_7d[6])
    return pos, rot


def _rotation_to_rotvec(R: np.ndarray) -> np.ndarray:
    """旋转矩阵 → 旋转向量 (rx, ry, rz)，单位 rad"""
    return Rotation.from_matrix(R).as_rotvec()


class ArmConverter:
    """单臂坐标转换器"""

    def __init__(self, name: str):
        self.name = name
        self._init_pos = None   # XR 初始位置
        self._init_rot = None   # XR 初始旋转矩阵
        self.is_calibrated = False

    def calibrate(self, pose_7d):
        """记录当前手柄位姿为基准（Grip 单击时调用）"""
        self._init_pos, self._init_rot = _pose7d_to_pos_rot(pose_7d)
        self.is_calibrated = True
        print(f"  [{self.name}] XR 初始位置: {self._init_pos.round(4)}")

    def reset(self):
        self.is_calibrated = False
        self._init_pos = None
        self._init_rot = None

    def compute_twist(self, pose_7d) -> dict:
        """
        计算相对初始位姿的增量，返回 Twist dict：
          {"linear":  {"x": dx, "y": dy, "z": dz},
           "angular": {"x": rx, "y": ry, "z": rz}}
        """
        if not self.is_calibrated:
            return _zero_twist()

        cur_pos, cur_rot = _pose7d_to_pos_rot(pose_7d)

        # ── 位置增量 ──────────────────────────────
        delta_pos_xr = cur_pos - self._init_pos
        if np.linalg.norm(delta_pos_xr) > MAX_DELTA_POS:
            delta_pos_xr = np.zeros(3)
        delta_pos = XR_TO_ROBOT_POS @ delta_pos_xr * TRANSLATION_SCALE

        # ── 旋转增量 ──────────────────────────────
        # delta_R_xr = cur_rot @ init_rot^T  (XR 坐标系中的旋转增量)
        delta_rot_xr = cur_rot @ self._init_rot.T
        # 映射到机械臂坐标系
        delta_rot_robot = XR_TO_ROBOT_ROT @ delta_rot_xr @ XR_TO_ROBOT_ROT.T
        rotvec = _rotation_to_rotvec(delta_rot_robot) * ROTATION_SCALE

        return {
            "linear":  {"x": float(delta_pos[0]), "y": float(delta_pos[1]), "z": float(delta_pos[2])},
            "angular": {"x": float(rotvec[0]),    "y": float(rotvec[1]),    "z": float(rotvec[2])},
        }


def _zero_twist() -> dict:
    return {
        "linear":  {"x": 0.0, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
    }


# ─────────────────────────────────────────────
# 单个 Grip 键的双击检测器
# ─────────────────────────────────────────────

class GripDetector:
    """
    检测单个 Grip 键的状态，每次调用 update() 返回:
      is_active     : bool  当前是否按住
      request_init  : bool  单击上升沿（需要校准）
      reset_to_zero : bool  双击上升沿（需要回零），仅发一帧 True
    """

    def __init__(self, name: str):
        self.name = name
        self._prev_pressed = False
        self._last_press_time = 0.0

    def update(self, grip_value: float):
        pressed = grip_value > GRIP_THRESHOLD
        request_init  = False
        reset_to_zero = False

        # 上升沿
        if pressed and not self._prev_pressed:
            now = time.time()
            if now - self._last_press_time < DOUBLE_TAP_WINDOW:
                reset_to_zero = True
                print(f"\n[{self.name} Grip×2] 双击 → 回零!")
            else:
                request_init = True
                print(f"\n[{self.name} Grip] 按下 → 激活 + 校准")
            self._last_press_time = now

        # 下降沿
        if not pressed and self._prev_pressed:
            print(f"\n[{self.name} Grip] 松开 → 停止")

        self._prev_pressed = pressed
        return pressed, request_init, reset_to_zero