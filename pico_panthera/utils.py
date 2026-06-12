# ─────────────────────────────────────────────
# XR 位姿 → Twist 增量
# ─────────────────────────────────────────────

import numpy as np
from scipy.spatial.transform import Rotation
import time

from pico_config import (
    TRANSLATION_SCALE, ROTATION_SCALE, MAX_DELTA_POS,
    DEADZONE_POS_M, DEADZONE_ROT_RAD,
    FILTER_ALPHA,
    GRIP_THRESHOLD, DOUBLE_TAP_WINDOW,
)


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


def _rotation_to_euler(R: np.ndarray) -> np.ndarray:
    return Rotation.from_matrix(R).as_euler('xyz')


def _clamp_vec(vec: np.ndarray, max_norm: float) -> np.ndarray:
    norm = np.linalg.norm(vec)
    if norm > max_norm:
        return vec * (max_norm / norm)
    return vec


def _apply_deadzone_vec(vec: np.ndarray, threshold: float) -> np.ndarray:
    norm = np.linalg.norm(vec)
    if norm < threshold:
        return np.zeros_like(vec)
    return vec * ((norm - threshold) / norm)


class PoseEMAFilter:
    def __init__(self, alpha: float):
        self._alpha = alpha
        self._pos: np.ndarray | None = None
        self._rot: Rotation | None   = None

    def reset(self, pos: np.ndarray, rot_matrix: np.ndarray):
        self._pos = pos.copy()
        self._rot = Rotation.from_matrix(rot_matrix)

    def update(self, pos: np.ndarray, rot_matrix: np.ndarray):
        cur_rot = Rotation.from_matrix(rot_matrix)
        self._pos += self._alpha * (pos - self._pos)
        r_delta = cur_rot * self._rot.inv()
        self._rot = Rotation.from_rotvec(r_delta.as_rotvec() * self._alpha) * self._rot
        return self._pos.copy(), self._rot.as_matrix()


class ArmConverter:
    """单臂坐标转换器"""

    def __init__(self, name: str,
                 xr_to_robot_pos: np.ndarray,
                 xr_to_robot_rot: np.ndarray,
                 pos_sign: np.ndarray,
                 rot_sign: np.ndarray):
        self.name = name
        self._xr_to_robot_pos = xr_to_robot_pos
        self._xr_to_robot_rot = xr_to_robot_rot
        self._pos_sign = pos_sign
        self._rot_sign = rot_sign
        self._init_pos = None
        self._init_rot = None
        self.is_calibrated = False
        self._filter = PoseEMAFilter(FILTER_ALPHA)
        self._last_pos: np.ndarray | None = None
        self._last_rotvec: np.ndarray | None = None

    def calibrate(self, pose_7d):
        self._init_pos, self._init_rot = _pose7d_to_pos_rot(pose_7d)
        self._filter.reset(self._init_pos, self._init_rot)
        self._last_pos    = np.zeros(3)
        self._last_rotvec = np.zeros(3)
        self.is_calibrated = True
        print(f"  [{self.name}] XR 初始位置: {self._init_pos.round(4)}")

    def reset(self):
        self.is_calibrated = False
        self._init_pos    = None
        self._init_rot    = None
        self._last_pos    = None
        self._last_rotvec = None

    def compute_twist(self, pose_7d, headset_pose_7d) -> dict:
        if not self.is_calibrated:
            return _zero_twist()

        raw_pos, raw_rot = _pose7d_to_pos_rot(pose_7d)
        cur_pos, cur_rot = self._filter.update(raw_pos, raw_rot)

        _, head_rot = _pose7d_to_pos_rot(headset_pose_7d)
        head_yaw_angle = Rotation.from_matrix(head_rot).as_euler('YXZ')[0]
        R_yaw = Rotation.from_euler('Y', head_yaw_angle).as_matrix()

        delta_pos_xr = _clamp_vec(cur_pos - self._init_pos, MAX_DELTA_POS)
        raw_pos_out  = self._xr_to_robot_pos @ (R_yaw.T @ delta_pos_xr) * self._pos_sign
        pos_change = _apply_deadzone_vec(raw_pos_out - self._last_pos, DEADZONE_POS_M)
        self._last_pos = self._last_pos + pos_change

        delta_rot_xr   = cur_rot @ self._init_rot.T
        delta_rot_user = R_yaw.T @ delta_rot_xr @ R_yaw
        delta_rot_robot = self._xr_to_robot_rot @ delta_rot_user @ self._xr_to_robot_rot.T
        euler = _rotation_to_euler(delta_rot_robot)
        raw_rotvec_out = Rotation.from_euler('xyz', euler * self._rot_sign).as_rotvec()
        rot_change = _apply_deadzone_vec(raw_rotvec_out - self._last_rotvec, DEADZONE_ROT_RAD)
        self._last_rotvec = self._last_rotvec + rot_change

        delta_pos = self._last_pos * TRANSLATION_SCALE
        rotvec    = self._last_rotvec * ROTATION_SCALE

        return {
            "linear":  {"x": float(delta_pos[0]), "y": float(delta_pos[1]), "z": float(delta_pos[2])},
            "angular": {"x": float(rotvec[0]),    "y": float(rotvec[1]),    "z": float(rotvec[2])},
        }


def _zero_twist() -> dict:
    return {
        "linear":  {"x": 0.0, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
    }


class GripDetector:
    """检测单个 Grip 键的状态，每次调用 update() 返回:
      is_active, request_init, reset_to_zero
    """

    def __init__(self, name: str):
        self.name = name
        self._prev_pressed = False
        self._last_press_time = 0.0

    def update(self, grip_value: float):
        pressed = grip_value > GRIP_THRESHOLD
        request_init  = False
        reset_to_zero = False

        if pressed and not self._prev_pressed:
            now = time.time()
            if now - self._last_press_time < DOUBLE_TAP_WINDOW:
                reset_to_zero = True
                print(f"\n[{self.name} Grip×2] 双击 → 回零!")
            else:
                request_init = True
                print(f"\n[{self.name} Grip] 按下 → 激活 + 校准")
            self._last_press_time = now

        if not pressed and self._prev_pressed:
            print(f"\n[{self.name} Grip] 松开 → 停止")

        self._prev_pressed = pressed
        return pressed, request_init, reset_to_zero
