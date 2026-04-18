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


from config import (
    XR_TO_ROBOT_POS, XR_TO_ROBOT_ROT,
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


def _rotation_to_rotvec(R: np.ndarray) -> np.ndarray:
    """旋转矩阵 → 旋转向量 (rx, ry, rz)，单位 rad"""
    return Rotation.from_matrix(R).as_rotvec()


def _clamp_vec(vec: np.ndarray, max_norm: float) -> np.ndarray:
    """幅值超过 max_norm 时按比例截断，方向不变。"""
    norm = np.linalg.norm(vec)
    if norm > max_norm:
        return vec * (max_norm / norm)
    return vec


def _apply_deadzone_vec(vec: np.ndarray, threshold: float) -> np.ndarray:
    """
    连续型死区：幅值 < threshold 时返回零向量；
    幅值 >= threshold 时输出从 0 线性增长，方向不变。
    output = (norm - threshold) / norm * vec
    """
    norm = np.linalg.norm(vec)
    if norm < threshold:
        return np.zeros_like(vec)
    return vec * ((norm - threshold) / norm)


class PoseEMAFilter:
    """
    对手柄位姿做 EMA 低通滤波，抑制生理性手抖（8–12 Hz）。
    位置：分量级 EMA。
    旋转：在切空间做 EMA（旋转向量缩放后合成），保证插值在 SO(3) 上连续。
    """

    def __init__(self, alpha: float):
        self._alpha = alpha
        self._pos: np.ndarray | None = None
        self._rot: Rotation | None   = None

    def reset(self, pos: np.ndarray, rot_matrix: np.ndarray):
        """校准时调用，用当前原始位姿初始化滤波器状态，避免启动瞬态。"""
        self._pos = pos.copy()
        self._rot = Rotation.from_matrix(rot_matrix)

    def update(self, pos: np.ndarray, rot_matrix: np.ndarray):
        """输入原始位姿，返回滤波后的 (pos, rot_matrix)。"""
        cur_rot = Rotation.from_matrix(rot_matrix)
        # 位置 EMA
        self._pos += self._alpha * (pos - self._pos)
        # 旋转 EMA：在切空间插值，等价于 slerp(prev, cur, alpha)
        r_delta = cur_rot * self._rot.inv()
        self._rot = Rotation.from_rotvec(r_delta.as_rotvec() * self._alpha) * self._rot
        return self._pos.copy(), self._rot.as_matrix()


class ArmConverter:
    """单臂坐标转换器"""

    def __init__(self, name: str):
        self.name = name
        self._init_pos = None   # XR 初始位置
        self._init_rot = None   # XR 初始旋转矩阵
        self.is_calibrated = False
        self._filter = PoseEMAFilter(FILTER_ALPHA)

    def calibrate(self, pose_7d):
        """记录当前手柄位姿为基准（Grip 单击时调用）"""
        self._init_pos, self._init_rot = _pose7d_to_pos_rot(pose_7d)
        self._filter.reset(self._init_pos, self._init_rot)
        self.is_calibrated = True
        print(f"  [{self.name}] XR 初始位置: {self._init_pos.round(4)}")

    def reset(self):
        self.is_calibrated = False
        self._init_pos = None
        self._init_rot = None

    def compute_twist(self, pose_7d, headset_pose_7d) -> dict:
        """
        计算相对初始位姿的增量，返回 Twist dict：
          {"linear":  {"x": dx, "y": dy, "z": dz},
           "angular": {"x": rx, "y": ry, "z": rz}}
        headset_pose_7d 用于提取用户当前水平朝向（yaw），使位移/旋转增量
        相对于用户面朝方向，而非 XR 世界坐标系固定轴。
        """
        if not self.is_calibrated:
            return _zero_twist()

        raw_pos, raw_rot = _pose7d_to_pos_rot(pose_7d)
        cur_pos, cur_rot = self._filter.update(raw_pos, raw_rot)

        # ── 提取头部 yaw 旋转矩阵（XR 世界 Y 轴为垂直轴）──────
        _, head_rot = _pose7d_to_pos_rot(headset_pose_7d)
        head_yaw_angle = Rotation.from_matrix(head_rot).as_euler('YXZ')[0]
        R_yaw = Rotation.from_euler('Y', head_yaw_angle).as_matrix()
        # R_yaw.T 将世界坐标系向量变换到用户朝向坐标系

        # ── 位置增量 ──────────────────────────────
        delta_pos_xr = cur_pos - self._init_pos
        delta_pos_xr = _clamp_vec(delta_pos_xr, MAX_DELTA_POS)
        delta_pos_xr = _apply_deadzone_vec(delta_pos_xr, DEADZONE_POS_M)
        # 先转到用户朝向坐标系，再映射到机械臂坐标系
        delta_pos = XR_TO_ROBOT_POS @ (R_yaw.T @ delta_pos_xr) * TRANSLATION_SCALE

        # ── 旋转增量 ──────────────────────────────
        # delta_R_xr = cur_rot @ init_rot^T  (XR 世界坐标系中的旋转增量)
        delta_rot_xr = cur_rot @ self._init_rot.T
        # 先将旋转增量转到用户朝向坐标系，再映射到机械臂坐标系
        delta_rot_user = R_yaw.T @ delta_rot_xr @ R_yaw
        delta_rot_robot = XR_TO_ROBOT_ROT @ delta_rot_user @ XR_TO_ROBOT_ROT.T
        rotvec = _apply_deadzone_vec(
            _rotation_to_rotvec(delta_rot_robot), DEADZONE_ROT_RAD
        ) * ROTATION_SCALE

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