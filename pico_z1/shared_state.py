# ─────────────────────────────────────────────
#  shared_state.py  —  线程安全的共享状态（Z1 版）
# ─────────────────────────────────────────────

import time
import threading
from typing import Optional
import numpy as np
from scipy.spatial.transform import Rotation

from z1_config import (
    TRANSLATION_SCALE, ROTATION_SCALE,
    WATCHDOG_TIMEOUT, FINE_SCALE,
    KP_LINEAR, KP_ANGULAR,
    MAX_LINEAR_VEL, MAX_ANGULAR_VEL,
)


class SharedState:
    def __init__(self):
        self._lock = threading.Lock()

        # 校准基准（Grip 按下时记录机械臂当前位姿）
        self.calibration_position: Optional[np.ndarray] = None
        self.calibration_rotation: Optional[np.ndarray] = None

        # 目标位姿（calibration + Pico 偏移）
        self.target_position = np.zeros(3)
        self.target_rotation = np.eye(3, dtype=float)

        # 当前 FK 位姿（由控制循环更新）
        self.fk_position = np.zeros(3)
        self.fk_rotation = np.eye(3, dtype=float)

        # 待处理指令
        self._pending_twist      = None   # (linear_xyz, angular_xyz)
        self._pending_gripper    = None   # float [0, 2]，由 pico trigger 映射
        self._pending_fine_delta = None   # (dx, dy)，摇杆精细控制

        self.reset_requested = False
        self.last_cmd_time   = 0.0

        # 可调参数（param_server 实时修改）
        self.translation_scale: float = TRANSLATION_SCALE
        self.rotation_scale:    float = ROTATION_SCALE
        self.kp_linear:         float = KP_LINEAR
        self.kp_angular:        float = KP_ANGULAR
        self.max_linear_vel:    float = MAX_LINEAR_VEL
        self.max_angular_vel:   float = MAX_ANGULAR_VEL
        self.fine_scale:        float = FINE_SCALE

    # ── 校准 ──────────────────────────────────

    def set_calibration_pose(self, pos: np.ndarray, rot: np.ndarray):
        with self._lock:
            self.calibration_position = np.array(pos, dtype=float)
            self.calibration_rotation = np.array(rot, dtype=float)
            self.target_position      = self.calibration_position.copy()
            self.target_rotation      = self.calibration_rotation.copy()
            self._pending_twist       = None
        print("[State] 校准基准已更新")

    def clear_calibration(self):
        with self._lock:
            self.calibration_position = None
            self.calibration_rotation = None
            self._pending_twist       = None
        print("[State] 校准已清除，请重新 Grip 校准")

    def is_calibrated(self) -> bool:
        with self._lock:
            return self.calibration_position is not None

    # ── Twist（Pico 位姿偏移）─────────────────

    def push_twist(self, linear_xyz, angular_xyz):
        with self._lock:
            self._pending_twist = (np.array(linear_xyz), np.array(angular_xyz))
            self.last_cmd_time  = time.time()

    def pop_twist(self):
        with self._lock:
            t = self._pending_twist
            self._pending_twist = None
        return t

    def set_target_from_offset(self, linear_xyz, angular_xyz):
        """将 Pico 的偏移量（相对校准基准）转换为目标位姿。"""
        if self.calibration_position is None:
            return
        pos_offset = np.array(linear_xyz) * self.translation_scale
        rot_offset = Rotation.from_rotvec(
            np.array(angular_xyz) * self.rotation_scale
        ).as_matrix()
        with self._lock:
            self.target_position = self.calibration_position + pos_offset
            self.target_rotation = rot_offset @ self.calibration_rotation

    # ── 夹爪 ──────────────────────────────────

    def push_gripper(self, pos: float):
        with self._lock:
            self._pending_gripper = float(pos)

    def pop_gripper(self) -> Optional[float]:
        with self._lock:
            g = self._pending_gripper
            self._pending_gripper = None
        return g

    # ── 摇杆精细控制 ──────────────────────────

    def push_fine_delta(self, xy):
        with self._lock:
            self._pending_fine_delta = np.array(xy[:2], dtype=float)
            self.last_cmd_time = time.time()

    def pop_fine_delta(self) -> Optional[np.ndarray]:
        with self._lock:
            d = self._pending_fine_delta
            self._pending_fine_delta = None
        return d

    def apply_fine_delta_to_target(self, axis_xy: np.ndarray):
        """将摇杆 xy 作为世界坐标系 x/y 平动增量叠加到 target。"""
        if self.calibration_position is None:
            return
        step = self.fine_scale / 50.0   # 假设 50 Hz，将 m/s 转换为单步
        with self._lock:
            self.target_position[0] += axis_xy[1] * step
            self.target_position[1] -= axis_xy[0] * step

    # ── FK 状态 ────────────────────────────────

    def set_fk(self, pos: np.ndarray, rot: np.ndarray):
        with self._lock:
            self.fk_position = np.array(pos, dtype=float)
            self.fk_rotation = np.array(rot, dtype=float)

    def get_fk(self):
        with self._lock:
            return self.fk_position.copy(), self.fk_rotation.copy()

    # ── 目标 ──────────────────────────────────

    def get_target(self):
        with self._lock:
            return self.target_position.copy(), self.target_rotation.copy()

    # ── Watchdog ──────────────────────────────

    def is_watchdog_ok(self) -> bool:
        with self._lock:
            if self.last_cmd_time == 0.0:
                return False
            return (time.time() - self.last_cmd_time) < WATCHDOG_TIMEOUT
