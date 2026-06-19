# ─────────────────────────────────────────────
#  shared_state.py  —  线程安全的共享状态（Z1 MoveL 版）
# ─────────────────────────────────────────────

import time
import threading
from typing import Optional
import numpy as np
from scipy.spatial.transform import Rotation

from z1_config import (
    TRANSLATION_SCALE, ROTATION_SCALE,
    WATCHDOG_TIMEOUT, FINE_SCALE,
)


class SharedState:
    def __init__(self):
        self._lock = threading.Lock()

        # 校准基准（Grip 按下时机械臂当前末端的 4x4 齐次矩阵）
        self._T_cal: Optional[np.ndarray] = None

        # 目标位姿（4x4 齐次矩阵，calibration + Pico 偏移）
        self._T_target: Optional[np.ndarray] = None

        # 待处理指令
        self._pending_twist:      Optional[tuple]      = None
        self._pending_gripper:    Optional[float]      = None
        self._pending_fine_delta: Optional[np.ndarray] = None

        self.reset_requested = False
        self.last_cmd_time   = 0.0

        # 可调参数（param_server 实时修改）
        self.translation_scale: float = TRANSLATION_SCALE
        self.rotation_scale:    float = ROTATION_SCALE
        self.fine_scale:        float = FINE_SCALE

    # ── 校准 ──────────────────────────────────

    def set_calibration(self, T_fk: np.ndarray):
        """以当前 FK 矩阵作为校准基准。"""
        with self._lock:
            self._T_cal    = T_fk.copy()
            self._T_target = T_fk.copy()
            self._pending_twist = None
        print("[State] 校准基准已更新")

    def clear_calibration(self):
        with self._lock:
            self._T_cal    = None
            self._T_target = None
            self._pending_twist = None
        print("[State] 校准已清除，请重新 Grip 校准")

    def is_calibrated(self) -> bool:
        with self._lock:
            return self._T_cal is not None

    # ── Twist（Pico 位姿偏移）─────────────────

    def push_twist(self, linear_xyz, angular_xyz):
        with self._lock:
            self._pending_twist = (np.array(linear_xyz), np.array(angular_xyz))
            self.last_cmd_time  = time.time()

    def pop_twist(self) -> Optional[tuple]:
        with self._lock:
            t = self._pending_twist
            self._pending_twist = None
        return t

    def set_target_from_offset(self, linear_xyz, angular_xyz):
        """calibration_T + Pico 偏移 → 新目标矩阵。"""
        with self._lock:
            if self._T_cal is None:
                return
            pos_offset = np.array(linear_xyz) * self.translation_scale
            rot_offset = Rotation.from_rotvec(
                np.array(angular_xyz) * self.rotation_scale
            ).as_matrix()

            T = self._T_cal.copy()
            T[:3, 3]  = self._T_cal[:3, 3] + pos_offset
            T[:3, :3] = rot_offset @ self._T_cal[:3, :3]
            self._T_target = T

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
        """摇杆 xy → 世界坐标系 x/y 平动叠加到 target。"""
        step = self.fine_scale / 50.0
        with self._lock:
            if self._T_target is None:
                return
            self._T_target[0, 3] += axis_xy[1] * step
            self._T_target[1, 3] -= axis_xy[0] * step

    # ── 夹爪 ──────────────────────────────────

    def push_gripper(self, pos: float):
        with self._lock:
            self._pending_gripper = float(pos)

    def pop_gripper(self) -> Optional[float]:
        with self._lock:
            g = self._pending_gripper
            self._pending_gripper = None
        return g

    # ── 目标矩阵 ──────────────────────────────

    def get_target_matrix(self) -> np.ndarray:
        with self._lock:
            return self._T_target.copy()

    # ── Watchdog ──────────────────────────────

    def is_watchdog_ok(self) -> bool:
        with self._lock:
            if self.last_cmd_time == 0.0:
                return False
            return (time.time() - self.last_cmd_time) < WATCHDOG_TIMEOUT
