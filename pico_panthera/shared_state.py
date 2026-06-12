# ─────────────────────────────────────────────
#  shared_state.py  —  线程安全的共享状态
# ─────────────────────────────────────────────

import time
import threading
import numpy as np
from scipy.spatial.transform import Rotation

_R_CONSTRAINED = Rotation.from_euler('y', np.pi / 2).as_matrix()
_R_FREE = np.eye(3, dtype=float)

from panthera_config import (
    TRANSLATION_SCALE, ROTATION_SCALE,
    WATCHDOG_TIMEOUT, CONTROL_RATE,
    TRACKING_GAIN_HZ, DAMPING_RATIO,
    MAX_LINEAR_VEL, MAX_ANGULAR_VEL,
    FINE_SCALE, FINE_ROTATION_SCALE,
)


class SharedState:
    def __init__(self):
        self._lock = threading.Lock()

        self._max_linear_step  = MAX_LINEAR_VEL  / CONTROL_RATE
        self._max_angular_step = MAX_ANGULAR_VEL / CONTROL_RATE
        self.tracking_gain_hz: float = TRACKING_GAIN_HZ
        self.damping_ratio:    float = DAMPING_RATIO

        self._prev_linear_error  = np.zeros(3)
        self._prev_angular_error = np.zeros(3)

        self.target_position = np.zeros(3)
        self.target_rotation = np.eye(3, dtype=float)

        self._smooth_position = self.target_position.copy()
        self._smooth_rotation = self.target_rotation.copy()

        self.calibration_position: np.ndarray | None = None
        self.calibration_rotation: np.ndarray | None = None

        self._pending_twist = None
        self._pending_fine_delta = None

        self.gripper_cmd: float | None = None
        self.gripper_vel: float = 3.0   # rad/s，可由 param_server 实时调节

        # 运动模式：0=自由，1=约束z轴旋转(基准Ry90°)，2=仅平动(基准Ry90°，无旋转)，3=约束z轴旋转(无初始旋转)
        self.motion_mode: int = 0

        self.translation_scale:    float = TRANSLATION_SCALE
        self.rotation_scale:       float = ROTATION_SCALE
        self.fine_mode:            int   = 0
        self.fine_scale:           float = FINE_SCALE
        self.fine_rotation_scale:  float = FINE_ROTATION_SCALE

        self.action_locked: bool = False
        self.reset_requested = False
        self.goto_joint_pos: list | None = None

        self.last_cmd_time = 0.0

        self.joint_positions  = [0.0] * 6
        self.joint_velocities = [0.0] * 6
        self.gripper_position = 0.0
        self.gripper_velocity = 0.0
        self.fk_position      = [0.0, 0.0, 0.0]
        self.fk_rotation      = np.eye(3)
        self.stamp            = {"secs": 0, "nsecs": 0}

        self.last_valid_joint_pos = [0.0] * 6

    def request_goto_joint_pos(self, joint_pos: list):
        with self._lock:
            self.goto_joint_pos = list(joint_pos)
            self.action_locked = True

    def request_goto_height_z(self, z: float) -> bool:
        """将 target_position[2] 设为 z，保持当前旋转，锁定 Pico 输入。
        未校准时返回 False。"""
        with self._lock:
            if self.calibration_position is None:
                return False
            self.target_position[2] = float(z)
            self.action_locked = True
        return True

    def rotation_error_magnitude(self, target_rot: np.ndarray) -> float:
        with self._lock:
            r_cur = Rotation.from_matrix(self._smooth_rotation)
        return (Rotation.from_matrix(target_rot) * r_cur.inv()).magnitude()

    def gripper_error(self, target_pos: float) -> float:
        with self._lock:
            return abs(self.gripper_position - target_pos)

    def set_calibration_pose(self, pos, rot):
        with self._lock:
            self.calibration_position = np.array(pos, dtype=float)
            self.target_position  = self.calibration_position.copy()
            self._smooth_position = self.calibration_position.copy()
            if self.motion_mode == 0:
                self.calibration_rotation = np.array(rot, dtype=float)
                self.target_rotation  = self.calibration_rotation.copy()
                self._smooth_rotation = self.calibration_rotation.copy()
            else:
                self.calibration_rotation = self.target_rotation.copy()
            self._prev_linear_error  = np.zeros(3)
            self._prev_angular_error = np.zeros(3)
            self._pending_twist      = None
        print("[State] 校准基准已更新")

    def is_calibrated(self) -> bool:
        with self._lock:
            return self.calibration_position is not None

    def push_twist(self, linear_xyz, angular_xyz):
        with self._lock:
            self._pending_twist = (np.array(linear_xyz), np.array(angular_xyz))
            self.last_cmd_time = time.time()

    def pop_twist(self):
        with self._lock:
            twist = self._pending_twist
            self._pending_twist = None
        return twist

    def push_fine_delta(self, xy):
        with self._lock:
            self._pending_fine_delta = np.array(xy[:2], dtype=float)
            self.last_cmd_time = time.time()

    def pop_fine_delta(self):
        with self._lock:
            delta = self._pending_fine_delta
            self._pending_fine_delta = None
        return delta

    def apply_fine_delta_to_target(self, axis_xy):
        if self.action_locked or self.calibration_position is None:
            return
        with self._lock:
            if self.fine_mode == 0:
                step = self.fine_scale / CONTROL_RATE
                self.target_position[0] += axis_xy[1] * step
                self.target_position[1] -= axis_xy[0] * step
            elif self.fine_mode == 1:
                angle = axis_xy[0] * self.fine_rotation_scale / CONTROL_RATE
                r_delta = Rotation.from_euler('z', angle).as_matrix()
                self.target_rotation = r_delta @ self.target_rotation
            else:
                step = self.fine_scale / CONTROL_RATE
                eef_x = self.target_rotation[:, 0]
                eef_y = self.target_rotation[:, 1]
                self.target_position += axis_xy[1] * step * eef_x
                self.target_position -= axis_xy[0] * step * eef_y

    def set_motion_mode(self, mode: int):
        _LABELS = ["自由旋转", "约束z轴旋转(基准Ry90°)", "仅平动(基准Ry90°)", "约束z轴旋转(无初始旋转)"]
        with self._lock:
            prev = self.motion_mode
            self.motion_mode = mode
            if mode in (1, 2) and prev not in (1, 2):
                self.target_rotation = _R_CONSTRAINED.copy()
            elif mode == 3 and prev != 3:
                self.target_rotation = _R_FREE.copy()
            elif mode == 0 and prev != 0:
                self.calibration_rotation = self._smooth_rotation.copy()
        print(f"[State] 运动模式: {_LABELS[mode]}")
        if mode == 0 and prev != 0:
            print("[State] 请重新 grip-init 以继续旋转控制")

    def set_target_from_offset(self, linear_xyz, angular_xyz):
        if self.action_locked or self.calibration_position is None:
            return
        pos_offset = np.array(linear_xyz) * self.translation_scale
        mode = self.motion_mode

        if mode in (1, 3):
            filtered = np.array([0.0, 0.0, angular_xyz[2]])
            rot_offset = Rotation.from_rotvec(filtered * self.rotation_scale).as_matrix()
            with self._lock:
                self.target_position = self.calibration_position + pos_offset
                self.target_rotation = rot_offset @ self.calibration_rotation
        elif mode == 2:
            with self._lock:
                self.target_position = self.calibration_position + pos_offset
                self.target_rotation = _R_CONSTRAINED.copy()
        else:
            rot_offset = Rotation.from_rotvec(np.array(angular_xyz) * self.rotation_scale).as_matrix()
            with self._lock:
                self.target_position = self.calibration_position + pos_offset
                self.target_rotation = rot_offset @ self.calibration_rotation

    def step_smooth_target(self):
        kp = self.tracking_gain_hz / CONTROL_RATE
        kd = self.damping_ratio * kp

        with self._lock:
            err  = self.target_position - self._smooth_position
            step = kp * err + kd * (err - self._prev_linear_error)
            mag  = np.linalg.norm(step)
            if mag > self._max_linear_step:
                step = step / mag * self._max_linear_step
            self._prev_linear_error = err.copy()
            self._smooth_position  += step

            r_curr = Rotation.from_matrix(self._smooth_rotation)
            r_tgt  = Rotation.from_matrix(self.target_rotation)
            err_rv = (r_tgt * r_curr.inv()).as_rotvec()
            omega  = kp * err_rv + kd * (err_rv - self._prev_angular_error)
            ang_mag = np.linalg.norm(omega)
            if ang_mag > self._max_angular_step:
                omega = omega / ang_mag * self._max_angular_step
            self._prev_angular_error = err_rv.copy()
            self._smooth_rotation = (Rotation.from_rotvec(omega) * r_curr).as_matrix()

            return self._smooth_position.copy(), self._smooth_rotation.copy()

    def get_target(self):
        with self._lock:
            return self.target_position.copy(), self.target_rotation.copy()

    def reset_target_to(self, position, rotation):
        with self._lock:
            self.target_position     = np.array(position)
            self.target_rotation     = np.array(rotation, dtype=float)
            self._smooth_position    = self.target_position.copy()
            self._smooth_rotation    = self.target_rotation.copy()
            self._prev_linear_error  = np.zeros(3)
            self._prev_angular_error = np.zeros(3)

    def is_watchdog_ok(self):
        with self._lock:
            if self.last_cmd_time == 0.0:
                return False
            return (time.time() - self.last_cmd_time) < WATCHDOG_TIMEOUT

    def set_robot_state(self, joint_pos, joint_vel, fk_pos, fk_rot,
                        gripper_pos: float = 0.0, gripper_vel: float = 0.0):
        t_ns = time.time_ns()
        with self._lock:
            self.joint_positions  = list(joint_pos)
            self.joint_velocities = list(joint_vel)
            self.gripper_position = gripper_pos
            self.gripper_velocity = gripper_vel
            self.fk_position      = list(fk_pos)
            self.fk_rotation      = fk_rot.copy()
            self.stamp            = {
                "secs":  t_ns // 1_000_000_000,
                "nsecs": t_ns  % 1_000_000_000,
            }

    def get_robot_state(self):
        with self._lock:
            return (
                list(self.joint_positions),
                list(self.joint_velocities),
                self.gripper_position,
                self.gripper_velocity,
                list(self.fk_position),
                self.fk_rotation.copy(),
                self.stamp.copy(),
            )
