# ─────────────────────────────────────────────
#  shared_state.py  —  线程安全的共享状态
# ─────────────────────────────────────────────

import time
import threading
import numpy as np
from scipy.spatial.transform import Rotation

# 约束模式基准朝向：绕 y 轴旋转 90°
_R_CONSTRAINED = Rotation.from_euler('y', np.pi / 2).as_matrix()

from config import (
    TRANSLATION_SCALE, ROTATION_SCALE,
    WATCHDOG_TIMEOUT, CONTROL_RATE,
    TRACKING_GAIN_HZ, DAMPING_RATIO,
    MAX_LINEAR_VEL, MAX_ANGULAR_VEL,
)

_N_JOINTS = 7   # Nero 有 7 个关节


class SharedState:
    def __init__(self):
        self._lock = threading.Lock()

        # 每控制周期的速度上限
        self._max_linear_step  = MAX_LINEAR_VEL  / CONTROL_RATE
        self._max_angular_step = MAX_ANGULAR_VEL / CONTROL_RATE
        # Smooth target PD 增益（可运行时调节）
        self.tracking_gain_hz: float = TRACKING_GAIN_HZ
        self.damping_ratio:    float = DAMPING_RATIO

        # 上一周期误差，用于计算 D 项
        self._prev_linear_error  = np.zeros(3)
        self._prev_angular_error = np.zeros(3)

        # Hard target（由 Pico 偏移量直接设置）
        self.target_position = np.zeros(3)
        self.target_rotation = np.eye(3, dtype=float)

        # Smooth target：每控制周期向 hard target 限速步进
        self._smooth_position = self.target_position.copy()
        self._smooth_rotation = self.target_rotation.copy()

        # 校准基准：Pico init 时记录的机械臂末端位姿
        self.calibration_position: np.ndarray | None = None
        self.calibration_rotation: np.ndarray | None = None

        # 待处理的 Pico 偏移量（取最新值，不累加）
        self._pending_twist = None

        # 夹爪指令：1=open，-1=close，0=idle
        self.gripper_cmd: int = 0

        # 运动模式：0=自由，1=约束z轴旋转(基准Ry90°)，2=仅平动(基准Ry90°，无旋转)
        self.motion_mode: int = 0

        # 可运行时调节的运动缩放参数
        self.translation_scale: float = TRANSLATION_SCALE
        self.rotation_scale:    float = ROTATION_SCALE

        # 预制动作执行期间锁住 Pico 输入
        self.action_locked: bool = False

        # 物理复位请求
        self.reset_requested = False

        # Watchdog
        self.last_cmd_time = 0.0

        # 机器人当前状态（供发布线程读取）
        self.joint_positions  = [0.0] * _N_JOINTS
        self.joint_velocities = [0.0] * _N_JOINTS
        self.gripper_position = 0.0
        self.gripper_velocity = 0.0
        self.fk_position      = [0.0, 0.0, 0.0]
        self.fk_rotation      = np.eye(3)
        self.stamp            = {"secs": 0, "nsecs": 0}

        # 上一次有效关节位置（Nero 不做软件 IK，主要用于状态追踪）
        self.last_valid_joint_pos = [0.0] * _N_JOINTS

    # ── 动作辅助 ──────────────────────────────────
    def rotation_error_magnitude(self, target_rot: np.ndarray) -> float:
        with self._lock:
            r_cur = Rotation.from_matrix(self._smooth_rotation)
        return (Rotation.from_matrix(target_rot) * r_cur.inv()).magnitude()

    def gripper_error(self, target_pos: float) -> float:
        with self._lock:
            return abs(self.gripper_position - target_pos)

    # ── Calibration ────────────────────────────
    def set_calibration_pose(self, pos, rot):
        with self._lock:
            self.calibration_position = np.array(pos, dtype=float)
            self.calibration_rotation = np.array(rot, dtype=float)
            self.target_position    = self.calibration_position.copy()
            self.target_rotation    = self.calibration_rotation.copy()
            self._smooth_position   = self.calibration_position.copy()
            self._smooth_rotation   = self.calibration_rotation.copy()
            self._prev_linear_error  = np.zeros(3)
            self._prev_angular_error = np.zeros(3)
            self._pending_twist      = None
        print("[State] 校准基准已更新")

    def is_calibrated(self) -> bool:
        with self._lock:
            return self.calibration_position is not None

    # ── Pico 偏移量 ────────────────────────────
    def push_twist(self, linear_xyz, angular_xyz):
        with self._lock:
            self._pending_twist = (np.array(linear_xyz), np.array(angular_xyz))
            self.last_cmd_time = time.time()

    def pop_twist(self):
        with self._lock:
            twist = self._pending_twist
            self._pending_twist = None
        return twist

    def set_motion_mode(self, mode: int):
        """切换运动模式。"""
        _LABELS = ["自由旋转", "约束z轴旋转(基准Ry90°)", "仅平动(基准Ry90°)"]
        with self._lock:
            prev = self.motion_mode
            self.motion_mode = mode
            if mode != 0 and prev == 0:
                self.target_rotation = _R_CONSTRAINED.copy()
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

        if mode == 1:
            filtered = np.array([0.0, 0.0, angular_xyz[2]])
            rot_offset = Rotation.from_rotvec(filtered * self.rotation_scale).as_matrix()
            with self._lock:
                self.target_position = self.calibration_position + pos_offset
                self.target_rotation = rot_offset @ _R_CONSTRAINED
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
        """将 smooth target 向 hard target 步进一个控制周期，返回 (pos, rot)。"""
        kp = self.tracking_gain_hz / CONTROL_RATE
        kd = self.damping_ratio * kp

        with self._lock:
            # 线性 PD
            err  = self.target_position - self._smooth_position
            step = kp * err + kd * (err - self._prev_linear_error)
            mag  = np.linalg.norm(step)
            if mag > self._max_linear_step:
                step = step / mag * self._max_linear_step
            self._prev_linear_error = err.copy()
            self._smooth_position  += step

            # 旋转 PD
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

    def get_smooth_pose(self):
        """返回当前 smooth target（不步进）。"""
        with self._lock:
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

    # ── Watchdog ───────────────────────────────
    def is_watchdog_ok(self):
        with self._lock:
            if self.last_cmd_time == 0.0:
                return False
            return (time.time() - self.last_cmd_time) < WATCHDOG_TIMEOUT

    # ── Robot state ────────────────────────────
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
