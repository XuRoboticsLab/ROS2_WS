# ─────────────────────────────────────────────
#  shared_state.py  —  线程安全的共享状态
# ─────────────────────────────────────────────

import time
import threading
import numpy as np
from scipy.spatial.transform import Rotation

# 约束模式基准朝向：绕 y 轴旋转 90°（end-effector x 轴竖直向下）
_R_CONSTRAINED = Rotation.from_euler('y', np.pi / 2).as_matrix()

from config import (
    TRANSLATION_SCALE, ROTATION_SCALE,
    WATCHDOG_TIMEOUT, CONTROL_RATE,
    TRACKING_GAIN_HZ, DAMPING_RATIO,
    MAX_LINEAR_VEL, MAX_ANGULAR_VEL,
)


class SharedState:
    def __init__(self):
        self._lock = threading.Lock()

        # 每控制周期的速度上限
        self._max_linear_step  = MAX_LINEAR_VEL  / CONTROL_RATE
        self._max_angular_step = MAX_ANGULAR_VEL / CONTROL_RATE
        # PD 增益（per-cycle）
        self._kp = TRACKING_GAIN_HZ / CONTROL_RATE
        self._kd = DAMPING_RATIO * self._kp

        # 上一周期误差，用于计算 D 项
        self._prev_linear_error  = np.zeros(3)
        self._prev_angular_error = np.zeros(3)

        # Hard target（由 Pico 偏移量直接设置）
        self.target_position = np.zeros(3)
        self.target_rotation = np.eye(3, dtype=float)

        # Smooth target：每控制周期向 hard target 限速步进，IK 使用此值
        self._smooth_position = self.target_position.copy()
        self._smooth_rotation = self.target_rotation.copy()

        # 校准基准：Pico init 时记录的机械臂末端位姿
        self.calibration_position: np.ndarray | None = None
        self.calibration_rotation: np.ndarray | None = None

        # 待处理的 Pico 偏移量（位置控制：取最新值，不累加）
        self._pending_twist = None

        # 夹爪目标位置 (rad)：0.0=完全闭合，2.0=完全张开；None=尚未收到指令
        self.gripper_cmd: float | None = None

        # 旋转约束模式：False=自由旋转，True=仅允许 x 轴旋转 + 基准朝向 Ry(90°)
        self.constrained_mode: bool = False

        # 物理复位请求（reset 信号）
        self.reset_requested = False

        # Watchdog
        self.last_cmd_time = 0.0

        # 机器人当前状态（供发布线程读取）
        self.joint_positions  = [0.0] * 6
        self.joint_velocities = [0.0] * 6
        self.gripper_position = 0.0
        self.gripper_velocity = 0.0
        self.fk_position      = [0.0, 0.0, 0.0]
        self.fk_rotation      = np.eye(3)
        self.stamp            = {"secs": 0, "nsecs": 0}

        # 上一次有效 IK 结果
        self.last_valid_joint_pos = [0.0] * 6

    # ── Calibration ────────────────────────────
    def set_calibration_pose(self, pos, rot):
        """Pico init 时调用：记录当前末端位姿为位置控制基准。"""
        with self._lock:
            self.calibration_position = np.array(pos, dtype=float)
            self.calibration_rotation = np.array(rot, dtype=float)
            # 同步 hard/smooth target，避免校准后出现初始跳变
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
        """接收 Pico 发来的绝对位移偏移量（相对于校准基准），取最新值。"""
        with self._lock:
            self._pending_twist = (np.array(linear_xyz), np.array(angular_xyz))
            self.last_cmd_time = time.time()

    def pop_twist(self):
        with self._lock:
            twist = self._pending_twist
            self._pending_twist = None
        return twist

    def set_constrained_mode(self, enabled: bool):
        """切换旋转约束模式。
        进入时：target rotation 立即设为 Ry(90°)，arm 平滑跟踪过去。
        退出时：以当前 smooth rotation 为新的旋转基准，arm 保持原地，用户需重新 grip-init。
        """
        with self._lock:
            prev = self.constrained_mode
            self.constrained_mode = enabled
            if enabled and not prev:
                self.target_rotation = _R_CONSTRAINED.copy()
            elif not enabled and prev:
                # 退出约束：以当前到达的旋转为新的 calibration，避免跳变
                self.calibration_rotation = self._smooth_rotation.copy()
        if enabled and not prev:
            print("[State] 进入旋转约束模式 (仅x轴旋转, 基准Ry90°)")
        elif not enabled and prev:
            print("[State] 退出旋转约束模式 (自由旋转, 请重新 grip-init 以继续旋转控制)")

    def set_target_from_offset(self, linear_xyz, angular_xyz):
        """位置控制：target = 校准基准 + Pico 偏移量（直接设置，不累加）。
        约束模式下：旋转仅保留 x 分量，基准朝向固定为 _R_CONSTRAINED。
        """
        if self.calibration_position is None:
            return
        pos_offset = np.array(linear_xyz) * TRANSLATION_SCALE

        if self.constrained_mode:
            # 只保留 x 轴旋转，y/z 分量清零
            filtered = np.array([0.0, 0.0, angular_xyz[2]])
            rot_offset = Rotation.from_rotvec(filtered * ROTATION_SCALE).as_matrix()
            with self._lock:
                self.target_position = self.calibration_position + pos_offset
                self.target_rotation = rot_offset @ _R_CONSTRAINED
        else:
            rot_offset = Rotation.from_rotvec(np.array(angular_xyz) * ROTATION_SCALE).as_matrix()
            with self._lock:
                self.target_position = self.calibration_position + pos_offset
                self.target_rotation = rot_offset @ self.calibration_rotation

    def step_smooth_target(self):
        """将 smooth target 向 hard target 步进一个控制周期，返回 (pos, rot) 供 IK 使用。

        线性部分：PD 步进 + 速度饱和裁剪；
        旋转部分：旋转向量 PD 步进 + 角速度饱和裁剪（SLERP 方向）。
        """
        with self._lock:
            # ── 线性 PD ───────────────────────────────
            err  = self.target_position - self._smooth_position
            step = self._kp * err + self._kd * (err - self._prev_linear_error)
            mag  = np.linalg.norm(step)
            if mag > self._max_linear_step:
                step = step / mag * self._max_linear_step
            self._prev_linear_error = err.copy()
            self._smooth_position  += step

            # ── 旋转 PD（误差用 rotvec 表示）────────────
            r_curr = Rotation.from_matrix(self._smooth_rotation)
            r_tgt  = Rotation.from_matrix(self.target_rotation)
            err_rv = (r_tgt * r_curr.inv()).as_rotvec()
            omega  = self._kp * err_rv + self._kd * (err_rv - self._prev_angular_error)
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
        """同步重置 hard/smooth target 和 PD 历史，避免跳变。"""
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
