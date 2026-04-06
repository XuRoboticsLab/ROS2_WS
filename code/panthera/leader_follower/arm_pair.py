# ─────────────────────────────────────────────
#  arm_pair.py  —  单对主从臂控制逻辑
# ─────────────────────────────────────────────

import numpy as np
from config import (
    KP, KD, GRIPPER_KP, GRIPPER_KD,
    FC, FV, VEL_THRESHOLD, TOR_THRESHOLD, TAU_LIMIT,
)


class ArmPair:
    """封装一对主从臂的控制逻辑，每次调用 step() 执行一帧控制"""

    _ZERO_KP = None   # 延迟初始化（需要知道 motor_count）
    _ZERO_KD = None

    def __init__(self, leader, follower, name: str = ""):
        self.leader   = leader
        self.follower = follower
        self.name     = name

        n = leader.motor_count
        self._zero_pos = [0.0] * n
        self._zero_vel = [0.0] * n
        self._zero_kp  = [0.0] * n
        self._zero_kd  = [0.0] * n

    def step(self):
        """
        执行一帧主从控制。
        返回 (follower_joint_pos, follower_fk) 供发布线程使用。
        follower_fk 是 forward_kinematics() 的结果 dict，或 None（如不需要）。
        """
        leader   = self.leader
        follower = self.follower

        # ── 读取状态 ──────────────────────────
        leader_pos  = leader.get_current_pos()
        leader_vel  = leader.get_current_vel()
        follower_vel = follower.get_current_vel()

        # ── 力矩计算 ──────────────────────────
        leader_gra   = np.array(leader.get_Gravity())
        follower_gra = np.array(follower.get_Gravity())

        follower_torque = np.array(follower.get_current_torque())
        tor_diff = follower_torque - follower_gra
        tor_diff[np.abs(tor_diff) < TOR_THRESHOLD] = 0

        leader_tor   = leader_gra   + leader.get_friction_compensation(
            leader_vel, FC, FV, VEL_THRESHOLD)
        follower_tor = follower_gra + follower.get_friction_compensation(
            follower_vel, FC, FV, VEL_THRESHOLD)

        leader_tor   = np.clip(leader_tor,   -TAU_LIMIT, TAU_LIMIT)
        follower_tor = np.clip(follower_tor, -TAU_LIMIT, TAU_LIMIT)

        # ── 发送控制指令 ──────────────────────
        leader.pos_vel_tqe_kp_kd(
            self._zero_pos, self._zero_vel,
            leader_tor.tolist(), self._zero_kp, self._zero_kd
        )
        follower.pos_vel_tqe_kp_kd(
            leader_pos, leader_vel,
            follower_tor.tolist(), KP, KD
        )

        # ── 夹爪控制 ──────────────────────────
        leader_grip_pos = leader.get_current_pos_gripper()
        leader_grip_vel = leader.get_current_vel_gripper()
        follower_grip   = follower.get_current_state_gripper()

        grip_tor = follower.get_friction_compensation(
            leader_grip_vel, 0.06, 0.0, 0.15
        ) - follower_grip.torque * 0.5
        grip_tor = grip_tor if abs(grip_tor) >= 0.2 else 0.0

        leader.gripper_control_MIT(1.5, 0, grip_tor, 0.2, 0.02)
        follower.gripper_control_MIT(
            leader_grip_pos, leader_grip_vel, 0, GRIPPER_KP, GRIPPER_KD
        )

        # ── 读取 Follower 当前状态（供发布）──
        follower_joint_pos = follower.get_current_pos()
        follower_fk        = follower.forward_kinematics()

        return follower_joint_pos, follower_fk