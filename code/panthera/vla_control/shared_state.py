# ─────────────────────────────────────────────
#  shared_state.py  —  action queue + arm state cache
# ─────────────────────────────────────────────

import time
import threading
from collections import deque

import numpy as np


class ArmState:
    __slots__ = ("joint_pos", "joint_vel", "gripper_pos", "gripper_vel",
                 "fk_pos", "fk_quat", "stamp")

    def __init__(self):
        self.joint_pos:   list[float] = []
        self.joint_vel:   list[float] = []
        self.gripper_pos: float       = 0.0
        self.gripper_vel: float       = 0.0
        self.fk_pos:      list[float] = [0.0, 0.0, 0.0]
        self.fk_quat:     list[float] = [0.0, 0.0, 0.0, 1.0]   # [x,y,z,w]
        self.stamp:       dict        = {"sec": 0, "nanosec": 0}


class SharedState:
    def __init__(self):
        self._lock = threading.Lock()
        self._queue: deque[np.ndarray] = deque()
        self.right = ArmState()
        self.left  = ArmState()

    # ── Action queue ──────────────────────────

    def enqueue_actions(self, actions: np.ndarray):
        """actions: (chunk, 14) numpy array，按行入队。"""
        with self._lock:
            for i in range(actions.shape[0]):
                self._queue.append(actions[i].copy())

    def dequeue_action(self) -> np.ndarray | None:
        with self._lock:
            return self._queue.popleft() if self._queue else None

    def queue_size(self) -> int:
        with self._lock:
            return len(self._queue)

    # ── Arm state update（由 control loop 调用）──

    def _make_stamp(self) -> dict:
        t = time.time_ns()
        return {"sec": t // 1_000_000_000, "nanosec": t % 1_000_000_000}

    def update_right(self, joint_pos, joint_vel, gripper_pos, gripper_vel,
                     fk_pos, fk_quat):
        stamp = self._make_stamp()
        with self._lock:
            s = self.right
            s.joint_pos   = list(joint_pos)
            s.joint_vel   = list(joint_vel)
            s.gripper_pos = float(gripper_pos)
            s.gripper_vel = float(gripper_vel)
            s.fk_pos      = list(fk_pos)
            s.fk_quat     = list(fk_quat)
            s.stamp       = stamp

    def update_left(self, joint_pos, joint_vel, gripper_pos, gripper_vel,
                    fk_pos, fk_quat):
        stamp = self._make_stamp()
        with self._lock:
            s = self.left
            s.joint_pos   = list(joint_pos)
            s.joint_vel   = list(joint_vel)
            s.gripper_pos = float(gripper_pos)
            s.gripper_vel = float(gripper_vel)
            s.fk_pos      = list(fk_pos)
            s.fk_quat     = list(fk_quat)
            s.stamp       = stamp

    def snapshot_right(self) -> ArmState:
        out = ArmState()
        with self._lock:
            s = self.right
            out.joint_pos   = s.joint_pos[:]
            out.joint_vel   = s.joint_vel[:]
            out.gripper_pos = s.gripper_pos
            out.gripper_vel = s.gripper_vel
            out.fk_pos      = s.fk_pos[:]
            out.fk_quat     = s.fk_quat[:]
            out.stamp       = s.stamp.copy()
        return out

    def snapshot_left(self) -> ArmState:
        out = ArmState()
        with self._lock:
            s = self.left
            out.joint_pos   = s.joint_pos[:]
            out.joint_vel   = s.joint_vel[:]
            out.gripper_pos = s.gripper_pos
            out.gripper_vel = s.gripper_vel
            out.fk_pos      = s.fk_pos[:]
            out.fk_quat     = s.fk_quat[:]
            out.stamp       = s.stamp.copy()
        return out
