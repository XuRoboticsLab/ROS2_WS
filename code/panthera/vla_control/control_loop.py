# ─────────────────────────────────────────────
#  control_loop.py  —  VLA 动作执行 + 状态读取
#
#  支持三种模式（由 config.MODE 决定）：
#    dual   双臂，14 维动作 [right(6), right_grip, left(6), left_grip]
#    right  右臂，7 维动作  [joints(6), gripper(1)]
#    left   左臂，7 维动作  [joints(6), gripper(1)]
#
#  每个控制 tick：
#    1. 刷新电机状态（send_get / motor_send）
#    2. 从队列取一步动作；队列为空则 hold 上次目标
#    3. pos_vel_tqe_kp_kd（含重力补偿）下发目标关节
#    4. 读取当前状态写入 SharedState（供发布线程使用）
# ─────────────────────────────────────────────

import time
import threading

import numpy as np
from scipy.spatial.transform import Rotation

from vla_config import (
    CONTROL_HZ, MODE,
    KP, KD, GRIPPER_KP, GRIPPER_KD,
    RIGHT_JOINT_NAMES, LEFT_JOINT_NAMES,
)
from shared_state import SharedState


def _rot_to_quat(R) -> list:
    return Rotation.from_matrix(np.array(R, dtype=float)).as_quat().tolist()


def _safe_fk(robot) -> tuple[list, list]:
    try:
        fk   = robot.forward_kinematics()
        pos  = list(fk["position"])
        quat = _rot_to_quat(fk["rotation"])
    except Exception:
        pos  = [0.0, 0.0, 0.0]
        quat = [0.0, 0.0, 0.0, 1.0]
    return pos, quat


def _safe_gripper(robot) -> tuple[float, float]:
    try:
        g = robot.get_current_state_gripper()
        return float(g.position), float(g.velocity)
    except Exception:
        return 0.0, 0.0


# ══════════════════════════════════════════════════════════════════════════
#  单臂控制器
# ══════════════════════════════════════════════════════════════════════════

class ArmController:
    def __init__(self, robot, joint_names: list[str], kp, kd, gripper_kp, gripper_kd):
        self._robot      = robot
        self._n          = len(joint_names)
        self._kp         = list(kp)
        self._kd         = list(kd)
        self._gripper_kp = gripper_kp
        self._gripper_kd = gripper_kd
        self._zero_vel   = [0.0] * self._n
        self._last_joint_target:   list[float] | None = None
        self._last_gripper_target: float               = 0.0

    def _refresh(self):
        self._robot.send_get_motor_state_cmd()
        self._robot.motor_send_cmd()

    def _read_state(self):
        pos  = list(self._robot.get_current_pos())
        vel  = list(self._robot.get_current_vel())
        grav = list(self._robot.get_Gravity())
        return pos, vel, grav

    def _send_cmd(self, joint_target: list[float], grav: list[float], grip_target: float):
        self._robot.pos_vel_tqe_kp_kd(
            joint_target, self._zero_vel, grav, self._kp, self._kd
        )
        self._robot.gripper_control_MIT(
            grip_target, 0.0, 0.0, self._gripper_kp, self._gripper_kd
        )

    def step(self, target_joint: list[float], target_grip: float):
        self._last_joint_target   = list(target_joint)
        self._last_gripper_target = float(target_grip)

        self._refresh()
        pos, vel, grav = self._read_state()
        self._send_cmd(self._last_joint_target, grav, self._last_gripper_target)

        grip_p, grip_v = _safe_gripper(self._robot)
        fk_pos, fk_quat = _safe_fk(self._robot)
        return pos, vel, grip_p, grip_v, fk_pos, fk_quat

    def hold(self):
        self._refresh()
        pos, vel, grav = self._read_state()

        if self._last_joint_target is None:
            # 首次 hold：锁定当前位置
            self._last_joint_target = pos

        self._send_cmd(self._last_joint_target, grav, self._last_gripper_target)

        grip_p, grip_v = _safe_gripper(self._robot)
        fk_pos, fk_quat = _safe_fk(self._robot)
        return pos, vel, grip_p, grip_v, fk_pos, fk_quat


# ══════════════════════════════════════════════════════════════════════════
#  动作解包辅助
# ══════════════════════════════════════════════════════════════════════════

def _unpack_dual(action: np.ndarray):
    """14 维 HT 格式 → (right_joints, right_grip, left_joints, left_grip)"""
    return action[0:6].tolist(), float(action[6]), action[7:13].tolist(), float(action[13])


def _unpack_single(action: np.ndarray):
    """7 维单臂格式 → (joints, gripper)"""
    return action[0:6].tolist(), float(action[6])


# ══════════════════════════════════════════════════════════════════════════
#  主控制循环
# ══════════════════════════════════════════════════════════════════════════

def control_loop(
    right_ctrl:  ArmController | None,
    left_ctrl:   ArmController | None,
    state:       SharedState,
    stop_event:  threading.Event,
    trigger_fn:  callable,
):
    """
    根据 MODE 执行对应的动作解包策略：
      dual   → 右臂用 action[0:7]，左臂用 action[7:14]
      right  → 仅右臂，action[0:7]
      left   → 仅左臂，action[0:7]

    trigger_fn:
      当 action queue 从有动作变为空（执行完最后一步）时调用一次，
      向 VLA bridge 发送新的推理请求。启动时 queue 也为空，同样触发一次。
    """
    interval   = 1.0 / CONTROL_HZ
    tick       = 0
    # True：上一 tick 还在执行动作（或刚启动），确保首次空队列也会触发
    was_active = True

    while not stop_event.is_set():
        t0     = time.monotonic()
        action = state.dequeue_action()

        # ── 双臂模式 ──────────────────────────────────────────────────
        if MODE == "dual":
            if action is not None:
                rj, rg, lj, lg = _unpack_dual(action)
            if right_ctrl is not None:
                r = right_ctrl.step(rj, rg) if action is not None else right_ctrl.hold()
                state.update_right(*r)
            if left_ctrl is not None:
                l = left_ctrl.step(lj, lg) if action is not None else left_ctrl.hold()
                state.update_left(*l)

        # ── 右臂模式 ──────────────────────────────────────────────────
        elif MODE == "right":
            if right_ctrl is not None:
                if action is not None:
                    joints, grip = _unpack_single(action)
                    r = right_ctrl.step(joints, grip)
                else:
                    r = right_ctrl.hold()
                state.update_right(*r)

        # ── 左臂模式 ──────────────────────────────────────────────────
        elif MODE == "left":
            if left_ctrl is not None:
                if action is not None:
                    joints, grip = _unpack_single(action)
                    l = left_ctrl.step(joints, grip)
                else:
                    l = left_ctrl.hold()
                state.update_left(*l)

        # ── Trigger 逻辑：queue 从有变空时触发一次 ────────────────────
        if action is not None:
            was_active = True
        elif was_active:
            # 刚执行完最后一步（或启动时 queue 本来就空）
            trigger_fn()
            was_active = False

        # ── 状态打印（每 200 tick ≈ 4s @ 50Hz）────────────────────────
        if tick % 200 == 0:
            q_sz = state.queue_size()
            if MODE in ("dual", "right") and right_ctrl is not None:
                snap = state.snapshot_right()
                print(
                    f"\r[ctrl] tick={tick} mode={MODE} queue={q_sz} "
                    f"right={[round(v,3) for v in snap.joint_pos]}   ",
                    end="", flush=True,
                )
            elif MODE == "left" and left_ctrl is not None:
                snap = state.snapshot_left()
                print(
                    f"\r[ctrl] tick={tick} mode={MODE} queue={q_sz} "
                    f"left={[round(v,3) for v in snap.joint_pos]}   ",
                    end="", flush=True,
                )

        tick += 1
        elapsed = time.monotonic() - t0
        if interval - elapsed > 0:
            time.sleep(interval - elapsed)
