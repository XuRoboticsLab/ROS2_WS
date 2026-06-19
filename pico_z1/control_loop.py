# ─────────────────────────────────────────────
#  control_loop.py  —  Z1 Pico 控制循环（IK + LOWCMD 版）
#
#  策略：
#    1. 消费 Pico 偏移 → 更新目标 4x4 矩阵
#    2. armModel.inverseKinematics(T_target, q_cur, False) → q_target
#    3. 限制每拍最大关节步长（安全）
#    4. 重力补偿 inverseDynamics
#    5. arm.setArmCmd(q_cmd, qd, tau)
#    6. arm.setGripperCmd(pos, vel, tau)
# ─────────────────────────────────────────────

import time
import threading
from typing import Optional
import numpy as np

from z1_config import (
    CONTROL_RATE,
    START_CARTESIAN, START_GRIPPER, START_SPEED,
    MAX_JOINT_STEP,
    GRIPPER_OPEN_POS, GRIPPER_CLOSE_POS,
    GRIPPER_SPEED, GRIPPER_MAX_TAU,
)
from shared_state import SharedState


def _pico_to_gripper(gripper_val: float) -> float:
    """pico gripper_pos [0,2] → setGripperCmd 目标位置。
    0 = trigger 完全按下(闭合), 2 = trigger 松开(张开)
    """
    t = min(1.0, max(0.0, gripper_val / 2.0))   # 0=关, 1=开
    return GRIPPER_CLOSE_POS + t * (GRIPPER_OPEN_POS - GRIPPER_CLOSE_POS)


def _hold(arm, model):
    """保持当前关节位置（重力补偿）。"""
    q_cur = np.array(arm.q)
    tau   = model.inverseDynamics(q_cur, np.zeros(6), np.zeros(6), np.zeros(6))
    arm.setArmCmd(q_cur, np.zeros(6), tau)


def _goto_start(arm, armState):
    """复位：临时切回高层 CARTESIAN 模式 MoveL，再切回 LOWCMD。"""
    arm.setWait(True)
    arm.backToStart()
    arm.labelRun("forward")
    arm.startTrack(armState.CARTESIAN)
    arm.MoveL(START_CARTESIAN, START_GRIPPER, START_SPEED)
    arm.setFsmLowcmd()
    arm.setWait(False)


def control_loop(arm, armState, model, state: SharedState,
                 stop_event: threading.Event):
    """
    主控制循环，在独立线程中运行（进入前已调用 setFsmLowcmd）。
    """
    dt       = arm._ctrlComp.dt
    interval = 1.0 / CONTROL_RATE

    last_gripper = GRIPPER_OPEN_POS

    print(f"[Control] 控制循环启动，频率 {CONTROL_RATE:.0f} Hz，SDK dt={dt:.4f}s")
    print("[Control] 请按 Pico Grip 键激活并校准")

    while not stop_event.is_set():
        t0 = time.time()

        # 1. 复位
        with state._lock:
            do_reset = state.reset_requested
            state.reset_requested = False

        if do_reset:
            print("[Control] 复位中...")
            _goto_start(arm, armState)
            state.clear_calibration()
            last_gripper = GRIPPER_OPEN_POS
            print("[Control] 复位完成，请重新 Grip 校准")
            time.sleep(max(0.0, interval - (time.time() - t0)))
            continue

        # 2. 消费 Pico twist → 更新目标位姿
        twist = state.pop_twist()
        if twist is not None:
            state.set_target_from_offset(twist[0], twist[1])

        # 3. 摇杆精细控制
        fine = state.pop_fine_delta()
        if fine is not None:
            state.apply_fine_delta_to_target(fine)

        # 4. 夹爪目标
        raw_gripper = state.pop_gripper()
        if raw_gripper is not None:
            last_gripper = _pico_to_gripper(raw_gripper)

        # 5. 未校准 / watchdog 超时 → hold
        if not state.is_calibrated() or not state.is_watchdog_ok():
            _hold(arm, model)
            arm.setGripperCmd(last_gripper, GRIPPER_SPEED, GRIPPER_MAX_TAU)
            time.sleep(max(0.0, interval - (time.time() - t0)))
            continue

        # 6. IK：以当前关节角为近似初始点，提高收敛率
        T_target = state.get_target_matrix()
        q_cur    = np.array(arm.q)
        hasIK, q_target = model.inverseKinematics(T_target, q_cur, False)

        if not hasIK:
            # IK 无解（奇异或超工作空间）→ hold
            _hold(arm, model)
        else:
            q_target = np.array(q_target)

            # 每拍最大关节步长限制（安全截断，不改变方向）
            delta     = q_target - q_cur
            max_delta = np.max(np.abs(delta))
            if max_delta > MAX_JOINT_STEP:
                delta = delta * (MAX_JOINT_STEP / max_delta)

            q_cmd = q_cur + delta
            qd    = delta / dt

            # 关节限位保护
            model.jointProtect(q_cmd, qd)

            # 重力补偿
            tau = model.inverseDynamics(q_cmd, qd, np.zeros(6), np.zeros(6))

            arm.setArmCmd(q_cmd, qd, tau)

        # 7. 夹爪
        arm.setGripperCmd(last_gripper, GRIPPER_SPEED, GRIPPER_MAX_TAU)

        time.sleep(max(0.0, interval - (time.time() - t0)))
