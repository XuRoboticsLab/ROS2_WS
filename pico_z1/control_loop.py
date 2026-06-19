# ─────────────────────────────────────────────
#  control_loop.py  —  Z1 Pico 控制循环（MoveL 版）
#
#  策略：setWait(False) + 持续 MoveL
#    1. 消费 Pico 偏移 → 更新目标位姿（4x4 矩阵）
#    2. homoToPosture 转换为 [rx,ry,rz,x,y,z]
#    3. arm.MoveL(posture, gripper, speed)  —— 非阻塞
#    SDK 内部负责 IK / 关节限位 / 奇异处理
# ─────────────────────────────────────────────

import time
import threading
import numpy as np

from z1_config import (
    CONTROL_RATE,
    START_CARTESIAN, START_GRIPPER, START_SPEED,
    MOVEL_SPEED,
    GRIPPER_OPEN_POS, GRIPPER_CLOSE_POS,
)
from shared_state import SharedState


def _pico_gripper_to_movel(gripper_val):
    """pico gripper_pos [0,2] → MoveL gripper 参数 [CLOSE, OPEN]。
    pico: 0=trigger 按下(闭合), 2=trigger 松开(张开)
    """
    if gripper_val is None:
        return None
    t = min(1.0, max(0.0, gripper_val / 2.0))   # 0=闭合, 1=张开
    return GRIPPER_CLOSE_POS + t * (GRIPPER_OPEN_POS - GRIPPER_CLOSE_POS)


def _goto_start(arm, armState):
    arm.setWait(True)
    arm.backToStart()
    arm.labelRun("forward")
    arm.startTrack(armState.CARTESIAN)
    arm.MoveL(START_CARTESIAN, START_GRIPPER, START_SPEED)
    arm.setWait(False)


def control_loop(arm, armState, homoToPosture, state: SharedState,
                 stop_event: threading.Event):
    """
    主控制循环，在独立线程中运行。
    进入前已调用 startTrack(CARTESIAN) + setWait(False)。
    """
    interval         = 1.0 / CONTROL_RATE
    last_gripper_pos = GRIPPER_OPEN_POS

    print(f"[Control] 控制循环启动，频率 {CONTROL_RATE:.0f} Hz")
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
            last_gripper_pos = GRIPPER_OPEN_POS
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

        # 4. 夹爪
        raw_gripper = state.pop_gripper()
        converted   = _pico_gripper_to_movel(raw_gripper)
        if converted is not None:
            last_gripper_pos = converted

        # 5. 未校准 / watchdog 超时 → 跳过本次 MoveL，保持原位
        if not state.is_calibrated() or not state.is_watchdog_ok():
            time.sleep(max(0.0, interval - (time.time() - t0)))
            continue

        # 6. 目标矩阵 → posture → MoveL
        T_target        = state.get_target_matrix()
        target_posture  = homoToPosture(T_target)
        arm.MoveL(target_posture, last_gripper_pos, MOVEL_SPEED)

        time.sleep(max(0.0, interval - (time.time() - t0)))
