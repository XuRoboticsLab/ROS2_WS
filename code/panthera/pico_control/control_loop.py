# ─────────────────────────────────────────────
#  control_loop.py  —  50 Hz 位置控制循环
# ─────────────────────────────────────────────

import time
import threading
import numpy as np

from config import CONTROL_RATE, KP, KD, IK_MAX_JOINT_STEP, SAFE_JOINT_POS, SAFE_JOINT_VEL
from shared_state import SharedState


def _hold(robot, state: SharedState):
    """保持上一次有效关节位置 + 重力补偿。"""
    grav = np.array(robot.get_Gravity())
    robot.pos_vel_tqe_kp_kd(
        state.last_valid_joint_pos,
        [0.0] * robot.motor_count,
        grav, KP, KD,
    )


def _update_state(robot, state: SharedState):
    """读取 FK + 关节速度 + 夹爪状态并写入共享状态。"""
    fk      = robot.forward_kinematics()
    gripper = robot.get_current_state_gripper()
    state.set_robot_state(
        robot.get_current_pos(),
        robot.get_current_vel(),
        fk["position"],
        np.array(fk["rotation"], dtype=float),
        gripper_pos=gripper.position,
        gripper_vel=gripper.velocity,
    )


def control_loop(robot, state: SharedState, stop_event: threading.Event):
    """
    50 Hz 位置控制循环（Pico → Panthera 位置跟随）。

    逻辑：
      1. 物理复位（最高优先级）：移到安全位置，重置 target
      2. 夹爪指令：gripper_open / gripper_close（独立于校准状态）
      3. 消费 Pico 偏移量 → 更新 hard target（校准基准 + 偏移量）
      4. Watchdog 超时 或 尚未校准 → hold
      5. Smooth target 步进（PD + 速度限幅）→ IK → pos_vel_tqe_kp_kd
      6. 更新共享状态供发布线程读取
    """
    interval = 1.0 / CONTROL_RATE
    n_joints = robot.motor_count
    print(f"[Control] 控制循环启动，频率 {CONTROL_RATE:.0f} Hz")
    print("[Control] 等待 Pico init 信号以完成校准...")

    while not stop_event.is_set():
        t0 = time.time()

        # ── 1. 物理复位（最高优先级）────────────────
        with state._lock:
            do_reset = state.reset_requested
            state.reset_requested = False

        if do_reset:
            print("[Control] 复位中...")
            robot.Joint_Pos_Vel(SAFE_JOINT_POS, SAFE_JOINT_VEL, iswait=True)
            fk = robot.forward_kinematics()
            state.reset_target_to(fk["position"], fk["rotation"])
            state.last_valid_joint_pos = robot.get_current_pos()
            _update_state(robot, state)
            print("[Control] 复位完成，请按 Pico init 键重新校准")
            time.sleep(max(0.0, interval - (time.time() - t0)))
            continue

        # ── 2. 夹爪指令 ──────────────────────────
        with state._lock:
            gripper_cmd       = state.gripper_cmd
            state.gripper_cmd = 0

        if gripper_cmd == 1:
            robot.gripper_open()
        elif gripper_cmd == -1:
            robot.gripper_close()

        # ── 3. 消费 Pico 偏移量 ──────────────────
        twist = state.pop_twist()
        if twist is not None:
            state.set_target_from_offset(twist[0], twist[1])

        # ── 3. Watchdog / 未校准 → hold ──────────
        if not state.is_calibrated() or not state.is_watchdog_ok():
            _hold(robot, state)
            _update_state(robot, state)
            time.sleep(max(0.0, interval - (time.time() - t0)))
            continue

        # ── 4. Smooth target 步进 → IK ───────────
        target_pos, target_rot = state.step_smooth_target()

        joint_pos = robot.inverse_kinematics(
            target_pos.tolist(),
            target_rot,
            state.last_valid_joint_pos,
            multi_init=False,
        )

        if joint_pos is not None:
            # 关节空间速度限幅：防止 IK 在奇异点附近产生大幅跳变
            delta = np.array(joint_pos) - np.array(state.last_valid_joint_pos)
            max_delta = np.max(np.abs(delta))
            if max_delta > IK_MAX_JOINT_STEP:
                joint_pos = (np.array(state.last_valid_joint_pos)
                             + delta * (IK_MAX_JOINT_STEP / max_delta)).tolist()

            grav = np.array(robot.get_Gravity())
            robot.pos_vel_tqe_kp_kd(
                joint_pos, [0.0] * n_joints, grav, KP, KD
            )
            state.last_valid_joint_pos = joint_pos
        else:
            _hold(robot, state)

        # ── 5. 更新发布状态 ───────────────────────
        _update_state(robot, state)

        time.sleep(max(0.0, interval - (time.time() - t0)))
