# ─────────────────────────────────────────────
#  control_loop.py  —  50 Hz 位置控制循环
# ─────────────────────────────────────────────

import time
import threading
import numpy as np

from panthera_config import CONTROL_RATE, KP, KD, IK_MAX_JOINT_STEP, SAFE_JOINT_POS, SAFE_JOINT_VEL
from shared_state import SharedState


def _hold(robot, state: SharedState):
    grav = np.array(robot.get_Gravity())
    robot.pos_vel_tqe_kp_kd(
        state.last_valid_joint_pos,
        [0.0] * robot.motor_count,
        grav, KP, KD,
    )


def _update_state(robot, state: SharedState):
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
      1. 物理复位（最高优先级）
      2. 夹爪连续控制
      3. 消费 Pico 偏移量 → 更新 hard target
      4. Watchdog 超时 或 尚未校准 → hold
      5. Smooth target 步进（PD + 速度限幅）→ IK → pos_vel_tqe_kp_kd
      6. 更新共享状态
    """
    interval = 1.0 / CONTROL_RATE
    n_joints = robot.motor_count
    print(f"[Control] 控制循环启动，频率 {CONTROL_RATE:.0f} Hz")
    print("[Control] 等待 Pico init 信号以完成校准...")

    while not stop_event.is_set():
        t0 = time.time()

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

        with state._lock:
            goto_pos = state.goto_joint_pos
            state.goto_joint_pos = None

        if goto_pos is not None:
            print("[Control] 前往保存位姿...")
            robot.Joint_Pos_Vel(goto_pos, SAFE_JOINT_VEL, iswait=True)
            fk = robot.forward_kinematics()
            state.reset_target_to(fk["position"], fk["rotation"])
            state.last_valid_joint_pos = robot.get_current_pos()
            state.action_locked = False
            _update_state(robot, state)
            print("[Control] 已到达目标位姿")
            time.sleep(max(0.0, interval - (time.time() - t0)))
            continue

        with state._lock:
            gripper_pos = state.gripper_cmd

        if gripper_pos is not None:
            robot.gripper_control(pos=gripper_pos, vel=0.5, max_tqu=0.5)

        twist = state.pop_twist()
        if twist is not None:
            state.set_target_from_offset(twist[0], twist[1])

        fine = state.pop_fine_delta()
        if fine is not None:
            state.apply_fine_delta_to_target(fine)

        if not state.is_calibrated() or (not state.action_locked and not state.is_watchdog_ok()):
            _hold(robot, state)
            _update_state(robot, state)
            time.sleep(max(0.0, interval - (time.time() - t0)))
            continue

        target_pos, target_rot = state.step_smooth_target()

        joint_pos = robot.inverse_kinematics(
            target_pos.tolist(),
            target_rot,
            state.last_valid_joint_pos,
            multi_init=False,
        )

        if joint_pos is not None:
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

        _update_state(robot, state)

        time.sleep(max(0.0, interval - (time.time() - t0)))
