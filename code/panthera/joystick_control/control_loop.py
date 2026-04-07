# ─────────────────────────────────────────────
#  control_loop.py  —  50 Hz 控制循环
# ─────────────────────────────────────────────

import time
import threading
import numpy as np

from config import CONTROL_RATE, KP, KD, SAFE_JOINT_POS, SAFE_JOINT_VEL
from shared_state import SharedState


def _hold(robot, state: SharedState):
    """保持当前位置 + 重力补偿（watchdog / IK 无解时调用）"""
    grav = np.array(robot.get_Gravity())
    robot.pos_vel_tqe_kp_kd(
        state.last_valid_joint_pos,
        [0.0] * robot.motor_count,
        grav, KP, KD,
    )


def _update_state(robot, state: SharedState):
    """读取 FK + 关节速度 + 夹爪状态并写入共享状态"""
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
    interval = 1.0 / CONTROL_RATE
    print(f"[Control] 控制循环启动，频率 {CONTROL_RATE:.0f} Hz")

    while not stop_event.is_set():
        t0 = time.time()

        # ── 1. 复位（最高优先级）─────────────────
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
            print("[Control] 复位完成")
            time.sleep(max(0.0, interval - (time.time() - t0)))
            continue

        # ── 2. 夹爪 ──────────────────────────────
        with state._lock:
            gripper_cmd      = state.gripper_cmd
            state.gripper_cmd = 0

        # if gripper_cmd == 1:
        #     robot.open_gripper()    # ← 替换为实际 API
        # elif gripper_cmd == -1:
        #     robot.close_gripper()   # ← 替换为实际 API

        # ── 3. 消费 Twist 增量 ────────────────────
        twist = state.pop_twist()
        if twist is not None:
            state.apply_twist_to_target(twist[0], twist[1])

        # ── 4. Watchdog ───────────────────────────
        if not state.is_watchdog_ok():
            _hold(robot, state)
            _update_state(robot, state)
            time.sleep(max(0.0, interval - (time.time() - t0)))
            continue

        # ── 5. IK + 发送指令 ──────────────────────
        target_pos, target_rot = state.get_target()
        joint_pos = robot.inverse_kinematics(
            target_pos.tolist(),
            target_rot,
            state.last_valid_joint_pos,
            multi_init=False,
        )

        if joint_pos is not None:
            grav = np.array(robot.get_Gravity())
            robot.pos_vel_tqe_kp_kd(
                joint_pos, [0.0] * robot.motor_count, grav, KP, KD
            )
            state.last_valid_joint_pos = joint_pos
        else:
            _hold(robot, state)

        # ── 6. 更新发布状态 ───────────────────────
        _update_state(robot, state)

        time.sleep(max(0.0, interval - (time.time() - t0)))