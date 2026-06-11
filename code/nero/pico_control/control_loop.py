# ─────────────────────────────────────────────
#  control_loop.py  —  50 Hz 末端位姿控制循环
# ─────────────────────────────────────────────

import time
import threading
import numpy as np
from scipy.spatial.transform import Rotation

from config import (
    CONTROL_RATE,
    SAFE_JOINT_POS, SAFE_SPEED_PERCENT, SPEED_PERCENT,
    GRIPPER_MAX_WIDTH_M, GRIPPER_FORCE_N,
)
from shared_state import SharedState


def _rot_to_rpy(rot: np.ndarray) -> list:
    return Rotation.from_matrix(rot).as_euler('xyz').tolist()


def _rpy_to_rot(rpy) -> np.ndarray:
    return Rotation.from_euler('xyz', rpy).as_matrix()


def _wait_motion_done(robot, timeout: float = 10.0) -> bool:
    """轮询 motion_status，等待运动完成。"""
    # move_j 发出后 arm 需要约 100~300ms 才开始运动，先等进入运动状态再等完成。
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        s = robot.get_arm_status()
        if s is not None and s.msg.motion_status == 1:
            break
        time.sleep(0.05)
    while time.monotonic() < deadline:
        s = robot.get_arm_status()
        if s is not None and s.msg.motion_status == 0:
            return True
        time.sleep(0.05)
    return False


def _update_state(robot, gripper, state: SharedState):
    """读取 FK、关节角度、夹爪状态，写入共享状态。"""
    fp = robot.get_flange_pose()
    ja = robot.get_joint_angles()
    if fp is None or ja is None:
        return

    x, y, z, roll, pitch, yaw = fp.msg
    rot = _rpy_to_rot([roll, pitch, yaw])

    gripper_pos = 0.0
    if gripper is not None:
        gs = gripper.get_gripper_status()
        if gs is not None:
            gripper_pos = gs.msg.value

    state.set_robot_state(
        ja.msg,
        [0.0] * 7,
        [x, y, z],
        rot,
        gripper_pos=gripper_pos,
        gripper_vel=0.0,
    )
    state.last_valid_joint_pos = list(ja.msg)


def control_loop(robot, gripper, state: SharedState, stop_event: threading.Event):
    """
    50 Hz 末端位姿控制循环（Pico → Nero move_p）。

    逻辑：
      1. 物理复位（最高优先级）：move_j 到安全位置，重置 target
      2. 夹爪连续控制
      3. 消费 Pico 偏移量 → 更新 hard target
      4. Watchdog 超时 或 尚未校准 → hold（不发送新指令）
      5. Smooth target 步进 → RPY → move_p
      6. 更新共享状态供发布线程读取
    """
    interval = 1.0 / CONTROL_RATE
    print(f"[Control] 控制循环启动，频率 {CONTROL_RATE:.0f} Hz")
    print("[Control] 等待 Pico init 信号以完成校准...")

    while not stop_event.is_set():
        t0 = time.time()

        # ── 1. 物理复位 ──────────────────────────
        with state._lock:
            do_reset = state.reset_requested
            state.reset_requested = False

        if do_reset:
            print("[Control] 复位中...")
            robot.set_speed_percent(SAFE_SPEED_PERCENT)
            robot.set_motion_mode('j')
            robot.move_j(SAFE_JOINT_POS)
            if not _wait_motion_done(robot, timeout=15.0):
                print("[Control] 警告：复位等待超时，继续...")
            robot.set_speed_percent(SPEED_PERCENT)
            robot.set_motion_mode('p')

            fp = robot.get_flange_pose()
            if fp is not None:
                x, y, z, roll, pitch, yaw = fp.msg
                rot = _rpy_to_rot([roll, pitch, yaw])
                state.reset_target_to([x, y, z], rot)
            with state._lock:
                state.calibration_position = None
                state.calibration_rotation = None
            _update_state(robot, gripper, state)
            print("[Control] 复位完成，请按 Pico init 键重新校准")
            time.sleep(max(0.0, interval - (time.time() - t0)))
            continue

        # ── 2. 夹爪连续控制 ──────────────────────
        with state._lock:
            gripper_cmd = state.gripper_cmd

        if gripper_cmd is not None and gripper is not None:
            width = max(0.0, min(GRIPPER_MAX_WIDTH_M,
                                 gripper_cmd / 2.0 * GRIPPER_MAX_WIDTH_M))
            gripper.move_gripper_m(width, GRIPPER_FORCE_N)

        # ── 3. 消费 Pico 偏移量 ──────────────────
        twist = state.pop_twist()
        if twist is not None:
            state.set_target_from_offset(twist[0], twist[1])

        # ── 4. Watchdog / 未校准 → hold ──────────
        if not state.is_calibrated() or (not state.action_locked and not state.is_watchdog_ok()):
            _update_state(robot, gripper, state)
            time.sleep(max(0.0, interval - (time.time() - t0)))
            continue

        # ── 5. Smooth target 步进 → move_p ───────
        target_pos, target_rot = state.step_smooth_target()
        try:
            rpy = _rot_to_rpy(target_rot)
            robot.move_p([*target_pos.tolist(), *rpy])
        except Exception as e:
            print(f"[Control] move_p 异常: {e}")

        # ── 6. 更新发布状态 ───────────────────────
        _update_state(robot, gripper, state)

        time.sleep(max(0.0, interval - (time.time() - t0)))
