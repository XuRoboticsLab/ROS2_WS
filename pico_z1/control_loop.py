# ─────────────────────────────────────────────
#  control_loop.py  —  Z1 机械臂 Pico 控制循环
#
#  策略：P 控制器 + 笛卡尔速度指令
#    1. 从 SharedState 取最新 Pico 偏移 → 更新目标位姿
#    2. 正运动学获取当前末端位姿
#    3. 计算位置 / 旋转误差
#    4. 误差 × Kp 得到速度，clamp 到最大速度
#    5. 归一化方向 + 速度标量 → cartesianCtrlCmd
#    6. 夹爪：trigger 阈值映射到开合方向
# ─────────────────────────────────────────────

import time
import threading
import numpy as np
from scipy.spatial.transform import Rotation

from z1_config import (
    CONTROL_RATE, START_CARTESIAN, START_GRIPPER, START_SPEED,
)
from shared_state import SharedState

# 速度门控：误差低于此值时停止驱动，避免抖动
_MIN_LINEAR_THRESH  = 0.001   # 1 mm
_MIN_ANGULAR_THRESH = 0.005   # ~0.3°


def _gripper_dir(gripper_pos: float | None) -> float:
    """将 pico trigger 映射的夹爪位置 [0, 2] 转换为速度方向。
    trigger 完全按下(0) → 闭合(-1), 完全松开(2) → 张开(+1), 中间 → 保持(0)。
    """
    if gripper_pos is None:
        return 0.0
    if gripper_pos < 0.5:
        return -1.0
    if gripper_pos > 1.5:
        return 1.0
    return 0.0


def _compute_cmd(state: SharedState,
                 current_pos: np.ndarray,
                 current_rot: np.ndarray,
                 gripper_pos: float | None):
    """计算 cartesianCtrlCmd 的 cmd、ang_speed、lin_speed。"""
    target_pos, target_rot = state.get_target()

    # 线速度
    lin_err  = target_pos - current_pos
    lin_norm = np.linalg.norm(lin_err)
    if lin_norm > _MIN_LINEAR_THRESH:
        lin_speed = min(lin_norm * state.kp_linear, state.max_linear_vel)
        lin_dir   = lin_err / lin_norm
    else:
        lin_speed = 0.0
        lin_dir   = np.zeros(3)

    # 角速度
    rot_err  = (Rotation.from_matrix(target_rot)
                * Rotation.from_matrix(current_rot).inv()).as_rotvec()
    ang_norm = np.linalg.norm(rot_err)
    if ang_norm > _MIN_ANGULAR_THRESH:
        ang_speed = min(ang_norm * state.kp_angular, state.max_angular_vel)
        rot_dir   = rot_err / ang_norm
    else:
        ang_speed = 0.0
        rot_dir   = np.zeros(3)

    g_dir = _gripper_dir(gripper_pos)
    cmd   = list(rot_dir) + list(lin_dir) + [g_dir]
    return cmd, ang_speed, lin_speed


def _goto_start(arm, armState):
    """移动到预设起始位置（MoveL），并重新进入笛卡尔跟踪模式。"""
    arm.backToStart()
    arm.labelRun("forward")
    arm.startTrack(armState.CARTESIAN)
    arm.MoveL(START_CARTESIAN, START_GRIPPER, START_SPEED)


def control_loop(arm, armState, model, state: SharedState, stop_event: threading.Event):
    """
    主控制循环，在独立线程中运行。

    优先级：
      1. 复位（backToStart → 重新进入 CARTESIAN 模式）
      2. 消费 Pico twist → 更新 target
      3. 精细控制（摇杆）→ 叠加 target
      4. Watchdog 超时 或 未校准 → 发零速保持
      5. P 控制器 → cartesianCtrlCmd
    """
    interval = 1.0 / CONTROL_RATE
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
            print("[Control] 复位完成，请重新 Grip 校准")
            time.sleep(max(0.0, interval - (time.time() - t0)))
            continue

        # 2. 正运动学获取当前位姿
        try:
            fk = model.forwardKinematics(np.array(arm.q), 6)
            current_pos = np.array(fk[:3, 3], dtype=float)
            current_rot = np.array(fk[:3, :3], dtype=float)
            state.set_fk(current_pos, current_rot)
        except Exception as e:
            print(f"[Control] FK 失败: {e}")
            time.sleep(interval)
            continue

        # 3. 消费 Pico twist
        twist = state.pop_twist()
        if twist is not None:
            state.set_target_from_offset(twist[0], twist[1])

        # 4. 精细控制（摇杆）
        fine = state.pop_fine_delta()
        if fine is not None:
            state.apply_fine_delta_to_target(fine)

        # 5. 夹爪指令
        gripper_pos = state.pop_gripper()

        # 6. 未校准或 watchdog 超时 → 发零速
        if not state.is_calibrated() or not state.is_watchdog_ok():
            arm.cartesianCtrlCmd([0, 0, 0, 0, 0, 0, 0], 0.0, 0.0)
            time.sleep(max(0.0, interval - (time.time() - t0)))
            continue

        # 7. P 控制器 → 速度指令
        cmd, ang_speed, lin_speed = _compute_cmd(
            state, current_pos, current_rot, gripper_pos
        )
        arm.cartesianCtrlCmd(cmd, ang_speed, lin_speed)

        time.sleep(max(0.0, interval - (time.time() - t0)))
