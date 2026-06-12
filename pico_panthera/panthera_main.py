#!/usr/bin/env python3
# ─────────────────────────────────────────────
#  panthera_main.py  —  机械臂控制端入口（ZMQ 版）
#
#  运行: python panthera_main.py --config <config.yaml>
#  依赖: pyzmq, scipy, flask
# ─────────────────────────────────────────────
import os
import sys
import argparse

_parser = argparse.ArgumentParser(description="Panthera Pico Position Control (ZMQ)")
_parser.add_argument("--config", "-c", required=True, metavar="PATH", help="config.yaml 路径")
_args = _parser.parse_args()
os.environ["PICO_CONTROL_CONFIG"] = os.path.abspath(_args.config)

# Panthera_lib 在原始位置，追加到 sys.path 末尾（不能 insert(0) 否则会覆盖本目录同名模块）
_PANTHERA_LIB_DIR = os.path.join(
    os.path.dirname(__file__), "..", "code", "panthera", "pico_control"
)
sys.path.append(os.path.abspath(_PANTHERA_LIB_DIR))

import time
import threading
import numpy as np
from Panthera_lib import Panthera

from panthera_config import (
    MOTOR_CONFIG_PATH,
    IPC_ADDRESS, ARM_SIDE,
    SAFE_JOINT_POS, SAFE_JOINT_VEL,
    KP, KD,
    PARAM_SERVER_PORT, ARM_NAME, POSES_FILE,
)
from shared_state import SharedState
from ipc_bridge import ZmqSubscriber
from control_loop import control_loop
import param_server


def _make_callbacks(state: SharedState) -> dict:
    """构建 ZMQ 消息回调，语义与原 ros_bridge.py 完全一致。"""

    def on_cmd(msg):
        l, a = msg["linear"], msg["angular"]
        state.push_twist([l["x"], l["y"], l["z"]],
                         [a["x"], a["y"], a["z"]])

    def on_gripper(msg):
        if state.action_locked:
            return
        with state._lock:
            state.gripper_cmd = float(msg["data"])

    def on_reset(msg):
        if msg["data"]:
            with state._lock:
                state.reset_requested = True

    def on_init(msg):
        if msg["data"]:
            with state._lock:
                fk_pos = np.array(state.fk_position)
                fk_rot = state.fk_rotation.copy()
            state.set_calibration_pose(fk_pos, fk_rot)

    def on_fine_cmd(msg):
        l = msg["linear"]
        state.push_fine_delta([l["x"], l["y"]])

    return {
        "cmd":      on_cmd,
        "gripper":  on_gripper,
        "reset":    on_reset,
        "init":     on_init,
        "fine_cmd": on_fine_cmd,
    }


def init_robot() -> Panthera:
    print("[Init] 初始化机械臂...")
    config_path = os.path.abspath(MOTOR_CONFIG_PATH)
    robot = Panthera(config_path)
    robot.send_get_motor_state_cmd()
    robot.motor_send_cmd()
    time.sleep(0.3)

    print("[Init] 移动到安全位置...")
    ok = robot.Joint_Pos_Vel(SAFE_JOINT_POS, SAFE_JOINT_VEL, iswait=True)
    if not ok:
        raise RuntimeError("移动到安全位置失败")
    print("[Init] ✓ 已到达安全位置")
    time.sleep(0.5)

    robot.send_get_motor_state_cmd()
    robot.motor_send_cmd()
    time.sleep(0.1)
    return robot


def init_state(robot: Panthera) -> SharedState:
    state   = SharedState()
    fk      = robot.forward_kinematics()
    gripper = robot.get_current_state_gripper()
    state.reset_target_to(fk["position"], fk["rotation"])
    state.last_valid_joint_pos = robot.get_current_pos()
    state.set_robot_state(
        state.last_valid_joint_pos,
        robot.get_current_vel(),
        fk["position"],
        np.array(fk["rotation"], dtype=float),
        gripper_pos=gripper.position,
        gripper_vel=gripper.velocity,
    )
    print(f"[Init] 初始末端位置: {[f'{v:.3f}' for v in fk['position']]}")
    print("[Init] 请按 Pico init 键完成校准，之后即可开始位置控制")
    return state


def main():
    print("=" * 60)
    print(f"Panthera Pico Position Control 节点 ({ARM_NAME}, ZMQ 版)")
    print("=" * 60)

    robot = init_robot()
    state = init_state(robot)

    grav = np.array(robot.get_Gravity())
    robot.pos_vel_tqe_kp_kd(
        state.last_valid_joint_pos, [0.0] * robot.motor_count, grav, KP, KD
    )

    param_server.start(state, robot, port=PARAM_SERVER_PORT, arm_name=ARM_NAME, poses_file=POSES_FILE)

    print(f"\n[IPC] 连接 {IPC_ADDRESS}，订阅 {ARM_SIDE} 臂指令...")
    subscriber = ZmqSubscriber(ARM_SIDE, IPC_ADDRESS, _make_callbacks(state))

    stop_event = threading.Event()
    ctrl_thread = threading.Thread(
        target=control_loop,
        args=(robot, state, stop_event),
        daemon=True, name="ControlLoop",
    )
    ctrl_thread.start()

    print("\n[Main] 运行中，Ctrl+C 退出\n")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[Main] 退出信号...")
    finally:
        stop_event.set()
        ctrl_thread.join(timeout=2.0)
        subscriber.stop()
        print("[Main] 返回安全位置...")
        robot.Joint_Pos_Vel(SAFE_JOINT_POS, SAFE_JOINT_VEL, iswait=True)
        print("[Main] 完成")


if __name__ == "__main__":
    main()
