#!/usr/bin/env python3
# ─────────────────────────────────────────────
#  z1_main.py  —  Z1 机械臂控制端入口（IK + LOWCMD 版）
#
#  运行: python z1_main.py --config <arm_config.yaml>
#  依赖: unitree_arm_interface (Z1 SDK), pyzmq, scipy, flask
# ─────────────────────────────────────────────
import os
import sys
import argparse

_parser = argparse.ArgumentParser(description="Pico Z1 Arm Control")
_parser.add_argument("--config", "-c", required=True, metavar="PATH", help="arm config.yaml 路径")
_args = _parser.parse_args()
os.environ["PICO_Z1_ARM_CONFIG"] = os.path.abspath(_args.config)

import time
import threading
import numpy as np

from z1_config import (
    LIB_PATH, HAS_GRIPPER,
    IPC_ADDRESS, ARM_SIDE,
    START_CARTESIAN, START_GRIPPER, START_SPEED,
    PARAM_SERVER_PORT, ARM_NAME,
)
from shared_state import SharedState
from ipc_bridge import ZmqSubscriber
from control_loop import control_loop
import param_server

# Z1 SDK
sys.path.append(os.path.abspath(LIB_PATH))
import unitree_arm_interface


def _make_callbacks(state: SharedState, arm, model) -> dict:
    def on_cmd(msg):
        l, a = msg["linear"], msg["angular"]
        state.push_twist([l["x"], l["y"], l["z"]],
                         [a["x"], a["y"], a["z"]])

    def on_gripper(msg):
        state.push_gripper(float(msg["data"]))

    def on_reset(msg):
        if msg["data"]:
            with state._lock:
                state.reset_requested = True

    def on_init(msg):
        if msg["data"]:
            # 以当前末端 FK 矩阵作为校准基准
            T_fk = model.forwardKinematics(np.array(arm.q), 6)
            state.set_calibration(np.array(T_fk))

    def on_fine_cmd(msg):
        l = msg["linear"]
        state.push_fine_delta([l["x"], l["y"]])

    def on_emergency(msg):
        if msg.get("data"):
            print("\n[Emergency] 收到紧急停止信号")
            with state._lock:
                state.reset_requested = True

    return {
        "cmd":       on_cmd,
        "gripper":   on_gripper,
        "reset":     on_reset,
        "init":      on_init,
        "fine_cmd":  on_fine_cmd,
        "emergency": on_emergency,
    }


def init_arm():
    print("[Init] 初始化 Z1 机械臂...")
    arm      = unitree_arm_interface.ArmInterface(hasGripper=HAS_GRIPPER)
    armState = unitree_arm_interface.ArmFSMState
    model    = arm._ctrlComp.armModel

    arm.loopOn()
    arm.labelRun("forward")
    arm.startTrack(armState.CARTESIAN)

    print(f"[Init] 移动到初始位置 {START_CARTESIAN}...")
    arm.MoveL(START_CARTESIAN, START_GRIPPER, START_SPEED)
    print("[Init] ✓ 已到达初始位置")

    # 切换到 LOWCMD 模式，后续由控制循环负责发指令
    arm.setFsmLowcmd()
    print("[Init] 已切换到 LOWCMD 模式")

    return arm, armState, model


def main():
    print("=" * 60)
    print(f"Z1 Pico 控制节点 ({ARM_NAME}, IK+LOWCMD 版)")
    print("=" * 60)

    arm, armState, model = init_arm()
    state = SharedState()

    q0   = np.array(arm.q)
    T_fk = model.forwardKinematics(q0, 6)
    print(f"[Init] 初始末端位置 (xyz): {np.array(T_fk[:3, 3]).round(4)}")
    print("[Init] 请按 Pico Grip 键完成校准，之后即可开始控制")

    param_server.start(state, port=PARAM_SERVER_PORT, arm_name=ARM_NAME)

    print(f"\n[IPC] 连接 {IPC_ADDRESS}，订阅 {ARM_SIDE}:* ...")
    subscriber = ZmqSubscriber(ARM_SIDE, IPC_ADDRESS,
                               _make_callbacks(state, arm, model))

    stop_event  = threading.Event()
    ctrl_thread = threading.Thread(
        target=control_loop,
        args=(arm, armState, model, state, stop_event),
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
        try:
            arm.setWait(True)
            arm.backToStart()
            arm.loopOff()
        except Exception as e:
            print(f"[Main] 退出时机械臂操作失败: {e}")
        print("[Main] 完成")


if __name__ == "__main__":
    main()
