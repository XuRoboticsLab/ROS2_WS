#!/usr/bin/env python3
# ─────────────────────────────────────────────
#  main.py  —  入口：初始化 & 线程启动
#
#  用法：
#    python main.py --config configs/right.yaml
# ─────────────────────────────────────────────
import os
import sys
import argparse

_parser = argparse.ArgumentParser(description="Nero Pico Position Control")
_parser.add_argument(
    "--config", "-c",
    required=True,
    metavar="PATH",
    help="config.yaml 路径",
)
_args = _parser.parse_args()
os.environ["PICO_CONTROL_CONFIG"] = os.path.abspath(_args.config)

# 将 nero SDK 加入 Python 路径
_NERO_SDK = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../nero"))
if _NERO_SDK not in sys.path:
    sys.path.insert(0, _NERO_SDK)

import time
import threading
import numpy as np
import roslibpy
from scipy.spatial.transform import Rotation

from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel

from config import (
    CAN_CHANNEL, FIRMWARE_VERSION, EFFECTOR_KIND,
    SAFE_JOINT_POS, SAFE_SPEED_PERCENT, SPEED_PERCENT,
    ROSBRIDGE_HOST, ROSBRIDGE_PORT,
    PARAM_SERVER_PORT, ARM_NAME,
)
from shared_state import SharedState
from ros_bridge import RosSubscribers, publisher_thread
from control_loop import control_loop, _wait_motion_done, _rpy_to_rot
import param_server


def _init_robot():
    print("[Init] 连接 Nero 机械臂...")
    nero_cfg = create_agx_arm_config(
        robot=ArmModel.NERO,
        firmeware_version=FIRMWARE_VERSION,
        interface="socketcan",
        channel=CAN_CHANNEL,
    )
    robot = AgxArmFactory.create_arm(nero_cfg)
    robot.connect()
    print(f"[Init] ✓ 已连接，CAN 通道: {robot.get_channel()}")

    robot._auto_set_motion_mode_enabled = False

    print("[Init] 使能电机...")
    deadline = time.monotonic() + 5.0
    enabled = False
    while time.monotonic() < deadline:
        if robot.enable():
            enabled = True
            break
        time.sleep(0.05)
    if not enabled:
        print("[Init] 警告：电机使能未完全确认，继续执行...")
    else:
        print("[Init] ✓ 电机已使能")

    gripper = None
    if EFFECTOR_KIND:
        print(f"[Init] 初始化夹爪 ({EFFECTOR_KIND})...")
        gripper = robot.init_effector(EFFECTOR_KIND)
        print("[Init] ✓ 夹爪已初始化")

    print("[Init] 移动到安全位置...")
    robot.set_speed_percent(SAFE_SPEED_PERCENT)
    robot.move_j(SAFE_JOINT_POS)
    if not _wait_motion_done(robot, timeout=15.0):
        raise RuntimeError("移动到安全位置超时")
    print("[Init] ✓ 已到达安全位置")

    robot.set_speed_percent(SPEED_PERCENT)
    robot.set_motion_mode('p')

    return robot, gripper


def _init_state(robot, gripper) -> SharedState:
    fp = None
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        fp = robot.get_flange_pose()
        if fp is not None:
            break
        time.sleep(0.05)
    if fp is None:
        raise RuntimeError("无法获取末端位姿，请检查机械臂连接")

    x, y, z, roll, pitch, yaw = fp.msg
    rot = _rpy_to_rot([roll, pitch, yaw])

    state = SharedState()
    state.reset_target_to([x, y, z], rot)

    ja = robot.get_joint_angles()
    if ja is not None:
        state.last_valid_joint_pos = list(ja.msg)

    gripper_pos = 0.0
    if gripper is not None:
        gs = gripper.get_gripper_status()
        if gs is not None:
            gripper_pos = gs.msg.value

    state.set_robot_state(
        state.last_valid_joint_pos,
        [0.0] * 7,
        [x, y, z],
        rot,
        gripper_pos=gripper_pos,
    )
    print(f"[Init] 初始末端位置: {[f'{v:.3f}' for v in [x, y, z]]}")
    print("[Init] 请按 Pico init 键完成校准，之后即可开始位置控制")
    return state


def main():
    print("=" * 60)
    print("Nero Pico Position Control 节点")
    print("=" * 60)

    robot, gripper = _init_robot()
    state = _init_state(robot, gripper)

    param_server.start(state, port=PARAM_SERVER_PORT, arm_name=ARM_NAME)

    print(f"\n[ROS] 连接 {ROSBRIDGE_HOST}:{ROSBRIDGE_PORT}...")
    ros = roslibpy.Ros(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
    ros.run()
    if not ros.is_connected:
        raise RuntimeError("rosbridge 连接失败")
    print("[ROS] ✓ 已连接")

    subscribers = RosSubscribers(ros, state)
    stop_event  = threading.Event()

    threads = [
        threading.Thread(
            target=publisher_thread,
            args=(ros, state, stop_event),
            daemon=True, name="StatePublisher",
        ),
        threading.Thread(
            target=control_loop,
            args=(robot, gripper, state, stop_event),
            daemon=True, name="ControlLoop",
        ),
    ]
    for t in threads:
        t.start()

    print("\n[Main] 运行中，Ctrl+C 退出\n")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[Main] 退出信号...")
    finally:
        stop_event.set()
        for t in threads:
            t.join(timeout=2.0)
        subscribers.unsubscribe_all()
        ros.terminate()

        print("[Main] 返回安全位置...")
        robot.set_speed_percent(SAFE_SPEED_PERCENT)
        robot.set_motion_mode('j')
        robot.move_j(SAFE_JOINT_POS)
        _wait_motion_done(robot, timeout=10.0)

        print("[Main] 下电...")
        robot.disable()
        robot.disconnect()
        print("[Main] 完成")


if __name__ == "__main__":
    main()
