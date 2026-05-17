#!/usr/bin/env python3
# ─────────────────────────────────────────────
#  main.py  —  VLA 动作执行节点
#
#  功能：
#    1. 订阅 /vla/actions（std_msgs/Float32MultiArray），将动作 chunk 入队
#    2. 以 control_hz 频率逐步执行队列中的关节目标位置
#    3. 队列为空时保持当前位置（重力补偿）
#    4. 以 publish_hz 频率向 ROS 发布左右臂的 JointState + PoseStamped
#
#  用法:
#    python main.py --config /path/to/vla_control.yaml
# ─────────────────────────────────────────────

# !! argparse 必须在所有本地 import 之前 !!
import argparse
import os
import sys

_parser = argparse.ArgumentParser(description="VLA Control Node")
_parser.add_argument("--config", "-c", required=True, metavar="PATH",
                     help="vla_control.yaml 路径")
_args = _parser.parse_args()
os.environ["VLA_CONTROL_CONFIG"] = os.path.abspath(_args.config)

import time
import threading

import roslibpy

sys.path.insert(0, os.path.dirname(__file__))
from Panthera_lib import Panthera  # noqa: E402

from vla_config import (
    ROSBRIDGE_HOST, ROSBRIDGE_PORT,
    RIGHT_MOTOR_CFG, RIGHT_JOINT_NAMES,
    LEFT_MOTOR_CFG,  LEFT_JOINT_NAMES,
    SAFE_JOINT_POS, SAFE_JOINT_VEL,
    KP, KD, GRIPPER_KP, GRIPPER_KD,
    TOPIC_VLA_ACTIONS,
)
from shared_state import SharedState
from control_loop import ArmController, control_loop
from ros_bridge import JointStatePublisher, make_actions_callback, make_trigger_publisher


# ══════════════════════════════════════════════════════════════════════════
#  初始化辅助
# ══════════════════════════════════════════════════════════════════════════

def _init_robot(motor_cfg: str, label: str) -> Panthera:
    print(f"[Init] 初始化 {label} 机械臂: {motor_cfg}")
    robot = Panthera(motor_cfg)
    robot.send_get_motor_state_cmd()
    robot.motor_send_cmd()
    time.sleep(0.3)
    print(f"[Init] {label} 机械臂就绪")
    return robot


def _move_to_safe(robot: Panthera, label: str):
    print(f"[Init] {label} 移动到安全位置...")
    ok = robot.Joint_Pos_Vel(SAFE_JOINT_POS, SAFE_JOINT_VEL, iswait=True)
    if not ok:
        sys.exit(f"[Init] ✗ {label} 移动到安全位置失败，退出")
    robot.send_get_motor_state_cmd()
    robot.motor_send_cmd()
    time.sleep(0.3)
    print(f"[Init] {label} ✓ 已到达安全位置")


# ══════════════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════════════

def main():
    print("=" * 60)
    print("VLA Control Node")
    print("=" * 60)

    # ── 初始化机械臂 ─────────────────────────────────────────────────────
    right_robot = None
    left_robot  = None

    if RIGHT_MOTOR_CFG and RIGHT_JOINT_NAMES:
        right_robot = _init_robot(RIGHT_MOTOR_CFG, "右臂")
        _move_to_safe(right_robot, "右臂")

    if LEFT_MOTOR_CFG and LEFT_JOINT_NAMES:
        left_robot = _init_robot(LEFT_MOTOR_CFG, "左臂")
        _move_to_safe(left_robot, "左臂")

    if right_robot is None and left_robot is None:
        sys.exit("[Init] ✗ 未配置任何机械臂，检查 arms.right / arms.left 配置")

    # ── 构造控制器 ───────────────────────────────────────────────────────
    right_ctrl = (
        ArmController(right_robot, RIGHT_JOINT_NAMES, KP, KD, GRIPPER_KP, GRIPPER_KD)
        if right_robot else None
    )
    left_ctrl = (
        ArmController(left_robot, LEFT_JOINT_NAMES, KP, KD, GRIPPER_KP, GRIPPER_KD)
        if left_robot else None
    )

    # ── 共享状态 ──────────────────────────────────────────────────────────
    state = SharedState()

    # ── 连接 rosbridge ───────────────────────────────────────────────────
    print(f"\n[ROS] 连接 {ROSBRIDGE_HOST}:{ROSBRIDGE_PORT}...")
    ros = roslibpy.Ros(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
    ros.run()
    if not ros.is_connected:
        sys.exit("[ROS] ✗ rosbridge 连接失败")
    print("[ROS] ✓ 已连接")

    # ── 订阅 VLA actions ─────────────────────────────────────────────────
    actions_sub = roslibpy.Topic(ros, TOPIC_VLA_ACTIONS, "std_msgs/Float32MultiArray")
    actions_sub.subscribe(make_actions_callback(state))
    print(f"[ROS] 已订阅 {TOPIC_VLA_ACTIONS}")

    # ── 状态发布器 + Trigger 发布器 ──────────────────────────────────────
    publisher = JointStatePublisher(ros)
    trigger_pub, trigger_fn = make_trigger_publisher(ros)

    # ── 启动线程 ──────────────────────────────────────────────────────────
    stop_event = threading.Event()

    ctrl_thread = threading.Thread(
        target=control_loop,
        args=(right_ctrl, left_ctrl, state, stop_event, trigger_fn),
        daemon=True, name="ControlLoop",
    )
    pub_thread = threading.Thread(
        target=publisher.run,
        args=(state, stop_event),
        daemon=True, name="StatePublisher",
    )

    ctrl_thread.start()
    pub_thread.start()
    print("\n[Main] 运行中，Ctrl+C 退出\n")

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n\n[Main] 退出信号...")
    finally:
        stop_event.set()
        ctrl_thread.join(timeout=2.0)
        pub_thread.join(timeout=2.0)
        actions_sub.unsubscribe()
        publisher.unadvertise_all()
        trigger_pub.unadvertise()
        ros.terminate()

        # 返回安全位置
        print("[Main] 返回安全位置...")
        if right_robot:
            right_robot.Joint_Pos_Vel(SAFE_JOINT_POS, SAFE_JOINT_VEL, iswait=True)
        if left_robot:
            left_robot.Joint_Pos_Vel(SAFE_JOINT_POS, SAFE_JOINT_VEL, iswait=True)
        print("[Main] 完成")


if __name__ == "__main__":
    main()
