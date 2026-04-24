#!/usr/bin/env python3
import sys
import os
import json
import time
import argparse
import numpy as np
from scipy.spatial.transform import Rotation as R

_script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_script_dir, "joystick_control"))

from Panthera_lib import Panthera


def init_robot(config_path: str) -> Panthera:
    print("[Init] 初始化机械臂...")
    robot = Panthera(config_path)
    robot.send_get_motor_state_cmd()
    robot.motor_send_cmd()
    time.sleep(0.3)
    return robot


def parse_position(entry: dict) -> tuple:
    joints = [
        float(entry["joint_1"]),
        float(entry["joint_2"]),
        float(entry["joint_3"]),
        float(entry["joint_4"]),
        float(entry["joint_5"]),
        float(entry["joint_6"]),
    ]
    return joints, float(entry["gripper"])


def refresh_and_record(robot: Panthera) -> dict:
    robot.send_get_motor_state_cmd()
    robot.motor_send_cmd()
    time.sleep(0.05)

    joint_pos = robot.get_current_pos().tolist()
    gripper_pos = robot.get_current_pos_gripper()
    fk = robot.forward_kinematics()

    rotation_matrix = np.array(fk["rotation"]).tolist()
    quaternion_xyzw = R.from_matrix(np.array(fk["rotation"])).as_quat().tolist()

    return {
        "joint_positions": joint_pos,
        "gripper_position": gripper_pos,
        "end_effector": {
            "position": fk["position"],
            "rotation_matrix": rotation_matrix,
            "quaternion_xyzw": quaternion_xyzw,
        },
    }


def main():
    parser = argparse.ArgumentParser(
        description="依次移动机械臂到 rounded_positions.json 中的各位置并记录状态"
    )
    parser.add_argument("--config", "-c", required=True, metavar="PATH",
                        help="机械臂 YAML 配置文件路径")
    parser.add_argument("--positions", "-p", default="rounded_positions.json",
                        metavar="PATH", help="目标位置 JSON 文件（默认 rounded_positions.json）")
    parser.add_argument("--output", "-o", default="recorded_states.json",
                        metavar="PATH", help="输出 JSON 文件（默认 recorded_states.json）")
    parser.add_argument("--move-duration", type=float, default=3.0,
                        help="每次关节运动的时间（秒，默认 3.0）")
    parser.add_argument("--settle-time", type=float, default=3.0,
                        help="到位后等待稳定的时间（秒，默认 3.0）")
    args = parser.parse_args()

    robot = init_robot(os.path.abspath(args.config))

    positions_path = args.positions if os.path.isabs(args.positions) else \
        os.path.join(_script_dir, args.positions)
    with open(positions_path, "r") as f:
        positions = json.load(f)

    keys = sorted(positions.keys())
    print(f"\n[Main] 共 {len(keys)} 个目标位置: {keys}")

    results = {}

    for i, key in enumerate(keys):
        print(f"\n[{i+1}/{len(keys)}] 移动到 {key} ...")
        joint_target, gripper_target = parse_position(positions[key])

        # 速度按比例计算使各关节同步到达
        current_pos = robot.get_current_pos()
        deltas = np.abs(np.array(joint_target) - current_pos)
        max_delta = deltas.max()
        if max_delta < 1e-4:
            vel = [0.1] * robot.motor_count
        else:
            vel = (deltas / args.move_duration).tolist()
            vel = [max(v, 0.05) for v in vel]

        ok = robot.Joint_Pos_Vel(joint_target, vel, iswait=True)
        if not ok:
            print(f"  [警告] 关节运动失败，跳过 {key}")
            continue

        robot.gripper_control(gripper_target, vel=0.3)

        print(f"  已到达，等待 {args.settle_time}s 稳定...")
        time.sleep(args.settle_time)

        state = refresh_and_record(robot)
        results[key] = state

        print(f"  关节位置: {[f'{v:.4f}' for v in state['joint_positions']]}")
        print(f"  夹爪位置: {state['gripper_position']:.4f}")
        print(f"  末端位置: {[f'{v:.4f}' for v in state['end_effector']['position']]}")

    output_path = args.output if os.path.isabs(args.output) else \
        os.path.join(_script_dir, args.output)
    with open(output_path, "w") as f:
        json.dump(results, f, indent=2)

    print(f"\n[Main] 完成，已保存 {len(results)} 条记录到 {output_path}")


if __name__ == "__main__":
    main()
