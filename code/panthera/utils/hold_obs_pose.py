#!/usr/bin/env python3
"""
hold_obs_pose.py — 将机械臂移动到感知臂观测点位 3 并保持

目标关节角（Observation Point id=3，前方俯视）：
  [-0.12001, 1.10961, 1.43948, -0.77974, -0.09927, -0.08042]

用法：
  python hold_obs_pose.py /home/xuroboticslab/ws/Panthera-HT_SDK/panthera_python/robot_param/Percep.yaml
"""

import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "joystick_control"))
from Panthera_lib import Panthera  # noqa: E402


# ============================================================
# 目标位姿（Observation Point id=3）
# ============================================================

TARGET_JOINT_ANGLES = [
    -0.12000884115695953,
     1.1096105575561523,
     1.439477801322937,
    -0.7797433137893677,
    -0.09927432984113693,
    -0.08042477071285248,
]

# ============================================================
# 控制参数
# ============================================================

MOVEJ_DURATION    = 5.0
MOVEJ_MAX_TORQUE  = [21.0, 36.0, 36.0, 21.0, 10.0, 10.0]
CONTROL_HZ        = 50

KP = [10.0, 21.0, 21.0, 16.0, 13.0, 1.0]
KD = [1.0,  2.0,  2.0,  0.9,  0.8,  0.1]
TAU_LIMIT = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0])


# ============================================================
# 辅助
# ============================================================

def init_arm(yaml_path: str) -> Panthera:
    arm = Panthera(os.path.abspath(yaml_path))
    arm.send_get_motor_state_cmd()
    arm.motor_send_cmd()
    time.sleep(0.3)
    return arm


def move_to_target(arm: Panthera):
    print(f"[hold_obs_pose] 移动到目标位置...")
    print(f"  目标关节角: {[round(v, 4) for v in TARGET_JOINT_ANGLES]}")
    arm.moveJ(
        TARGET_JOINT_ANGLES,
        duration=MOVEJ_DURATION,
        max_tqu=MOVEJ_MAX_TORQUE,
        iswait=True,
    )
    time.sleep(0.5)
    actual = list(arm.get_current_pos())
    print(f"  实际关节角: {[round(v, 4) for v in actual]}")
    print("[hold_obs_pose] ✓ 已到达目标位置，进入保持模式（Ctrl-C 退出）")


def hold_loop(arm: Panthera):
    dt = 1.0 / CONTROL_HZ
    zero_vel = [0.0] * 6

    while True:
        t0 = time.monotonic()

        arm.send_get_motor_state_cmd()
        arm.motor_send_cmd()

        grav = list(arm.get_Gravity())
        tau  = np.clip(np.array(grav), -TAU_LIMIT, TAU_LIMIT).tolist()

        arm.pos_vel_tqe_kp_kd(
            TARGET_JOINT_ANGLES,
            zero_vel,
            tau,
            KP,
            KD,
        )

        elapsed = time.monotonic() - t0
        sleep_t = dt - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)


# ============================================================
# 入口
# ============================================================

def main():
    if len(sys.argv) != 2:
        sys.exit(f"用法: python {sys.argv[0]} <Percep.yaml路径>")

    yaml_path = sys.argv[1]

    print("=" * 55)
    print(f"hold_obs_pose — Observation Point id=3 (前方俯视)")
    print(f"yaml: {yaml_path}")
    print("=" * 55)

    arm = init_arm(yaml_path)

    try:
        move_to_target(arm)
        hold_loop(arm)
    except KeyboardInterrupt:
        print("\n[hold_obs_pose] Ctrl-C — 退出")
    finally:
        print("[hold_obs_pose] 完成")


if __name__ == "__main__":
    main()
