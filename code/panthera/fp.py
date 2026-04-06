#!/usr/bin/env python3
"""
双主双从臂遥操作程序
Leader_1 (CAN6) -> Follower_1 (CAN1)
Leader_2 (CAN7) -> Follower_2 (CAN2)
"""
import time
import sys
import os
import numpy as np
from Panthera_lib import Panthera


def control_arm_pair(leader, follower, zero_pos, zero_vel, zero_kp, zero_kd,
                     kp, kd, gripper_kp, gripper_kd, Fc, Fv, vel_threshold, tor_threshold, pair_name=""):
    """控制一对主从臂"""
    # 获取主臂位置速度
    leader_positions = leader.get_current_pos()
    leader_velocity = leader.get_current_vel()
    follower_velocity = follower.get_current_vel()

    # 获取从臂力矩计算反馈力矩
    follower_torque = follower.get_current_torque()

    # 计算重力力矩
    leader_gra = leader.get_Gravity()
    follower_gra = follower.get_Gravity()

    # 计算从臂受到的外力
    tor_diff = np.array(follower_torque) - np.array(follower_gra)
    tor_diff[np.abs(tor_diff) < tor_threshold] = 0

    # 主臂力矩 (无力反馈模式)
    leader_tor = np.array(leader_gra) + leader.get_friction_compensation(leader_velocity, Fc, Fv, vel_threshold)

    # 从臂力矩
    follower_tor = np.array(follower_gra) + follower.get_friction_compensation(follower_velocity, Fc, Fv, vel_threshold)

    # 力矩限幅
    tau_limit = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0])
    leader_tor = np.clip(leader_tor, -tau_limit, tau_limit)
    follower_tor = np.clip(follower_tor, -tau_limit, tau_limit)

    # 运行控制
    leader.pos_vel_tqe_kp_kd(zero_pos, zero_vel, leader_tor, zero_kp, zero_kd)
    follower.pos_vel_tqe_kp_kd(leader_positions, leader_velocity, follower_tor, kp, kd)

    # 夹爪控制
    leader_gripper_positions = leader.get_current_pos_gripper()
    leader_gripper_velocity = leader.get_current_vel_gripper()
    follower_gripper = follower.get_current_state_gripper()
    gripper_torque = follower.get_friction_compensation(leader_gripper_velocity, 0.06, 0.0, 0.15) - follower_gripper.torque * 0.5
    # 注意：原代码此处用了 tor_diff，应该是对 gripper_torque 做阈值判断
    gripper_torque_filtered = gripper_torque if abs(gripper_torque) >= 0.2 else 0
    leader.gripper_control_MIT(1.5, 0, gripper_torque_filtered, 0.2, 0.02)
    follower.gripper_control_MIT(leader_gripper_positions, leader_gripper_velocity, 0, gripper_kp, gripper_kd)

    # 打印信息
    print(f"===== {pair_name} =====")
    for i in range(leader.motor_count):
        print(f"  关节{i+1}: 位置={leader_positions[i]:7.3f} rad, 速度={leader_velocity[i]:7.3f} rad/s")
    print(f"  反馈力矩: {tor_diff}")
    print(f"  夹爪力矩: {gripper_torque:7.3f} Nm")


if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # 创建4个机器人实例
    # Leader_1 (CAN6) -> Follower_1 (CAN1)
    # Leader_2 (CAN7) -> Follower_2 (CAN2)
    Leader_1 = Panthera(os.path.join(script_dir, "../robot_param/Leader_1.yaml"))   # CAN6
    Leader_2 = Panthera(os.path.join(script_dir, "../robot_param/Leader_2.yaml"))   # CAN7
    Follower_1 = Panthera(os.path.join(script_dir, "../robot_param/Follower_1.yaml"))  # CAN1
    Follower_2 = Panthera(os.path.join(script_dir, "../robot_param/Follower_2.yaml"))  # CAN2

    # 共用参数（假设4条臂规格一致，如不同可分别配置）
    motor_count = Leader_1.motor_count
    zero_pos = [0.0] * motor_count
    zero_vel = [0.0] * motor_count
    zero_kp = [0.0] * motor_count
    zero_kd = [0.0] * motor_count
    kp = [10.0, 21.0, 21.0, 16.0, 13.0, 1.0]
    kd = [1.0, 2.0, 2.0, 0.9, 0.8, 0.1]
    gripper_kp = 4.0
    gripper_kd = 0.4

    Fc = np.array([0.15, 0.12, 0.12, 0.12, 0.04, 0.04])
    Fv = np.array([0.05, 0.05, 0.05, 0.03, 0.02, 0.02])
    vel_threshold = 0.02
    tor_threshold = np.array([0.5, 1.0, 1.0, 0.5, 0.3, 0.3])

    try:
        while True:
            control_arm_pair(Leader_1, Follower_1,
                             zero_pos, zero_vel, zero_kp, zero_kd,
                             kp, kd, gripper_kp, gripper_kd,
                             Fc, Fv, vel_threshold, tor_threshold,
                             pair_name="Leader_1(CAN6) -> Follower_1(CAN1)")

            control_arm_pair(Leader_2, Follower_2,
                             zero_pos, zero_vel, zero_kp, zero_kd,
                             kp, kd, gripper_kp, gripper_kd,
                             Fc, Fv, vel_threshold, tor_threshold,
                             pair_name="Leader_2(CAN7) -> Follower_2(CAN2)")

            print('-' * 50)
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n\n程序被中断")
        print("所有电机已停止")
    except Exception as e:
        print(f"\n错误: {e}")