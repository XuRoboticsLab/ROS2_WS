#!/usr/bin/env python3
"""
机械臂控制端 - 双臂版 (运行在机械臂 conda 环境)

双臂控制:
  右手 Grip按住:   右臂跟随右手柄
  左手 Grip按住:   左臂跟随左手柄
  各自独立校准、独立回零、独立夹爪控制
  左右臂可以同时或单独操作

硬件:
  右臂: CAN1 (serial_id: 1, Follower.yaml)
  左臂: CAN2 (serial_id: 2, Follower_left.yaml)

运行: conda activate robot_env && python 7_xr_cartesian_pos_control.py
先启动 xr_server.py, 再启动本程序。
"""

import time
import sys
import os
import numpy as np
from Panthera_lib import Panthera

# 共享内存通信模块
from xr_robot_shm import XRSharedMemReader


# ======================== 工具函数 ========================

def quat_to_rotation_matrix(qx, qy, qz, qw):
    """四元数转3x3旋转矩阵"""
    norm = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm < 1e-10:
        return np.eye(3)
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
    R = np.array([
        [1 - 2*(qy*qy + qz*qz),   2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),       1 - 2*(qx*qx + qz*qz),   2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),       2*(qy*qz + qx*qw),       1 - 2*(qx*qx + qy*qy)]
    ])
    return R


def xr_pose_to_pos_rot(pose_7d):
    """XR 7D位姿 → 位置(3,) + 旋转矩阵(3,3)"""
    pos = np.array(pose_7d[:3], dtype=np.float64)
    rot = quat_to_rotation_matrix(pose_7d[3], pose_7d[4], pose_7d[5], pose_7d[6])
    return pos, rot


def compute_delta_rotation(R_current, R_initial):
    """旋转增量: delta_R = R_current * R_initial^T"""
    return R_current @ R_initial.T


def apply_delta_rotation(R_robot_init, delta_R):
    """应用旋转增量: R_target = delta_R * R_robot_init"""
    return delta_R @ R_robot_init


# ======================== 坐标系映射 ========================
# 校准结果 (两条臂使用相同映射, 如果安装方向不同需要各自一个)

XR_TO_ROBOT_POS = np.array([
    [+0.0, +0.0, +1.0],   # 机械臂 X (前)
    [+1.0, +0.0, +0.0],   # 机械臂 Y (左)
    [+0.0, +1.0, +0.0],   # 机械臂 Z (上)
])

XR_TO_ROBOT_ROT = XR_TO_ROBOT_POS.copy()

# 位置缩放
POS_SCALE = 1

# Grip 阈值
GRIP_THRESHOLD = 0.5

# 双击检测窗口 (秒)
DOUBLE_TAP_WINDOW = 0.4


# ======================== 单臂控制器 ========================

class ArmController:
    """单条机械臂的遥操作控制器 (左右臂各一个实例)"""

    def __init__(self, robot, name="arm"):
        self.robot = robot
        self.name = name

        # 状态
        self.is_calibrated = False
        self.is_resetting = False

        # 初始位姿
        self.xr_init_pos = None
        self.xr_init_rot = None
        self.robot_init_pos = None
        self.robot_init_rot = None

        # 目标关节角
        self.last_valid_joints = np.array(robot.get_current_pos(), dtype=np.float64)

        # 零位关节角
        self.zero_joints = np.array([0.0] * 6, dtype=np.float64)

        # 控制参数
        self.kp = [10.0, 21.0, 21.0, 16.0, 13.0, 1.0]
        self.kd = [1.0, 2.0, 2.0, 0.9, 0.8, 0.1]
        self.gripper_kp = 4.0
        self.gripper_kd = 0.4

        # 摩擦补偿
        self.Fc = np.array([0.15, 0.12, 0.12, 0.12, 0.04, 0.04])
        self.Fv = np.array([0.05, 0.05, 0.05, 0.03, 0.02, 0.02])
        self.vel_threshold = 0.02

        # 力矩限幅
        self.tau_limit = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0])

        # 安全限幅
        self.max_joint_delta_per_step = 0.05

        # 统计
        self.ik_fail_count = 0
        self.ik_total_count = 0

        # Grip 状态 (用于在机械臂端做上升沿检测)
        self.prev_grip_pressed = False
        self.last_grip_press_time = 0

    def calibrate(self, xr_pose_7d):
        """校准"""
        print(f"\n--- [{self.name}] 校准 ---")
        self.xr_init_pos, self.xr_init_rot = xr_pose_to_pos_rot(xr_pose_7d)
        fk = self.robot.forward_kinematics()
        self.robot_init_pos = np.array(fk['position'], dtype=np.float64)
        self.robot_init_rot = np.array(fk['rotation'], dtype=np.float64)
        self.last_valid_joints = np.array(self.robot.get_current_pos(), dtype=np.float64)
        self.is_calibrated = True
        self.ik_fail_count = 0
        self.ik_total_count = 0
        print(f"  手柄: [{self.xr_init_pos[0]:.4f}, {self.xr_init_pos[1]:.4f}, {self.xr_init_pos[2]:.4f}]")
        print(f"  机械臂: [{self.robot_init_pos[0]:.4f}, {self.robot_init_pos[1]:.4f}, {self.robot_init_pos[2]:.4f}]")

    def compute_target(self, xr_pose_7d):
        """计算目标末端位姿"""
        xr_pos, xr_rot = xr_pose_to_pos_rot(xr_pose_7d)
        delta_pos_xr = xr_pos - self.xr_init_pos
        delta_pos_robot = XR_TO_ROBOT_POS @ delta_pos_xr * POS_SCALE
        if np.linalg.norm(delta_pos_robot) > 0.3:
            delta_pos_robot = np.zeros(3)
        target_pos = self.robot_init_pos + delta_pos_robot
        delta_rot_xr = compute_delta_rotation(xr_rot, self.xr_init_rot)
        delta_rot_robot = XR_TO_ROBOT_ROT @ delta_rot_xr @ XR_TO_ROBOT_ROT.T
        target_rot = apply_delta_rotation(self.robot_init_rot, delta_rot_robot)
        return target_pos, target_rot

    def step(self, xr_pose_7d, trigger_value):
        """执行一步控制"""
        target_pos, target_rot = self.compute_target(xr_pose_7d)
        self.ik_total_count += 1
        solved = self.robot.inverse_kinematics(target_pos, target_rot, self.last_valid_joints)
        ik_ok = solved is not None
        if ik_ok:
            solved = np.array(solved, dtype=np.float64)
            joint_delta = np.clip(
                solved - self.last_valid_joints,
                -self.max_joint_delta_per_step,
                self.max_joint_delta_per_step
            )
            self.last_valid_joints = self.last_valid_joints + joint_delta
        else:
            self.ik_fail_count += 1
        self._send_control(self.last_valid_joints)
        gripper_pos = (1.0 - trigger_value) * 2.0
        self.robot.gripper_control_MIT(gripper_pos, 0, 0, self.gripper_kp, self.gripper_kd)
        return target_pos, ik_ok

    def hold_position(self):
        """保持当前位置"""
        self._send_control(self.last_valid_joints)

    def move_to_zero(self):
        """平滑回零, 返回 True=仍在运动, False=已到达"""
        delta = self.zero_joints - self.last_valid_joints
        max_delta = 0.002
        if np.max(np.abs(delta)) < 0.01:
            self.last_valid_joints = self.zero_joints.copy()
            self._send_control(self.last_valid_joints)
            return False
        delta = np.clip(delta, -max_delta, max_delta)
        self.last_valid_joints = self.last_valid_joints + delta
        self._send_control(self.last_valid_joints)
        return True

    def handle_grip(self, grip_value, xr_pose_7d, trigger_value):
        """处理单条臂的完整Grip逻辑
        
        Args:
            grip_value: 该手的grip值
            xr_pose_7d: 该手的位姿
            trigger_value: 该手的trigger值
        
        Returns:
            target_pos: 当前目标位置 (用于反馈), 或 None
            ik_ok: IK是否成功
        """
        grip_pressed = grip_value > GRIP_THRESHOLD
        
        # 上升沿检测
        if grip_pressed and not self.prev_grip_pressed:
            now = time.time()
            if now - self.last_grip_press_time < DOUBLE_TAP_WINDOW:
                # 双击 → 回零
                self.is_resetting = True
                self.is_calibrated = False
                print(f"\n[{self.name}] 双击Grip → 回零!")
            else:
                # 单击 → 校准
                self.is_resetting = False
                self.calibrate(xr_pose_7d)
            self.last_grip_press_time = now
        
        self.prev_grip_pressed = grip_pressed
        
        # 正在回零
        if self.is_resetting:
            still_moving = self.move_to_zero()
            if not still_moving:
                self.is_resetting = False
                print(f"\n[{self.name}] 已到达零位!")
            return None, True
        
        # Grip按住且已校准 → 跟踪
        if grip_pressed and self.is_calibrated:
            target_pos, ik_ok = self.step(xr_pose_7d, trigger_value)
            return target_pos, ik_ok
        
        # 未激活 → 保持位置
        self.hold_position()
        return None, True

    def _send_control(self, target_joints):
        """发送关节控制指令"""
        if target_joints is None:
            return
        vel = self.robot.get_current_vel()
        gra = self.robot.get_Gravity()
        friction = self.robot.get_friction_compensation(vel, self.Fc, self.Fv, self.vel_threshold)
        tor = np.clip(np.array(gra) + friction, -self.tau_limit, self.tau_limit)
        zero_vel = [0.0] * self.robot.motor_count
        self.robot.pos_vel_tqe_kp_kd(
            target_joints.tolist(), zero_vel, tor.tolist(),
            self.kp, self.kd
        )

    @property
    def ik_success_rate(self):
        if self.ik_total_count == 0:
            return 100.0
        return (1 - self.ik_fail_count / self.ik_total_count) * 100


# ======================== 主函数 ========================

def main():
    print("=" * 60)
    print("机械臂控制端 (双臂)")
    print("=" * 60)

    # ---- 初始化双臂 ----
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # 右臂: CAN1 (serial_id: 1)
    print("\n初始化右臂 (CAN1)...")
    config_path_R = os.path.join(script_dir, "../robot_param/Right.yaml")
    robot_R = Panthera(config_path_R)
    print("[OK] 右臂初始化完成")

    # 左臂: CAN2 (serial_id: 2)
    print("\n初始化左臂 (CAN2)...")
    config_path_L = os.path.join(script_dir, "../robot_param/Left.yaml")
    robot_L = Panthera(config_path_L)
    print("[OK] 左臂初始化完成")

    # ---- 连接共享内存 ----
    print("\n连接共享内存 (请确保 xr_server.py 已启动)...")
    reader = XRSharedMemReader()

    # ---- 创建控制器 ----
    ctrl_R = ArmController(robot_R, name="右臂")
    ctrl_L = ArmController(robot_L, name="左臂")

    print("\n" + "-" * 60)
    print("准备就绪! 双臂遥操作")
    print("  右手 Grip按住: 控制右臂")
    print("  左手 Grip按住: 控制左臂")
    print("  可以同时操作, 也可以单独操作")
    print("  连按两次 Grip: 对应臂回零")
    print("-" * 60 + "\n")

    loop_count = 0

    try:
        while True:
            # ---- 读取共享内存 ----
            data = reader.read_xr_data()

            # 数据超时
            if not data['is_fresh']:
                # 回零中继续回零
                if ctrl_R.is_resetting:
                    if not ctrl_R.move_to_zero():
                        ctrl_R.is_resetting = False
                        print("\n[右臂] 已到达零位!")
                else:
                    ctrl_R.hold_position()
                
                if ctrl_L.is_resetting:
                    if not ctrl_L.move_to_zero():
                        ctrl_L.is_resetting = False
                        print("\n[左臂] 已到达零位!")
                else:
                    ctrl_L.hold_position()
                
                if loop_count % 500 == 0:
                    print(f"\r[等待] XR 数据超时...    ", end="", flush=True)
                loop_count += 1
                time.sleep(0.001)
                continue

            # 紧急停止
            if data['emergency_stop']:
                ctrl_R.is_resetting = False
                ctrl_L.is_resetting = False
                ctrl_R.hold_position()
                ctrl_L.hold_position()
                if loop_count % 500 == 0:
                    print(f"\r[急停] 双臂已停止! 按 Grip 恢复    ", end="", flush=True)
                loop_count += 1
                time.sleep(0.001)
                continue

            # ---- 右臂控制 ----
            target_pos_R, ik_ok_R = ctrl_R.handle_grip(
                data['right_grip'], data['right_pose'], data['right_trigger']
            )

            # ---- 左臂控制 ----
            target_pos_L, ik_ok_L = ctrl_L.handle_grip(
                data['left_grip'], data['left_pose'], data['left_trigger']
            )

            # ---- 写入反馈 ----
            # 用当前FK位置作为反馈
            ee_r = target_pos_R if target_pos_R is not None else np.zeros(3)
            ee_l = target_pos_L if target_pos_L is not None else np.zeros(3)
            
            # 综合IK成功率
            total_ik = ctrl_R.ik_total_count + ctrl_L.ik_total_count
            total_fail = ctrl_R.ik_fail_count + ctrl_L.ik_fail_count
            combined_ik_rate = (1 - total_fail / max(total_ik, 1)) * 100

            reader.write_robot_feedback(
                ee_pos_right=ee_r,
                ee_pos_left=ee_l,
                ik_success_rate=combined_ik_rate,
                connected=True,
                ik_ok=ik_ok_R and ik_ok_L,
                tracking=(data['right_grip'] > GRIP_THRESHOLD or 
                         data['left_grip'] > GRIP_THRESHOLD),
            )

            # ---- 打印 (20Hz) ----
            if loop_count % 50 == 0:
                r_status = "回零" if ctrl_R.is_resetting else ("控制" if data['right_grip'] > GRIP_THRESHOLD and ctrl_R.is_calibrated else "待机")
                l_status = "回零" if ctrl_L.is_resetting else ("控制" if data['left_grip'] > GRIP_THRESHOLD and ctrl_L.is_calibrated else "待机")
                
                # 获取实际末端位置
                info_parts = [f"右:{r_status}"]
                if r_status == "控制":
                    fk_r = robot_R.forward_kinematics()
                    if fk_r:
                        p = fk_r['position']
                        info_parts.append(f"[{p[0]:.3f},{p[1]:.3f},{p[2]:.3f}]")
                    info_parts.append(f"IK:{ctrl_R.ik_success_rate:.0f}%")
                
                info_parts.append(f" 左:{l_status}")
                if l_status == "控制":
                    fk_l = robot_L.forward_kinematics()
                    if fk_l:
                        p = fk_l['position']
                        info_parts.append(f"[{p[0]:.3f},{p[1]:.3f},{p[2]:.3f}]")
                    info_parts.append(f"IK:{ctrl_L.ik_success_rate:.0f}%")
                
                print(f"\r{''.join(info_parts)}    ", end="", flush=True)

            loop_count += 1
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n\n双臂控制端退出")
        if ctrl_R.ik_total_count > 0:
            print(f"  右臂 IK: {ctrl_R.ik_total_count}次, 成功率{ctrl_R.ik_success_rate:.1f}%")
        if ctrl_L.ik_total_count > 0:
            print(f"  左臂 IK: {ctrl_L.ik_total_count}次, 成功率{ctrl_L.ik_success_rate:.1f}%")
    finally:
        reader.close()
        print("所有电机已停止")


if __name__ == "__main__":
    main()