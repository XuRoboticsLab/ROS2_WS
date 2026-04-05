#!/usr/bin/env python3
"""
XR 数据发送端 - 双臂版 (运行在 XR conda 环境)

控制逻辑:
  右手 Grip键按住:   右臂跟随右手柄运动
  左手 Grip键按住:   左臂跟随左手柄运动
  Grip键松开:        对应机械臂停止
  Grip键连按两次:    对应机械臂回零位
  右手 Trigger键:    右臂夹爪
  左手 Trigger键:    左臂夹爪
  Y键:               紧急停止 (双臂同时)
  Ctrl+C:            退出

运行: conda activate xr_env && python xr_server.py
"""

import time
import sys
import numpy as np

# XRT SDK
try:
    import xrobotoolkit_sdk as xrt
except ImportError:
    print("错误: xrobotoolkit_sdk 未安装, 请确认在正确的 conda 环境中运行")
    sys.exit(1)

# 共享内存通信模块
from xr_robot_shm import XRSharedMemWriter


class GripDoubleTapDetector:
    """单个Grip键的双击检测器"""
    
    def __init__(self, name, threshold=0.5, double_tap_window=0.4):
        self.name = name
        self.threshold = threshold
        self.double_tap_window = double_tap_window
        self.prev_pressed = False
        self.last_press_time = 0
    
    def update(self, grip_value):
        """更新状态，返回 (is_active, request_init, reset_to_zero)"""
        pressed = grip_value > self.threshold
        request_init = False
        reset_to_zero = False
        
        # 上升沿
        if pressed and not self.prev_pressed:
            now = time.time()
            if now - self.last_press_time < self.double_tap_window:
                # 双击
                reset_to_zero = True
                print(f"\n[{self.name} Grip×2] 双击 → 回零!")
            else:
                # 单击
                request_init = True
                print(f"\n[{self.name} Grip] 按下 → 激活 + 校准")
            self.last_press_time = now
        
        # 下降沿
        if not pressed and self.prev_pressed:
            print(f"\n[{self.name} Grip] 松开 → 停止")
        
        self.prev_pressed = pressed
        
        is_active = pressed
        return is_active, request_init, reset_to_zero


def main():
    print("=" * 60)
    print("XR 数据发送端 (双臂)")
    print("=" * 60)
    print("控制说明:")
    print("  右手 Grip: 按住=控制右臂, 连按两次=右臂回零")
    print("  左手 Grip: 按住=控制左臂, 连按两次=左臂回零")
    print("  Trigger:   各自控制夹爪")
    print("  Y键:       紧急停止 (双臂)")
    print("  Ctrl+C:    退出\n")

    # 初始化 XRT SDK
    try:
        xrt.init()
        print("[OK] XRoboToolkit SDK 初始化成功")
    except Exception as e:
        print(f"[FAIL] XRT SDK 初始化失败: {e}")
        sys.exit(1)

    # 创建共享内存
    writer = XRSharedMemWriter()

    # 双击检测器 (左右各一个)
    right_grip_detector = GripDoubleTapDetector("右手")
    left_grip_detector = GripDoubleTapDetector("左手")

    # 状态
    emergency_stop = False
    prev_Y = False

    # 回零信号 (只发一帧)
    right_reset_to_zero = False
    left_reset_to_zero = False

    loop_count = 0
    print("\n等待 Pico 4 Ultra 连接...\n")

    try:
        while True:
            # ---- 读取 XR 数据 ----
            try:
                right_pose = list(xrt.get_right_controller_pose())
                left_pose = list(xrt.get_left_controller_pose())
                right_trigger = xrt.get_right_trigger()
                left_trigger = xrt.get_left_trigger()
                right_grip = xrt.get_right_grip()
                left_grip = xrt.get_left_grip()

                A = xrt.get_A_button()
                B = xrt.get_B_button()
                X = xrt.get_X_button()
                Y = xrt.get_Y_button()
            except Exception as e:
                if loop_count % 500 == 0:
                    print(f"\r[警告] XR 数据读取失败: {e}    ", end="", flush=True)
                time.sleep(0.01)
                loop_count += 1
                continue

            # ---- 右手 Grip 逻辑 ----
            r_active, r_request_init, r_reset = right_grip_detector.update(right_grip)
            if r_reset:
                right_reset_to_zero = True
                r_active = False

            # ---- 左手 Grip 逻辑 ----
            l_active, l_request_init, l_reset = left_grip_detector.update(left_grip)
            if l_reset:
                left_reset_to_zero = True
                l_active = False

            # ---- Y键: 紧急停止 (双臂同时) ----
            if Y and not prev_Y:
                emergency_stop = True
                print("\n[Y] !!!紧急停止!!! (按 Grip 恢复)")
            prev_Y = Y

            # 任意一手 grip 按下时解除急停
            if emergency_stop and (r_request_init or l_request_init):
                emergency_stop = False
                print("[Grip] 急停已解除")

            if emergency_stop:
                r_active = False
                l_active = False

            # ---- 合并标志位 ----
            # is_active: 任一手激活即 True (机械臂端根据各自grip值判断)
            # request_init: 任一手请求校准
            # reset_to_zero: 任一手请求回零
            # 为了让机械臂端区分左右，我们将左右的独立信号编码:
            #   is_active 仍用于整体状态
            #   具体哪只手激活，机械臂端通过 grip 值判断 (>0.5)
            #   request_init 和 reset_to_zero 也通过 grip 值的上升沿在机械臂端判断
            is_active = r_active or l_active
            request_init = r_request_init or l_request_init
            reset_to_zero = right_reset_to_zero or left_reset_to_zero

            # ---- 写入共享内存 ----
            buttons_dict = {
                'A': A, 'B': B, 'X': X, 'Y': Y,
                'left_menu': False, 'right_menu': False,
                'left_stick': False, 'right_stick': False,
            }
            try:
                buttons_dict['left_menu'] = xrt.get_left_menu_button()
                buttons_dict['right_menu'] = xrt.get_right_menu_button()
                buttons_dict['left_stick'] = xrt.get_left_axis_click()
                buttons_dict['right_stick'] = xrt.get_right_axis_click()
            except Exception:
                pass

            writer.write_xr_data(
                right_pose=right_pose,
                left_pose=left_pose,
                right_trigger=right_trigger,
                left_trigger=left_trigger,
                right_grip=right_grip,
                left_grip=left_grip,
                buttons_dict=buttons_dict,
                is_active=is_active,
                request_init=request_init,
                emergency_stop=emergency_stop,
                reset_to_zero=reset_to_zero,
            )

            # 回零信号只发一帧
            if right_reset_to_zero:
                right_reset_to_zero = False
            if left_reset_to_zero:
                left_reset_to_zero = False

            # ---- 读取机械臂反馈 ----
            fb = writer.read_robot_feedback()

            # ---- 打印状态 (20Hz) ----
            if loop_count % 50 == 0:
                if emergency_stop:
                    status_str = "急停"
                else:
                    r_str = "控制" if r_active else "待机"
                    l_str = "控制" if l_active else "待机"
                    status_str = f"右:{r_str} 左:{l_str}"

                if fb and fb['robot_connected']:
                    ee_r = fb['ee_pos_right']
                    ee_l = fb['ee_pos_left']
                    robot_str = (f"R:[{ee_r[0]:.3f},{ee_r[1]:.3f},{ee_r[2]:.3f}] "
                                f"L:[{ee_l[0]:.3f},{ee_l[1]:.3f},{ee_l[2]:.3f}] "
                                f"IK:{fb['ik_success_rate']:.0f}%")
                else:
                    robot_str = "机械臂:未连接"

                print(f"\r[{status_str}] "
                      f"Rgrip:{right_grip:.2f} Lgrip:{left_grip:.2f} "
                      f"| {robot_str}    ", end="", flush=True)

            loop_count += 1
            time.sleep(0.002)

    except KeyboardInterrupt:
        print("\n\nXR 发送端退出")
    finally:
        try:
            writer.write_xr_data(
                right_pose=[0]*7, left_pose=[0]*7,
                right_trigger=0, left_trigger=0,
                right_grip=0, left_grip=0,
                buttons_dict={},
                is_active=False, emergency_stop=True,
            )
        except Exception:
            pass
        writer.close()
        try:
            xrt.close()
            print("XRT SDK 已关闭")
        except Exception:
            pass


if __name__ == "__main__":
    main()