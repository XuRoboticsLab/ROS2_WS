#!/usr/bin/env python3
# ─────────────────────────────────────────────
#  main.py  —  XR 发布端入口
#
#  运行: conda activate xr_env && python main.py
#  依赖: xrobotoolkit_sdk, roslibpy, scipy
# ─────────────────────────────────────────────
import os
import argparse

_parser = argparse.ArgumentParser(description="Pico ROS Bridge")
_parser.add_argument(
    "--config", "-c",
    required=True,
    metavar="PATH",
    help="config.yaml 路径",
)
_args = _parser.parse_args()
os.environ["PICO_CONFIG"] = os.path.abspath(_args.config)


import time
import sys
import roslibpy

try:
    import xrobotoolkit_sdk as xrt
except ImportError:
    print("错误: xrobotoolkit_sdk 未安装，请确认在正确的 conda 环境中运行")
    sys.exit(1)

from config import ROSBRIDGE_HOST, ROSBRIDGE_PORT, GRIP_THRESHOLD
from utils import GripDetector, ArmConverter
from ros_publisher import XRRosPublisher


GRIPPER_MAX_RAD = 2.0  # 完全张开 (rad)


def _trigger_to_gripper_pos(trigger: float) -> float:
    """trigger [0, 1] → gripper position [0.0, 2.0] rad
    松开 trigger (0) = 夹爪张开 (2.0 rad)
    按下 trigger (1) = 夹爪闭合 (0.0 rad)
    """
    return (1.0 - max(0.0, min(1.0, trigger))) * GRIPPER_MAX_RAD


def main():
    print("=" * 60)
    print("XR 发布端 (双臂) — roslibpy 版")
    print("=" * 60)
    print("  右手 Grip 按住:   控制右臂")
    print("  左手 Grip 按住:   控制左臂")
    print("  Grip 连按两次:    对应臂回零")
    print("  右手 Trigger:     连续控制右夹爪 (松开=张开, 按下=闭合)")
    print("  左手 Trigger:     连续控制左夹爪 (松开=张开, 按下=闭合)")
    print("  右手 B:           右臂旋转约束模式 开/关 (x轴朝下 + 仅x轴旋转)")
    print("  左手 Y:           左臂旋转约束模式 开/关 (x轴朝下 + 仅x轴旋转)")
    print("  Ctrl+C:           退出\n")

    # ── 初始化 XRT SDK ────────────────────────
    try:
        xrt.init()
        print("[OK] XRoboToolkit SDK 初始化成功")
    except Exception as e:
        print(f"[FAIL] XRT SDK 初始化失败: {e}")
        sys.exit(1)

    # ── 连接 rosbridge ────────────────────────
    print(f"\n[ROS] 连接 {ROSBRIDGE_HOST}:{ROSBRIDGE_PORT}...")
    ros = roslibpy.Ros(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
    ros.run()
    if not ros.is_connected:
        print("[ROS] ✗ 连接失败，退出")
        sys.exit(1)
    print("[ROS] ✓ 已连接\n")

    publisher   = XRRosPublisher(ros)
    right_grip  = GripDetector("右手")
    left_grip   = GripDetector("左手")
    right_conv  = ArmConverter("右臂")
    left_conv   = ArmConverter("左臂")

    # 约束模式状态（B=右臂, Y=左臂）
    right_constrained = False
    left_constrained  = False
    prev_B = False
    prev_Y = False

    loop_count = 0

    print("等待 Pico 4 Ultra 连接...\n")

    try:
        while True:
            # ── 读取 XR 数据 ──────────────────
            try:
                right_pose    = list(xrt.get_right_controller_pose())
                left_pose     = list(xrt.get_left_controller_pose())
                headset_pose  = list(xrt.get_headset_pose())
                right_grip_v  = xrt.get_right_grip()
                left_grip_v   = xrt.get_left_grip()
                right_trigger = xrt.get_right_trigger()
                left_trigger  = xrt.get_left_trigger()
                B_btn         = xrt.get_B_button()   # 右臂约束模式
                Y_btn         = xrt.get_Y_button()   # 左臂约束模式
            except Exception as e:
                if loop_count % 500 == 0:
                    print(f"\r[警告] XR 数据读取失败: {e}    ", end="", flush=True)
                time.sleep(0.01)
                loop_count += 1
                continue

            # ── 右手 Grip 逻辑 ────────────────
            r_active, r_init, r_reset = right_grip.update(right_grip_v)

            if r_reset:
                right_conv.reset()
                publisher.publish_reset("right", True)
            elif r_init:
                right_conv.calibrate(right_pose)
                publisher.publish_init("right")

            # ── 左手 Grip 逻辑 ────────────────
            l_active, l_init, l_reset = left_grip.update(left_grip_v)

            if l_reset:
                left_conv.reset()
                publisher.publish_reset("left", True)
            elif l_init:
                left_conv.calibrate(left_pose)
                publisher.publish_init("left")

            # ── 发布 cmd ─────────────────────
            if r_active and right_conv.is_calibrated:
                publisher.publish_cmd("right", right_conv.compute_twist(right_pose, headset_pose))

            if l_active and left_conv.is_calibrated:
                publisher.publish_cmd("left", left_conv.compute_twist(left_pose, headset_pose))

            # ── 旋转约束模式切换（上升沿触发）──────
            if B_btn and not prev_B:
                right_constrained = not right_constrained
                print(f"\n[右臂] 旋转约束模式: {'ON' if right_constrained else 'OFF'}")
            if Y_btn and not prev_Y:
                left_constrained = not left_constrained
                print(f"\n[左臂] 旋转约束模式: {'ON' if left_constrained else 'OFF'}")
            prev_B = B_btn
            prev_Y = Y_btn

            # ── 发布夹爪 ─────────────────────
            publisher.publish_gripper("right", _trigger_to_gripper_pos(right_trigger))
            publisher.publish_gripper("left",  _trigger_to_gripper_pos(left_trigger))

            # ── 发布旋转约束模式 ──────────────
            publisher.publish_mode("right", right_constrained)
            publisher.publish_mode("left",  left_constrained)

            # ── 打印状态 (20 Hz) ─────────────
            if loop_count % 50 == 0:
                r_str = "控制" if (r_active and right_conv.is_calibrated) else "待机"
                l_str = "控制" if (l_active and left_conv.is_calibrated)  else "待机"
                r_mode = "约束" if right_constrained else "自由"
                l_mode = "约束" if left_constrained  else "自由"
                print(f"\r右:{r_str}[{r_mode}] grip={right_grip_v:.2f} trig={right_trigger:.2f}  "
                      f"左:{l_str}[{l_mode}] grip={left_grip_v:.2f} trig={left_trigger:.2f}    ",
                      end="", flush=True)

            loop_count += 1
            time.sleep(0.002)   # ~500 Hz 上限，rosbridge 会自然限速

    except KeyboardInterrupt:
        print("\n\nXR 发布端退出")
    finally:
        publisher.publish_emergency(True)   # 退出时触发急停
        publisher.unadvertise_all()
        ros.terminate()
        try:
            xrt.close()
            print("XRT SDK 已关闭")
        except Exception:
            pass


if __name__ == "__main__":
    main()