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
    default=None,
    metavar="PATH",
    help="config.yaml 路径",
)
_args = _parser.parse_args()
if _args.config:
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


def _gripper_value(trigger: float) -> int:
    """trigger > 0.5 → open(1), 否则 close(-1)"""
    return 1 if trigger > 0.5 else -1


def main():
    print("=" * 60)
    print("XR 发布端 (双臂) — roslibpy 版")
    print("=" * 60)
    print("  右手 Grip 按住:   控制右臂")
    print("  左手 Grip 按住:   控制左臂")
    print("  Grip 连按两次:    对应臂回零")
    print("  Trigger:          夹爪开合")
    print("  Y 键:             紧急停止")
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

    emergency_stop = False
    prev_Y         = False
    loop_count     = 0

    print("等待 Pico 4 Ultra 连接...\n")

    try:
        while True:
            # ── 读取 XR 数据 ──────────────────
            try:
                right_pose    = list(xrt.get_right_controller_pose())
                left_pose     = list(xrt.get_left_controller_pose())
                right_trigger = xrt.get_right_trigger()
                left_trigger  = xrt.get_left_trigger()
                right_grip_v  = xrt.get_right_grip()
                left_grip_v   = xrt.get_left_grip()
                Y             = xrt.get_Y_button()
            except Exception as e:
                if loop_count % 500 == 0:
                    print(f"\r[警告] XR 数据读取失败: {e}    ", end="", flush=True)
                time.sleep(0.01)
                loop_count += 1
                continue

            # ── Y 键：紧急停止 ────────────────
            if Y and not prev_Y:
                emergency_stop = True
                publisher.publish_emergency(True)
                print("\n[Y] !!! 紧急停止 !!! (按 Grip 恢复)")
            prev_Y = Y

            if emergency_stop:
                # 任意手 Grip 单击解除急停
                r_active, r_init, _ = right_grip.update(right_grip_v)
                l_active, l_init, _ = left_grip.update(left_grip_v)
                if r_init or l_init:
                    emergency_stop = False
                    publisher.publish_emergency(False)
                    print("[Grip] 急停已解除")
                time.sleep(0.002)
                loop_count += 1
                continue

            # ── 右手 Grip 逻辑 ────────────────
            r_active, r_init, r_reset = right_grip.update(right_grip_v)

            if r_reset:
                right_conv.reset()
                publisher.publish_reset("right", True)
            elif r_init:
                right_conv.calibrate(right_pose)

            # ── 左手 Grip 逻辑 ────────────────
            l_active, l_init, l_reset = left_grip.update(left_grip_v)

            if l_reset:
                left_conv.reset()
                publisher.publish_reset("left", True)
            elif l_init:
                left_conv.calibrate(left_pose)

            # ── 发布 cmd ─────────────────────
            if r_active and right_conv.is_calibrated:
                publisher.publish_cmd("right", right_conv.compute_twist(right_pose))

            if l_active and left_conv.is_calibrated:
                publisher.publish_cmd("left", left_conv.compute_twist(left_pose))

            # ── 发布夹爪 ─────────────────────
            publisher.publish_gripper("right", _gripper_value(right_trigger))
            publisher.publish_gripper("left",  _gripper_value(left_trigger))

            # ── 打印状态 (20 Hz) ─────────────
            if loop_count % 50 == 0:
                r_str = "控制" if (r_active and right_conv.is_calibrated) else "待机"
                l_str = "控制" if (l_active and left_conv.is_calibrated)  else "待机"
                print(f"\r右:{r_str} grip={right_grip_v:.2f}  "
                      f"左:{l_str} grip={left_grip_v:.2f}    ",
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