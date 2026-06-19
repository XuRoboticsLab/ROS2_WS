#!/usr/bin/env python3
# ─────────────────────────────────────────────
#  pico_main.py  —  XR 发布端（单臂，ZMQ 版）
#
#  运行: conda activate xr_env && python pico_main.py --config <pico_config.yaml>
#  依赖: xrobotoolkit_sdk, pyzmq, scipy
# ─────────────────────────────────────────────
import os
import argparse

_parser = argparse.ArgumentParser(description="Pico Z1 Publisher")
_parser.add_argument("--config", "-c", required=True, metavar="PATH", help="pico config.yaml 路径")
_args = _parser.parse_args()
os.environ["PICO_Z1_CONFIG"] = os.path.abspath(_args.config)

import time
import sys

try:
    import xrobotoolkit_sdk as xrt
except ImportError:
    print("错误: xrobotoolkit_sdk 未安装，请确认在正确的 conda 环境中运行")
    sys.exit(1)

from pico_config import (
    IPC_ADDRESS, AXIS_THRESHOLD, HAND_SIDE,
    XR_TO_ROBOT_POS, XR_TO_ROBOT_ROT, POS_SIGN, ROT_SIGN,
)
from utils import GripDetector, ArmConverter
from ipc_bridge import ZmqPublisher

GRIPPER_MAX_RAD = 2.0


def _trigger_to_gripper_pos(trigger: float) -> float:
    """trigger [0, 1] → gripper_pos [2.0, 0.0] (松开=张开=2.0, 按下=闭合=0.0)"""
    return (1.0 - max(0.0, min(1.0, trigger))) * GRIPPER_MAX_RAD


def _get_poses(side: str):
    """根据控制手选择 XR SDK 的读取函数。"""
    if side == "right":
        return (
            xrt.get_right_controller_pose,
            xrt.get_right_grip,
            xrt.get_right_trigger,
            xrt.get_right_axis,
        )
    else:
        return (
            xrt.get_left_controller_pose,
            xrt.get_left_grip,
            xrt.get_left_trigger,
            xrt.get_left_axis,
        )


def main():
    print("=" * 60)
    print(f"XR 发布端 — Z1 机械臂控制 ({HAND_SIDE}手)")
    print("=" * 60)
    print(f"  {HAND_SIDE}手 Grip 按住:    控制机械臂")
    print(f"  {HAND_SIDE}手 Grip 连按两次: 回零")
    print(f"  {HAND_SIDE}手 Trigger:      控制夹爪 (松开=张开, 按下=闭合)")
    print(f"  Grip 松开时摇杆: 精细平动")
    print("  Ctrl+C:         退出\n")

    try:
        xrt.init()
        print("[OK] XRoboToolkit SDK 初始化成功")
    except Exception as e:
        print(f"[FAIL] XRT SDK 初始化失败: {e}")
        sys.exit(1)

    publisher = ZmqPublisher(IPC_ADDRESS)
    grip_det  = GripDetector(f"{HAND_SIDE}手")
    converter = ArmConverter("Z1", XR_TO_ROBOT_POS, XR_TO_ROBOT_ROT, POS_SIGN, ROT_SIGN)

    get_ctrl_pose, get_grip, get_trigger, get_axis = _get_poses(HAND_SIDE)

    # 等待订阅端连接
    time.sleep(0.1)

    loop_count = 0
    print("等待 Pico 4 Ultra 连接...\n")

    try:
        while True:
            try:
                ctrl_pose    = list(get_ctrl_pose())
                headset_pose = list(xrt.get_headset_pose())
                grip_v       = get_grip()
                trigger_v    = get_trigger()
                axis         = list(get_axis())
            except Exception as e:
                if loop_count % 500 == 0:
                    print(f"\r[警告] XR 数据读取失败: {e}    ", end="", flush=True)
                time.sleep(0.01)
                loop_count += 1
                continue

            is_active, do_init, do_reset = grip_det.update(grip_v)

            if do_reset:
                converter.reset()
                publisher.publish_reset(HAND_SIDE, True)
            elif do_init:
                converter.calibrate(ctrl_pose)
                publisher.publish_init(HAND_SIDE)

            if is_active and converter.is_calibrated:
                publisher.publish_cmd(HAND_SIDE, converter.compute_twist(ctrl_pose, headset_pose))

            publisher.publish_gripper(HAND_SIDE, _trigger_to_gripper_pos(trigger_v))

            # Grip 松开时，摇杆发送精细位移指令
            if not is_active:
                ax, ay = axis[0], axis[1]
                if max(abs(ax), abs(ay)) > AXIS_THRESHOLD:
                    if abs(ax) >= abs(ay):
                        publisher.publish_fine_cmd(HAND_SIDE, ax, 0.0)
                    else:
                        publisher.publish_fine_cmd(HAND_SIDE, 0.0, ay)

            if loop_count % 50 == 0:
                state_str = "控制" if (is_active and converter.is_calibrated) else "待机"
                print(f"\r状态:{state_str} grip={grip_v:.2f} trig={trigger_v:.2f}    ",
                      end="", flush=True)

            loop_count += 1
            time.sleep(0.002)

    except KeyboardInterrupt:
        print("\n\nXR 发布端退出")
    finally:
        publisher.publish_emergency(True)
        publisher.close()
        try:
            xrt.close()
            print("XRT SDK 已关闭")
        except Exception:
            pass


if __name__ == "__main__":
    main()
