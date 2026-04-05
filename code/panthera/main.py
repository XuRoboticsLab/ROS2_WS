#!/usr/bin/env python3
# ─────────────────────────────────────────────
#  main.py  —  入口：初始化 & 线程启动
# ─────────────────────────────────────────────
 
import time
import threading
import numpy as np
import roslibpy
from Panthera_lib import Panthera
import os
 
from config import (
    ROSBRIDGE_HOST, ROSBRIDGE_PORT,
    SAFE_JOINT_POS, SAFE_JOINT_VEL,
    KP, KD,
)
from shared_state import SharedState
from ros_bridge import RosSubscribers, publisher_thread
from control_loop import control_loop

def init_robot() -> Panthera:
    print("[Init] 初始化机械臂...")
    config_path = os.path.abspath("/home/xuroboticslab/ws/Panthera-HT_SDK/panthera_python/robot_param/Right.yaml")
    robot = Panthera(config_path)
    robot.send_get_motor_state_cmd()
    robot.motor_send_cmd()
    time.sleep(0.3)

    print("[Init] 移动到安全位置...")
    ok = robot.Joint_Pos_Vel(SAFE_JOINT_POS, SAFE_JOINT_VEL, iswait=True)
    if not ok:
        raise RuntimeError("移动到安全位置失败")
    print("[Init] ✓ 已到达安全位置")
    time.sleep(0.5)

    robot.send_get_motor_state_cmd()
    robot.motor_send_cmd()
    time.sleep(0.1)
    return robot


def init_state(robot: Panthera) -> SharedState:
    state = SharedState()
    fk = robot.forward_kinematics()
    state.reset_target_to(fk["position"], fk["rotation"])
    state.last_valid_joint_pos = robot.get_current_pos()
    state.set_robot_state(
        state.last_valid_joint_pos,
        fk["position"],
        np.array(fk["rotation"], dtype=float),
    )
    print(f"[Init] 初始末端位置: {[f'{v:.3f}' for v in fk['position']]}")
    return state


def main():
    print("=" * 60)
    print("Panthera ROS Bridge 节点")
    print("=" * 60)

    robot = init_robot()
    state = init_state(robot)

    # 先稳定住机械臂
    grav = np.array(robot.get_Gravity())
    robot.pos_vel_tqe_kp_kd(
        state.last_valid_joint_pos, [0.0] * robot.motor_count, grav, KP, KD
    )

    # 连接 rosbridge
    print(f"\n[ROS] 连接 {ROSBRIDGE_HOST}:{ROSBRIDGE_PORT}...")
    ros = roslibpy.Ros(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
    ros.run()
    if not ros.is_connected:
        raise RuntimeError("rosbridge 连接失败")
    print("[ROS] ✓ 已连接")

    subscribers = RosSubscribers(ros, state)
    stop_event  = threading.Event()

    threads = [
        threading.Thread(target=publisher_thread,
                         args=(ros, state, stop_event),
                         daemon=True, name="StatePublisher"),
        threading.Thread(target=control_loop,
                         args=(robot, state, stop_event),
                         daemon=True, name="ControlLoop"),
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
        robot.Joint_Pos_Vel(SAFE_JOINT_POS, SAFE_JOINT_VEL, iswait=True)
        print("[Main] 完成")


if __name__ == "__main__":
    main()