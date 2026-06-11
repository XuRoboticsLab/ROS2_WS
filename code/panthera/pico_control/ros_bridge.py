# ─────────────────────────────────────────────
#  ros_bridge.py  —  ROS 订阅回调 + state 发布线程
# ─────────────────────────────────────────────

import time
import threading
import numpy as np
import roslibpy
from scipy.spatial.transform import Rotation

from config import (
    PUBLISH_RATE,
    JOINT_NAMES,
    TOPIC_CMD, TOPIC_GRIPPER, TOPIC_RESET, TOPIC_INIT, TOPIC_FINE_CMD,
    TOPIC_JOINTS, TOPIC_EE,
)
from shared_state import SharedState


def _rot_to_quat(R) -> list:
    """3x3 旋转矩阵 → [x, y, z, w]"""
    return Rotation.from_matrix(R).as_quat().tolist()


# ─────────────────────────────────────────────
#  订阅回调
# ─────────────────────────────────────────────
def make_cmd_callback(state: SharedState):
    """接收 Pico 发来的位姿偏移量（相对于校准基准的绝对偏移）。"""
    def callback(msg):
        l = msg["linear"]
        a = msg["angular"]
        state.push_twist([l["x"], l["y"], l["z"]],
                         [a["x"], a["y"], a["z"]])
    return callback


def make_gripper_callback(state: SharedState):
    def callback(msg):
        if state.action_locked:
            return
        with state._lock:
            state.gripper_cmd = float(msg["data"])
    return callback


def make_reset_callback(state: SharedState):
    """Pico reset 信号：触发物理复位到安全位置。"""
    def callback(msg):
        if msg["data"]:
            with state._lock:
                state.reset_requested = True
    return callback


def make_init_callback(state: SharedState):
    """Pico init 信号：记录当前末端位姿为位置控制校准基准。"""
    def callback(msg):
        if msg["data"]:
            with state._lock:
                fk_pos = np.array(state.fk_position)
                fk_rot = state.fk_rotation.copy()
            state.set_calibration_pose(fk_pos, fk_rot)
    return callback


def make_fine_cmd_callback(state: SharedState):
    """操纵杆精细控制：接收归一化轴值 [x, y]（不按 Grip 时）。"""
    def callback(msg):
        l = msg["linear"]
        state.push_fine_delta([l["x"], l["y"]])
    return callback


# ─────────────────────────────────────────────
#  订阅管理
# ─────────────────────────────────────────────
class RosSubscribers:
    def __init__(self, ros: roslibpy.Ros, state: SharedState):
        self._subs = []

        def _sub(topic, msg_type, cb):
            if not topic:
                return
            t = roslibpy.Topic(ros, topic, msg_type)
            t.subscribe(cb)
            self._subs.append(t)

        _sub(TOPIC_CMD,      "geometry_msgs/Twist", make_cmd_callback(state))
        _sub(TOPIC_GRIPPER,  "std_msgs/Float32",    make_gripper_callback(state))
        _sub(TOPIC_RESET,    "std_msgs/Bool",       make_reset_callback(state))
        _sub(TOPIC_INIT,     "std_msgs/Bool",       make_init_callback(state))
        _sub(TOPIC_FINE_CMD, "geometry_msgs/Twist", make_fine_cmd_callback(state))
        active = [t for t in [TOPIC_CMD, TOPIC_GRIPPER, TOPIC_RESET, TOPIC_INIT, TOPIC_FINE_CMD] if t]
        print(f"[ROS] 已订阅 {', '.join(active)}")

    def unsubscribe_all(self):
        for sub in self._subs:
            sub.unsubscribe()


# ─────────────────────────────────────────────
#  State 发布线程
# ─────────────────────────────────────────────
def publisher_thread(ros: roslibpy.Ros, state: SharedState,
                     stop_event: threading.Event):
    joint_pub = roslibpy.Topic(ros, TOPIC_JOINTS, "sensor_msgs/JointState")
    ee_pub    = roslibpy.Topic(ros, TOPIC_EE,     "geometry_msgs/PoseStamped")
    interval  = 1.0 / PUBLISH_RATE

    while not stop_event.is_set():
        t0 = time.time()

        if ros.is_connected:
            joint_pos, joint_vel, grip_pos, grip_vel, fk_pos, fk_rot, stamp = \
                state.get_robot_state()

            all_names = JOINT_NAMES + ["gripper"]
            all_pos   = joint_pos + [grip_pos]
            all_vel   = joint_vel + [grip_vel]

            joint_pub.publish(roslibpy.Message({
                "header":   {"stamp": stamp, "frame_id": "base_link"},
                "name":     all_names,
                "position": all_pos,
                "velocity": all_vel,
                "effort":   [0.0] * len(all_names),
            }))

            q = _rot_to_quat(fk_rot)
            ee_pub.publish(roslibpy.Message({
                "header": {"stamp": stamp, "frame_id": "base_link"},
                "pose": {
                    "position":    {"x": fk_pos[0], "y": fk_pos[1], "z": fk_pos[2]},
                    "orientation": {"x": q[0], "y": q[1], "z": q[2], "w": q[3]},
                },
            }))

        time.sleep(max(0.0, interval - (time.time() - t0)))
