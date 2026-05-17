# ─────────────────────────────────────────────
#  ros_bridge.py  —  订阅 VLA actions + 发布 joint states + 发布 trigger
# ─────────────────────────────────────────────

import time
import threading

import numpy as np
import roslibpy

from vla_config import (
    PUBLISH_HZ,
    TOPIC_VLA_ACTIONS, TOPIC_TRIGGER, VLA_PROMPT,
    RIGHT_JOINT_NAMES, RIGHT_TOPIC_JOINTS, RIGHT_TOPIC_EE,
    LEFT_JOINT_NAMES,  LEFT_TOPIC_JOINTS,  LEFT_TOPIC_EE,
)
from shared_state import SharedState


# ── 订阅 VLA actions ──────────────────────────────────────────────────────

def make_actions_callback(state: SharedState):
    """解析 std_msgs/Float32MultiArray，将每步动作入队。
    action_dim 从 layout.dim[1].size 读取（7 或 14），无需硬编码。
    """
    def callback(msg):
        try:
            data = np.array(msg["data"], dtype=np.float32)
            dims = msg.get("layout", {}).get("dim", [])

            if len(dims) >= 2 and dims[1]["size"] > 0:
                action_dim = dims[1]["size"]
                chunk_size = dims[0]["size"]
                actions = data.reshape(chunk_size, action_dim)
            else:
                # fallback：按常见维度自动推断
                for dim in (14, 7):
                    if len(data) % dim == 0:
                        actions = data.reshape(-1, dim)
                        break
                else:
                    raise ValueError(f"无法推断 action 维度，数据长度={len(data)}")

            state.enqueue_actions(actions)
            print(f"\n[ROS] 收到 VLA actions chunk {actions.shape}")
        except Exception as e:
            print(f"\n[ROS] actions 解析失败: {e}")
    return callback


# ── Trigger 发布 ──────────────────────────────────────────────────────────

def make_trigger_publisher(ros: roslibpy.Ros) -> tuple[roslibpy.Topic, callable]:
    """返回 (topic_handle, trigger_fn)。
    trigger_fn() 向 /vla/trigger 发送一次 std_msgs/String（prompt 来自 config）。
    """
    pub = roslibpy.Topic(ros, TOPIC_TRIGGER, "std_msgs/String")

    def trigger_fn():
        pub.publish(roslibpy.Message({"data": VLA_PROMPT}))
        print(f"\n[ROS] 已发送 trigger → {TOPIC_TRIGGER} | prompt='{VLA_PROMPT}'")

    return pub, trigger_fn


# ── 发布线程 ──────────────────────────────────────────────────────────────

class JointStatePublisher:
    """为左右两臂维护 roslibpy Topic，以固定频率发布 JointState + PoseStamped。"""

    def __init__(self, ros: roslibpy.Ros):
        self._ros = ros
        self._pubs_joint = {}
        self._pubs_ee    = {}

        if RIGHT_JOINT_NAMES:
            self._pubs_joint["right"] = roslibpy.Topic(
                ros, RIGHT_TOPIC_JOINTS, "sensor_msgs/JointState"
            )
            self._pubs_ee["right"] = roslibpy.Topic(
                ros, RIGHT_TOPIC_EE, "geometry_msgs/PoseStamped"
            )
            print(f"[ROS] 右臂 joints → {RIGHT_TOPIC_JOINTS}")

        if LEFT_JOINT_NAMES:
            self._pubs_joint["left"] = roslibpy.Topic(
                ros, LEFT_TOPIC_JOINTS, "sensor_msgs/JointState"
            )
            self._pubs_ee["left"] = roslibpy.Topic(
                ros, LEFT_TOPIC_EE, "geometry_msgs/PoseStamped"
            )
            print(f"[ROS] 左臂 joints → {LEFT_TOPIC_JOINTS}")

    def _publish_arm(self, side: str, names: list[str], arm_state):
        if side not in self._pubs_joint:
            return
        stamp     = arm_state.stamp
        all_names = names + ["gripper"]
        all_pos   = arm_state.joint_pos + [arm_state.gripper_pos]
        all_vel   = arm_state.joint_vel + [arm_state.gripper_vel]

        self._pubs_joint[side].publish(roslibpy.Message({
            "header":   {"stamp": stamp, "frame_id": "base_link"},
            "name":     all_names,
            "position": all_pos,
            "velocity": all_vel,
            "effort":   [0.0] * len(all_names),
        }))

        q = arm_state.fk_quat
        p = arm_state.fk_pos
        self._pubs_ee[side].publish(roslibpy.Message({
            "header": {"stamp": stamp, "frame_id": "base_link"},
            "pose": {
                "position":    {"x": p[0], "y": p[1], "z": p[2]},
                "orientation": {"x": q[0], "y": q[1], "z": q[2], "w": q[3]},
            },
        }))

    def run(self, state: SharedState, stop_event: threading.Event):
        interval = 1.0 / PUBLISH_HZ
        while not stop_event.is_set():
            t0 = time.time()
            if self._ros.is_connected:
                if RIGHT_JOINT_NAMES:
                    self._publish_arm("right", RIGHT_JOINT_NAMES, state.snapshot_right())
                if LEFT_JOINT_NAMES:
                    self._publish_arm("left",  LEFT_JOINT_NAMES,  state.snapshot_left())
            time.sleep(max(0.0, interval - (time.time() - t0)))

    def unadvertise_all(self):
        for pub in list(self._pubs_joint.values()) + list(self._pubs_ee.values()):
            pub.unadvertise()
