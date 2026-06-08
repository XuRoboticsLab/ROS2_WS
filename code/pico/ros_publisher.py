# ─────────────────────────────────────────────
#  ros_publisher.py  —  roslibpy 发布封装
# ─────────────────────────────────────────────

import roslibpy
from config import (
    TOPIC_RIGHT_CMD, TOPIC_LEFT_CMD,
    TOPIC_RIGHT_GRIPPER, TOPIC_LEFT_GRIPPER,
    TOPIC_RIGHT_RESET, TOPIC_LEFT_RESET,
    TOPIC_RIGHT_INIT, TOPIC_LEFT_INIT,
    TOPIC_RIGHT_MODE, TOPIC_LEFT_MODE,
    TOPIC_EMERGENCY,
)


class XRRosPublisher:
    def __init__(self, ros: roslibpy.Ros):
        self._ros = ros
        self._pubs = {
            "right_cmd":     roslibpy.Topic(ros, TOPIC_RIGHT_CMD,     "geometry_msgs/Twist"),
            "left_cmd":      roslibpy.Topic(ros, TOPIC_LEFT_CMD,      "geometry_msgs/Twist"),
            "right_gripper": roslibpy.Topic(ros, TOPIC_RIGHT_GRIPPER, "std_msgs/Float32"),
            "left_gripper":  roslibpy.Topic(ros, TOPIC_LEFT_GRIPPER,  "std_msgs/Float32"),
            "right_reset":   roslibpy.Topic(ros, TOPIC_RIGHT_RESET,   "std_msgs/Bool"),
            "left_reset":    roslibpy.Topic(ros, TOPIC_LEFT_RESET,    "std_msgs/Bool"),
            "right_init":    roslibpy.Topic(ros, TOPIC_RIGHT_INIT,    "std_msgs/Bool"),
            "left_init":     roslibpy.Topic(ros, TOPIC_LEFT_INIT,     "std_msgs/Bool"),
            "right_mode":    roslibpy.Topic(ros, TOPIC_RIGHT_MODE,    "std_msgs/Bool"),
            "left_mode":     roslibpy.Topic(ros, TOPIC_LEFT_MODE,     "std_msgs/Bool"),
            "emergency":     roslibpy.Topic(ros, TOPIC_EMERGENCY,     "std_msgs/Bool"),
        }

    def _pub(self, key, data: dict):
        if self._ros.is_connected:
            self._pubs[key].publish(roslibpy.Message(data))

    def publish_cmd(self, side: str, twist: dict):
        """side: 'right' | 'left'"""
        self._pub(f"{side}_cmd", twist)

    def publish_gripper(self, side: str, pos: float):
        """pos: 0.0=fully closed, 2.0=fully open (rad)"""
        self._pub(f"{side}_gripper", {"data": float(pos)})

    def publish_reset(self, side: str, value: bool = True):
        self._pub(f"{side}_reset", {"data": value})

    def publish_init(self, side: str):
        """通知机器人记录当前末端位姿为校准基准（Grip 单击时调用）。"""
        self._pub(f"{side}_init", {"data": True})

    def publish_mode(self, side: str, constrained: bool):
        """constrained: True=约束模式(仅x轴旋转), False=自由旋转"""
        self._pub(f"{side}_mode", {"data": bool(constrained)})

    def publish_emergency(self, active: bool):
        self._pub("emergency", {"data": active})

    def unadvertise_all(self):
        for pub in self._pubs.values():
            pub.unadvertise()