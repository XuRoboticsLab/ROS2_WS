# ─────────────────────────────────────────────
#  ros_publisher.py  —  roslibpy 发布封装
# ─────────────────────────────────────────────

import roslibpy
from config import (
    TOPIC_RIGHT_CMD, TOPIC_LEFT_CMD,
    TOPIC_RIGHT_GRIPPER, TOPIC_LEFT_GRIPPER,
    TOPIC_RIGHT_RESET, TOPIC_LEFT_RESET,
    TOPIC_EMERGENCY,
)


class XRRosPublisher:
    def __init__(self, ros: roslibpy.Ros):
        self._ros = ros
        self._pubs = {
            "right_cmd":     roslibpy.Topic(ros, TOPIC_RIGHT_CMD,     "geometry_msgs/Twist"),
            "left_cmd":      roslibpy.Topic(ros, TOPIC_LEFT_CMD,      "geometry_msgs/Twist"),
            "right_gripper": roslibpy.Topic(ros, TOPIC_RIGHT_GRIPPER, "std_msgs/Int8"),
            "left_gripper":  roslibpy.Topic(ros, TOPIC_LEFT_GRIPPER,  "std_msgs/Int8"),
            "right_reset":   roslibpy.Topic(ros, TOPIC_RIGHT_RESET,   "std_msgs/Bool"),
            "left_reset":    roslibpy.Topic(ros, TOPIC_LEFT_RESET,    "std_msgs/Bool"),
            "emergency":     roslibpy.Topic(ros, TOPIC_EMERGENCY,     "std_msgs/Bool"),
        }

    def _pub(self, key, data: dict):
        if self._ros.is_connected:
            self._pubs[key].publish(roslibpy.Message(data))

    def publish_cmd(self, side: str, twist: dict):
        """side: 'right' | 'left'"""
        self._pub(f"{side}_cmd", twist)

    def publish_gripper(self, side: str, value: int):
        """value: 1=open, -1=close, 0=idle"""
        self._pub(f"{side}_gripper", {"data": value})

    def publish_reset(self, side: str, value: bool = True):
        self._pub(f"{side}_reset", {"data": value})

    def publish_emergency(self, active: bool):
        self._pub("emergency", {"data": active})

    def unadvertise_all(self):
        for pub in self._pubs.values():
            pub.unadvertise()