# ─────────────────────────────────────────────
#  ros_publisher.py  —  Follower state 发布（所有对共用一个线程）
# ─────────────────────────────────────────────

import time
import threading
import numpy as np
import roslibpy
from scipy.spatial.transform import Rotation

from config import PUBLISH_HZ, PAIRS


def _rot_to_quat(R: np.ndarray) -> list:
    return Rotation.from_matrix(R).as_quat().tolist()   # [x, y, z, w]


class FollowerStatePublisher:
    """
    为每对主从臂的 Follower 维护一组 roslibpy Topic，
    以固定频率发布 JointState + PoseStamped。

    外部（控制循环）调用 update(pair_name, joint_pos, fk) 更新缓存，
    发布线程从缓存读取并发布。
    """

    def __init__(self, ros: roslibpy.Ros):
        self._ros   = ros
        self._lock  = threading.Lock()
        self._cache = {}   # pair_name → {"joint_pos", "fk_pos", "fk_rot", "stamp"}

        # 为每对臂创建 Topic
        self._joint_pubs = {}
        self._ee_pubs    = {}
        for p in PAIRS:
            name = p["name"]
            self._joint_pubs[name] = roslibpy.Topic(
                ros, p["topics"]["joint_states"], "sensor_msgs/JointState"
            )
            self._ee_pubs[name] = roslibpy.Topic(
                ros, p["topics"]["end_effector"], "geometry_msgs/PoseStamped"
            )
            self._cache[name] = None   # 尚无数据
        print(f"[ROS] 已注册 {len(PAIRS)} 对 Follower 发布器")

    def update(self, pair_name: str, joint_pos: list, fk: dict):
        """由控制循环调用，更新某一对的最新状态"""
        if fk is None:
            return
        t_ns = time.time_ns()
        with self._lock:
            self._cache[pair_name] = {
                "joint_pos": list(joint_pos),
                "fk_pos":    list(fk["position"]),
                "fk_rot":    np.array(fk["rotation"], dtype=float),
                "stamp": {
                    "sec":     t_ns // 1_000_000_000,
                    "nanosec": t_ns  % 1_000_000_000,
                },
            }

    def _publish_once(self):
        """发布所有对的当前缓存"""
        with self._lock:
            snapshot = {k: v for k, v in self._cache.items() if v is not None}

        for name, data in snapshot.items():
            stamp     = data["stamp"]
            joint_pos = data["joint_pos"]
            fk_pos    = data["fk_pos"]
            fk_rot    = data["fk_rot"]
            q         = _rot_to_quat(fk_rot)

            self._joint_pubs[name].publish(roslibpy.Message({
                "header":   {"stamp": stamp, "frame_id": "base_link"},
                "name":     [f"joint_{i+1}" for i in range(len(joint_pos))],
                "position": joint_pos,
                "velocity": [0.0] * len(joint_pos),
                "effort":   [0.0] * len(joint_pos),
            }))

            self._ee_pubs[name].publish(roslibpy.Message({
                "header": {"stamp": stamp, "frame_id": "base_link"},
                "pose": {
                    "position":    {"x": fk_pos[0], "y": fk_pos[1], "z": fk_pos[2]},
                    "orientation": {"x": q[0], "y": q[1], "z": q[2], "w": q[3]},
                }
            }))

    def run(self, stop_event: threading.Event):
        """发布线程主循环"""
        interval = 1.0 / PUBLISH_HZ
        while not stop_event.is_set():
            t0 = time.time()
            if self._ros.is_connected:
                self._publish_once()
            time.sleep(max(0.0, interval - (time.time() - t0)))

    def unadvertise_all(self):
        for pub in list(self._joint_pubs.values()) + list(self._ee_pubs.values()):
            pub.unadvertise()