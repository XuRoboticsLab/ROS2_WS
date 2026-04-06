# ─────────────────────────────────────────────
#  config.py  —  从根目录 config.yaml 读取 leader_follower 配置
# ─────────────────────────────────────────────

import os
import numpy as np
import yaml

_YAML_PATH = os.environ.get(
    "LEADER_FOLLOWER_CONFIG",
    os.path.join(os.path.dirname(__file__), "..", "config.yaml")
)

with open(_YAML_PATH, "r") as f:
    _cfg = yaml.safe_load(f)["leader_follower"]

# ── ROS Bridge ────────────────────────────────
ROSBRIDGE_HOST = _cfg["rosbridge"]["host"]
ROSBRIDGE_PORT = _cfg["rosbridge"]["port"]

# ── 发布频率 ──────────────────────────────────
PUBLISH_HZ = float(_cfg["publish_hz"])

# ── 主从对列表 ────────────────────────────────
# 每个元素: {"name", "leader", "follower", "topics": {"joint_states", "end_effector"}}
# leader/follower 路径解析为相对于 config.yaml 所在目录的绝对路径
_cfg_dir = os.path.dirname(os.path.abspath(_YAML_PATH))

PAIRS = []
for p in _cfg["pairs"]:
    PAIRS.append({
        "name":     p["name"],
        "leader":   os.path.abspath(p["leader"]),
        "follower": os.path.abspath(p["follower"]),
        "topics":   dict(p["topics"]),
    })

# ── 控制参数 ──────────────────────────────────
_ctrl = _cfg["control"]

KP         = list(_ctrl["kp"])
KD         = list(_ctrl["kd"])
GRIPPER_KP = float(_ctrl["gripper_kp"])
GRIPPER_KD = float(_ctrl["gripper_kd"])

FC            = np.array(_ctrl["friction"]["Fc"],  dtype=float)
FV            = np.array(_ctrl["friction"]["Fv"],  dtype=float)
VEL_THRESHOLD = float(_ctrl["friction"]["vel_threshold"])
TOR_THRESHOLD = np.array(_ctrl["tor_threshold"],   dtype=float)

TAU_LIMIT = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0])