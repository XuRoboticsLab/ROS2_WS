# ─────────────────────────────────────────────
#  config.py  —  读取 xr_publisher 配置
# ─────────────────────────────────────────────

import os
import numpy as np
import yaml

_YAML_PATH = os.environ.get("PICO_CONFIG")
if not _YAML_PATH:
    raise RuntimeError("未指定 config 路径，请通过 --config 传入 config.yaml 路径")

with open(_YAML_PATH, "r") as f:
    _cfg = yaml.safe_load(f)

# ── ROS Bridge ────────────────────────────────
ROSBRIDGE_HOST = _cfg["rosbridge"]["host"]
ROSBRIDGE_PORT = _cfg["rosbridge"]["port"]

# ── Grip ──────────────────────────────────────
GRIP_THRESHOLD      = float(_cfg["grip"]["threshold"])
DOUBLE_TAP_WINDOW   = float(_cfg["grip"]["double_tap_window_s"])

# ── 缩放 ──────────────────────────────────────
TRANSLATION_SCALE = float(_cfg["scale"]["translation_m"])
ROTATION_SCALE    = float(_cfg["scale"]["rotation_rad"])
MAX_DELTA_POS     = float(_cfg["scale"]["max_delta_pos_m"])

# ── 死区 ──────────────────────────────────────
DEADZONE_POS_M   = float(_cfg["deadzone"]["pos_m"])
DEADZONE_ROT_RAD = float(_cfg["deadzone"]["rot_rad"])

# ── 低通滤波 ──────────────────────────────────
FILTER_ALPHA = float(_cfg["filter"]["alpha"])

# ── 坐标系映射 ────────────────────────────────
def _load_coord(side: str):
    c = _cfg["coord_mapping"][side]
    pos      = np.array(c["pos"],                          dtype=float)
    rot      = np.array(c["rot"],                          dtype=float)
    pos_sign = np.array(c.get("pos_sign", [1, 1, 1]),     dtype=float)
    rot_sign = np.array(c.get("rot_sign", [1, 1, 1]),     dtype=float)
    return pos, rot, pos_sign, rot_sign

XR_TO_ROBOT_POS_RIGHT, XR_TO_ROBOT_ROT_RIGHT, POS_SIGN_RIGHT, ROT_SIGN_RIGHT = _load_coord("right")
XR_TO_ROBOT_POS_LEFT,  XR_TO_ROBOT_ROT_LEFT,  POS_SIGN_LEFT,  ROT_SIGN_LEFT  = _load_coord("left")

# ── Topics ────────────────────────────────────
TOPIC_RIGHT_CMD     = _cfg["topics"]["publish"]["right_cmd"]
TOPIC_LEFT_CMD      = _cfg["topics"]["publish"]["left_cmd"]
TOPIC_RIGHT_GRIPPER = _cfg["topics"]["publish"]["right_gripper"]
TOPIC_LEFT_GRIPPER  = _cfg["topics"]["publish"]["left_gripper"]
TOPIC_RIGHT_RESET   = _cfg["topics"]["publish"]["right_reset"]
TOPIC_LEFT_RESET    = _cfg["topics"]["publish"]["left_reset"]
TOPIC_EMERGENCY     = _cfg["topics"]["publish"]["emergency"]
TOPIC_RIGHT_INIT    = _cfg["topics"]["publish"]["right_init"]
TOPIC_LEFT_INIT     = _cfg["topics"]["publish"]["left_init"]