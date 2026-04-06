# ─────────────────────────────────────────────
#  config.py  —  从根目录 config.yaml 读取 xr_publisher 配置
# ─────────────────────────────────────────────

import os
import numpy as np
import yaml

_YAML_PATH = os.environ.get(
    "PICO_CONFIG",
    os.path.join(os.path.dirname(__file__), "../..", "config.yaml")
)

with open(_YAML_PATH, "r") as f:
    _cfg = yaml.safe_load(f)["xr_publisher"]

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

# ── 坐标系映射 ────────────────────────────────
XR_TO_ROBOT_POS = np.array(_cfg["coord_mapping"]["pos"], dtype=float)
XR_TO_ROBOT_ROT = np.array(_cfg["coord_mapping"]["rot"], dtype=float)

# ── Topics ────────────────────────────────────
TOPIC_RIGHT_CMD     = _cfg["topics"]["right_cmd"]
TOPIC_LEFT_CMD      = _cfg["topics"]["left_cmd"]
TOPIC_RIGHT_GRIPPER = _cfg["topics"]["right_gripper"]
TOPIC_LEFT_GRIPPER  = _cfg["topics"]["left_gripper"]
TOPIC_RIGHT_RESET   = _cfg["topics"]["right_reset"]
TOPIC_LEFT_RESET    = _cfg["topics"]["left_reset"]
TOPIC_EMERGENCY     = _cfg["topics"]["emergency"]