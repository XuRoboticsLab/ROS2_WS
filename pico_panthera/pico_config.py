# ─────────────────────────────────────────────
#  pico_config.py  —  读取 pico 侧配置
# ─────────────────────────────────────────────

import os
import numpy as np
import yaml

_YAML_PATH = os.environ.get("PICO_CONFIG")
if not _YAML_PATH:
    raise RuntimeError("未指定 config 路径，请通过 --config 传入 config.yaml 路径")

with open(_YAML_PATH, "r") as f:
    _cfg = yaml.safe_load(f)

# ── IPC ───────────────────────────────────────
IPC_ADDRESS = str(_cfg["ipc"]["address"])

# ── Grip ──────────────────────────────────────
GRIP_THRESHOLD    = float(_cfg["grip"]["threshold"])
DOUBLE_TAP_WINDOW = float(_cfg["grip"]["double_tap_window_s"])

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
XR_TO_ROBOT_POS = np.array(_cfg["coord_mapping"]["pos"], dtype=float)
XR_TO_ROBOT_ROT = np.array(_cfg["coord_mapping"]["rot"], dtype=float)
ROT_SIGN        = np.array(_cfg["coord_mapping"].get("rot_sign", [1, 1, 1]), dtype=float)
POS_SIGN        = np.array(_cfg["coord_mapping"].get("pos_sign", [1, 1, 1]), dtype=float)

# ── 操纵杆精细控制 ────────────────────────────
AXIS_THRESHOLD = float(_cfg.get("joystick", {}).get("axis_threshold", 0.15))
