# ─────────────────────────────────────────────
#  config.py  —  读取配置
# ─────────────────────────────────────────────

import os
import yaml

_YAML_PATH = os.environ.get("PICO_CONTROL_CONFIG")
if not _YAML_PATH:
    raise RuntimeError("未指定 config 路径，请通过 --config 传入 config.yaml 路径")

with open(_YAML_PATH, "r") as f:
    _cfg = yaml.safe_load(f)

# ── Nero 机械臂连接 ─────────────────────────────
CAN_CHANNEL         = str(_cfg["nero"]["channel"])
FIRMWARE_VERSION    = str(_cfg["nero"].get("firmware_version", "v112"))
EFFECTOR_KIND       = _cfg["nero"].get("effector")          # None 或 "agx_gripper"
GRIPPER_MAX_WIDTH_M = float(_cfg["nero"].get("gripper_max_width_m", 0.07))
GRIPPER_FORCE_N     = float(_cfg["nero"].get("gripper_force_n", 1.0))
SPEED_PERCENT       = int(_cfg["nero"].get("speed_percent", 30))
SAFE_SPEED_PERCENT  = int(_cfg["nero"].get("safe_speed_percent", 10))

# ── ROS Bridge ────────────────────────────────
ROSBRIDGE_HOST = _cfg["rosbridge"]["host"]
ROSBRIDGE_PORT = _cfg["rosbridge"]["port"]

# ── 频率 ──────────────────────────────────────
CONTROL_RATE     = float(_cfg["rates"]["control_hz"])
PUBLISH_RATE     = float(_cfg["rates"]["publish_hz"])
WATCHDOG_TIMEOUT = float(_cfg["rates"]["watchdog_timeout_s"])

# ── 安全位置 ──────────────────────────────────
SAFE_JOINT_POS = list(_cfg["safe_position"]["joint_pos_rad"])

# ── Pico 偏移缩放 ─────────────────────────────
TRANSLATION_SCALE = float(_cfg["twist_scale"]["translation_m"])
ROTATION_SCALE    = float(_cfg["twist_scale"]["rotation_rad"])

# ── Smooth target PD 参数 ─────────────────────
MAX_LINEAR_VEL   = float(_cfg["smoothing"]["max_linear_vel_m_s"])
MAX_ANGULAR_VEL  = float(_cfg["smoothing"]["max_angular_vel_rad_s"])
TRACKING_GAIN_HZ = float(_cfg["smoothing"]["tracking_gain_hz"])
DAMPING_RATIO    = float(_cfg["smoothing"]["damping_ratio"])

# ── 关节名称 ──────────────────────────────────
JOINT_NAMES = list(_cfg["joints"]["names"])

# ── Topics ────────────────────────────────────
PARAM_SERVER_PORT = int(_cfg.get("param_server_port", 8082))
ARM_NAME          = str(_cfg.get("arm_name", "Nero"))

TOPIC_CMD     = _cfg["topics"]["subscribe"]["cmd"]
TOPIC_GRIPPER = _cfg["topics"]["subscribe"]["gripper"]
TOPIC_RESET   = _cfg["topics"]["subscribe"]["reset"]
TOPIC_INIT    = _cfg["topics"]["subscribe"].get("init", "")
TOPIC_JOINTS  = _cfg["topics"]["publish"]["joints"]
TOPIC_EE      = _cfg["topics"]["publish"]["ee"]
