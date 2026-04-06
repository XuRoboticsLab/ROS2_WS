# ─────────────────────────────────────────────
#  config.py  —  从根目录 config.yaml 读取配置
# ─────────────────────────────────────────────

import os
import yaml

_YAML_PATH = os.environ.get(
    "PANTHERA_CONFIG",
    os.path.join(os.path.dirname(__file__), "../..", "config.yaml")
)

with open(_YAML_PATH, "r") as f:
    _cfg = yaml.safe_load(f)["panthera"]

MOTOR_CONFIG_PATH = str(_cfg["motot_config_path"])

# ── ROS Bridge ────────────────────────────────
ROSBRIDGE_HOST = _cfg["rosbridge"]["host"]
ROSBRIDGE_PORT = _cfg["rosbridge"]["port"]

# ── 频率 ──────────────────────────────────────
CONTROL_RATE     = float(_cfg["rates"]["control_hz"])
PUBLISH_RATE     = float(_cfg["rates"]["publish_hz"])
WATCHDOG_TIMEOUT = float(_cfg["rates"]["watchdog_timeout_s"])

# ── IK 控制参数 ───────────────────────────────
KP = list(_cfg["ik_control"]["kp"])
KD = list(_cfg["ik_control"]["kd"])

# ── 安全位置 ──────────────────────────────────
SAFE_JOINT_POS = list(_cfg["safe_position"]["joint_pos"])
SAFE_JOINT_VEL = list(_cfg["safe_position"]["joint_vel"])

# ── Twist 缩放 ────────────────────────────────
TRANSLATION_SCALE = float(_cfg["twist_scale"]["translation_m"])
ROTATION_SCALE    = float(_cfg["twist_scale"]["rotation_rad"])

# ── 关节名称 ──────────────────────────────────
JOINT_NAMES = list(_cfg["joints"]["names"])

# ── Topics ────────────────────────────────────
TOPIC_CMD     = _cfg["topics"]["cmd"]
TOPIC_GRIPPER = _cfg["topics"]["gripper"]
TOPIC_RESET   = _cfg["topics"]["reset"]
TOPIC_JOINTS  = _cfg["topics"]["joints"]
TOPIC_EE      = _cfg["topics"]["ee"]
