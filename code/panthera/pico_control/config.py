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

MOTOR_CONFIG_PATH = str(_cfg["motor_config_path"])

# ── ROS Bridge ────────────────────────────────
ROSBRIDGE_HOST = _cfg["rosbridge"]["host"]
ROSBRIDGE_PORT = _cfg["rosbridge"]["port"]

# ── 频率 ──────────────────────────────────────
CONTROL_RATE     = float(_cfg["rates"]["control_hz"])
PUBLISH_RATE     = float(_cfg["rates"]["publish_hz"])
WATCHDOG_TIMEOUT = float(_cfg["rates"]["watchdog_timeout_s"])

# ── PD 增益（传给 pos_vel_tqe_kp_kd）─────────
KP = list(_cfg["ik_control"]["kp"])
KD = list(_cfg["ik_control"]["kd"])

# ── IK 关节步长限幅 ───────────────────────────
IK_MAX_JOINT_STEP = float(_cfg["ik_params"]["max_joint_step_rad"])

# ── 安全位置 ──────────────────────────────────
SAFE_JOINT_POS = list(_cfg["safe_position"]["joint_pos"])
SAFE_JOINT_VEL = list(_cfg["safe_position"]["joint_vel"])

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
TOPIC_CMD     = _cfg["topics"]["subscribe"]["cmd"]
TOPIC_GRIPPER = _cfg["topics"]["subscribe"]["gripper"]
TOPIC_RESET   = _cfg["topics"]["subscribe"]["reset"]
TOPIC_INIT    = _cfg["topics"]["subscribe"].get("init", "")
TOPIC_MODE    = _cfg["topics"]["subscribe"].get("mode", "")
TOPIC_JOINTS  = _cfg["topics"]["publish"]["joints"]
TOPIC_EE      = _cfg["topics"]["publish"]["ee"]
