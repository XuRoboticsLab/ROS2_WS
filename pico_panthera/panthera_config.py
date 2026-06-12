# ─────────────────────────────────────────────
#  panthera_config.py  —  读取机械臂侧配置
# ─────────────────────────────────────────────

import os
import yaml

_YAML_PATH = os.environ.get("PICO_CONTROL_CONFIG")
if not _YAML_PATH:
    raise RuntimeError("未指定 config 路径，请通过 --config 传入 config.yaml 路径")

with open(_YAML_PATH, "r") as f:
    _cfg = yaml.safe_load(f)

MOTOR_CONFIG_PATH = str(_cfg["motor_config_path"])

# ── IPC ───────────────────────────────────────
IPC_ADDRESS = str(_cfg["ipc"]["address"])
ARM_SIDE    = str(_cfg["ipc"]["arm_side"])   # "right" 或 "left"

# ── 频率 ──────────────────────────────────────
CONTROL_RATE     = float(_cfg["rates"]["control_hz"])
PUBLISH_RATE     = float(_cfg["rates"].get("publish_hz", 100.0))  # 保留字段，当前未使用
WATCHDOG_TIMEOUT = float(_cfg["rates"]["watchdog_timeout_s"])

# ── PD 增益 ───────────────────────────────────
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

# ── 参数服务器 ────────────────────────────────
PARAM_SERVER_PORT = int(_cfg.get("param_server_port", 8080))
ARM_NAME          = str(_cfg.get("arm_name", "机械臂"))

# ── 操纵杆精细控制 ────────────────────────────
FINE_SCALE          = float(_cfg.get("fine_control", {}).get("scale_m_per_s", 0.1))
FINE_ROTATION_SCALE = float(_cfg.get("fine_control", {}).get("rotation_scale_rad_per_s", 0.5))

POSES_FILE = str(_cfg.get("poses_file", ""))
