# ─────────────────────────────────────────────
#  config.py  —  读取 vla_control 配置
#  main.py 在 import 前设置 VLA_CONTROL_CONFIG 环境变量
# ─────────────────────────────────────────────

import os
import yaml

_YAML_PATH = os.environ.get("VLA_CONTROL_CONFIG")
if not _YAML_PATH:
    raise RuntimeError("未指定 config，请通过 --config 传入 vla_control.yaml 路径")

with open(_YAML_PATH) as f:
    _cfg = yaml.safe_load(f)

# ── ROS Bridge ────────────────────────────────
ROSBRIDGE_HOST = str(_cfg["rosbridge"]["host"])
ROSBRIDGE_PORT = int(_cfg["rosbridge"]["port"])

# ── 运行频率 ──────────────────────────────────
CONTROL_HZ = float(_cfg["rates"]["control_hz"])
PUBLISH_HZ  = float(_cfg["rates"]["publish_hz"])

# ── 臂配置（right / left 均可为 None） ────────
_cfg_dir = os.path.dirname(os.path.abspath(_YAML_PATH))

def _abs(path):
    return os.path.abspath(path) if path else None

_arms = _cfg.get("arms", {})

RIGHT_MOTOR_CFG  = _abs(_arms.get("right", {}).get("motor_config_path"))
RIGHT_JOINT_NAMES = list(_arms.get("right", {}).get("joint_names", []))
RIGHT_TOPIC_JOINTS = _arms.get("right", {}).get("topics", {}).get("joint_states", "/vla_robot/right/joint_states")
RIGHT_TOPIC_EE     = _arms.get("right", {}).get("topics", {}).get("end_effector",  "/vla_robot/right/end_effector")

LEFT_MOTOR_CFG   = _abs(_arms.get("left", {}).get("motor_config_path"))
LEFT_JOINT_NAMES  = list(_arms.get("left", {}).get("joint_names", []))
LEFT_TOPIC_JOINTS  = _arms.get("left", {}).get("topics", {}).get("joint_states", "/vla_robot/left/joint_states")
LEFT_TOPIC_EE      = _arms.get("left", {}).get("topics", {}).get("end_effector",  "/vla_robot/left/end_effector")

# ── 安全位置 ──────────────────────────────────
SAFE_JOINT_POS = list(_cfg["safe_position"]["joint_pos"])
SAFE_JOINT_VEL = list(_cfg["safe_position"]["joint_vel"])

# ── 控制参数 ──────────────────────────────────
_ctrl = _cfg["control"]
KP         = list(_ctrl["kp"])
KD         = list(_ctrl["kd"])
GRIPPER_KP = float(_ctrl["gripper_kp"])
GRIPPER_KD = float(_ctrl["gripper_kd"])
EXEC_VEL   = float(_ctrl.get("exec_vel", 1.0))

# ── 控制模式 ──────────────────────────────────
# "dual"  双臂，接收 14 维动作
# "right" 右臂，接收 7 维动作
# "left"  左臂，接收 7 维动作
MODE = str(_cfg.get("mode", "dual")).lower()
if MODE not in ("dual", "left", "right"):
    raise ValueError(f"mode 必须是 dual/left/right，当前: '{MODE}'")

# ── Topics ────────────────────────────────────
TOPIC_VLA_ACTIONS = str(_cfg["topics"]["subscribe"]["vla_actions"])
TOPIC_TRIGGER     = str(_cfg["topics"]["publish"]["trigger"])

# ── VLA prompt ────────────────────────────────
VLA_PROMPT = str(_cfg.get("prompt", ""))
