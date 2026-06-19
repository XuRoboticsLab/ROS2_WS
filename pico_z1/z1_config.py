# ─────────────────────────────────────────────
#  z1_config.py  —  读取 Z1 机械臂侧配置
# ─────────────────────────────────────────────

import os
import yaml

_YAML_PATH = os.environ.get("PICO_Z1_ARM_CONFIG")
if not _YAML_PATH:
    raise RuntimeError("未指定 config 路径，请通过 --config 传入 config.yaml 路径")

with open(_YAML_PATH, "r") as f:
    _cfg = yaml.safe_load(f)

# ── IPC ───────────────────────────────────────
IPC_ADDRESS = str(_cfg["ipc"]["address"])
ARM_SIDE    = str(_cfg["ipc"]["arm_side"])

# ── Z1 SDK ────────────────────────────────────
LIB_PATH    = str(_cfg["z1"]["lib_path"])
HAS_GRIPPER = bool(_cfg["z1"].get("has_gripper", True))

# ── 频率 / watchdog ───────────────────────────
CONTROL_RATE     = float(_cfg["control"]["rate_hz"])
WATCHDOG_TIMEOUT = float(_cfg["control"]["watchdog_timeout_s"])

# ── IK 关节步长限制 ───────────────────────────
MAX_JOINT_STEP = float(_cfg["control"]["max_joint_step_rad"])

# ── Pico 偏移缩放（param_server 可实时调节）──
TRANSLATION_SCALE = float(_cfg["scale"]["translation_m"])
ROTATION_SCALE    = float(_cfg["scale"]["rotation_rad"])

# ── 摇杆精细控制 ─────────────────────────────
FINE_SCALE = float(_cfg.get("fine_control", {}).get("scale_m_per_s", 0.05))

# ── 初始位置（笛卡尔 [rx,ry,rz,x,y,z]）──────
START_CARTESIAN = list(_cfg["start_pose"]["cartesian"])
START_GRIPPER   = float(_cfg["start_pose"].get("gripper", 0.0))
START_SPEED     = float(_cfg["start_pose"].get("speed", 0.3))

# ── 夹爪 ──────────────────────────────────────
GRIPPER_OPEN_POS  = float(_cfg.get("gripper", {}).get("open_pos",   0.0))
GRIPPER_CLOSE_POS = float(_cfg.get("gripper", {}).get("close_pos", -1.0))
GRIPPER_SPEED     = float(_cfg.get("gripper", {}).get("speed_rad_s", 2.0))
GRIPPER_MAX_TAU   = float(_cfg.get("gripper", {}).get("max_torque",  2.0))

# ── 参数服务器 ────────────────────────────────
PARAM_SERVER_PORT = int(_cfg.get("param_server", {}).get("port", 8082))
ARM_NAME          = str(_cfg.get("param_server", {}).get("arm_name", "Z1"))
