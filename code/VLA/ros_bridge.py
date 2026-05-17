"""
VLA ROS Bridge — roslibpy 版，支持三种控制模式

mode: "dual"   双臂，14 维动作 [right(6), right_grip, left(6), left_grip]
mode: "right"  右臂，7 维动作  [joints(6), gripper(1)]
mode: "left"   左臂，7 维动作  [joints(6), gripper(1)]

数据流:
  [trigger /vla/trigger]  std_msgs/String  → policy.infer() → publish /vla/actions
  相机 + 关节状态订阅根据 mode 自动选取

用法:
  python ros_bridge.py --config /path/to/vla_ros_bridge.yaml
"""

import argparse
import base64
import logging
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import roslibpy
import yaml

sys.path.append(".")
from openpi.policies import policy_config as _policy_config
from openpi.training import config as _config

logging.basicConfig(
    level=logging.INFO,
    format="[VLA-ROS] %(asctime)s %(levelname)s %(message)s",
)
logger = logging.getLogger(__name__)

DUAL_ACTION_DIM   = 14   # [right(6), right_grip, left(6), left_grip]
SINGLE_ACTION_DIM = 7    # [joints(6), gripper(1)]

VALID_MODES = ("dual", "left", "right")


# ══════════════════════════════════════════════════════════════════════════
#  Thread-safe caches
# ══════════════════════════════════════════════════════════════════════════

class _ImageCache:
    def __init__(self):
        self._lock = threading.Lock()
        self._img: np.ndarray | None = None

    def update(self, img: np.ndarray):
        with self._lock:
            self._img = img

    def get(self) -> np.ndarray | None:
        with self._lock:
            return self._img.copy() if self._img is not None else None


class _JointStateCache:
    def __init__(self, joint_names: list[str], gripper_name: str = "gripper"):
        self._joint_names  = joint_names
        self._gripper_name = gripper_name
        self._lock = threading.Lock()
        self._joint_pos: list[float] | None = None
        self._gripper_pos: float = 1.0

    def update(self, msg: dict):
        pos_map = dict(zip(msg["name"], msg["position"]))
        with self._lock:
            self._joint_pos   = [pos_map.get(n, 0.0) for n in self._joint_names]
            self._gripper_pos = float(pos_map.get(self._gripper_name, 1.0))

    def get(self) -> tuple[list[float] | None, float]:
        with self._lock:
            jp = self._joint_pos.copy() if self._joint_pos is not None else None
            return jp, self._gripper_pos


# ══════════════════════════════════════════════════════════════════════════
#  Image decode helpers
# ══════════════════════════════════════════════════════════════════════════

def _decode_image(msg: dict) -> np.ndarray | None:
    try:
        h   = int(msg["height"])
        w   = int(msg["width"])
        enc = msg["encoding"].lower()
        raw = base64.b64decode(msg["data"])

        if enc in ("bgr8", "rgb8"):
            img = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 3)
            if enc == "bgr8":
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        elif enc == "mono8":
            img = cv2.cvtColor(
                np.frombuffer(raw, dtype=np.uint8).reshape(h, w),
                cv2.COLOR_GRAY2RGB,
            )
        else:
            logger.warning("不支持的图像编码: %s", enc)
            return None
        return img
    except Exception as e:
        logger.warning("图像解码失败: %s", e)
        return None


def _resize(img: np.ndarray, h: int, w: int) -> np.ndarray:
    if img.shape[0] != h or img.shape[1] != w:
        img = cv2.resize(img, (w, h), interpolation=cv2.INTER_LINEAR)
    return img


# ══════════════════════════════════════════════════════════════════════════
#  VLA ROS Bridge
# ══════════════════════════════════════════════════════════════════════════

class VLARosBridge:
    def __init__(self, cfg: dict, policy):
        self._policy = policy

        self._mode = str(cfg["mode"]).lower()
        if self._mode not in VALID_MODES:
            raise ValueError(f"mode 必须是 {VALID_MODES}，当前: '{self._mode}'")

        ros_cfg = cfg["rosbridge"]
        self._ros = roslibpy.Ros(host=ros_cfg["host"], port=int(ros_cfg["port"]))

        model_cfg = cfg["model"]
        self._img_h = int(model_cfg.get("image_height", 180))
        self._img_w = int(model_cfg.get("image_width",  320))
        self._fallback_prompt = str(model_cfg.get("fallback_prompt", ""))

        joints_cfg   = cfg["joints"]
        gripper_name = str(joints_cfg.get("gripper_name", "gripper"))

        # ── 按模式创建关节/相机缓存 ────────────────────────────────────
        self._extern_cache = _ImageCache()

        if self._mode in ("dual", "right"):
            self._right_joints_cache  = _JointStateCache(
                list(joints_cfg["right_names"]), gripper_name
            )
            self._right_wrist_cache = _ImageCache()
        else:
            self._right_joints_cache  = None
            self._right_wrist_cache = None

        if self._mode in ("dual", "left"):
            self._left_joints_cache  = _JointStateCache(
                list(joints_cfg["left_names"]), gripper_name
            )
            self._left_wrist_cache = _ImageCache()
        else:
            self._left_joints_cache  = None
            self._left_wrist_cache = None

        self._topics_cfg  = cfg["topics"]
        self._infer_lock  = threading.Lock()
        self._infer_count = 0

    # ── 通用回调工厂 ──────────────────────────────────────────────────────

    def _img_cb(self, cache: _ImageCache):
        def cb(msg):
            img = _decode_image(msg)
            if img is not None:
                cache.update(img)
        return cb

    def _joint_cb(self, cache: _JointStateCache):
        def cb(msg):
            cache.update(msg)
        return cb

    def _trigger_cb(self, msg):
        prompt = str(msg.get("data", "") or self._fallback_prompt) or self._fallback_prompt
        logger.info("收到 trigger | mode=%s | prompt='%s'", self._mode, prompt)
        threading.Thread(target=self._infer, args=(prompt,), daemon=True).start()

    # ── 推理调度 ──────────────────────────────────────────────────────────

    def _infer(self, prompt: str):
        if not self._infer_lock.acquire(blocking=False):
            logger.warning("推理跳过：上一次推理仍在进行")
            return
        try:
            if self._mode == "dual":
                self._infer_dual(prompt)
            else:
                self._infer_single(prompt)
        finally:
            self._infer_lock.release()

    # ── 双臂推理（14 维） ─────────────────────────────────────────────────

    def _infer_dual(self, prompt: str):
        extern_img = self._extern_cache.get()
        left_img   = self._left_wrist_cache.get()
        right_img  = self._right_wrist_cache.get()
        right_joints, right_gripper = self._right_joints_cache.get()
        left_joints,  left_gripper  = self._left_joints_cache.get()

        missing = [n for n, v in [
            ("extern_camera", extern_img), ("left_wrist", left_img),
            ("right_wrist", right_img),    ("right_joints", right_joints),
            ("left_joints", left_joints),
        ] if v is None]
        if missing:
            logger.warning("双臂推理跳过：数据未就绪 %s", missing)
            return

        extern_img = _resize(extern_img, self._img_h, self._img_w)
        left_img   = _resize(left_img,   self._img_h, self._img_w)
        right_img  = _resize(right_img,  self._img_h, self._img_w)

        # HT format state: [right(6), right_grip, left(6), left_grip]
        state = np.concatenate([
            np.array(right_joints,    dtype=np.float32),
            np.array([right_gripper], dtype=np.float32),
            np.array(left_joints,     dtype=np.float32),
            np.array([left_gripper],  dtype=np.float32),
        ])

        inputs = {
            "images": {
                "cam_extern": extern_img.transpose(2, 0, 1),
                "cam_left":   left_img.transpose(2, 0, 1),
                "cam_right":  right_img.transpose(2, 0, 1),
            },
            "state":  state,
            "prompt": prompt,
        }

        actions = self._run_policy(inputs, DUAL_ACTION_DIM)
        if actions is None:
            return

        logger.info(
            "双臂推理 #%d | shape=%s | "
            "right_j=%s grip=%.1f | left_j=%s grip=%.1f",
            self._infer_count, actions.shape,
            np.round(actions[0, 0:6], 4), actions[0, 6],
            np.round(actions[0, 7:13], 4), actions[0, 13],
        )
        self._publish_actions(actions)

    # ── 单臂推理（7 维） ──────────────────────────────────────────────────

    def _infer_single(self, prompt: str):
        arm_side     = self._mode                   # "left" or "right"
        joints_cache = (self._right_joints_cache
                        if arm_side == "right" else self._left_joints_cache)
        wrist_cache  = (self._right_wrist_cache
                        if arm_side == "right" else self._left_wrist_cache)

        extern_img          = self._extern_cache.get()
        wrist_img           = wrist_cache.get()
        joints, gripper_pos = joints_cache.get()

        missing = [n for n, v in [
            ("extern_camera", extern_img),
            (f"{arm_side}_wrist", wrist_img),
            (f"{arm_side}_joints", joints),
        ] if v is None]
        if missing:
            logger.warning("%s 臂推理跳过：数据未就绪 %s", arm_side, missing)
            return

        extern_img = _resize(extern_img, self._img_h, self._img_w)
        wrist_img  = _resize(wrist_img,  self._img_h, self._img_w)

        # state: [joints(6), gripper(1)]
        state = np.concatenate([
            np.array(joints,         dtype=np.float32),
            np.array([gripper_pos],  dtype=np.float32),
        ])

        # HTSingleArmInputs 期望：cam_left = 腕部相机，cam_extern = 外部相机
        inputs = {
            "images": {
                "cam_left":   wrist_img.transpose(2, 0, 1),
                "cam_extern": extern_img.transpose(2, 0, 1),
            },
            "state":  state,
            "prompt": prompt,
        }

        actions = self._run_policy(inputs, SINGLE_ACTION_DIM)
        if actions is None:
            return

        logger.info(
            "%s 臂推理 #%d | shape=%s | joints=%s grip=%.1f",
            arm_side, self._infer_count,
            actions.shape,
            np.round(actions[0, 0:6], 4), actions[0, 6],
        )
        self._publish_actions(actions)

    # ── 公共推理执行 ──────────────────────────────────────────────────────

    def _run_policy(self, inputs: dict, effective_dim: int) -> np.ndarray | None:
        try:
            t0      = time.time()
            result  = self._policy.infer(inputs)
            elapsed = time.time() - t0
            actions = result["actions"]
        except Exception as e:
            logger.error("policy.infer() 失败: %s", e)
            return None

        if actions.ndim == 1:
            actions = actions[np.newaxis, :]
        actions = actions[:, :effective_dim].astype(np.float32)

        self._infer_count += 1
        logger.info("推理 #%d | shape=%s | elapsed=%.3fs",
                    self._infer_count, actions.shape, elapsed)
        return actions

    # ── 发布动作 ──────────────────────────────────────────────────────────

    def _publish_actions(self, actions: np.ndarray):
        chunk, dim = actions.shape
        pub_topic  = self._topics_cfg["publish"]["actions"]
        self._actions_pub.publish(roslibpy.Message({
            "layout": {
                "dim": [
                    {"label": "chunk",      "size": chunk, "stride": chunk * dim},
                    {"label": "action_dim", "size": dim,   "stride": dim},
                ],
                "data_offset": 0,
            },
            "data": actions.flatten().tolist(),
        }))
        logger.info("已发布 actions (%dx%d) → %s", chunk, dim, pub_topic)

    # ── 主入口 ────────────────────────────────────────────────────────────

    def run(self):
        logger.info("mode=%s | 连接 rosbridge %s:%s ...",
                    self._mode, self._ros.host, self._ros.port)
        self._ros.run()
        if not self._ros.is_connected:
            raise RuntimeError("rosbridge 连接失败")
        logger.info("rosbridge 已连接")

        sub_cfg  = self._topics_cfg["subscribe"]
        cam_cfg  = self._topics_cfg["cameras"]

        self._actions_pub = roslibpy.Topic(
            self._ros, self._topics_cfg["publish"]["actions"],
            "std_msgs/Float32MultiArray",
        )

        # ── 构建订阅列表（根据 mode） ──────────────────────────────────
        subs: list[roslibpy.Topic] = []

        def _sub(topic: str, msg_type: str, cb):
            t = roslibpy.Topic(self._ros, topic, msg_type)
            t.subscribe(cb)
            subs.append(t)
            logger.info("已订阅 %s", topic)

        # 外部相机（所有模式都需要）
        _sub(cam_cfg["extern"], "sensor_msgs/Image",
             self._img_cb(self._extern_cache))

        # 触发 topic（所有模式都需要）
        _sub(sub_cfg["trigger"], "std_msgs/String", self._trigger_cb)

        # 右臂相关
        if self._mode in ("dual", "right"):
            _sub(cam_cfg["right_wrist"], "sensor_msgs/Image",
                 self._img_cb(self._right_wrist_cache))
            _sub(sub_cfg["right_joints"], "sensor_msgs/JointState",
                 self._joint_cb(self._right_joints_cache))

        # 左臂相关
        if self._mode in ("dual", "left"):
            _sub(cam_cfg["left_wrist"], "sensor_msgs/Image",
                 self._img_cb(self._left_wrist_cache))
            _sub(sub_cfg["left_joints"], "sensor_msgs/JointState",
                 self._joint_cb(self._left_joints_cache))

        logger.info("共订阅 %d 个 topic，等待 trigger（%s）...",
                    len(subs), sub_cfg["trigger"])

        try:
            while self._ros.is_connected:
                time.sleep(0.5)
        except KeyboardInterrupt:
            logger.info("收到退出信号...")
        finally:
            for s in subs:
                s.unsubscribe()
            self._actions_pub.unadvertise()
            self._ros.terminate()
            logger.info("已关闭")


# ══════════════════════════════════════════════════════════════════════════
#  Entry point
# ══════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="VLA ROS Bridge")
    parser.add_argument("--config", "-c", required=True, help="config.yaml 路径")
    args = parser.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f) or {}

    model_cfg = cfg["model"]
    logger.info("加载 VLA 模型: %s  (mode=%s)", model_cfg["config_name"], cfg.get("mode"))
    config = _config.get_config(model_cfg["config_name"])
    policy = _policy_config.create_trained_policy(
        config, Path(model_cfg["checkpoint_dir"])
    )
    logger.info("模型加载完成")

    VLARosBridge(cfg, policy).run()


if __name__ == "__main__":
    main()
