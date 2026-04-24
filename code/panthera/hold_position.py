#!/usr/bin/env python3
"""
hold_position.py — 机械臂定点保持控制器

两种模式让机械臂停在目标位置，之后持续发布 JointState + PoseStamped：

  --mode 1 (默认): 前 N 秒零重力（人工摆臂），到时间后自动锁定当前位置
  --mode 2:        读取 JSON 快照文件，运动到该关节角度后锁定

运行期间持续向 rosbridge 发布：
  • sensor_msgs/JointState    → topics.publish.joints
  • geometry_msgs/PoseStamped → topics.publish.ee

用法:
  python hold_position.py --config path/to/config.yaml
  python hold_position.py --config path/to/config.yaml --mode 2 --snapshot state.json
  python hold_position.py --config path/to/config.yaml --mode 1 --gravity-secs 8
"""

import argparse
import json
import os
import sys
import threading
import time

import numpy as np
import roslibpy
import yaml
from scipy.spatial.transform import Rotation

# ── 使用与 joystick_control 相同的 Panthera_lib ──────────────────────────
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "joystick_control"))
from Panthera_lib import Panthera  # noqa: E402

# ── 默认参数 ─────────────────────────────────────────────────────────────
DEFAULT_CONTROL_HZ   = 50
DEFAULT_PUBLISH_HZ   = 50
DEFAULT_GRAVITY_SECS = 5.0
DEFAULT_MOVEJ_SECS   = 4.0   # mode 2 运动到目标的时长


# ═══════════════════════════════════════════════════════════════════════════
#  配置加载
# ═══════════════════════════════════════════════════════════════════════════

def load_config(yaml_path: str) -> dict:
    with open(yaml_path) as f:
        return yaml.safe_load(f) or {}


# ═══════════════════════════════════════════════════════════════════════════
#  主控制器
# ═══════════════════════════════════════════════════════════════════════════

class HoldPositionController:
    """
    Phase 1 (mode 1): gravity-comp 窗口，人工摆臂
    Phase 1 (mode 2): 运动到快照位置
    Phase 2 (共用):   pos_vel_tqe_kp_kd 锁定 + 持续发布
    """

    def __init__(
        self,
        robot: Panthera,
        ros: roslibpy.Ros,
        *,
        topic_joints: str,
        topic_ee: str,
        joint_names: list,
        kp: list,
        kd: list,
        control_hz: float,
        publish_hz: float,
        mode: int,
        gravity_secs: float,
        snapshot_path: str | None,
    ):
        self.robot        = robot
        self.ros          = ros
        self.joint_names  = joint_names          # 不含 gripper
        self.n_joints     = len(joint_names)
        self.kp           = kp
        self.kd           = kd
        self.control_hz   = control_hz
        self.publish_hz   = publish_hz
        self.mode         = mode
        self.gravity_secs = gravity_secs
        self.snapshot_path = snapshot_path

        self._stop   = threading.Event()

        # 锁定目标（control 线程写，hold_step 读）
        self._lock_pos     = None   # list[float], 长度 = n_joints
        self._lock_gripper = None   # float

        # 发布缓存（control 线程写，publisher 线程读）
        self._state_mu   = threading.Lock()
        self._joint_pos  = [0.0] * self.n_joints
        self._joint_vel  = [0.0] * self.n_joints
        self._grip_pos   = 0.0
        self._grip_vel   = 0.0
        self._fk_pos     = [0.0, 0.0, 0.0]
        self._fk_quat    = [0.0, 0.0, 0.0, 1.0]   # [x, y, z, w]
        self._stamp      = {"sec": 0, "nanosec": 0}

        # roslibpy 发布者
        self._pub_joints = roslibpy.Topic(ros, topic_joints, "sensor_msgs/JointState")
        self._pub_ee     = roslibpy.Topic(ros, topic_ee,     "geometry_msgs/PoseStamped")

    # ──────────────────────────────────────────────────────────────────────
    #  对外入口
    # ──────────────────────────────────────────────────────────────────────

    def run(self):
        pub_thread  = threading.Thread(target=self._publisher_loop, daemon=True, name="StatePublisher")
        ctrl_thread = threading.Thread(target=self._control_loop,   daemon=True, name="ControlLoop")

        pub_thread.start()
        ctrl_thread.start()

        try:
            while ctrl_thread.is_alive():
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\n[hold_position] Ctrl-C — 正在退出...")
        finally:
            self._stop.set()
            pub_thread.join(timeout=2.0)
            self._pub_joints.unadvertise()
            self._pub_ee.unadvertise()

    # ──────────────────────────────────────────────────────────────────────
    #  控制主线程
    # ──────────────────────────────────────────────────────────────────────

    def _control_loop(self):
        # ── Phase 1: 到达目标位置 ──────────────────────────────────────
        if self.mode == 1:
            self._phase_gravity_comp()
        else:
            self._phase_move_to_snapshot()

        if self._stop.is_set():
            return

        print("[hold_position] ✓ 位置已锁定，持续发布中（Ctrl-C 退出）")

        # ── Phase 2: 锁定保持 ──────────────────────────────────────────
        dt = 1.0 / self.control_hz
        while not self._stop.is_set():
            t0 = time.monotonic()
            self._hold_step()
            elapsed = time.monotonic() - t0
            sleep_t = dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    # ──────────────────────────────────────────────────────────────────────
    #  Phase 1 — Mode 1: 零重力窗口
    # ──────────────────────────────────────────────────────────────────────

    def _phase_gravity_comp(self):
        print(
            f"[hold_position] Mode 1: {self.gravity_secs:.1f}s 零重力模式 — "
            "请手动将机械臂移动到目标位置..."
        )
        dt       = 1.0 / self.control_hz
        deadline = time.monotonic() + self.gravity_secs
        prev_sec = int(self.gravity_secs) + 1

        while time.monotonic() < deadline and not self._stop.is_set():
            t0 = time.monotonic()

            self._refresh_motor_state()
            pos  = list(self.robot.get_current_pos())
            vel  = list(self.robot.get_current_vel())
            grav = list(self.robot.get_Gravity())

            # kp=0, kd=0 → 纯重力补偿，人可自由移动
            self.robot.pos_vel_tqe_kp_kd(
                pos,
                [0.0] * self.n_joints,
                grav,
                [0.0] * self.n_joints,
                [0.0] * self.n_joints,
            )

            self._update_state(pos, vel)

            remaining = deadline - time.monotonic()
            cur_sec   = max(0, int(remaining))
            if cur_sec < prev_sec:
                prev_sec = cur_sec
                print(f"  剩余 {cur_sec}s ...")

            time.sleep(max(0.0, dt - (time.monotonic() - t0)))

        if self._stop.is_set():
            return

        # 读取当前位置作为锁定目标
        self._refresh_motor_state()
        pos     = list(self.robot.get_current_pos())
        gripper = self.robot.get_current_state_gripper()

        self._lock_pos     = pos
        self._lock_gripper = float(gripper.position)
        print(f"[hold_position] 锁定关节角度: {[round(v, 4) for v in pos]}")

    # ──────────────────────────────────────────────────────────────────────
    #  Phase 1 — Mode 2: 读取快照并运动
    # ──────────────────────────────────────────────────────────────────────

    def _phase_move_to_snapshot(self):
        print(f"[hold_position] Mode 2: 读取快照 '{self.snapshot_path}'")
        with open(self.snapshot_path) as f:
            snap = json.load(f)

        pos_map     = dict(zip(snap["name"], snap["position"]))
        target_arm  = [pos_map[n] for n in self.joint_names]
        target_grip = float(pos_map.get("gripper", 0.0))

        # 取快照中的速度作为参考运动速度（若有），否则用安全默认值
        vel_map = dict(zip(snap.get("name", []), snap.get("velocity", [])))
        move_vel = [abs(vel_map.get(n, 0.5)) or 0.5 for n in self.joint_names]

        print(f"[hold_position] 目标关节角度: {[round(v, 4) for v in target_arm]}")
        print(f"[hold_position] 正在运动到目标位置...")

        ok = self.robot.Joint_Pos_Vel(target_arm, move_vel, iswait=True)
        if not ok:
            print("[hold_position] ⚠ Joint_Pos_Vel 返回失败，检查目标是否超出限位")

        self._lock_pos     = target_arm
        self._lock_gripper = target_grip
        print(f"[hold_position] ✓ 已到达目标位置")

    # ──────────────────────────────────────────────────────────────────────
    #  Phase 2: 单次保持控制节拍
    # ──────────────────────────────────────────────────────────────────────

    def _hold_step(self):
        self._refresh_motor_state()
        pos  = list(self.robot.get_current_pos())
        vel  = list(self.robot.get_current_vel())
        grav = list(self.robot.get_Gravity())

        self.robot.pos_vel_tqe_kp_kd(
            self._lock_pos,
            [0.0] * self.n_joints,
            grav,
            self.kp,
            self.kd,
        )
        self._update_state(pos, vel)

    # ──────────────────────────────────────────────────────────────────────
    #  辅助：刷新电机状态（参考 main.py 的初始化模式）
    # ──────────────────────────────────────────────────────────────────────

    def _refresh_motor_state(self):
        self.robot.send_get_motor_state_cmd()
        self.robot.motor_send_cmd()

    # ──────────────────────────────────────────────────────────────────────
    #  辅助：更新发布缓存
    # ──────────────────────────────────────────────────────────────────────

    def _update_state(self, joint_pos: list, joint_vel: list):
        try:
            fk  = self.robot.forward_kinematics()
            pos = list(fk["position"])
            q   = Rotation.from_matrix(np.array(fk["rotation"], dtype=float)).as_quat().tolist()
        except Exception:
            pos = [0.0, 0.0, 0.0]
            q   = [0.0, 0.0, 0.0, 1.0]

        try:
            gripper = self.robot.get_current_state_gripper()
            grip_p  = float(gripper.position)
            grip_v  = float(gripper.velocity)
        except Exception:
            grip_p = self._lock_gripper or 0.0
            grip_v = 0.0

        t_ns = time.time_ns()
        stamp = {"sec": t_ns // 1_000_000_000, "nanosec": t_ns % 1_000_000_000}

        with self._state_mu:
            self._joint_pos = list(joint_pos)
            self._joint_vel = list(joint_vel)
            self._grip_pos  = grip_p
            self._grip_vel  = grip_v
            self._fk_pos    = pos
            self._fk_quat   = q
            self._stamp     = stamp

    # ──────────────────────────────────────────────────────────────────────
    #  发布线程
    # ──────────────────────────────────────────────────────────────────────

    def _publisher_loop(self):
        interval = 1.0 / self.publish_hz
        while not self._stop.is_set():
            t0 = time.time()
            if self.ros.is_connected:
                self._publish_once()
            time.sleep(max(0.0, interval - (time.time() - t0)))

    def _publish_once(self):
        with self._state_mu:
            joint_pos = self._joint_pos[:]
            joint_vel = self._joint_vel[:]
            grip_pos  = self._grip_pos
            grip_vel  = self._grip_vel
            fk_pos    = self._fk_pos[:]
            q         = self._fk_quat[:]
            stamp     = self._stamp.copy()

        all_names = self.joint_names + ["gripper"]
        all_pos   = joint_pos + [grip_pos]
        all_vel   = joint_vel + [grip_vel]

        self._pub_joints.publish(roslibpy.Message({
            "header":   {"stamp": stamp, "frame_id": "base_link"},
            "name":     all_names,
            "position": all_pos,
            "velocity": all_vel,
            "effort":   [0.0] * len(all_names),
        }))

        self._pub_ee.publish(roslibpy.Message({
            "header": {"stamp": stamp, "frame_id": "base_link"},
            "pose": {
                "position":    {"x": fk_pos[0], "y": fk_pos[1], "z": fk_pos[2]},
                "orientation": {"x": q[0],      "y": q[1],      "z": q[2], "w": q[3]},
            },
        }))


# ═══════════════════════════════════════════════════════════════════════════
#  入口
# ═══════════════════════════════════════════════════════════════════════════

def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--config", "-c", required=True, metavar="PATH",
                   help="config.yaml 路径（与 joystick_control 共用同一格式）")
    p.add_argument("--mode", type=int, choices=[1, 2], default=1,
                   help="1=零重力摆位后锁定（默认）；2=读取快照后锁定")
    p.add_argument("--snapshot", type=str, default=None, metavar="PATH",
                   help="mode 2 所需的 JointState JSON 快照路径")
    p.add_argument("--gravity-secs", type=float, default=DEFAULT_GRAVITY_SECS,
                   metavar="N",
                   help=f"mode 1 零重力窗口时长（秒，默认 {DEFAULT_GRAVITY_SECS}）")
    return p.parse_args()


def main():
    args = parse_args()

    if args.mode == 2 and not args.snapshot:
        sys.exit("错误：mode 2 需要通过 --snapshot 指定 JSON 快照路径")

    # ── 加载配置 ────────────────────────────────────────────────────────
    cfg           = load_config(args.config)
    motor_cfg     = str(cfg["motor_config_path"])
    host          = cfg["rosbridge"]["host"]
    port          = cfg["rosbridge"]["port"]
    control_hz    = float(cfg.get("rates", {}).get("control_hz",  DEFAULT_CONTROL_HZ))
    publish_hz    = float(cfg.get("rates", {}).get("publish_hz",  DEFAULT_PUBLISH_HZ))
    kp            = list(cfg["ik_control"]["kp"])
    kd            = list(cfg["ik_control"]["kd"])
    joint_names   = list(cfg["joints"]["names"])
    topic_joints  = cfg["topics"]["publish"]["joints"]
    topic_ee      = cfg["topics"]["publish"]["ee"]
    safe_pos      = list(cfg["safe_position"]["joint_pos"])
    safe_vel      = list(cfg["safe_position"]["joint_vel"])

    print("=" * 60)
    print("Panthera 定点保持控制器")
    print(f"  Mode:        {args.mode}")
    if args.mode == 1:
        print(f"  零重力窗口:  {args.gravity_secs}s")
    else:
        print(f"  快照文件:    {args.snapshot}")
    print(f"  控制频率:    {control_hz} Hz")
    print(f"  发布频率:    {publish_hz} Hz")
    print("=" * 60)

    # ── 初始化机械臂 ────────────────────────────────────────────────────
    print("\n[Init] 初始化机械臂...")
    os.environ["PANTHERA_CONFIG"] = os.path.abspath(args.config)
    robot = Panthera(os.path.abspath(motor_cfg))
    robot.send_get_motor_state_cmd()
    robot.motor_send_cmd()
    time.sleep(0.3)

    print("[Init] 移动到安全位置...")
    ok = robot.Joint_Pos_Vel(safe_pos, safe_vel, iswait=True)
    if not ok:
        sys.exit("[Init] ✗ 移动到安全位置失败，退出")
    print("[Init] ✓ 已到达安全位置")
    time.sleep(0.5)

    robot.send_get_motor_state_cmd()
    robot.motor_send_cmd()
    time.sleep(0.1)

    # ── 连接 rosbridge ──────────────────────────────────────────────────
    print(f"\n[ROS] 连接 {host}:{port}...")
    ros = roslibpy.Ros(host=host, port=port)
    ros.run()
    if not ros.is_connected:
        sys.exit("[ROS] ✗ rosbridge 连接失败")
    print("[ROS] ✓ 已连接")

    # ── 启动控制器 ──────────────────────────────────────────────────────
    controller = HoldPositionController(
        robot         = robot,
        ros           = ros,
        topic_joints  = topic_joints,
        topic_ee      = topic_ee,
        joint_names   = joint_names,
        kp            = kp,
        kd            = kd,
        control_hz    = control_hz,
        publish_hz    = publish_hz,
        mode          = args.mode,
        gravity_secs  = args.gravity_secs,
        snapshot_path = args.snapshot,
    )

    try:
        controller.run()
    finally:
        print("\n[Main] 返回安全位置...")
        robot.Joint_Pos_Vel(safe_pos, safe_vel, iswait=True)
        ros.terminate()
        print("[Main] 完成")


if __name__ == "__main__":
    main()
