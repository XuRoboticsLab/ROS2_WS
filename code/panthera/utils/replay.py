#!/usr/bin/env python3
"""
replay_trajectory.py — 从记录的 JointState JSON 文件重放轨迹

用法:
  python replay_trajectory.py --config path/to/config.yaml --session path/to/session_dir
  python replay_trajectory.py --config path/to/config.yaml --session path/to/session_dir \\
      --joint-name joint_state --speed-scale 1.0 --dry-run

目录结构（data_collector_node 的输出格式）:
  <session_dir>/
    000001/
      <timestamp_ns>_joint_state.json
      <timestamp_ns>_image.npy
      ...
    000002/
      ...

脚本只读取每个快照目录里名字匹配 joint_name 的 .json 文件。
"""

import argparse
import json
import os
import sys
import time
from pathlib import Path

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "joystick_control"))
from Panthera_lib import Panthera  # noqa: E402

# ── 默认值 ─────────────────────────────────────────────────────────────────
DEFAULT_SPEED_SCALE  = 1.0   # 1.0 = 按原始时间间隔重放
DEFAULT_GOTO_VEL     = 0.5   # 移动到起始帧时每个关节的速度 (rad/s)
DEFAULT_REPLAY_VEL   = 1.0   # 逐帧重放时每个关节的最大速度 (rad/s)
DEFAULT_JOINT_NAME   = "joint_state"  # 匹配文件名中的 <name> 部分


# ═══════════════════════════════════════════════════════════════════════════
#  配置 & 数据加载
# ═══════════════════════════════════════════════════════════════════════════

def load_config(yaml_path: str) -> dict:
    with open(yaml_path) as f:
        return yaml.safe_load(f) or {}


def load_session(session_dir: Path, joint_name: str) -> list[dict]:
    """
    遍历 session_dir 下所有快照子目录，收集匹配 joint_name 的 JSON 文件。

    返回按时间戳排序的列表，每项为:
      {
        "ts_ns":    int,          # 文件名中的纳秒时间戳
        "position": list[float],  # 关节角度（不含 gripper）
        "gripper":  float,        # gripper 位置
        "names":    list[str],    # 原始 name 列表
      }
    """
    snapshot_dirs = sorted(
        p for p in session_dir.iterdir()
        if p.is_dir() and p.name.isdigit()
    )

    if not snapshot_dirs:
        sys.exit(f"[ERROR] session 目录下没有找到快照子目录: {session_dir}")

    frames = []
    for snap_dir in snapshot_dirs:
        # 找匹配的 JSON 文件：<ts_ns>_<joint_name>.json
        candidates = list(snap_dir.glob(f"*_{joint_name}.json"))
        if not candidates:
            print(f"  [WARN] {snap_dir.name}: 未找到 '{joint_name}' JSON，跳过")
            continue
        if len(candidates) > 1:
            print(f"  [WARN] {snap_dir.name}: 找到多个匹配文件，使用第一个")

        json_path = candidates[0]
        try:
            with open(json_path) as f:
                data = json.load(f)
        except Exception as e:
            print(f"  [WARN] 读取 {json_path} 失败: {e}，跳过")
            continue

        # 从文件名提取时间戳
        ts_ns = int(json_path.stem.split("_")[0])

        names    = data["name"]
        position = data["position"]
        pos_map  = dict(zip(names, position))

        frames.append({
            "ts_ns":   ts_ns,
            "names":   names,
            "pos_map": pos_map,
        })

    if not frames:
        sys.exit(f"[ERROR] 没有成功加载任何帧，请检查 --joint-name 参数和目录结构")

    frames.sort(key=lambda x: x["ts_ns"])
    print(f"[INFO] 共加载 {len(frames)} 帧，"
          f"时长约 {(frames[-1]['ts_ns'] - frames[0]['ts_ns']) / 1e9:.2f}s")
    return frames


# ═══════════════════════════════════════════════════════════════════════════
#  重放逻辑
# ═══════════════════════════════════════════════════════════════════════════

def build_arm_target(frame: dict, joint_names: list[str]) -> list[float]:
    """从帧的 pos_map 中取出机械臂关节角度（不含 gripper）。"""
    pos_map = frame["pos_map"]
    missing = [n for n in joint_names if n not in pos_map]
    if missing:
        raise KeyError(f"帧中缺少关节: {missing}")
    return [float(pos_map[n]) for n in joint_names]


def replay(
    robot: Panthera,
    frames: list[dict],
    joint_names: list[str],
    safe_pos: list[float],
    safe_vel: list[float],
    goto_vel: float,
    replay_vel: float,
    speed_scale: float,
    dry_run: bool,
):
    n = len(joint_names)

    # ── 步骤 1: 移动到安全位置 ────────────────────────────────────────────
    print("\n[Step 1] 移动到安全位置...")
    if not dry_run:
        ok = robot.Joint_Pos_Vel(safe_pos, safe_vel, iswait=True)
        if not ok:
            sys.exit("[ERROR] 移动到安全位置失败，退出")
    print("  ✓ 已到达安全位置")
    time.sleep(0.5)

    # ── 步骤 2: 快速移动到第一帧 ──────────────────────────────────────────
    first_frame = frames[0]
    first_target = build_arm_target(first_frame, joint_names)
    first_grip   = float(first_frame["pos_map"].get("gripper", 0.0))

    print(f"\n[Step 2] 快速移动到起始帧...")
    print(f"  关节角度: {[round(v, 4) for v in first_target]}")
    print(f"  Gripper:  {first_grip:.4f}")

    goto_vels = [goto_vel] * n
    if not dry_run:
        ok = robot.Joint_Pos_Vel(first_target, goto_vels, iswait=True)
        if not ok:
            print("  [WARN] 移动到起始帧返回失败，继续执行...")
        robot.send_get_motor_state_cmd()
        robot.motor_send_cmd()
    print("  ✓ 已到达起始帧")
    time.sleep(1.0)

    # ── 步骤 3: 逐帧重放 ──────────────────────────────────────────────────
    print(f"\n[Step 3] 开始逐帧重放（speed_scale={speed_scale}）...")
    print("  按 Ctrl-C 可随时中止\n")

    replay_vels = [replay_vel] * n
    t_start_wall = time.monotonic()
    t_start_traj = frames[0]["ts_ns"] / 1e9  # 轨迹起始时间（秒）

    try:
        for i, frame in enumerate(frames):
            traj_t  = (frame["ts_ns"] / 1e9 - t_start_traj) / speed_scale
            wall_t  = time.monotonic() - t_start_wall

            # 等待到该帧应该执行的时刻
            wait = traj_t - wall_t
            if wait > 0:
                time.sleep(wait)

            target = build_arm_target(frame, joint_names)
            gripper = float(frame["pos_map"].get("gripper", 0.0))

            print(
                f"\r  帧 {i+1:>5}/{len(frames)}  "
                f"traj_t={traj_t:7.3f}s  "
                f"pos=[{', '.join(f'{v:.3f}' for v in target)}]  "
                f"grip={gripper:.3f}",
                end="",
                flush=True,
            )

            if not dry_run:
                robot.Joint_Pos_Vel(target, replay_vels, iswait=False)
                robot.send_get_motor_state_cmd()
                robot.motor_send_cmd()

    except KeyboardInterrupt:
        print("\n\n  [中止] 用户中断重放")

    print(f"\n\n[Step 3] 重放完成，共执行 {i+1} 帧")

    # ── 步骤 4: 返回安全位置 ──────────────────────────────────────────────
    print("\n[Step 4] 返回安全位置...")
    if not dry_run:
        robot.Joint_Pos_Vel(safe_pos, safe_vel, iswait=True)
    print("  ✓ 完成")


# ═══════════════════════════════════════════════════════════════════════════
#  CLI & 入口
# ═══════════════════════════════════════════════════════════════════════════

def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--config", "-c", required=True, metavar="PATH",
                   help="config.yaml 路径（与 hold_position 共用同一格式）")
    p.add_argument("--session", "-s", required=True, metavar="PATH",
                   help="session 目录路径，即 <output_dir>/<YYYYMMDD_HHMMSS>/")
    p.add_argument("--joint-name", default=DEFAULT_JOINT_NAME, metavar="NAME",
                   help=f"JSON 文件名中 <name> 部分（默认: '{DEFAULT_JOINT_NAME}'）")
    p.add_argument("--speed-scale", type=float, default=DEFAULT_SPEED_SCALE,
                   metavar="F",
                   help=f"重放速度倍率，1.0=原速，0.5=半速（默认: {DEFAULT_SPEED_SCALE}）")
    p.add_argument("--goto-vel", type=float, default=DEFAULT_GOTO_VEL,
                   metavar="V",
                   help=f"移动到起始帧时的关节速度 rad/s（默认: {DEFAULT_GOTO_VEL}）")
    p.add_argument("--replay-vel", type=float, default=DEFAULT_REPLAY_VEL,
                   metavar="V",
                   help=f"逐帧重放时的最大关节速度 rad/s（默认: {DEFAULT_REPLAY_VEL}）")
    p.add_argument("--dry-run", action="store_true",
                   help="只打印帧信息，不实际发送命令给机械臂")
    return p.parse_args()


def main():
    args = parse_args()

    session_dir = Path(args.session)
    if not session_dir.is_dir():
        sys.exit(f"[ERROR] session 目录不存在: {session_dir}")

    # ── 加载配置 ────────────────────────────────────────────────────────
    cfg         = load_config(args.config)
    motor_cfg   = str(cfg["motor_config_path"])
    joint_names = list(cfg["joints"]["names"])          # 不含 gripper
    safe_pos    = list(cfg["safe_position"]["joint_pos"])
    safe_vel    = list(cfg["safe_position"]["joint_vel"])

    print("=" * 60)
    print("轨迹重放工具")
    print(f"  Session:     {session_dir}")
    print(f"  Joint name:  {args.joint_name}")
    print(f"  Speed scale: {args.speed_scale}x")
    print(f"  Goto vel:    {args.goto_vel} rad/s")
    print(f"  Replay vel:  {args.replay_vel} rad/s")
    print(f"  Dry run:     {args.dry_run}")
    print("=" * 60)

    # ── 加载轨迹帧 ──────────────────────────────────────────────────────
    frames = load_session(session_dir, args.joint_name)

    # ── 初始化机械臂（dry-run 时跳过）──────────────────────────────────
    robot = None
    if not args.dry_run:
        print("\n[Init] 初始化机械臂...")
        os.environ["PANTHERA_CONFIG"] = os.path.abspath(args.config)
        robot = Panthera(os.path.abspath(motor_cfg))
        robot.send_get_motor_state_cmd()
        robot.motor_send_cmd()
        time.sleep(0.3)
        print("[Init] ✓ 机械臂就绪")
    else:
        print("\n[Init] Dry-run 模式，跳过机械臂初始化")

    # ── 执行重放 ────────────────────────────────────────────────────────
    try:
        replay(
            robot      = robot,
            frames     = frames,
            joint_names= joint_names,
            safe_pos   = safe_pos,
            safe_vel   = safe_vel,
            goto_vel   = args.goto_vel,
            replay_vel = args.replay_vel,
            speed_scale= args.speed_scale,
            dry_run    = args.dry_run,
        )
    except Exception as e:
        print(f"\n[ERROR] 重放过程中出现异常: {e}")
        if robot and not args.dry_run:
            print("[Safety] 尝试返回安全位置...")
            robot.Joint_Pos_Vel(safe_pos, safe_vel, iswait=True)
        raise


if __name__ == "__main__":
    main()