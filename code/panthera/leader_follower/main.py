#!/usr/bin/env python3
# ─────────────────────────────────────────────
#  main.py  —  双主双从（任意 N 对）遥操作 + ROS 发布
#
#  用法:
#    python main.py                        # 使用默认 config.yaml
#    python main.py --config /path/to/cfg  # 指定 config 路径
# ─────────────────────────────────────────────

# !! argparse 必须在所有本地 import 之前 !!
import argparse
import os

_parser = argparse.ArgumentParser(description="Leader-Follower ROS Bridge")
_parser.add_argument(
    "--config", "-c",
    default=None,
    metavar="PATH",
    help="config.yaml 路径（默认：本文件上一级目录的 config.yaml）",
)
_args = _parser.parse_args()
if _args.config:
    os.environ["LEADER_FOLLOWER_CONFIG"] = os.path.abspath(_args.config)

import time
import threading
import roslibpy
from Panthera_lib import Panthera

from config import ROSBRIDGE_HOST, ROSBRIDGE_PORT, PAIRS
from arm_pair import ArmPair
from ros_publisher import FollowerStatePublisher


def init_pairs() -> list[ArmPair]:
    """按 config 初始化所有主从对"""
    pairs = []
    for p in PAIRS:
        print(f"\n[Init] 初始化 {p['name']}...")
        leader   = Panthera(p["leader"])
        follower = Panthera(p["follower"])
        pairs.append(ArmPair(leader, follower, name=p["name"]))
        print(f"  Leader:   {p['leader']}")
        print(f"  Follower: {p['follower']}")
        print(f"  Topics:   {p['topics']}")
    return pairs


def control_loop(pairs: list[ArmPair],
                 publisher: FollowerStatePublisher,
                 stop_event: threading.Event):
    """主控制循环：依次步进所有主从对，并更新发布缓存"""
    print(f"[Control] 控制循环启动，共 {len(pairs)} 对主从臂")
    loop_count = 0

    while not stop_event.is_set():
        t0 = time.time()

        for pair in pairs:
            try:
                joint_pos, fk = pair.step()
                publisher.update(pair.name, joint_pos, fk)
            except Exception as e:
                print(f"\n[Control] {pair.name} 控制异常: {e}")

        # 打印（每 500 帧 ≈ 0.5s）
        if loop_count % 500 == 0:
            status_parts = []
            for pair in pairs:
                fk = pair.follower.forward_kinematics()
                if fk:
                    p = fk["position"]
                    status_parts.append(
                        f"{pair.name}: [{p[0]:.3f},{p[1]:.3f},{p[2]:.3f}]"
                    )
            if status_parts:
                print(f"\r{'  |  '.join(status_parts)}    ", end="", flush=True)

        loop_count += 1
        elapsed = time.time() - t0
        time.sleep(max(0.0, 0.001 - elapsed))   # ~1kHz 上限


def main():
    print("=" * 60)
    print("Leader-Follower ROS Bridge")
    print("=" * 60)

    # ── 初始化主从对 ─────────────────────────
    pairs = init_pairs()
    print(f"\n[Init] ✓ 共初始化 {len(pairs)} 对主从臂")

    # ── 连接 rosbridge ───────────────────────
    print(f"\n[ROS] 连接 {ROSBRIDGE_HOST}:{ROSBRIDGE_PORT}...")
    ros = roslibpy.Ros(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
    ros.run()
    if not ros.is_connected:
        raise RuntimeError("rosbridge 连接失败")
    print("[ROS] ✓ 已连接")

    publisher  = FollowerStatePublisher(ros)
    stop_event = threading.Event()

    pub_thread  = threading.Thread(
        target=publisher.run,
        args=(stop_event,),
        daemon=True, name="StatePublisher"
    )
    ctrl_thread = threading.Thread(
        target=control_loop,
        args=(pairs, publisher, stop_event),
        daemon=True, name="ControlLoop"
    )

    pub_thread.start()
    ctrl_thread.start()
    print("\n[Main] 运行中，Ctrl+C 退出\n")

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n\n[Main] 退出信号...")
    finally:
        stop_event.set()
        pub_thread.join(timeout=2.0)
        ctrl_thread.join(timeout=2.0)
        publisher.unadvertise_all()
        ros.terminate()
        print("[Main] 完成")


if __name__ == "__main__":
    main()