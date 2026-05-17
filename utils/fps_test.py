"""
用 roslibpy 订阅一个 Image topic，每秒打印一次实际接收帧率。

用法：
  python ros_image_fps.py --topic /camera/color/image_raw
  python ros_image_fps.py --topic /camera/color/image_raw --host 192.168.1.10
"""

import time
import threading
import argparse
import base64

import numpy as np
import roslibpy

# ── 计数器 ──────────────────────────────────────────────────
_lock = threading.Lock()
_count = 0
_last_time = time.time()
global msg_list
msg_list = []

def callback(msg):
    global msg_list
    msg_list.append(msg)
    global _count
    with _lock:
        _count += 1


def stats_loop():
    global _count, _last_time
    while True:
        time.sleep(1.0)
        now = time.time()
        with _lock:
            fps = _count / (now - _last_time)
            cnt = _count
            _count = 0
            _last_time = now
        h = msg_size.get("h", "?")
        w = msg_size.get("w", "?")
        enc = msg_size.get("enc", "?")
        print(f"FPS: {fps:.1f}  |  frames in last second: {cnt}  |  {w}x{h}  enc={enc}")


# 用于记录最后一帧的元信息
msg_size = {}


def callback_with_info(msg):
    global _count
    msg_size["h"] = msg.get("height", "?")
    msg_size["w"] = msg.get("width", "?")
    msg_size["enc"] = msg.get("encoding", "?")
    with _lock:
        _count += 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/extern/color/image_raw")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=9090)
    args = parser.parse_args()

    print(f"连接 {args.host}:{args.port}，订阅 {args.topic} ...")

    client = roslibpy.Ros(host=args.host, port=args.port)
    t = threading.Thread(target=client.run_forever, daemon=True)
    t.start()

    deadline = time.time() + 10.0
    while not client.is_connected and time.time() < deadline:
        time.sleep(0.1)
    if not client.is_connected:
        print("无法连接到 ROS bridge")
        return

    print("已连接，开始统计帧率（Ctrl+C 退出）...\n")

    sub = roslibpy.Topic(client, args.topic, "sensor_msgs/Image")
    sub.subscribe(callback_with_info)

    stats = threading.Thread(target=stats_loop, daemon=True)
    stats.start()

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        sub.unsubscribe()
        client.terminate()


if __name__ == "__main__":
    main()
