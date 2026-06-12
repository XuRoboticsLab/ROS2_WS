# ─────────────────────────────────────────────
#  ipc_bridge.py  —  ZeroMQ 进程间通信
#
#  替代 roslibpy，同机器两进程通信无需 ROS。
#  PUB/SUB 模式，与 ROS topic 语义一致（即发即忘，无缓冲）。
#
#  话题格式："{side}:{channel}"，如 "right:cmd"、"left:gripper"
#  emergency 无 side 前缀，全局生效。
# ─────────────────────────────────────────────

import json
import threading

try:
    import zmq
except ImportError:
    raise ImportError("pyzmq 未安装，请运行: pip install pyzmq")


class ZmqPublisher:
    """Pico 侧发布端：绑定 PUB socket，发布所有遥控指令。"""

    def __init__(self, address: str):
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.bind(address)
        print(f"[IPC] 发布端已绑定: {address}")

    def _send(self, topic: str, data):
        msg = topic.encode() + b" " + json.dumps(data).encode()
        self._sock.send(msg, zmq.NOBLOCK)

    def publish_cmd(self, side: str, twist: dict):
        self._send(f"{side}:cmd", twist)

    def publish_gripper(self, side: str, pos: float):
        self._send(f"{side}:gripper", {"data": float(pos)})

    def publish_reset(self, side: str, value: bool = True):
        self._send(f"{side}:reset", {"data": value})

    def publish_init(self, side: str):
        self._send(f"{side}:init", {"data": True})

    def publish_fine_cmd(self, side: str, x: float, y: float):
        self._send(f"{side}:fine_cmd", {
            "linear":  {"x": x, "y": y, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
        })

    def publish_emergency(self, active: bool):
        self._send("emergency", {"data": active})

    def close(self):
        self._sock.close()


class ZmqSubscriber:
    """Panthera 侧订阅端：连接 SUB socket，后台线程分发消息到回调。

    回调字典格式: {channel: callable}
    channel 为去掉 side 前缀后的名称，如 "cmd"、"gripper" 等。
    """

    def __init__(self, side: str, address: str, callbacks: dict):
        self._side = side
        self._callbacks = callbacks
        self._stop = threading.Event()

        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.SUB)
        self._sock.setsockopt_string(zmq.SUBSCRIBE, f"{side}:")
        self._sock.setsockopt_string(zmq.SUBSCRIBE, "emergency")
        # 接收超时 200 ms，让 stop_event 可以被检测到
        self._sock.setsockopt(zmq.RCVTIMEO, 200)
        self._sock.connect(address)
        print(f"[IPC] 订阅端已连接: {address}，监听 {side}:* + emergency")

        self._thread = threading.Thread(
            target=self._run, daemon=True, name="ZmqSubscriber"
        )
        self._thread.start()

    def _run(self):
        prefix = f"{self._side}:".encode()
        while not self._stop.is_set():
            try:
                raw = self._sock.recv()
            except zmq.Again:
                continue
            except zmq.ZMQError:
                break

            sep = raw.find(b" ")
            if sep == -1:
                continue
            topic_b = raw[:sep]
            try:
                data = json.loads(raw[sep + 1:])
            except Exception:
                continue

            if topic_b.startswith(prefix):
                channel = topic_b[len(prefix):].decode()
            elif topic_b == b"emergency":
                channel = "emergency"
            else:
                continue

            cb = self._callbacks.get(channel)
            if cb is not None:
                try:
                    cb(data)
                except Exception as e:
                    print(f"[IPC] 回调 {channel} 异常: {e}")

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        self._sock.close()
