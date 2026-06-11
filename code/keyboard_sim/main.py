#!/usr/bin/env python3
"""
main.py — 键盘模拟 Pico 控制器
用于在没有 Pico VR 头显时测试 panthera pico_control 代码。

运行: python main.py [--config /path/to/pico.yaml]
     python main.py [--host localhost --port 9090]
"""

import argparse
import curses
import sys
import threading
import time

import roslibpy
import yaml

# ── 运动步长 ─────────────────────────────────
STEP_LINEAR  = 0.005   # 每帧位移增量 (m)
STEP_ANGULAR = 0.01    # 每帧旋转增量 (rad)
PUBLISH_HZ   = 50

# ── 默认 Topics（与 pico.yaml 一致）─────────
DEFAULT_TOPICS = {
    "right_cmd":     "/joystick/right/cmd",
    "left_cmd":      "/joystick/left/cmd",
    "right_gripper": "/joystick/right/gripper",
    "left_gripper":  "/joystick/left/gripper",
    "right_reset":   "/joystick/right/reset",
    "left_reset":    "/joystick/left/reset",
    "right_init":    "/joystick/right/init",
    "left_init":     "/joystick/left/init",
    "emergency":     "/joystick/emergency",
}

MOTION_KEYS = set("wsadqeikjluo")


class ArmState:
    def __init__(self):
        self.dx = self.dy = self.dz = 0.0
        self.rx = self.ry = self.rz = 0.0
        self.gripper_open = True

    def zero(self):
        self.dx = self.dy = self.dz = 0.0
        self.rx = self.ry = self.rz = 0.0

    def apply_key(self, key: str):
        if   key == "w": self.dx += STEP_LINEAR
        elif key == "s": self.dx -= STEP_LINEAR
        elif key == "a": self.dy += STEP_LINEAR
        elif key == "d": self.dy -= STEP_LINEAR
        elif key == "q": self.dz += STEP_LINEAR
        elif key == "e": self.dz -= STEP_LINEAR
        elif key == "i": self.rx += STEP_ANGULAR
        elif key == "k": self.rx -= STEP_ANGULAR
        elif key == "j": self.ry += STEP_ANGULAR
        elif key == "l": self.ry -= STEP_ANGULAR
        elif key == "u": self.rz += STEP_ANGULAR
        elif key == "o": self.rz -= STEP_ANGULAR

    def twist(self) -> dict:
        return {
            "linear":  {"x": self.dx, "y": self.dy, "z": self.dz},
            "angular": {"x": self.rx, "y": self.ry, "z": self.rz},
        }

    def gripper_val(self) -> float:
        return 0.0 if self.gripper_open else 2.0


class Simulator:
    def __init__(self, host: str, port: int, topics: dict):
        self.arms = {"right": ArmState(), "left": ArmState()}
        self.active = "right"
        self.emergency = False
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.log: list[str] = []

        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run()
        if not self.ros.is_connected:
            raise RuntimeError(f"无法连接到 rosbridge {host}:{port}")

        self._pubs = {}
        type_map = {
            "cmd": "geometry_msgs/Twist",
            "gripper": "std_msgs/Float32",
            "reset": "std_msgs/Bool",
            "init": "std_msgs/Bool",
            "emergency": "std_msgs/Bool",
        }
        for key, topic in topics.items():
            suffix = key.split("_", 1)[-1] if "_" in key else key
            msg_type = type_map.get(suffix, "std_msgs/Bool")
            self._pubs[key] = roslibpy.Topic(self.ros, topic, msg_type)

    def _pub(self, key: str, data: dict):
        p = self._pubs.get(key)
        if p and self.ros.is_connected:
            p.publish(roslibpy.Message(data))

    def publish_loop(self):
        interval = 1.0 / PUBLISH_HZ
        while not self._stop.is_set():
            t0 = time.time()
            if not self.emergency:
                with self._lock:
                    arm = self.active
                    state = self.arms[arm]
                    self._pub(f"{arm}_cmd",     state.twist())
                    self._pub(f"{arm}_gripper", {"data": state.gripper_val()})
            time.sleep(max(0.0, interval - (time.time() - t0)))

    def _log(self, msg: str):
        self.log = (self.log + [msg])[-5:]

    def handle_key(self, key: str) -> bool:
        """Return True to quit."""
        with self._lock:
            arm = self.active
            state = self.arms[arm]

            if key in MOTION_KEYS:
                state.apply_key(key)

            elif key == "\t":
                self.active = "left" if arm == "right" else "right"
                self._log(f"切换到 {self.active.upper()} 臂")

            elif key == " ":
                state.zero()
                self._pub(f"{arm}_init", {"data": True})
                self._log(f"[{arm}] Init 已发送，偏移归零")

            elif key == "r":
                state.zero()
                self._pub(f"{arm}_reset", {"data": True})
                self._log(f"[{arm}] Reset 已发送")

            elif key == "g":
                state.gripper_open = not state.gripper_open
                self._log(f"[{arm}] 夹爪: {'张开' if state.gripper_open else '闭合'}")

            elif key == "z":
                state.zero()
                self._log(f"[{arm}] 偏移归零")

            elif key == "\x1b":  # ESC
                self.emergency = not self.emergency
                self._pub("emergency", {"data": self.emergency})
                self._log(f"急停: {'开启' if self.emergency else '关闭'}")

            elif key == "x":
                return True

        return False

    def shutdown(self):
        self._stop.set()
        self._pub("emergency", {"data": True})
        self.ros.terminate()


# ─────────────────────────────────────────────
#  curses UI
# ─────────────────────────────────────────────

def run_ui(stdscr, sim: Simulator):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(20)   # ~50 Hz poll

    while True:
        ch = stdscr.getch()
        if ch != -1:
            try:
                key = chr(ch).lower()
            except (ValueError, OverflowError):
                key = ""
            if key and sim.handle_key(key):
                break

        # ── 绘制界面 ─────────────────────────
        stdscr.erase()
        h, w = stdscr.getmaxyx()

        with sim._lock:
            arm = sim.active
            st  = sim.arms[arm]
            emg = sim.emergency
            log = list(sim.log)

        rows = [
            "=" * 56,
            "  键盘 Pico 模拟器",
            "=" * 56,
            f"  当前臂: {arm.upper():<6}  |  {'!! 急停 !!' if emg else '正常运行'}",
            f"  夹爪:   {'张开' if st.gripper_open else '闭合'}",
            f"  linear : x={st.dx:+.4f}  y={st.dy:+.4f}  z={st.dz:+.4f}",
            f"  angular: x={st.rx:+.4f}  y={st.ry:+.4f}  z={st.rz:+.4f}",
            "-" * 56,
            "  平移:  W/S=x  A/D=y  Q/E=z",
            "  旋转:  I/K=rx J/L=ry U/O=rz",
            "  Tab: 切换左右臂  G: 夹爪开关  Z: 偏移归零",
            "  Space: Init(校准)  R: Reset(回零)  Esc: 急停",
            "  X: 退出",
            "-" * 56,
            "  日志:",
        ] + [f"    {l}" for l in log]

        for i, line in enumerate(rows):
            if i < h - 1:
                stdscr.addstr(i, 0, line[: w - 1])

        stdscr.refresh()


# ─────────────────────────────────────────────
#  入口
# ─────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="键盘模拟 Pico 控制器")
    parser.add_argument("--config", "-c", metavar="PATH",
                        help="pico.yaml 配置文件（可选）")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=9090)
    args = parser.parse_args()

    host, port = args.host, args.port
    topics = DEFAULT_TOPICS.copy()

    if args.config:
        with open(args.config) as f:
            cfg = yaml.safe_load(f)
        host = cfg.get("rosbridge", {}).get("host", host)
        port = cfg.get("rosbridge", {}).get("port", port)
        for k, v in cfg.get("topics", {}).get("publish", {}).items():
            if k in topics:
                topics[k] = v

    print(f"连接到 rosbridge {host}:{port} ...")
    try:
        sim = Simulator(host, port, topics)
    except RuntimeError as e:
        print(e)
        sys.exit(1)
    print("已连接。启动 UI...\n")

    pub_thread = threading.Thread(target=sim.publish_loop, daemon=True)
    pub_thread.start()

    try:
        curses.wrapper(lambda scr: run_ui(scr, sim))
    finally:
        sim.shutdown()
        print("已断开连接，退出。")


if __name__ == "__main__":
    main()
