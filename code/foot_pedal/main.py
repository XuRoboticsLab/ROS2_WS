"""
foot_pedal/main.py
------------------
Standalone script (run with sudo) that listens to USB foot pedals via evdev and
publishes std_msgs/Empty on configured topics through a rosbridge websocket.

Requirements:
    pip install evdev roslibpy pyyaml

Usage:
    sudo python3 main.py --config configs/config.yaml

Prerequisites:
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml
"""

import argparse
import sys
import time
import threading

import yaml
import evdev
from evdev import InputDevice, categorize, ecodes
import roslibpy


def find_devices(specs: list[tuple[str, str | None]]) -> dict[str, InputDevice]:
    """
    Find devices matching the given specs.

    specs: list of (name_substring, phys_or_None)
    Returns: {device_key: InputDevice}
        device_key = phys if provided in spec, else name_substring
    """
    # Build {device_key: (name, phys_or_None)} for all unique specs
    needed: dict[str, tuple[str, str | None]] = {}
    for name, phys in specs:
        key = phys if phys else name
        needed[key] = (name, phys)

    matched: dict[str, InputDevice] = {}
    all_found = []
    for path in evdev.list_devices():
        try:
            dev = InputDevice(path)
        except PermissionError:
            print(f"WARNING: Permission denied on {path} — try running with sudo", flush=True)
            continue
        except Exception:
            continue
        all_found.append(f"{path}: {dev.name}  phys={dev.phys}")
        for dev_key, (name, phys) in needed.items():
            if dev_key in matched:
                continue
            if name not in dev.name:
                continue
            if phys is not None and dev.phys != phys:
                continue
            matched[dev_key] = dev
            break

    if len(matched) != len(needed):
        missing = [k for k in needed if k not in matched]
        print(f"  Not found: {missing}", flush=True)
        print(f"  Available devices:", flush=True)
        for d in all_found:
            print(f"    {d}", flush=True)
    return matched


def listen_device(dev: InputDevice, key_topic_map: dict[int, roslibpy.Topic]) -> None:
    """Block-read events from dev and publish on key release."""
    print(f"Listening on [{dev.name}] @ {dev.path}", flush=True)
    try:
        for event in dev.read_loop():
            if event.type != ecodes.EV_KEY:
                continue
            key_event = categorize(event)
            # value 0 = release, 1 = press, 2 = repeat
            if key_event.keystate != evdev.events.KeyEvent.key_up:
                continue
            topic = key_topic_map.get(key_event.scancode)
            if topic is not None:
                topic.publish(roslibpy.Message({}))
                print(
                    f"\r[{dev.name}] key {key_event.keycode} → {topic.name}",
                    end="",
                    flush=True,
                )
    except OSError as exc:
        print(f"[{dev.name}] device disconnected: {exc}", flush=True)


def main() -> None:
    # --- Load config ---
    parser = argparse.ArgumentParser(description="Foot pedal ROS publisher")
    parser.add_argument("--config", "-c", required=True, metavar="PATH", help="config.yaml 路径")
    args = parser.parse_args()

    with open(args.config, "r") as f:
        cfg = yaml.safe_load(f)

    rosbridge_host = cfg["rosbridge"]["host"]
    rosbridge_port = cfg["rosbridge"]["port"]

    # (device_key, evdev_key_code) → topic
    # device_key = phys if specified in config, else device name substring
    pedal_config: dict[tuple[str, int], str] = {
        (p.get("phys") or p["device"], ecodes.ecodes[p["key"]]): p["topic"]
        for p in cfg["pedals"]
    }

    # --- Connect to rosbridge ---
    client = roslibpy.Ros(host=rosbridge_host, port=rosbridge_port)
    client.run()
    for _ in range(20):
        if client.is_connected:
            break
        time.sleep(0.5)
    else:
        print(
            f"ERROR: Could not connect to rosbridge at "
            f"ws://{rosbridge_host}:{rosbridge_port}",
            flush=True,
        )
        print("  ros2 launch rosbridge_server rosbridge_websocket_launch.xml", flush=True)
        sys.exit(1)

    print(f"Connected to rosbridge ws://{rosbridge_host}:{rosbridge_port}", flush=True)

    # --- Advertise all unique topics ---
    unique_topics: dict[str, roslibpy.Topic] = {}
    for topic_name in pedal_config.values():
        if topic_name not in unique_topics:
            t = roslibpy.Topic(client, topic_name, "std_msgs/Empty")
            t.advertise()
            unique_topics[topic_name] = t
            print(f"Advertising {topic_name}", flush=True)

    # --- Find devices ---
    # Build specs: (name, phys_or_None) for each unique device_key
    dev_key_to_spec: dict[str, tuple[str, str | None]] = {
        (p.get("phys") or p["device"]): (p["device"], p.get("phys"))
        for p in cfg["pedals"]
    }
    specs = list(dev_key_to_spec.values())
    devices = find_devices(specs)

    if not devices:
        print("ERROR: No matching pedal devices found. Is the pedal plugged in?", flush=True)
        for t in unique_topics.values():
            t.unadvertise()
        client.terminate()
        sys.exit(1)

    for dev_key in dev_key_to_spec:
        if dev_key not in devices:
            print(f"WARNING: Device '{dev_key}' not found, skipping.", flush=True)

    # --- Build per-device key → topic maps ---
    device_key_maps: dict[str, dict[int, roslibpy.Topic]] = {}
    for (dev_name, key_code), topic_name in pedal_config.items():
        if dev_name not in devices:
            continue
        device_key_maps.setdefault(dev_name, {})[key_code] = unique_topics[topic_name]

    # --- Grab devices so key events don't leak to the OS ---
    for dev in devices.values():
        try:
            dev.grab()
        except Exception as exc:
            print(f"WARNING: Could not grab {dev.name}: {exc}", flush=True)

    print("Foot pedals ready.", flush=True)

    # --- Spawn one listener thread per device ---
    for dev_name, dev in devices.items():
        threading.Thread(
            target=listen_device,
            args=(dev, device_key_maps[dev_name]),
            daemon=True,
        ).start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        for dev in devices.values():
            try:
                dev.ungrab()
                dev.close()
            except Exception:
                pass
        for t in unique_topics.values():
            t.unadvertise()
        client.terminate()
        print("Foot pedal publisher stopped.", flush=True)


if __name__ == "__main__":
    main()
