"""
identify_pedals.py
------------------
Press each foot pedal to identify its path and phys.

Usage:
    sudo python3 identify_pedals.py
"""

import threading
import evdev
from evdev import InputDevice, categorize, ecodes


def find_km_devices() -> list[InputDevice]:
    devices = []
    for path in evdev.list_devices():
        try:
            d = InputDevice(path)
            if 'KM-key08' in d.name:
                devices.append(d)
        except Exception:
            pass
    return devices


def listen(dev: InputDevice):
    try:
        for event in dev.read_loop():
            if event.type != ecodes.EV_KEY:
                continue
            key_event = categorize(event)
            if key_event.keystate != evdev.events.KeyEvent.key_down:
                continue
            print(f"\n  Pressed!")
            print(f"  path : {dev.path}")
            print(f"  phys : {dev.phys}")
            print(f"  key  : {key_event.keycode}")
            print()
    except OSError:
        pass


def main():
    devices = find_km_devices()
    if not devices:
        print("No KM-key08 devices found. Try running with sudo.")
        return

    print(f"Found {len(devices)} KM-key08 device(s):\n")
    for d in devices:
        print(f"  {d.path}  phys={d.phys}")
    print("\nPress any pedal to identify it (Ctrl+C to quit)...\n")

    threads = []
    for dev in devices:
        t = threading.Thread(target=listen, args=(dev,), daemon=True)
        t.start()
        threads.append(t)

    try:
        for t in threads:
            t.join()
    except KeyboardInterrupt:
        print("\nDone.")


if __name__ == '__main__':
    main()
