"""
foot_pedal/main.py
------------------
Standalone script (run with sudo) that listens to a USB foot pedal and
publishes std_msgs/Empty on /foot_pedal/press through a rosbridge websocket.

Requirements:
    pip install keyboard roslibpy

Usage:
    sudo python3 main.py

Prerequisites:
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml
"""

import sys
import time

import keyboard
import roslibpy


def main():
    # Connect to rosbridge
    client = roslibpy.Ros(host='localhost', port=9090)
    client.run()
    for _ in range(20):
        if client.is_connected:
            break
        time.sleep(0.5)
    else:
        print("ERROR: Could not connect to rosbridge at ws://localhost:9090", flush=True)
        print("  ros2 launch rosbridge_server rosbridge_websocket_launch.xml", flush=True)
        sys.exit(1)

    print("Connected to rosbridge. Publishing to /foot_pedal/press", flush=True)

    publisher = roslibpy.Topic(client, '/foot_pedal/press', 'std_msgs/Empty')
    publisher.advertise()

    print("Foot pedal ready. Press the pedal to publish.", flush=True)

    def on_press(_event):
        publisher.publish(roslibpy.Message({}))
        print("Pedal pressed → published.", flush=True)

    keyboard.on_release_key('space', on_press)

    try:
        keyboard.wait()   # blocks forever until KeyboardInterrupt
    except KeyboardInterrupt:
        pass
    finally:
        keyboard.unhook_all()
        publisher.unadvertise()
        client.terminate()
        print("Foot pedal publisher stopped.", flush=True)


if __name__ == '__main__':
    main()
