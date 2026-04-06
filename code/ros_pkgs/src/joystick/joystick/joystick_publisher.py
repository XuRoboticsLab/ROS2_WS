#!/usr/bin/env python3
"""
ROS2 Humble joystick publisher.

Reads a gamepad via pygame and publishes control commands.

Deps:
  pip install pygame pyyaml

Topics published:
  /joystick/cmd  (geometry_msgs/Twist)  linear=translation, angular=rotation
  /joystick/gripper  (std_msgs/Int8)    1=open, -1=close, 0=idle
  /joystick/reset    (std_msgs/Bool)    True = reset triggered

Axis / button mapping:
  axis0  left stick X     → cmd.linear.x
  axis1  left stick Y     → cmd.linear.y
  axis2  LT               → cmd.linear.z (down)
  axis5  RT               → cmd.linear.z (up)
  axis3  right stick X    → cmd.angular.x (roll)
  axis4  right stick Y    → cmd.angular.y (pitch)
  b1     B                → cmd.angular.z (yaw -)
  b2     X                → cmd.angular.z (yaw +)
  b9     LB               → gripper close
  b10    RB               → gripper open
  b0     A                → reset

Config path resolution (first match wins):
  1. CLI flag:       --config /path/to/config.yaml
  2. ROS parameter:  --ros-args -p config_path:=/path/to/config.yaml

Config YAML example:
  device_id: 0
  publish_rate: 100.0
  trans_scale: 1
  rot_scale: 1
"""

import sys
import argparse
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int8

import pygame


class JoystickPublisher(Node):

    def __init__(self, cfg_data: dict):
        super().__init__('joystick_publisher')

        def cfg(key, default):
            return cfg_data.get(key, default)

        self.device_id   = int(cfg('device_id',    0))
        publish_rate     = float(cfg('publish_rate', 100.0))
        self.trans_scale = float(cfg('trans_scale',  1))
        self.rot_scale   = float(cfg('rot_scale',    1))

        topics = cfg('topics', {})
        topic_cmd     = topics.get('cmd',     '/joystick/cmd')
        topic_gripper = topics.get('gripper', '/joystick/gripper')
        topic_reset   = topics.get('reset',   '/joystick/reset')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub_cmd     = self.create_publisher(Twist, topic_cmd,     qos)
        self.pub_gripper = self.create_publisher(Int8,  topic_gripper, qos)
        self.pub_reset   = self.create_publisher(Bool,  topic_reset,   qos)

        self._init_pygame()

        self.create_timer(1.0 / publish_rate, self.timer_callback)
        self.get_logger().info(
            f'Started | device={self.device_id} | '
            f'trans_scale={self.trans_scale} | rot_scale={self.rot_scale} | '
            f'{publish_rate:.0f} Hz'
        )

    # ── Init ─────────────────────────────────────────────────────────────────

    def _init_pygame(self):
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            raise RuntimeError('No joystick detected')

        self.joystick = pygame.joystick.Joystick(self.device_id)
        self.joystick.init()
        self.get_logger().info('Joystick opened')

    # ── Timer callback ────────────────────────────────────────────────────────

    def timer_callback(self):
        pygame.event.pump()

        # Axes
        axis0 = self.joystick.get_axis(0)   # left stick X
        axis1 = self.joystick.get_axis(1)   # left stick Y
        axis2 = self.joystick.get_axis(2)   # LT
        axis3 = self.joystick.get_axis(3)   # right stick X
        axis4 = self.joystick.get_axis(4)   # right stick Y
        axis5 = self.joystick.get_axis(5)   # RT

        # Buttons
        b0  = self.joystick.get_button(0)   # A - reset
        b1  = self.joystick.get_button(1)   # B - yaw -
        b2  = self.joystick.get_button(2)   # X - yaw +
        b9  = self.joystick.get_button(9)   # LB - gripper close
        b10 = self.joystick.get_button(10)  # RB - gripper open

        # Translation
        dx = axis0 * self.trans_scale
        dy = -axis1 * self.trans_scale
        dz = ((axis5 + 1) / 2 - (axis2 + 1) / 2) * self.trans_scale  # RT - LT

        # Rotation
        droll  = axis3 * self.rot_scale
        dpitch = axis4 * self.rot_scale
        if b1:
            dyaw = float(-self.rot_scale)
        elif b2:
            dyaw = float(self.rot_scale)
        else:
            dyaw = 0.0

        # Gripper
        if b10:
            gripper = 1
        elif b9:
            gripper = -1
        else:
            gripper = 0

        # Publish cmd
        cmd = Twist()
        cmd.linear.x  = dx
        cmd.linear.y  = dy
        cmd.linear.z  = dz
        cmd.angular.x = droll
        cmd.angular.y = dpitch
        cmd.angular.z = dyaw
        self.pub_cmd.publish(cmd)

        # Publish gripper
        gripper_msg = Int8()
        gripper_msg.data = gripper
        self.pub_gripper.publish(gripper_msg)

        # Publish reset
        reset_msg = Bool()
        reset_msg.data = bool(b0)
        self.pub_reset.publish(reset_msg)

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        pygame.quit()
        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description='ROS2 Joystick Publisher')
    parser.add_argument(
        '--config', '-c',
        default=None,
        help='Path to config.yaml. '
             'Falls back to the ROS parameter "config_path" if not given.',
    )
    # parse only our own args, leave the rest for rclpy
    known, remaining = parser.parse_known_args(args=args)

    rclpy.init(args=remaining)

    # ── Resolve config path (CLI flag → ROS param → error) ────────────────
    if known.config:
        config_path = known.config
    else:
        tmp = rclpy.create_node('_config_reader')
        tmp.declare_parameter('config_path', '')
        config_path = tmp.get_parameter('config_path').get_parameter_value().string_value
        tmp.destroy_node()

    if not config_path:
        print(
            'ERROR: No config path provided. '
            'Use --config /path/to/config.yaml or '
            "set the 'config_path' ROS parameter via --ros-args -p config_path:=...",
            file=sys.stderr,
        )
        rclpy.shutdown()
        sys.exit(1)

    # ── Load YAML inline ──────────────────────────────────────────────────
    try:
        with open(config_path, 'r') as f:
            cfg_data = yaml.safe_load(f) or {}
    except Exception as e:
        print(f"ERROR loading config '{config_path}': {e}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    node = JoystickPublisher(cfg_data)
    node.get_logger().info(f'Loaded config from: {config_path}')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()