#!/usr/bin/env python3
"""
ROS2 Humble joystick publisher.

Reads a gamepad via pygame and publishes control commands.

Deps:
  pip install pygame

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

Params (--ros-args -p key:=value):
  device_id    : pygame joystick index  (default: 0)
  publish_rate : Hz                     (default: 50.0)
  trans_scale  : translation step size  (default: 0.02)
  rot_scale    : rotation step size     (default: 0.1)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int8

import pygame


class JoystickPublisher(Node):

    def __init__(self):
        super().__init__('joystick_publisher')

        # Declare and load parameters
        self.declare_parameter('device_id',    0)
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('trans_scale',  1)
        self.declare_parameter('rot_scale',    1)

        self.device_id   = self.get_parameter('device_id').value
        publish_rate     = self.get_parameter('publish_rate').value
        self.trans_scale = self.get_parameter('trans_scale').value
        self.rot_scale   = self.get_parameter('rot_scale').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub_cmd     = self.create_publisher(Twist, '/joystick/cmd',     qos)
        self.pub_gripper = self.create_publisher(Int8,  '/joystick/gripper', qos)
        self.pub_reset   = self.create_publisher(Bool,  '/joystick/reset',   qos)

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
            dyaw = -self.rot_scale
        elif b2:
            dyaw = self.rot_scale
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
    rclpy.init(args=args)
    node = JoystickPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()