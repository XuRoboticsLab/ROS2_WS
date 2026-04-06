#!/usr/bin/env python3
"""
ROS2 Humble camera publisher — RGB + depth via Intel RealSense.
Supports multiple cameras: run one node per camera, distinguished by serial_number.

Deps:
  pip install pyrealsense2 pyyaml
  sudo apt install ros-humble-cv-bridge ros-humble-image-transport

Topics published (under /<camera_name>/):
  color/image_raw   (sensor_msgs/Image)
  color/camera_info (sensor_msgs/CameraInfo)
  depth/image_raw   (sensor_msgs/Image)      16UC1, mm
  depth/camera_info (sensor_msgs/CameraInfo)

Config path resolution (first match wins):
  1. CLI flag:        --config /path/to/config.yaml
  2. ROS parameter:   --ros-args -p config_path:=/path/to/config.yaml

Config YAML example:
  serial_number: '136622073828'
  camera_name: camera
  enable_depth: false
  publish_rate: 30.0
  image_width: 640
  image_height: 480

Multiple cameras example (see multi_camera.launch.py):
  ros2 run <pkg> camera --config /path/to/cam0.yaml
  ros2 run <pkg> camera --config /path/to/cam1.yaml
"""

import sys
import argparse
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import pyrealsense2 as rs


def rs_timestamp_to_ros_time(ts_ms: float) -> Time:
    """
    Convert a RealSense timestamp (milliseconds, float) to a ROS Time message.

    RealSense get_timestamp() returns something like: 1775452963818.0874 (ms)
    ROS Time has: sec (int32) + nanosec (uint32)
    """
    ts_ns = int(ts_ms * 1_000_000)   # ms -> ns
    sec    = ts_ns // 1_000_000_000
    nanosec = ts_ns %  1_000_000_000
    return Time(sec=sec, nanosec=nanosec)


class CameraPublisher(Node):

    def __init__(self, cfg_data: dict):
        super().__init__('camera')

        def cfg(key, default):
            return cfg_data.get(key, default)

        self.serial       = str(cfg('serial_number', '136622073828'))
        self.camera_name  = str(cfg('camera_name',   'camera'))
        self.enable_depth = bool(cfg('enable_depth',  False))
        publish_rate      = float(cfg('publish_rate',  30.0))
        self.img_w        = int(cfg('image_width',   640))
        self.img_h        = int(cfg('image_height',  480))

        # ── ROS setup ──────────────────────────────────────────────────────
        self.bridge   = CvBridge()
        self.frame_id = f'{self.camera_name}_link'

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        ns = self.camera_name
        self.pub_rgb        = self.create_publisher(Image,      f'/{ns}/color/image_raw',   qos)
        self.pub_rgb_info   = self.create_publisher(CameraInfo, f'/{ns}/color/camera_info', qos)
        if self.enable_depth:
            self.pub_depth      = self.create_publisher(Image,      f'/{ns}/depth/image_raw',   qos)
            self.pub_depth_info = self.create_publisher(CameraInfo, f'/{ns}/depth/camera_info', qos)

        self._init_realsense()

        self.create_timer(1.0 / publish_rate, self.timer_callback)
        self.get_logger().info(
            f'Started | serial={self.serial or "auto"} | ns=/{ns} | '
            f'depth={self.enable_depth} | {self.img_w}x{self.img_h} @ {publish_rate:.1f} Hz'
        )

    # ── Camera init ──────────────────────────────────────────────────────────

    def _init_realsense(self):
        self.rs_pipeline = rs.pipeline()
        cfg = rs.config()

        if self.serial:
            cfg.enable_device(self.serial)

        cfg.enable_stream(rs.stream.color, self.img_w, self.img_h, rs.format.bgr8, 30)
        if self.enable_depth:
            cfg.enable_stream(rs.stream.depth, self.img_w, self.img_h, rs.format.z16, 30)
        profile = self.rs_pipeline.start(cfg)

        actual_serial = profile.get_device().get_info(rs.camera_info.serial_number)
        self.get_logger().info(f'RealSense opened | serial={actual_serial}')

        self._color_intr = (
            profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        )
        if self.enable_depth:
            self._depth_intr = (
                profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
            )
            self.rs_align = rs.align(rs.stream.color)

    # ── Timer callback ────────────────────────────────────────────────────────

    def timer_callback(self):
        try:
            frames = self.rs_pipeline.wait_for_frames()
        except Exception as e:
            self.get_logger().warn(f'Frame timeout: {e}')
            return

        if self.enable_depth:
            frames = self.rs_align.process(frames)

        color_frame = frames.get_color_frame()
        if not color_frame:
            self.get_logger().warn('No color frame, skipping.')
            return

        # Use RealSense hardware timestamp (ms float) instead of ROS clock
        stamp = rs_timestamp_to_ros_time(frames.get_timestamp())

        color_image = np.asanyarray(color_frame.get_data())   # (H,W,3) uint8
        header = Header(stamp=stamp, frame_id=self.frame_id)

        rgb_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        rgb_msg.header = header
        self.pub_rgb.publish(rgb_msg)
        self.pub_rgb_info.publish(self._to_camera_info(header, self._color_intr))

        if self.enable_depth:
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                self.get_logger().warn('No depth frame, skipping.')
                return
            depth_image = np.asanyarray(depth_frame.get_data())   # (H,W) uint16, mm
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')
            depth_msg.header = header
            self.pub_depth.publish(depth_msg)
            self.pub_depth_info.publish(self._to_camera_info(header, self._depth_intr))

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _to_camera_info(self, header: Header, intr) -> CameraInfo:
        """Convert a RealSense intrinsics object to CameraInfo."""
        fx, fy = intr.fx, intr.fy
        cx, cy = intr.ppx, intr.ppy
        msg = CameraInfo(header=header, width=intr.width, height=intr.height)
        msg.distortion_model = 'plumb_bob'
        msg.d = list(intr.coeffs)
        msg.k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
        msg.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        msg.p = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]
        return msg

    def destroy_node(self):
        self.rs_pipeline.stop()
        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description='ROS2 RealSense Camera Publisher')
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

    try:
        with open(config_path, 'r') as f:
            cfg_data = yaml.safe_load(f) or {}
    except Exception as e:
        print(f"ERROR loading config '{config_path}': {e}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    node = CameraPublisher(cfg_data)
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