#!/usr/bin/env python3
"""
ROS2 Humble camera publisher — RGB + depth via Intel RealSense.
Supports multiple cameras: run one node per camera, distinguished by serial_number.

Deps:
  pip install pyrealsense2
  sudo apt install ros-humble-cv-bridge ros-humble-image-transport

Topics published (under /<camera_name>/):
  color/image_raw   (sensor_msgs/Image)
  color/camera_info (sensor_msgs/CameraInfo)
  depth/image_raw   (sensor_msgs/Image)      16UC1, mm
  depth/camera_info (sensor_msgs/CameraInfo)

Params (--ros-args -p key:=value):
  serial_number : device serial number, '' = first available (default: '')
  camera_name   : topic namespace and TF prefix  (default: camera)
  enable_depth  : publish depth stream           (default: False)
  publish_rate  : Hz                             (default: 30.0)
  image_width   :                                (default: 640)
  image_height  :                                (default: 480)

Multiple cameras example (see multi_camera.launch.py):
  ros2 run <pkg> camera --ros-args -p serial_number:=123456 -p camera_name:=cam0
  ros2 run <pkg> camera --ros-args -p serial_number:=789012 -p camera_name:=cam1
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import pyrealsense2 as rs


class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera')

        # Declare and load parameters
        self.declare_parameter('serial_number', '136622073828')
        self.declare_parameter('camera_name',   'camera')
        self.declare_parameter('enable_depth',  False)
        self.declare_parameter('publish_rate',  30.0)
        self.declare_parameter('image_width',   640)
        self.declare_parameter('image_height',  480)

        self.serial       = self.get_parameter('serial_number').value
        self.camera_name  = self.get_parameter('camera_name').value
        self.enable_depth = self.get_parameter('enable_depth').value
        publish_rate      = self.get_parameter('publish_rate').value
        self.img_w        = self.get_parameter('image_width').value
        self.img_h        = self.get_parameter('image_height').value

        self.bridge  = CvBridge()
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

        # Pin to a specific device when serial_number is provided
        if self.serial:
            cfg.enable_device(self.serial)

        cfg.enable_stream(rs.stream.color, self.img_w, self.img_h, rs.format.bgr8, 30)
        if self.enable_depth:
            cfg.enable_stream(rs.stream.depth, self.img_w, self.img_h, rs.format.z16, 30)
        profile = self.rs_pipeline.start(cfg)

        # Read actual serial from the active device (useful when serial='' / auto)
        actual_serial = profile.get_device().get_info(rs.camera_info.serial_number)
        self.get_logger().info(f'RealSense opened | serial={actual_serial}')

        # Cache intrinsics from device profile
        self._color_intr = (
            profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        )
        if self.enable_depth:
            self._depth_intr = (
                profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
            )
            # Align depth to color frame
            self.rs_align = rs.align(rs.stream.color)

    # ── Timer callback ────────────────────────────────────────────────────────

    def timer_callback(self):
        stamp = self.get_clock().now().to_msg()

        try:
            frames = self.rs_pipeline.wait_for_frames()
        except Exception as e:
            self.get_logger().warn(f'Frame timeout: {e}')
            return

        # Align and extract color
        if self.enable_depth:
            frames = self.rs_align.process(frames)

        color_frame = frames.get_color_frame()
        if not color_frame:
            self.get_logger().warn('No color frame, skipping.')
            return

        color_image = np.asanyarray(color_frame.get_data())  # (H,W,3) uint8
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
            depth_image = np.asanyarray(depth_frame.get_data())  # (H,W) uint16, mm
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
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()