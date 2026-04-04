#!/usr/bin/env python3
"""
ROS2 Humble Franka MuJoCo simulation node.

Subscribes:
  /joystick/cmd     (geometry_msgs/Twist)   translation + rotation deltas
  /joystick/gripper (std_msgs/Int8)         1=open, -1=close, 0=idle
  /joystick/reset   (std_msgs/Bool)         True = reset to home pose

Publishes:
  /franka/state      (robot_interfaces/SingleArmState7DOF)
  /franka/wrist_cam  (sensor_msgs/Image)  rgb8
  /franka/extern_cam (sensor_msgs/Image)  rgb8

Params (--ros-args -p key:=value):
  world_xml    : path to MuJoCo world.xml  (default: resolved from package assets)
  control_rate : Hz                        (default: 500.0)
  state_rate   : Hz                        (default: 60.0)
  camera_rate  : Hz                        (default: 30.0)
  viewer_rate  : Hz                        (default: 60.0)
  enable_viewer: show GLFW window          (default: True)
  trans_scale  : translation step          (default: 0.05)
  rot_scale    : rotation step             (default: 0.2)
  image_width  :                           (default: 640)
  image_height :                           (default: 480)

Note on OpenGL contexts:
  GLFW and offscreen rendering each need their own MjrContext bound to their
  own OpenGL context. The viewer thread owns the GLFW context + viewer_context,
  while the ROS camera timer uses the offscreen context. Both share MjModel and
  MjData (protected by sim_lock), but never share an MjrContext.
"""

import os
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from robot_interfaces.msg import SingleArmState7DOF

import numpy as np
import mujoco
import glfw
from scipy.spatial.transform import Rotation as R


QPOS0 = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
K     = [600.0, 600.0, 600.0, 30.0, 30.0, 30.0]


class FrankaNode(Node):

    def __init__(self):
        super().__init__('franka_node')

        self.declare_parameter('world_xml',     '')
        self.declare_parameter('control_rate',  500.0)
        self.declare_parameter('state_rate',    60.0)
        self.declare_parameter('camera_rate',   30.0)
        self.declare_parameter('viewer_rate',   15.0)
        self.declare_parameter('enable_viewer', True)
        self.declare_parameter('trans_scale',   0.05)
        self.declare_parameter('rot_scale',     0.2)
        self.declare_parameter('image_width',   640)
        self.declare_parameter('image_height',  480)

        world_xml          = self.get_parameter('world_xml').value
        control_rate       = self.get_parameter('control_rate').value
        state_rate         = self.get_parameter('state_rate').value
        camera_rate        = self.get_parameter('camera_rate').value
        self.viewer_rate   = self.get_parameter('viewer_rate').value
        self.enable_viewer = self.get_parameter('enable_viewer').value
        self.trans_scale   = self.get_parameter('trans_scale').value
        self.rot_scale     = self.get_parameter('rot_scale').value
        self.img_w         = self.get_parameter('image_width').value
        self.img_h         = self.get_parameter('image_height').value

        if not world_xml:
            pkg_share = get_package_share_directory('mujoco_single_franka')
            world_xml = os.path.join(pkg_share, 'assets', 'world.xml')

        self._init_mujoco(world_xml)

        self.latest_cmd     = np.zeros(6)
        self.latest_gripper = 0
        self.do_reset       = False
        self.gripper_open   = True
        self.sim_lock       = threading.Lock()
        self.bridge         = CvBridge()
        self._viewer_running = False

        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # GLFW viewer runs in its own thread — must own its OpenGL context
        if self.enable_viewer:
            self._viewer_thread = threading.Thread(
                target=self._viewer_loop, daemon=True
            )
            self._viewer_thread.start()

        self.create_subscription(Twist, '/joystick/cmd',     self._cb_cmd,     qos_sub)
        self.create_subscription(Int8,  '/joystick/gripper', self._cb_gripper, qos_sub)
        self.create_subscription(Bool,  '/joystick/reset',   self._cb_reset,   qos_sub)

        self.pub_state      = self.create_publisher(SingleArmState7DOF, '/franka/state',      qos_pub)
        self.pub_wrist_cam  = self.create_publisher(Image,              '/franka/wrist_cam',  qos_pub)
        self.pub_extern_cam = self.create_publisher(Image,              '/franka/extern_cam', qos_pub)

        self.create_timer(1.0 / control_rate, self._control_loop)
        self.create_timer(1.0 / state_rate,   self._state_loop)
        self.create_timer(1.0 / camera_rate,  self._camera_loop)

        

        self.get_logger().info(
            f'Franka node started | ctrl={control_rate:.0f} Hz | '
            f'state={state_rate:.0f} Hz | cam={camera_rate:.0f} Hz | '
            f'viewer={"on" if self.enable_viewer else "off"}'
        )

    # ── MuJoCo init ───────────────────────────────────────────────────────────

    def _init_mujoco(self, world_xml: str):
        print(world_xml)
        self.model  = mujoco.MjModel.from_xml_path(world_xml)
        self.data   = mujoco.MjData(self.model)
        self.scene  = mujoco.MjvScene(self.model, maxgeom=10000)

        # Offscreen context for camera publishing (used by ROS timer thread)
        # self.offscreen_context = mujoco.GLContext(self.img_w, self.img_h)
        # self.offscreen_context.make_current()
        # self.context = mujoco.MjrContext(
        #     self.model, mujoco.mjtFontScale.mjFONTSCALE_100
        # )

        self.wrist_cam  = self._make_fixed_cam('wrist_image_left')
        self.extern_cam = self._make_fixed_cam('extern')

        for i, q in enumerate(QPOS0):
            self.data.joint(f'panda_joint{i+1}').qpos = q
        self._set_gripper(True)
        mujoco.mj_forward(self.model, self.data)
        self.get_logger().info(f'MuJoCo model loaded: {world_xml}')

    def _make_fixed_cam(self, name: str) -> mujoco.MjvCamera:
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        cam.fixedcamid = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_CAMERA, name
        )
        return cam

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _set_gripper(self, open: bool):
        val = 0.04 if open else 0.0
        self.data.actuator('pos_panda_finger_joint1').ctrl = val
        self.data.actuator('pos_panda_finger_joint2').ctrl = val
        self.gripper_open = open

    def _pd_control(self, xpos_d: np.ndarray, xquat_d: np.ndarray):
        xpos  = self.data.body('panda_hand').xpos
        xquat = self.data.body('panda_hand').xquat
        jacp  = np.zeros((3, self.model.nv))
        jacr  = np.zeros((3, self.model.nv))
        body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, 'panda_hand'
        )
        mujoco.mj_jacBody(self.model, self.data, jacp, jacr, body_id)

        error = np.zeros(6)
        error[:3] = xpos_d - xpos
        res = np.zeros(3)
        mujoco.mju_subQuat(res, xquat, xquat_d)
        mujoco.mju_rotVecQuat(res, res, xquat)
        error[3:] = -res

        J = np.concatenate((jacp, jacr))
        v = J @ self.data.qvel
        for i in range(1, 8):
            dof  = self.model.joint(f'panda_joint{i}').dofadr
            bias = self.data.joint(f'panda_joint{i}').qfrc_bias
            ctrl = bias
            ctrl += J[:, dof].T @ np.diag(K) @ error
            ctrl -= J[:, dof].T @ np.diag(2 * np.sqrt(K)) @ v
            self.data.actuator(f'panda_joint{i}').ctrl = ctrl

    def _render_cam_offscreen(self, cam: mujoco.MjvCamera) -> np.ndarray:
        """Render a camera to offscreen buffer. Call with sim_lock held."""
        opt      = mujoco.MjvOption()
        pert     = mujoco.MjvPerturb()
        viewport = mujoco.MjrRect(0, 0, self.img_w, self.img_h)
        mujoco.mjv_updateScene(
            self.model, self.data, opt, pert, cam,
            mujoco.mjtCatBit.mjCAT_ALL, self.scene
        )
        mujoco.mjr_render(viewport, self.scene, self.context)
        rgb = np.zeros((self.img_h, self.img_w, 3), dtype=np.uint8)
        mujoco.mjr_readPixels(rgb, None, viewport, self.context)
        return np.flipud(rgb)

    # ── Subscribers ───────────────────────────────────────────────────────────

    def _cb_cmd(self, msg: Twist):
        self.latest_cmd = np.array([
            msg.linear.x,  msg.linear.y,  msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z,
        ])

    def _cb_gripper(self, msg: Int8):
        self.latest_gripper = msg.data

    def _cb_reset(self, msg: Bool):
        if msg.data:
            self.do_reset = True

    # ── Control loop (500 Hz) ─────────────────────────────────────────────────

    def _control_loop(self):
        with self.sim_lock:
            if self.do_reset:
                for i, q in enumerate(QPOS0):
                    self.data.joint(f'panda_joint{i+1}').qpos = q
                mujoco.mj_forward(self.model, self.data)
                self.do_reset = False
                self.get_logger().info('Reset to home pose')
                return

            if self.latest_gripper == 1:
                self._set_gripper(True)
            elif self.latest_gripper == -1:
                self._set_gripper(False)

            dx, dy, dz, droll, dpitch, dyaw = self.latest_cmd

            xpos_0  = self.data.body('panda_hand').xpos.copy()
            xquat_0 = self.data.body('panda_hand').xquat.copy()

            xpos_d = xpos_0 + np.array([dx, dy, dz]) * self.trans_scale

            dq    = R.from_euler('xyz', np.array([droll, dpitch, dyaw]) * self.rot_scale).as_quat()
            dq_mj = np.array([dq[3], dq[0], dq[1], dq[2]])
            xquat_d = np.zeros(4)
            mujoco.mju_mulQuat(xquat_d, dq_mj, xquat_0)

            self._pd_control(xpos_d, xquat_d)
            mujoco.mj_step(self.model, self.data)

    # ── State publish loop (60 Hz) ────────────────────────────────────────────

    def _state_loop(self):
        with self.sim_lock:
            q        = np.array([self.data.joint(f'panda_joint{i}').qpos[0] for i in range(1, 8)])
            xpos     = self.data.body('panda_hand').xpos.copy()
            xquat_mj = self.data.body('panda_hand').xquat.copy()

        xquat = np.array([xquat_mj[1], xquat_mj[2], xquat_mj[3], xquat_mj[0]])

        msg = SingleArmState7DOF()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.joint_position  = q.tolist()
        msg.eef_position    = xpos.tolist()
        msg.eef_orientation = xquat.tolist()
        msg.gripper_open    = self.gripper_open
        self.pub_state.publish(msg)

    # ── Camera publish loop (30 Hz) ───────────────────────────────────────────

    def _camera_loop(self):
        with self.sim_lock:
            wrist_rgb  = self._render_cam_offscreen(self.wrist_cam)
            extern_rgb = self._render_cam_offscreen(self.extern_cam)

        stamp = self.get_clock().now().to_msg()

        wrist_msg = self.bridge.cv2_to_imgmsg(wrist_rgb, encoding='rgb8')
        wrist_msg.header.stamp = stamp
        self.pub_wrist_cam.publish(wrist_msg)

        extern_msg = self.bridge.cv2_to_imgmsg(extern_rgb, encoding='rgb8')
        extern_msg.header.stamp = stamp
        self.pub_extern_cam.publish(extern_msg)

    # ── GLFW viewer loop (separate thread) ───────────────────────────────────

    def _viewer_loop(self):
        """
        Runs entirely in its own thread. Creates its own GLFW window and
        MjrContext — never touches the offscreen context owned by the ROS timers.
        """
        glfw.init()
        glfw.window_hint(glfw.SAMPLES, 8)
        window = glfw.create_window(1280, 720, 'Franka Sim', None, None)
        glfw.make_context_current(window)

        # Viewer gets its own MjrContext bound to the GLFW OpenGL context
        self.context = mujoco.MjrContext(
            self.model, mujoco.mjtFontScale.mjFONTSCALE_100
        )
        
        opt  = mujoco.MjvOption()
        pert = mujoco.MjvPerturb()

        self._viewer_running = True
        interval = 1.0 / self.viewer_rate

        import time
        while not glfw.window_should_close(window) and self._viewer_running:
            t0 = time.time()

            w, h = glfw.get_framebuffer_size(window)
            viewport = mujoco.MjrRect(0, 0, w, h)

            with self.sim_lock:
                mujoco.mjv_updateScene(
                    self.model, self.data, opt, pert, self.extern_cam,
                    mujoco.mjtCatBit.mjCAT_ALL, self.scene
                )

            mujoco.mjr_render(viewport, self.scene, self.context)
            glfw.swap_buffers(window)
            glfw.poll_events()

            elapsed = time.time() - t0
            if elapsed < interval:
                time.sleep(interval - elapsed)

        glfw.destroy_window(window)
        glfw.terminate()

    def destroy_node(self):
        self._viewer_running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FrankaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

