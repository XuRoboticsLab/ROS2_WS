"""
data_collector_node.py
----------------------
ROS2 node that:
  1. Reads config.yaml (topics, recording frequency, output dir).
  2. Dynamically subscribes to every configured topic using the correct
     message type imported at runtime.
  3. On each timer tick (= 1 / recording_frequency), uses the primary
     topic's latest message as the reference timestamp and saves the
     nearest message from every other topic to disk.
  4. Files are named:  <timestamp_ns>_<name>.<ext>
       Images  → .npy
       Others  → .json
"""

import importlib
import time
import threading
from pathlib import Path
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy,
    QoSHistoryPolicy, QoSDurabilityPolicy
)

from .config_loader import load_config, CollectorConfig, TopicConfig
from .msg_buffer import MsgBuffer, get_msg_stamp, ros_time_to_sec
from .utils import save_msg


# ── helpers ───────────────────────────────────────────────────────────────────

def import_msg_type(msg_type_str: str):
    """
    Dynamically import a ROS2 message class from a string like
    'sensor_msgs/Image' or 'geometry_msgs/PoseStamped'.
    """
    parts = msg_type_str.strip().split('/')
    if len(parts) != 2:
        raise ValueError(
            f"msg_type must be '<package>/<MsgName>', got: {msg_type_str}"
        )
    pkg, cls_name = parts
    module = importlib.import_module(f'{pkg}.msg')
    return getattr(module, cls_name)


def make_qos(depth: int = 10) -> QoSProfile:
    """Best-effort QoS profile suitable for sensor data."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


# ── node ──────────────────────────────────────────────────────────────────────

class DataCollectorNode(Node):

    def __init__(self, config: CollectorConfig):
        super().__init__('data_collector_node')
        self._cfg = config
        self._recording = False
        self._save_lock = threading.Lock()
        self._snapshot_index = 0

        # Prepare output directory
        self._cfg.recording.output_dir.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(
            f"Output directory: {self._cfg.recording.output_dir}"
        )

        # Build a buffer and subscriber for each configured topic
        self._buffers: dict[str, MsgBuffer] = {}
        self._subs = []
        for tc in self._cfg.topics:
            self._setup_subscriber(tc)

        # Timer at the recording frequency
        period = 1.0 / self._cfg.recording.frequency
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"DataCollector ready. "
            f"Recording at {self._cfg.recording.frequency} Hz. "
            f"Primary topic: '{self._cfg.primary_topic.topic}'. "
            f"Watching {len(self._cfg.topics)} topic(s)."
        )

    # ── subscription setup ────────────────────────────────────────────────────

    def _setup_subscriber(self, tc: TopicConfig):
        try:
            msg_class = import_msg_type(tc.msg_type)
        except Exception as e:
            self.get_logger().error(
                f"Cannot import msg type '{tc.msg_type}': {e}"
            )
            return

        buf = MsgBuffer(maxlen=self._cfg.recording.buffer_size)
        self._buffers[tc.topic] = buf

        qos = make_qos()
        sub = self.create_subscription(
            msg_class,
            tc.topic,
            lambda msg, topic=tc.topic: self._on_msg(msg, topic),
            qos,
        )
        self._subs.append(sub)
        self.get_logger().info(
            f"Subscribed to '{tc.topic}' [{tc.msg_type}] → name='{tc.name}'"
        )

    def _on_msg(self, msg, topic: str):
        recv_time = time.time()
        self._buffers[topic].push(msg, recv_time)

    # ── timer callback (recording tick) ──────────────────────────────────────

    def _on_timer(self):
        primary = self._cfg.primary_topic
        if primary is None:
            return

        primary_buf = self._buffers.get(primary.topic)
        if primary_buf is None or len(primary_buf) == 0:
            return  # no data yet

        # Get the second-to-last primary message as reference timestamp.
        # This gives other topics time to receive a nearby message before
        # we do the alignment lookup.
        recv_t, stamp, primary_msg = primary_buf.second_to_last()
        ref_time = stamp if stamp is not None else recv_t

        # Save all topics
        with self._save_lock:
            self._save_snapshot(ref_time)

    # ── snapshot save ─────────────────────────────────────────────────────────

    def _save_snapshot(self, ref_time: float):
        """
        For each topic, find the message nearest to ref_time and save it.
        Each snapshot gets its own sub-folder named by a zero-padded index.
        Filename uses each message's own timestamp so alignment can be verified.
        """
        self._snapshot_index += 1
        snapshot_dir = self._cfg.recording.output_dir / f"{self._snapshot_index:06d}"
        snapshot_dir.mkdir(parents=True, exist_ok=True)

        saved_count = 0
        for tc in self._cfg.topics:
            buf = self._buffers.get(tc.topic)
            if buf is None or len(buf) == 0:
                self.get_logger().warn(
                    f"No data in buffer for topic '{tc.topic}', skipping."
                )
                continue

            entry = buf.nearest_to(ref_time, use_header_stamp=True)
            if entry is None:
                continue

            recv_t, stamp, msg = entry

            # Use the message's own header stamp if available, else recv time
            msg_time = stamp if stamp is not None else recv_t
            ts_ns = int(msg_time * 1e9)

            # Build path (without extension – serializer adds it)
            stem = f"{ts_ns}_{tc.name}"
            path_stem = snapshot_dir / stem

            try:
                final_path = save_msg(msg, path_stem)
                saved_count += 1
                self.get_logger().debug(f"Saved: {final_path.name}")
            except Exception as e:
                self.get_logger().error(
                    f"Failed to save '{tc.topic}' → {stem}: {e}"
                )

        if saved_count > 0:
            self.get_logger().info(
                f"[t={ref_time:.3f}] Saved {saved_count}/{len(self._cfg.topics)} topics."
            )

    # ── public control API ────────────────────────────────────────────────────

    def start_recording(self):
        self._recording = True
        self.get_logger().info("Recording STARTED.")

    def stop_recording(self):
        self._recording = False
        self.get_logger().info("Recording STOPPED.")


# ── entrypoint ────────────────────────────────────────────────────────────────

def main(args=None):
    import argparse
    import sys

    parser = argparse.ArgumentParser(description='ROS2 Data Collector Node')
    parser.add_argument(
        '--config', '-c',
        default=None,
        help='Path to config.yaml. '
             'Falls back to the ROS parameter "config_path" if not given.'
    )
    # parse only our own args, leave the rest for rclpy
    known, remaining = parser.parse_known_args(args=args)

    rclpy.init(args=remaining)

    # Determine config path
    if known.config:
        config_path = known.config
    else:
        # Try to get it from a ROS parameter after node creation
        # We'll create a temporary node just to read the parameter
        tmp = rclpy.create_node('_config_reader')
        tmp.declare_parameter('config_path', '')
        config_path = tmp.get_parameter('config_path').get_parameter_value().string_value
        tmp.destroy_node()
        if not config_path:
            print(
                "ERROR: No config path provided. "
                "Use --config /path/to/config.yaml or "
                "set the 'config_path' ROS parameter.",
                file=sys.stderr,
            )
            rclpy.shutdown()
            sys.exit(1)

    try:
        config = load_config(config_path)
    except Exception as e:
        print(f"ERROR loading config '{config_path}': {e}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    node = DataCollectorNode(config)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()