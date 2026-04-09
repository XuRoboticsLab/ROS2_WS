"""
config_loader.py
----------------
Load and validate the data-collector config.yaml.

Expected YAML structure
-----------------------
recording:
  frequency: 10.0          # Hz – recording (trigger) rate
  output_dir: /tmp/rosbag_data
  buffer_size: 200         # optional, default 200

topics:
  - name: left_rgb
    topic: /camera_left/color/image_raw
    msg_type: sensor_msgs/Image
    is_primary: false       # set true on ONE topic to use as the master clock

  - name: robot_joints
    topic: /joint_states
    msg_type: sensor_msgs/JointState
    is_primary: true        # this topic drives timestamp alignment

  - name: ee_pose
    topic: /end_effector/pose
    msg_type: geometry_msgs/PoseStamped
    is_primary: false
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional
import yaml


@dataclass
class TopicConfig:
    name: str            # used in filenames
    topic: str           # ROS topic string
    msg_type: str        # e.g. "sensor_msgs/Image"
    is_primary: bool = False


@dataclass
class RecordingConfig:
    frequency: float           # Hz
    output_dir: Path
    buffer_size: int = 200
    foot_pedal_topic: str = '/foot_pedal/press'


@dataclass
class CollectorConfig:
    recording: RecordingConfig
    topics: List[TopicConfig]

    @property
    def primary_topic(self) -> Optional[TopicConfig]:
        primaries = [t for t in self.topics if t.is_primary]
        if not primaries:
            return None
        return primaries[0]


def load_config(path: str | Path) -> CollectorConfig:
    with open(str(path), 'r') as f:
        raw = yaml.safe_load(f)

    # ── recording section ──────────────────────────────────────────────────────
    rec_raw = raw.get('recording', {})
    recording = RecordingConfig(
        frequency=float(rec_raw['frequency']),
        output_dir=Path(rec_raw.get('output_dir', './tmp/ros_data')),
        buffer_size=int(rec_raw.get('buffer_size', 200)),
        foot_pedal_topic=str(rec_raw.get('foot_pedal_topic', '/foot_pedal/press')),
    )

    # ── topics section ─────────────────────────────────────────────────────────
    topics_raw = raw.get('topics', [])
    if not topics_raw:
        raise ValueError("Config must define at least one topic under 'topics:'")

    topics = []
    n_primary = 0
    for t in topics_raw:
        tc = TopicConfig(
            name=str(t['name']),
            topic=str(t['topic']),
            msg_type=str(t['msg_type']),
            is_primary=bool(t.get('is_primary', False)),
        )
        topics.append(tc)
        if tc.is_primary:
            n_primary += 1

    if n_primary > 1:
        raise ValueError("Only one topic may be marked is_primary: true")

    # If no primary is specified, default to the first topic
    if n_primary == 0:
        topics[0].is_primary = True

    return CollectorConfig(recording=recording, topics=topics)