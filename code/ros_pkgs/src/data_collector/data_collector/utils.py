"""
serializers.py
--------------
Convert ROS messages to saveable formats:
  - sensor_msgs/Image  →  numpy array  (saved as .npy)
  - everything else    →  dict         (saved as .json)
"""

import json
import numpy as np
from pathlib import Path


# ── numpy / image ─────────────────────────────────────────────────────────────

# Mapping from sensor_msgs/Image encoding to numpy dtype
_ENCODING_TO_DTYPE = {
    'rgb8':    (np.uint8,  3),
    'rgba8':   (np.uint8,  4),
    'bgr8':    (np.uint8,  3),
    'bgra8':   (np.uint8,  4),
    'mono8':   (np.uint8,  1),
    'mono16':  (np.uint16, 1),
    '16UC1':   (np.uint16, 1),
    '32FC1':   (np.float32, 1),
}


def image_msg_to_numpy(msg) -> np.ndarray:
    """
    Convert a sensor_msgs/Image message to a numpy array.
    Shape: (H, W) for single-channel, (H, W, C) for multi-channel.
    """
    encoding = msg.encoding
    if encoding not in _ENCODING_TO_DTYPE:
        # Fallback: treat raw bytes as uint8 flat array
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        return arr.reshape((msg.height, msg.step))

    dtype, channels = _ENCODING_TO_DTYPE[encoding]
    arr = np.frombuffer(bytes(msg.data), dtype=dtype)

    if channels == 1:
        return arr.reshape((msg.height, msg.width))
    else:
        return arr.reshape((msg.height, msg.width, channels))


def save_image(arr: np.ndarray, path: Path):
    np.save(str(path), arr)


# ── JSON / generic ─────────────────────────────────────────────────────────────

def msg_to_dict(msg) -> dict:
    """
    Recursively convert a ROS message to a plain Python dict.
    Works for any message type that exposes __slots__ or get_fields_and_field_types().
    """
    # Try the ROS2 introspection API first
    if hasattr(msg, 'get_fields_and_field_types'):
        result = {}
        for field in msg.get_fields_and_field_types().keys():
            value = getattr(msg, field)
            result[field] = _convert_value(value)
        return result

    # Fallback: __slots__
    if hasattr(msg, '__slots__'):
        result = {}
        for slot in msg.__slots__:
            value = getattr(msg, slot)
            result[slot] = _convert_value(value)
        return result

    return {'raw': str(msg)}


def _convert_value(value):
    """Recursively convert a field value to a JSON-serialisable type."""
    # Primitive types
    if isinstance(value, (bool, int, float, str)):
        return value

    # numpy / bytes arrays that may come from Array fields
    if isinstance(value, (bytes, bytearray)):
        return list(value)

    if isinstance(value, np.ndarray):
        return value.tolist()

    # Python sequences (list, tuple, array.array)
    if isinstance(value, (list, tuple)):
        return [_convert_value(v) for v in value]

    # Check for array.array (common in ROS2 for float64[] etc.)
    try:
        import array as _array
        if isinstance(value, _array.array):
            return list(value)
    except ImportError:
        pass

    # Nested ROS message
    if hasattr(value, 'get_fields_and_field_types') or hasattr(value, '__slots__'):
        return msg_to_dict(value)

    # Last resort
    return str(value)


def save_json(data: dict, path: Path):
    with open(str(path), 'w') as f:
        json.dump(data, f, indent=2)


# ── dispatch ───────────────────────────────────────────────────────────────────

def is_image_msg(msg) -> bool:
    """Return True if *msg* is a sensor_msgs/Image."""
    t = type(msg)
    return t.__module__.startswith('sensor_msgs') and t.__name__ == 'Image'


def save_msg(msg, path_without_ext: Path) -> Path:
    """
    Save a ROS message to disk and return the final path.
    Images → <path>.npy   (numpy array)
    Others → <path>.json  (dict)
    """
    if is_image_msg(msg):
        arr = image_msg_to_numpy(msg)
        final = path_without_ext.with_suffix('.npy')
        save_image(arr, final)
    else:
        data = msg_to_dict(msg)
        final = path_without_ext.with_suffix('.json')
        save_json(data, final)
    return final