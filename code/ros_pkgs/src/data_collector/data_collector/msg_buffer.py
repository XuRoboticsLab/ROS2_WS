"""
msg_buffer.py
-------------
Thread-safe ring buffer for storing the latest N messages per topic,
with timestamp-based nearest-message lookup.
"""

import threading
from collections import deque


def ros_time_to_sec(stamp) -> float:
    """Convert a ROS builtin_interfaces/Time stamp to float seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def get_msg_stamp(msg) -> float | None:
    """
    Extract the timestamp (as float seconds) from a message.
    Checks msg.header.stamp first, then msg.stamp, then returns None.
    """
    if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
        return ros_time_to_sec(msg.header.stamp)
    if hasattr(msg, 'stamp'):
        return ros_time_to_sec(msg.stamp)
    return None


class MsgBuffer:
    """
    Thread-safe buffer storing (receive_time, stamp, msg) tuples.
    - receive_time : float  – wall-clock time when the message arrived
    - stamp        : float | None – message header timestamp (if available)
    - msg          : any ROS message
    """

    def __init__(self, maxlen: int = 200):
        self._buf: deque = deque(maxlen=maxlen)
        self._lock = threading.Lock()

    def push(self, msg, receive_time: float):
        stamp = get_msg_stamp(msg)
        with self._lock:
            self._buf.append((receive_time, stamp, msg))

    def latest(self):
        """Return the most-recently received (receive_time, stamp, msg) or None."""
        with self._lock:
            if not self._buf:
                return None
            return self._buf[-1]

    def second_to_last(self):
        """
        Return the second-to-last received (receive_time, stamp, msg), or
        fall back to the latest if fewer than 2 messages are buffered.
        """
        with self._lock:
            if not self._buf:
                return None
            if len(self._buf) < 2:
                return self._buf[-1]
            return self._buf[-2]

    def nearest_to(self, target_time: float, use_header_stamp: bool = True):
        """
        Return the (receive_time, stamp, msg) whose timestamp is closest
        to *target_time*.

        Searches from the newest entry backwards. Because messages arrive in
        roughly chronological order, the diff forms a valley shape — once it
        starts increasing we have passed the minimum and can exit early.
        """
        with self._lock:
            if not self._buf:
                return None
            best = None
            best_diff = float('inf')
            for entry in reversed(self._buf):
                recv_t, stamp, msg = entry
                t = stamp if (use_header_stamp and stamp is not None) else recv_t
                diff = abs(t - target_time)
                if diff < best_diff:
                    best_diff = diff
                    best = entry
                elif diff > best_diff:
                    break
            return best

    def __len__(self):
        with self._lock:
            return len(self._buf)