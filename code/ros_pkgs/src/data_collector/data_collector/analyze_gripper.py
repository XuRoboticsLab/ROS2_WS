#!/usr/bin/env python3
"""
analyze_gripper.py
------------------
Extract and binarize gripper state from a data_collector recording.

Two methods:
  1. Mean threshold  — below mean → closed (0), above mean → open (1)
  2. Transition detection — a sudden position jump flips the state;
     jump threshold = transition_factor × median of all inter-frame jumps

Usage:
    python3 analyze_gripper.py                          # latest session
    python3 analyze_gripper.py --session ./data/0409/20250409_153012
    python3 analyze_gripper.py --config verify_config.yaml
"""

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import yaml


# ── file loading ───────────────────────────────────────────────────────────────

_FNAME_RE = re.compile(r'^(\d+)_(.+)\.json$')


def latest_session(data_dir: Path) -> Path:
    folders = sorted(
        (p for p in data_dir.iterdir() if p.is_dir()),
        key=lambda p: p.stat().st_mtime,
    )
    if not folders:
        sys.exit(f"No session folders found in {data_dir}")
    return folders[-1]


def load_joint_topic(session_dir: Path, topic_name: str,
                     gripper_joint_name: str) -> tuple:
    """
    Load all snapshots for *topic_name* and extract:
      ts       : (T,)  timestamps in seconds
      gripper  : (T,)  raw gripper position values

    Returns (ts, gripper) or (None, None) on failure.
    """
    folders = sorted(
        p for p in session_dir.iterdir()
        if p.is_dir() and p.name.isdigit()
    )
    if not folders:
        print(f"  [gripper] No snapshot folders in {session_dir}")
        return None, None

    ts_list, val_list = [], []
    gripper_idx = None

    for folder in folders:
        for fpath in folder.iterdir():
            m = _FNAME_RE.match(fpath.name)
            if not m:
                continue
            ts_ns, name = int(m.group(1)), m.group(2)
            if name != topic_name:
                continue

            try:
                with open(fpath) as f:
                    obj = json.load(f)
            except Exception as e:
                print(f"  [gripper] Failed to read {fpath.name}: {e}")
                continue

            # Find gripper index on first valid file
            if gripper_idx is None:
                names = obj.get('name', [])
                if gripper_joint_name in names:
                    gripper_idx = names.index(gripper_joint_name)
                else:
                    print(f"  [gripper] Joint '{gripper_joint_name}' not found "
                          f"in topic '{topic_name}'. Available: {names}")
                    return None, None

            positions = obj.get('position', [])
            if gripper_idx < len(positions):
                ts_list.append(ts_ns * 1e-9)
                val_list.append(positions[gripper_idx])

    if not ts_list:
        print(f"  [gripper] No data loaded for topic '{topic_name}'")
        return None, None

    # Sort by timestamp (snapshots are already ordered but be safe)
    order = np.argsort(ts_list)
    return np.array(ts_list)[order], np.array(val_list)[order]


# ── binarization ───────────────────────────────────────────────────────────────

def method_mean(values: np.ndarray) -> np.ndarray:
    """Below mean → 0 (closed), above mean → 1 (open)."""
    return (values >= values.mean()).astype(int)


def method_transition(values: np.ndarray,
                      transition_factor: float = 5.0) -> np.ndarray:
    """
    Detect sudden position jumps; each jump flips the gripper state.
    Initial state is determined by comparing the first value to the mean.
    """
    jumps = np.abs(np.diff(values))

    # Threshold: factor × median of nonzero jumps (ignores steady-state noise)
    nonzero = jumps[jumps > 0]
    if len(nonzero) == 0:
        # No movement at all — fall back to mean method
        return method_mean(values)

    threshold = transition_factor * np.median(nonzero)
    transition_mask = jumps > threshold

    # Build state array
    initial_state = int(values[0] >= values.mean())
    states = np.empty(len(values), dtype=int)
    states[0] = initial_state
    current = initial_state
    for i, is_transition in enumerate(transition_mask):
        if is_transition:
            current = 1 - current
        states[i + 1] = current

    return states


# ── plotting ───────────────────────────────────────────────────────────────────

def _fill_col(axes_col, t, values, state_mean, state_trans, topic_name):
    """Fill one column (3 axes) for a single topic."""
    ax_raw, ax_m1, ax_m2 = axes_col

    # raw position
    ax_raw.plot(t, values, linewidth=1.0, color='steelblue')
    ax_raw.axhline(values.mean(), color='tomato', linestyle='--',
                   linewidth=0.9, label=f'mean={values.mean():.3f}')
    ax_raw.set_title(topic_name, fontsize=10)
    ax_raw.set_ylabel("position (rad)")
    ax_raw.legend(fontsize=8)

    # method 1
    ax_m1.step(t, state_mean, where='post', linewidth=1.2, color='steelblue')
    ax_m1.fill_between(t, state_mean, step='post', alpha=0.25, color='steelblue')
    ax_m1.set_yticks([0, 1])
    ax_m1.set_yticklabels(['closed', 'open'])
    ax_m1.set_ylim(-0.1, 1.3)
    ax_m1.set_ylabel("Method 1")

    # method 2
    ax_m2.step(t, state_trans, where='post', linewidth=1.2, color='darkorange')
    ax_m2.fill_between(t, state_trans, step='post', alpha=0.25, color='darkorange')
    trans_idx = np.where(np.diff(state_trans) != 0)[0]
    if trans_idx.size:
        ax_m2.scatter(t[trans_idx + 1], state_trans[trans_idx + 1],
                      color='red', s=40, zorder=5, label='transition')
        ax_m2.legend(fontsize=8)
    ax_m2.set_yticks([0, 1])
    ax_m2.set_yticklabels(['closed', 'open'])
    ax_m2.set_ylim(-0.1, 1.3)
    ax_m2.set_ylabel("Method 2")
    ax_m2.set_xlabel("time (s)")


def plot_all_grippers(results: list):
    """
    results: [ (topic_name, ts, values, state_mean, state_trans), … ]
    Draws a single figure with 3 rows × N columns (one column per topic).
    """
    n = len(results)
    fig, axes = plt.subplots(3, n, figsize=(7 * n, 8), squeeze=False)
    fig.suptitle("Gripper State Analysis", fontsize=13)

    # Row labels on the left-most column
    row_labels = ["Raw position", "Method 1\n(mean threshold)",
                  "Method 2\n(transition detect)"]
    for row, label in enumerate(row_labels):
        axes[row][0].set_ylabel(f"{label}\n", fontsize=9)

    for col, (topic_name, ts, values, state_mean, state_trans) in enumerate(results):
        t = ts - ts[0]
        _fill_col(axes[:, col], t, values, state_mean, state_trans, topic_name)

    plt.tight_layout()
    plt.show()


# ── main ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Analyze gripper open/close state.')
    parser.add_argument('--config', '-c',
                        default=str('configs/verify_config.yaml'))
    parser.add_argument('--session', '-s', default=None,
                        help='Specific session folder (skips auto-detect)')
    args = parser.parse_args()

    cfg_path = Path(args.config)
    if not cfg_path.is_absolute():
        cfg_path = Path(__file__).parent / cfg_path

    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)

    g = cfg.get('gripper', {})
    joint_topics       = g.get('joint_topics', [])
    gripper_joint_name = g.get('gripper_joint_name', 'gripper')
    transition_factor  = float(g.get('transition_factor', 5.0))

    if not joint_topics:
        sys.exit("No joint_topics defined under 'gripper:' in the config.")

    # Resolve session directory
    if args.session:
        session_dir = Path(args.session)
    else:
        data_dir = Path(cfg.get('data_dir', './data'))
        if not data_dir.is_absolute():
            data_dir = Path(__file__).parent / data_dir
        session_dir = latest_session(data_dir)

    if not session_dir.is_dir():
        sys.exit(f"Session directory not found: {session_dir}")

    print(f"\nSession: {session_dir}")
    print(f"Gripper joint: '{gripper_joint_name}'  "
          f"transition factor: {transition_factor}\n")

    results = []
    for topic in joint_topics:
        print(f"── {topic} ──")
        ts, values = load_joint_topic(session_dir, topic, gripper_joint_name)
        if ts is None:
            continue

        state_mean  = method_mean(values)
        state_trans = method_transition(values, transition_factor)

        n_open_m1 = int(state_mean.sum())
        n_open_m2 = int(state_trans.sum())
        n_trans   = int(np.abs(np.diff(state_trans)).sum())
        print(f"  {len(ts)} frames   "
              f"min={values.min():.3f}  max={values.max():.3f}  "
              f"mean={values.mean():.3f}")
        print(f"  Method 1: {n_open_m1} open / {len(ts)-n_open_m1} closed frames")
        print(f"  Method 2: {n_open_m2} open / {len(ts)-n_open_m2} closed frames  "
              f"({n_trans} transitions)\n")

        results.append((topic, ts, values, state_mean, state_trans))

    if results:
        plot_all_grippers(results)


if __name__ == '__main__':
    main()
