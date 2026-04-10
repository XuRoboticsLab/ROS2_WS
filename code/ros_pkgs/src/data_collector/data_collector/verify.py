"""
verify.py
---------
Data quality checks for recordings produced by data_collector.
Called automatically after recording stops; prompts the user to keep or
discard the session directory.
"""

import json
import re
import shutil
import sys
from pathlib import Path
from typing import Optional

import numpy as np

try:
    import matplotlib
    matplotlib.use('TkAgg')   # non-blocking interactive backend
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
    HAS_PLOT = True
except ImportError:
    HAS_PLOT = False

try:
    from .analyze_gripper import load_joint_topic, method_mean, method_transition, plot_all_grippers
    HAS_GRIPPER = True
except ImportError:
    HAS_GRIPPER = False


# ── filename pattern:  <timestamp_ns>_<name>.<ext> ────────────────────────────
_FNAME_RE = re.compile(r'^(\d+)_(.+)\.(json|npy)$')


# ══════════════════════════════════════════════════════════════════════════════
# Loading
# ══════════════════════════════════════════════════════════════════════════════

def load_recording(root: Path) -> dict:
    """
    Walk snapshot sub-folders (000001, 000002, …) and return:
        { topic_name: [ {'snapshot', 'ts_ns', 'ts_sec', 'path', 'ext'}, … ] }
    """
    def parse(fname: str) -> Optional[tuple]:
        m = _FNAME_RE.match(fname)
        if not m:
            return None
        return int(m.group(1)), m.group(2), m.group(3)

    data: dict[str, list] = {}
    folders = sorted(p for p in root.iterdir() if p.is_dir() and p.name.isdigit())
    if not folders:
        print(f"[verify] No snapshot folders found in {root}")
        return data

    for folder in folders:
        idx = int(folder.name)
        for fpath in folder.iterdir():
            parsed = parse(fpath.name)
            if parsed is None:
                continue
            ts_ns, name, ext = parsed
            data.setdefault(name, []).append({
                'snapshot': idx,
                'ts_ns':    ts_ns,
                'ts_sec':   ts_ns * 1e-9,
                'path':     fpath,
                'ext':      ext,
            })

    for name in data:
        data[name].sort(key=lambda x: x['snapshot'])

    return data


# ══════════════════════════════════════════════════════════════════════════════
# Alignment check
# ══════════════════════════════════════════════════════════════════════════════

def check_alignment(data: dict, primary: str, warn_ms: float = 50.0) -> dict:
    if primary not in data:
        print(f"[ALIGN] Primary topic '{primary}' not found. "
              f"Available: {list(data.keys())}")
        return {}

    primary_ts = {e['snapshot']: e['ts_sec'] for e in data[primary]}

    results = {}
    for name, entries in data.items():
        if name == primary:
            continue
        diffs = []
        for e in entries:
            ref = primary_ts.get(e['snapshot'])
            if ref is None:
                continue
            diffs.append(abs(e['ts_sec'] - ref) * 1000.0)
        if not diffs:
            continue
        arr = np.array(diffs)
        results[name] = {
            'diffs_ms': arr,
            'mean_ms':  float(arr.mean()),
            'max_ms':   float(arr.max()),
            'p95_ms':   float(np.percentile(arr, 95)),
        }

    print("\n── Timestamp Alignment (vs primary: '{}') ──".format(primary))
    header = f"  {'Topic':<30} {'mean(ms)':>10} {'p95(ms)':>10} {'max(ms)':>10}"
    print(header)
    print("  " + "─" * (len(header) - 2))
    for name, r in sorted(results.items()):
        flag = "  ⚠" if r['max_ms'] > warn_ms else ""
        print(f"  {name:<30} {r['mean_ms']:>10.2f} {r['p95_ms']:>10.2f} "
              f"{r['max_ms']:>10.2f}{flag}")

    return results


# ══════════════════════════════════════════════════════════════════════════════
# Frame rate & jitter
# ══════════════════════════════════════════════════════════════════════════════

def check_framerate(data: dict, dropout_threshold: float = 3.0) -> dict:
    results = {}
    print("\n── Frame Rate & Jitter ──")
    header = (f"  {'Topic':<30} {'fps':>7} {'median_ms':>10} "
              f"{'std_ms':>8} {'max_ms':>8} {'dropouts':>9}")
    print(header)
    print("  " + "─" * (len(header) - 2))

    for name, entries in sorted(data.items()):
        ts = np.array([e['ts_sec'] for e in entries])
        if len(ts) < 2:
            continue
        gaps = np.diff(ts) * 1000.0
        median_gap = float(np.median(gaps))
        fps = 1000.0 / median_gap if median_gap > 0 else 0.0
        dropout_mask = gaps > dropout_threshold * median_gap
        dropout_indices = np.where(dropout_mask)[0]

        results[name] = {
            'ts':              ts,
            'gaps_ms':         gaps,
            'fps':             fps,
            'median_gap_ms':   median_gap,
            'std_gap_ms':      float(gaps.std()),
            'max_gap_ms':      float(gaps.max()),
            'dropout_indices': dropout_indices,
            'n_dropouts':      int(dropout_mask.sum()),
        }

        flag = (f"  ⚠ {dropout_mask.sum()} dropout(s)"
                if dropout_mask.any() else "")
        print(f"  {name:<30} {fps:>7.2f} {median_gap:>10.2f} "
              f"{gaps.std():>8.2f} {gaps.max():>8.2f}{flag}")

    return results


# ══════════════════════════════════════════════════════════════════════════════
# Trajectory smoothness
# ══════════════════════════════════════════════════════════════════════════════

def _finite_diff(values: np.ndarray, times: np.ndarray) -> np.ndarray:
    dt = np.diff(times)
    dv = np.diff(values, axis=0)
    dt = np.where(dt < 1e-9, 1e-9, dt)
    deriv = dv / dt[:, None] if values.ndim == 2 else dv / dt
    return np.concatenate([[deriv[0]], deriv])


def _load_json_field(path: Path, *keys):
    try:
        with open(path) as f:
            obj = json.load(f)
        for k in keys:
            obj = obj[k]
        return obj
    except Exception:
        return None


def _check_one_joint_topic(data: dict, joint_topic: str, results: dict):
    """Compute smoothness for a single joint topic and store into results."""
    if joint_topic not in data:
        print(f"  [joints] Topic '{joint_topic}' not found in recording.")
        return

    entries = data[joint_topic]
    ts, positions, names_out = [], [], None
    for e in entries:
        if e['ext'] != 'json':
            continue
        pos = _load_json_field(e['path'], 'position')
        if pos is None:
            continue
        if names_out is None:
            names_out = _load_json_field(e['path'], 'name') or []
        ts.append(e['ts_sec'])
        positions.append(pos)

    if len(ts) <= 3:
        print(f"  [joints] Not enough data in '{joint_topic}'")
        return

    ts_arr  = np.array(ts)
    pos_arr = np.array(positions)
    vel     = _finite_diff(pos_arr, ts_arr)
    acc     = _finite_diff(vel,     ts_arr)
    jerk    = _finite_diff(acc,     ts_arr)

    results[f'joints:{joint_topic}'] = {
        'ts': ts_arr, 'position': pos_arr,
        'velocity': vel, 'acceleration': acc, 'jerk': jerk,
        'joint_names': names_out or [],
        'topic': joint_topic,
    }

    print(f"\n  Joint states ('{joint_topic}')  —  {pos_arr.shape[1]} joints")
    print(f"  {'Joint':<20} {'vel_rms':>10} {'acc_rms':>10} "
          f"{'jerk_rms':>10} {'jerk_peak':>12}")
    print("  " + "─" * 68)
    for j in range(pos_arr.shape[1]):
        jname = (names_out[j] if names_out and j < len(names_out)
                 else f"joint_{j}")
        v_rms  = float(np.sqrt(np.mean(vel[:, j]**2)))
        a_rms  = float(np.sqrt(np.mean(acc[:, j]**2)))
        jr_rms = float(np.sqrt(np.mean(jerk[:, j]**2)))
        jr_pk  = float(np.max(np.abs(jerk[:, j])))
        flag   = "  ⚠" if jr_pk > 10 * jr_rms else ""
        print(f"  {jname:<20} {v_rms:>10.4f} {a_rms:>10.4f} "
              f"{jr_rms:>10.4f} {jr_pk:>12.4f}{flag}")


def _check_one_ee_topic(data: dict, ee_topic: str, results: dict):
    """Compute smoothness for a single EE topic and store into results."""
    if ee_topic not in data:
        print(f"  [ee] Topic '{ee_topic}' not found in recording.")
        return

    entries = data[ee_topic]
    ts, positions_xyz = [], []
    for e in entries:
        if e['ext'] != 'json':
            continue
        px = _load_json_field(e['path'], 'pose', 'position', 'x')
        py = _load_json_field(e['path'], 'pose', 'position', 'y')
        pz = _load_json_field(e['path'], 'pose', 'position', 'z')
        if None in (px, py, pz):
            continue
        ts.append(e['ts_sec'])
        positions_xyz.append([px, py, pz])

    if len(ts) <= 3:
        print(f"  [ee] Not enough data in '{ee_topic}'")
        return

    ts_arr  = np.array(ts)
    pos_arr = np.array(positions_xyz)
    vel     = _finite_diff(pos_arr, ts_arr)
    acc     = _finite_diff(vel,     ts_arr)
    jerk    = _finite_diff(acc,     ts_arr)
    speed   = np.linalg.norm(vel,  axis=1)
    accel_n = np.linalg.norm(acc,  axis=1)
    jerk_n  = np.linalg.norm(jerk, axis=1)

    results[f'ee:{ee_topic}'] = {
        'ts': ts_arr, 'position': pos_arr,
        'velocity': vel, 'acceleration': acc, 'jerk': jerk,
        'speed': speed, 'accel_norm': accel_n, 'jerk_norm': jerk_n,
        'topic': ee_topic,
    }

    print(f"\n  End-effector ('{ee_topic}')")
    print(f"  speed  — mean: {speed.mean():.4f} m/s   peak: {speed.max():.4f} m/s")
    print(f"  accel  — rms:  {np.sqrt(np.mean(accel_n**2)):.4f} m/s²  "
          f"peak: {accel_n.max():.4f} m/s²")
    jerk_rms = float(np.sqrt(np.mean(jerk_n**2)))
    jerk_pk  = float(jerk_n.max())
    flag = "  ⚠ large jerk spike" if jerk_pk > 10 * jerk_rms else ""
    print(f"  jerk   — rms:  {jerk_rms:.4f} m/s³  peak: {jerk_pk:.4f} m/s³{flag}")


def check_trajectory_smoothness(data: dict,
                                 joint_topics: list = None,
                                 ee_topics: list = None) -> dict:
    if joint_topics is None:
        joint_topics = ['joint_states']
    if ee_topics is None:
        ee_topics = ['ee_pose']

    results = {}
    print("\n── Trajectory Smoothness ──")

    for jt in joint_topics:
        _check_one_joint_topic(data, jt, results)

    for et in ee_topics:
        _check_one_ee_topic(data, et, results)

    return results


# ══════════════════════════════════════════════════════════════════════════════
# Image playback
# ══════════════════════════════════════════════════════════════════════════════
def play_image_topics(data: dict, image_topic_names: list) -> None:
    """
    For each image topic, load .npy frames in snapshot order and display them
    one by one with plt.pause(), like a video. Multiple topics are shown
    side-by-side. Blocks until all frames are shown.
    """
    if not HAS_PLOT:
        print("[play] matplotlib not available — skipping image playback.")
        return

    # Collect .npy paths per topic, ordered by snapshot index
    topic_paths: dict[str, list] = {}
    for name in image_topic_names:
        if name not in data:
            print(f"[play] Image topic '{name}' not found.")
            continue
        paths = [
            e['path']
            for e in sorted(data[name], key=lambda x: x['snapshot'])
            if e['ext'] == 'npy'
        ]
        if paths:
            topic_paths[name] = paths

    if not topic_paths:
        print("[play] No image data to play.")
        return

    n_topics = len(topic_paths)
    n_frames = max(len(p) for p in topic_paths.values())
    names    = list(topic_paths.keys())

    # Estimate FPS from timestamps of first topic
    first_ts = np.array([
        e['ts_sec']
        for e in sorted(data[names[0]], key=lambda x: x['snapshot'])
        if e['ext'] == 'npy'
    ])
    fps = (1.0 / float(np.median(np.diff(first_ts)))
           if len(first_ts) > 1 else 10.0)
    fps = max(1.0, min(fps, 120.0))
    pause_sec = 1.0 / fps

    print(f"\n── Image Playback  ({n_frames} frames @ {fps:.1f} fps) ──")

    fig, axes = plt.subplots(1, n_topics, figsize=(6 * n_topics, 5),
                             squeeze=False)
    axes = axes[0]
    fig.suptitle("Image Playback", fontsize=11)

    im_handles, title_handles = [], []
    for ax, name in zip(axes, names):
        first = np.load(str(topic_paths[name][0]))
        # BGR -> RGB conversion for correct color display
        if first.ndim == 3 and first.shape[2] == 3:
            first = first[..., ::-1]
        im = ax.imshow(first, cmap='gray' if first.ndim == 2 else None,
                        aspect='auto')
        ax.axis('off')
        title_handles.append(ax.set_title(name, fontsize=9))
        im_handles.append(im)

    plt.tight_layout()
    plt.show(block=False)

    for i in range(n_frames):
        for j, name in enumerate(names):
            paths = topic_paths[name]
            idx   = min(i, len(paths) - 1)
            frame = np.load(str(paths[idx]))
            # BGR -> RGB conversion for correct color display
            if frame.ndim == 3 and frame.shape[2] == 3:
                frame = frame[..., ::-1]
            im_handles[j].set_data(frame)
            title_handles[j].set_text(f"{name}  [{idx + 1}/{len(paths)}]")
        plt.pause(pause_sec)

    plt.close(fig)

# ══════════════════════════════════════════════════════════════════════════════
# Plotting
# ══════════════════════════════════════════════════════════════════════════════

def plot_all(align_results: dict, fr_results: dict, traj_results: dict,
             primary: str):
    if not HAS_PLOT:
        print("\n[plot] matplotlib not available — skipping plots.")
        return

    n_topics = len(fr_results)
    fig = plt.figure(figsize=(14, 4 + 3 * n_topics))
    fig.suptitle("Data Quality — Alignment & Frame Rate", fontsize=13)
    gs = gridspec.GridSpec(n_topics + 1, 2, figure=fig,
                           hspace=0.55, wspace=0.35)

    ax_align = fig.add_subplot(gs[0, :])
    if align_results:
        names = list(align_results.keys())
        diffs = [align_results[n]['diffs_ms'] for n in names]
        ax_align.boxplot(diffs, labels=names, vert=True)
        ax_align.set_ylabel("Alignment error (ms)")
        ax_align.set_title(f"Timestamp alignment vs primary '{primary}'")
        ax_align.tick_params(axis='x', rotation=30)
        ax_align.axhline(10, color='orange', linestyle='--', linewidth=0.8,
                         label='10 ms')
        ax_align.axhline(33, color='red', linestyle='--', linewidth=0.8,
                         label='33 ms (30 fps frame)')
        ax_align.legend(fontsize=8)

    for i, (name, r) in enumerate(sorted(fr_results.items())):
        row = i + 1
        ax_gap = fig.add_subplot(gs[row, 0])
        ax_gap.plot(r['gaps_ms'], linewidth=0.8, color='steelblue')
        if r['dropout_indices'].size:
            ax_gap.scatter(r['dropout_indices'],
                           r['gaps_ms'][r['dropout_indices']],
                           color='red', s=20, zorder=5, label='dropout')
            ax_gap.legend(fontsize=7)
        ax_gap.axhline(r['median_gap_ms'], color='green', linestyle='--',
                       linewidth=0.8)
        ax_gap.set_title(f"{name}  ({r['fps']:.1f} fps)", fontsize=9)
        ax_gap.set_xlabel("frame index", fontsize=8)
        ax_gap.set_ylabel("gap (ms)", fontsize=8)

        ax_hist = fig.add_subplot(gs[row, 1])
        ax_hist.hist(r['gaps_ms'], bins=40, color='steelblue', edgecolor='none')
        ax_hist.axvline(r['median_gap_ms'], color='green', linestyle='--',
                        linewidth=0.8)
        ax_hist.set_title(f"{name} — gap histogram", fontsize=9)
        ax_hist.set_xlabel("gap (ms)", fontsize=8)
        ax_hist.set_ylabel("count", fontsize=8)

    joint_results = {k: v for k, v in traj_results.items()
                     if k.startswith('joints:')}

    if traj_results:
        fig2, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=False)
        fig2.suptitle("Trajectory Smoothness", fontsize=13)

        for key, r in joint_results.items():
            ts     = r['ts'] - r['ts'][0]
            topic  = r['topic']
            for j in range(r['jerk'].shape[1]):
                jname = (r['joint_names'][j] if j < len(r['joint_names'])
                         else f"j{j}")
                axes[0].plot(ts, r['jerk'][:, j], linewidth=0.8,
                             label=f"{topic}/{jname}")
            axes[1].plot(ts, r['position'], linewidth=0.8,
                         label=topic)
        if joint_results:
            axes[0].set_title("Joint jerk (rad/s³)")
            axes[0].set_ylabel("jerk")
            axes[0].legend(fontsize=7, ncol=4)
            axes[0].set_xlabel("time (s)")
            axes[1].set_title("Joint positions (rad)")
            axes[1].set_ylabel("angle (rad)")
            axes[1].set_xlabel("time (s)")
            if len(joint_results) > 1:
                axes[1].legend(fontsize=7)

        ee_results = {k: v for k, v in traj_results.items()
                      if k.startswith('ee:')}
        for key, r in ee_results.items():
            ts = r['ts'] - r['ts'][0]
            axes[2].plot(ts, r['jerk_norm'], linewidth=0.8,
                         label=f"{r['topic']} jerk")
            axes[2].plot(ts, r['speed'], linewidth=0.8, alpha=0.7,
                         label=f"{r['topic']} speed")
        if ee_results:
            axes[2].set_title("End-effector speed & jerk norm")
            axes[2].set_ylabel("m/s  |  m/s³")
            axes[2].set_xlabel("time (s)")
            axes[2].legend(fontsize=8)

        plt.tight_layout()

    plt.show(block=False)


# ══════════════════════════════════════════════════════════════════════════════
# Main entry point (called from data_collector_node after recording stops)
# ══════════════════════════════════════════════════════════════════════════════

def run_verify_and_prompt(session_dir: Path, v,
                          image_topic_names: list = None,
                          wait_for_decision=None) -> bool:
    """
    Run quality checks on *session_dir*, optionally play image topics and plot,
    then ask the user whether to keep or delete the recording.

    Parameters
    ----------
    session_dir : Path
        The directory that was just recorded into.
    v : VerifyConfig
        Verify settings from the config.
    image_topic_names : list[str], optional
        Topic names (the 'name' field, not the ROS topic) whose msg_type is
        sensor_msgs/Image. If provided and v.plot is True, they are played
        back as a video before the static analysis plots.
    wait_for_decision : callable, optional
        Called to obtain the keep/delete decision. Must block until the user
        decides and return True (keep) or False (delete). When None, falls
        back to keyboard input().

    Returns
    -------
    bool
        True  → recording was kept.
        False → recording was deleted.
    """
    print(f"\n{'═'*60}")
    print(f"  Post-recording quality check")
    print(f"  Session: {session_dir}")
    print(f"{'═'*60}")

    data = load_recording(session_dir)
    if not data:
        print("[verify] No data found — nothing to verify.")
        return True

    print(f"\nLoaded {sum(len(vv) for vv in data.values())} entries across "
          f"{len(data)} topic(s):")
    for name, entries in sorted(data.items()):
        print(f"  {name:<30} {len(entries):>5} snapshots")

    align = check_alignment(data, primary=v.primary,
                             warn_ms=v.alignment_warn_ms)
    fr    = check_framerate(data, dropout_threshold=v.dropout_threshold)
    traj  = check_trajectory_smoothness(data,
                                        joint_topics=v.joint_topics,
                                        ee_topics=v.ee_topics)

    if v.plot:
        if image_topic_names:
            play_image_topics(data, image_topic_names)
        plot_all(align, fr, traj, primary=v.primary)

        if HAS_GRIPPER and v.gripper_joint_topics:
            print("\n── Gripper Analysis ──")
            gripper_results = []
            for topic_name in v.gripper_joint_topics:
                ts, values = load_joint_topic(
                    session_dir, topic_name, v.gripper_joint_name
                )
                if ts is None:
                    continue
                state_mean  = method_mean(values)
                state_trans = method_transition(values, v.gripper_transition_factor)
                n_open_m1 = int(state_mean.sum())
                n_open_m2 = int(state_trans.sum())
                n_trans   = int(np.abs(np.diff(state_trans)).sum())
                print(f"  {topic_name}: {len(ts)} frames  "
                      f"min={values.min():.3f}  max={values.max():.3f}  "
                      f"mean={values.mean():.3f}")
                print(f"    Method 1: {n_open_m1} open / {len(ts)-n_open_m1} closed")
                print(f"    Method 2: {n_open_m2} open / {len(ts)-n_open_m2} closed  "
                      f"({n_trans} transitions)")
                gripper_results.append(
                    (topic_name, ts, values, state_mean, state_trans)
                )
            if gripper_results:
                plot_all_grippers(gripper_results, block=False)

    print(f"\n{'═'*60}")

    # ── Decision ──────────────────────────────────────────────────────────────
    if wait_for_decision is not None:
        print("  Press SAVE pedal to keep, DELETE pedal to discard.", flush=True)
        if v.plot and HAS_PLOT:
            # Pump the Tk event loop while waiting so windows stay responsive
            import threading as _threading
            _done = _threading.Event()
            _result = [None]

            def _bg():
                _result[0] = wait_for_decision()
                _done.set()

            _threading.Thread(target=_bg, daemon=True).start()
            while not _done.is_set():
                plt.pause(0.1)
            keep = _result[0]
        else:
            keep = wait_for_decision()
    else:
        keep = None
        while keep is None:
            try:
                answer = input("  Keep this recording? [Y/n]: ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                answer = 'y'
                print()
            if answer in ('', 'y', 'yes'):
                keep = True
            elif answer in ('n', 'no'):
                keep = False
            else:
                print("  Please enter Y or N.")

    if keep:
        print(f"  Recording saved → {session_dir}")
    else:
        print(f"  Deleting {session_dir} …")
        shutil.rmtree(session_dir, ignore_errors=True)
        print(f"  Deleted.")

    print(f"{'═'*60}\n")
    if v.plot and HAS_PLOT:
        plt.close('all')
    return keep
