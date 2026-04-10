#!/usr/bin/env python3
"""
verify_recording.py
-------------------
Standalone post-recording quality check.

Usage:
    python3 verify_recording.py                        # uses verify_config.yaml
    python3 verify_recording.py --config my_cfg.yaml  # custom config
    python3 verify_recording.py --session ./data/0409/20250409_123456  # specific session
"""

import argparse
import shutil
import sys
from pathlib import Path

import yaml

# ── import verify functions from the data_collector package source ────────────

from verify import (
    load_recording,
    check_alignment,
    check_framerate,
    check_trajectory_smoothness,
    plot_all,
    play_image_topics,
)


# ── config loading ─────────────────────────────────────────────────────────────

def load_verify_config(path: Path) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def latest_session(data_dir: Path) -> Path:
    """Return the most recently modified session sub-folder."""
    folders = sorted(
        (p for p in data_dir.iterdir() if p.is_dir()),
        key=lambda p: p.stat().st_mtime,
    )
    if not folders:
        sys.exit(f"No session folders found in {data_dir}")
    return folders[-1]


# ── main ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Verify a data_collector recording.')
    parser.add_argument('--config', '-c',
                        default=str(Path(__file__).parent / 'verify_config.yaml'),
                        help='Path to verify_config.yaml')
    parser.add_argument('--session', '-s', default=None,
                        help='Path to a specific session folder (skips auto-detect)')
    args = parser.parse_args()

    cfg_path = Path(args.config)

    cfg = load_verify_config(cfg_path)

    v = cfg.get('verify', {})

    # ── resolve session directory ──────────────────────────────────────────────
    if args.session:
        session_dir = Path(args.session)
    else:
        data_dir = Path(cfg['data_dir'])
        session_dir = latest_session(data_dir)

    if not session_dir.is_dir():
        sys.exit(f"Session directory not found: {session_dir}")

    print(f"\n{'═'*60}")
    print(f"  Recording: {session_dir}")
    print(f"{'═'*60}")

    # ── load data ──────────────────────────────────────────────────────────────
    data = load_recording(session_dir)
    if not data:
        sys.exit("[verify] No data found in session directory.")

    print(f"\nLoaded {sum(len(vv) for vv in data.values())} entries across "
          f"{len(data)} topic(s):")
    for name, entries in sorted(data.items()):
        print(f"  {name:<30} {len(entries):>5} snapshots")

    # ── checks ────────────────────────────────────────────────────────────────
    primary          = v.get('primary', next(iter(data)))
    alignment_warn   = float(v.get('alignment_warn_ms', 50.0))
    dropout_thresh   = float(v.get('dropout_threshold', 3.0))
    joint_topics     = _as_list(v.get('joint_topics', []))
    ee_topics        = _as_list(v.get('ee_topics', []))
    do_plot          = bool(v.get('plot', True))
    image_topics     = _as_list(cfg.get('image_topics', []))

    align = check_alignment(data, primary=primary, warn_ms=alignment_warn)
    fr    = check_framerate(data, dropout_threshold=dropout_thresh)
    traj  = check_trajectory_smoothness(data,
                                        joint_topics=joint_topics or None,
                                        ee_topics=ee_topics or None)

    # ── plots & playback ───────────────────────────────────────────────────────
    if do_plot:
        if image_topics:
            play_image_topics(data, image_topics)
        plot_all(align, fr, traj, primary=primary)

    # ── Y/N prompt ─────────────────────────────────────────────────────────────
    print(f"\n{'═'*60}")
    while True:
        try:
            answer = input("  Keep this recording? [Y/n]: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            answer = 'y'
            print()

        if answer in ('', 'y', 'yes'):
            print(f"  Recording saved → {session_dir}")
            break
        elif answer in ('n', 'no'):
            print(f"  Deleting {session_dir} …")
            shutil.rmtree(session_dir, ignore_errors=True)
            print(f"  Deleted.")
            break
        else:
            print("  Please enter Y or N.")

    try:
        import matplotlib.pyplot as plt
        plt.close('all')
    except Exception:
        pass

    print(f"{'═'*60}\n")


def _as_list(value) -> list:
    if isinstance(value, list):
        return value
    if value:
        return [str(value)]
    return []


if __name__ == '__main__':
    main()
