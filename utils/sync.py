#!/usr/bin/env python3
"""
sync_watcher.py
Usage : python3 sync_watcher.py <local_folder> <ssh_target> [options]
Example: python3 sync_watcher.py /data/capture user@host:/remote/data

只监控第一层子目录（不深入遍历文件）。
某个子目录出现超过 --delay 秒后，用独立线程将其整个 rsync 过去，不阻塞轮询。
"""

import argparse
import logging
import subprocess
import sys
import threading
import time
from pathlib import Path

# ── 默认配置 ──────────────────────────────────────────────────────────────────
POLL_INTERVAL = 10    # 轮询间隔（秒）
MIN_AGE       = 70   # 子目录需存在多少秒才触发同步（10 分钟）

# ── 日志 ──────────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
    handlers=[logging.StreamHandler(sys.stdout)],
)
log = logging.getLogger(__name__)


# ── 参数 ──────────────────────────────────────────────────────────────────────
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "Watch first-level subdirectories and rsync each one to an SSH target "
            "after it has existed for --delay seconds. Each sync runs in its own thread."
        )
    )
    p.add_argument("local_folder", help="Local directory to watch")
    p.add_argument("ssh_target",   help="SSH destination, e.g. user@host:/remote/path")
    p.add_argument("--poll",  type=int, default=POLL_INTERVAL, metavar="SEC",
                   help=f"Poll interval in seconds (default: {POLL_INTERVAL})")
    p.add_argument("--delay", type=int, default=MIN_AGE, metavar="SEC",
                   help=f"Seconds a subdir must exist before syncing (default: {MIN_AGE})")
    p.add_argument("--port", type=int, default=22, metavar="PORT",
                   help="SSH port on the remote host (default: 22)")
    return p.parse_args()


# ── 同步线程函数 ──────────────────────────────────────────────────────────────
def sync_subdir_thread(subdir: Path, local_dir: Path, ssh_target: str,
                       ssh_port: int, entry: dict) -> None:
    """
    Rsync an entire subdirectory to ssh_target, preserving its relative path.
    Updates entry["status"] to "done" or "failed" when finished.
    """
    rel         = subdir.relative_to(local_dir)  # e.g. Path("20240425_153000")
    remote_dest = f"{ssh_target}/{rel.parent}/"  # destination parent on remote

    # No trailing slash on source → rsync transfers the directory itself (with its name)
    cmd = [
        "rsync", "-avz", "--mkpath",
        "-e", f"ssh -p {ssh_port}",
        str(subdir), remote_dest,
    ]

    log.info("[%s] Sync start  →  %s (port %d)", subdir.name, remote_dest, ssh_port)
    result = subprocess.run(cmd, capture_output=True, text=True)

    if result.returncode == 0:
        log.info("[%s] Sync done", subdir.name)
        entry["status"] = "done"
    else:
        log.warning("[%s] Sync FAILED (will retry next cycle)\n%s",
                    subdir.name, result.stderr.strip())
        entry["status"] = "failed"


# ── 主循环 ────────────────────────────────────────────────────────────────────
def watch(local_folder: str, ssh_target: str, poll: int, delay: int, ssh_port: int) -> None:
    local_dir = Path(local_folder).resolve()
    if not local_dir.is_dir():
        log.error("Local folder does not exist: %s", local_dir)
        sys.exit(1)

    ssh_target = ssh_target.rstrip("/")

    log.info("Starting sync_watcher")
    log.info("  Local  : %s", local_dir)
    log.info("  Target : %s", ssh_target)
    log.info("  Port   : %d", ssh_port)
    log.info("  Delay  : %ss  |  Poll: %ss", delay, poll)

    # state: subdir_name (str) → {
    #   "first_seen": float,
    #   "status": "waiting" | "syncing" | "done" | "failed"
    # }
    state: dict[str, dict] = {}

    while True:
        now = time.time()

        # ── 1. 扫描第一层子目录 ───────────────────────────────────────────────
        try:
            subdirs = {e.name: e for e in local_dir.iterdir() if e.is_dir()}
        except PermissionError as exc:
            log.warning("Cannot read local folder: %s", exc)
            subdirs = {}

        for name in subdirs:
            if name not in state:
                log.info("New subdir: %s", name)
                state[name] = {"first_seen": now, "status": "waiting"}

        # 找出当前所有子目录中 first_seen 最晚的那个（即"最新"）
        # 只考虑尚未完成同步的目录，已 done 的不再保护
        pending_names = {
            name for name in subdirs
            if state[name]["status"] not in ("done", "syncing")
        }
        newest_name: str | None = None
        if pending_names:
            newest_name = max(pending_names, key=lambda n: state[n]["first_seen"])

        for name, subdir in subdirs.items():
            entry = state[name]

            if entry["status"] == "done":
                continue  # 已完成，永远跳过

            if entry["status"] == "syncing":
                continue  # 线程还在跑，不重复启动

            # status 是 "waiting" 或 "failed"
            age = now - entry["first_seen"]

            if age < delay:
                continue  # 还没到延迟阈值

            # 即使达到阈值，如果是当前最新出现的目录，跳过本轮
            if name == newest_name:
                # log.info("[%s] Skipped — newest subdir (age %ds)", name, int(age))
                continue

            entry["status"] = "syncing"
            t = threading.Thread(
                target=sync_subdir_thread,
                args=(subdir, local_dir, ssh_target, ssh_port, entry),
                daemon=True,
                name=f"sync-{name}",
            )
            t.start()
            log.info("[%s] Thread launched (age %ds)", name, int(age))

        # ── 2. 清理已消失的子目录 ─────────────────────────────────────────────
        for name in [n for n in list(state) if n not in subdirs]:
            if state[name]["status"] not in ("done", "syncing"):
                log.info("Subdir removed before sync: %s", name)
            del state[name]

        time.sleep(poll)


# ── 入口 ──────────────────────────────────────────────────────────────────────
def main() -> None:
    args = parse_args()
    try:
        watch(args.local_folder, args.ssh_target, args.poll, args.delay, args.port)
    except KeyboardInterrupt:
        log.info("Stopped by user.")


if __name__ == "__main__":
    main()