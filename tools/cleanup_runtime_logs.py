#!/usr/bin/env python3
"""Clean old GameUAV runtime logs without touching active log files."""

import argparse
import os
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]


@dataclass
class DeleteItem:
    path: Path
    size: int
    reason: str
    kind: str


def format_bytes(value):
    units = ["B", "KB", "MB", "GB"]
    number = float(value)
    for unit in units:
        if number < 1024 or unit == units[-1]:
            return f"{number:.1f} {unit}" if unit != "B" else f"{int(number)} B"
        number /= 1024
    return f"{value} B"


def path_size(path):
    if path.is_file() or path.is_symlink():
        try:
            return path.stat().st_size
        except OSError:
            return 0
    total = 0
    for root, _dirs, files in os.walk(path):
        for name in files:
            file_path = Path(root) / name
            try:
                total += file_path.stat().st_size
            except OSError:
                pass
    return total


def newest_mtime(path):
    newest = 0.0
    if path.is_file() or path.is_symlink():
        try:
            return path.stat().st_mtime
        except OSError:
            return 0.0
    for root, dirs, files in os.walk(path):
        for name in dirs + files:
            item = Path(root) / name
            try:
                newest = max(newest, item.stat().st_mtime)
            except OSError:
                pass
    try:
        newest = max(newest, path.stat().st_mtime)
    except OSError:
        pass
    return newest


def run_lsof(root):
    if not root.exists():
        return set()
    try:
        result = subprocess.run(
            ["lsof", "+D", str(root)],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            check=False,
            timeout=20,
        )
    except (OSError, subprocess.TimeoutExpired):
        return set()
    paths = set()
    for line in result.stdout.splitlines()[1:]:
        parts = line.split(None, 8)
        if len(parts) >= 9:
            name = parts[8]
            if " (deleted)" in name:
                name = name.replace(" (deleted)", "")
            paths.add(str(Path(name).resolve()))
    return paths


def is_open(path, open_paths):
    resolved = str(path.resolve())
    if resolved in open_paths:
        return True
    if path.is_dir():
        prefix = resolved.rstrip("/") + "/"
        return any(item.startswith(prefix) for item in open_paths)
    return False


def collect_open_paths(roots):
    open_paths = set()
    for root in roots:
        open_paths.update(run_lsof(root))
    return open_paths


def collect_old_log_files(log_root, keep_days, open_paths):
    cutoff = time.time() - keep_days * 86400
    items = []
    if not log_root.exists():
        return items
    for path in log_root.rglob("*"):
        if not path.is_file():
            continue
        if is_open(path, open_paths):
            continue
        try:
            stat = path.stat()
        except OSError:
            continue
        if stat.st_mtime < cutoff:
            items.append(DeleteItem(path, stat.st_size, f"mtime older than {keep_days:g} days", "file"))
    return items


def collect_old_ros_runs(ros_log_root, keep_runs, open_paths):
    if not ros_log_root.exists():
        return []
    run_dirs = [path for path in ros_log_root.iterdir() if path.is_dir()]
    run_dirs.sort(key=newest_mtime, reverse=True)
    items = []
    for path in run_dirs[keep_runs:]:
        if is_open(path, open_paths):
            continue
        items.append(DeleteItem(path, path_size(path), f"older ROS run dir beyond newest {keep_runs}", "dir"))
    return items


def delete_item(item):
    if item.kind == "dir":
        shutil.rmtree(item.path)
    else:
        item.path.unlink()


def remove_empty_dirs(root):
    if not root.exists():
        return 0
    removed = 0
    for path in sorted((item for item in root.rglob("*") if item.is_dir()), key=lambda p: len(p.parts), reverse=True):
        try:
            path.rmdir()
            removed += 1
        except OSError:
            pass
    return removed


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=WORKSPACE_ROOT)
    parser.add_argument("--ros-log-root", type=Path, default=Path("/tmp/gameuav_ros_logs"))
    parser.add_argument("--keep-days", type=float, default=2.0, help="Keep logs newer than this many days under logs/")
    parser.add_argument("--ros-keep-runs", type=int, default=1, help="Keep this many newest inactive ROS run dirs, plus any active dirs")
    parser.add_argument("--list-limit", type=int, default=80, help="Maximum candidates to print")
    parser.add_argument("--execute", action="store_true", help="Actually delete candidates; default is dry-run")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    workspace = args.workspace.resolve()
    log_root = workspace / "logs"
    runtime_root = workspace / "runtime"
    ros_log_root = args.ros_log_root.resolve()
    roots = [path for path in [log_root, runtime_root, ros_log_root] if path.exists()]

    open_paths = collect_open_paths(roots)
    items = []
    items.extend(collect_old_log_files(log_root, args.keep_days, open_paths))
    items.extend(collect_old_ros_runs(ros_log_root, args.ros_keep_runs, open_paths))
    items.sort(key=lambda item: item.size, reverse=True)

    mode = "EXECUTE" if args.execute else "DRY-RUN"
    print(f"mode: {mode}")
    print(f"workspace: {workspace}")
    print(f"protected_open_paths: {len(open_paths)}")
    print(f"delete_candidates: {len(items)}")
    total = sum(item.size for item in items)
    print(f"candidate_bytes: {format_bytes(total)}")
    for item in items[: args.list_limit]:
        print(f"{format_bytes(item.size):>10}  {item.kind:<4}  {item.path}  # {item.reason}")
    if len(items) > args.list_limit:
        print(f"... {len(items) - args.list_limit} more candidates omitted; use --list-limit to show more")

    if not args.execute:
        print("dry-run only; pass --execute to delete these candidates")
        return 0

    deleted = 0
    failed = 0
    for item in items:
        if is_open(item.path, open_paths):
            print(f"skip-open: {item.path}")
            continue
        try:
            delete_item(item)
            deleted += 1
        except OSError as exc:
            failed += 1
            print(f"failed: {item.path}: {exc}", file=sys.stderr)
    empty_dirs = remove_empty_dirs(log_root)
    print(f"deleted_items: {deleted}")
    print(f"failed_items: {failed}")
    print(f"empty_log_dirs_removed: {empty_dirs}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
