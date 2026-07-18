#!/usr/bin/env python3
import os
import subprocess
import sys


def main() -> int:
    if len(sys.argv) < 2:
        print("[realsense_prelaunch] missing command to exec", file=sys.stderr)
        return 2

    if os.environ.get("GAMEUAV_REALSENSE_PRELAUNCH_RESET", "1") != "0":
        cmd = [
            "rosrun",
            "gameuav_usb_camera",
            "realsense_hw_reset",
            "--find-timeout",
            "8",
            "--settle-timeout",
            "7",
        ]
        serial = os.environ.get("GAMEUAV_REALSENSE_SERIAL", "")
        if serial:
            cmd.extend(["--serial", serial])

        print("[realsense_prelaunch] hardware reset before starting realsense nodelet", flush=True)
        try:
            completed = subprocess.run(cmd, check=False)
            if completed.returncode != 0:
                print(
                    f"[realsense_prelaunch] reset helper failed with code {completed.returncode}; "
                    "continuing with nodelet startup",
                    file=sys.stderr,
                    flush=True,
                )
        except Exception as exc:
            print(
                f"[realsense_prelaunch] reset helper failed: {exc}; continuing with nodelet startup",
                file=sys.stderr,
                flush=True,
            )

    os.execvp(sys.argv[1], sys.argv[1:])
    return 127


if __name__ == "__main__":
    raise SystemExit(main())
