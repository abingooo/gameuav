#!/usr/bin/env python3

import argparse
import sys
import time
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

from comm.udp_link.link import UdpLink


def main(argv=None):
    parser = argparse.ArgumentParser(description="Send test UDP heartbeat/state messages.")
    parser.add_argument("--source-id", default="uav1")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9001)
    parser.add_argument("--count", type=int, default=5)
    parser.add_argument("--interval", type=float, default=0.5)
    args = parser.parse_args(argv)

    link = UdpLink("0.0.0.0", 0)
    try:
        for index in range(args.count):
            link.send_heartbeat(
                args.source_id,
                args.host,
                args.port,
                payload={"seq": index},
            )
            link.send_state(
                args.source_id,
                args.host,
                args.port,
                state={
                    "uav_id": args.source_id,
                    "position": [float(index), 0.0, 1.0],
                    "velocity": [0.0, 0.0, 0.0],
                    "battery": {"percentage": 0.8, "voltage": 16.0},
                },
            )
            time.sleep(args.interval)
    finally:
        link.close()


if __name__ == "__main__":
    main()
