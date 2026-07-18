#!/usr/bin/env python3
import argparse
import sys
import time

import serial


DEFAULT_PORT = "/dev/gameuav_tiplight"


MODE_COMMANDS = {
    "1": "1",
    "default": "1",
    "2": "2",
    "ready": "2",
    "3": "3",
    "takeoff": "3",
    "4": "4",
    "hover": "4",
    "5": "5",
    "game": "5",
    "6": "6",
    "defense": "6",
    "7": "7",
    "enemy": "7",
    "8": "8",
    "abort": "8",
    "n": "n",
    "next": "n",
    "p": "p",
    "prev": "p",
    "previous": "p",
    "?": "?",
    "status": "?",
    "help": "?",
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Send drone status light commands to the ESP32 LED controller."
    )
    parser.add_argument(
        "command",
        choices=sorted(MODE_COMMANDS),
        help=(
            "1/default, 2/ready, 3/takeoff, 4/hover, 5/game, "
            "6/defense, 7/enemy, 8/abort, next, prev, status"
        ),
    )
    parser.add_argument(
        "-p",
        "--port",
        default=DEFAULT_PORT,
        help=f"serial device path, default: {DEFAULT_PORT}",
    )
    parser.add_argument(
        "-b",
        "--baud",
        type=int,
        default=115200,
        help="baud rate, default: 115200",
    )
    parser.add_argument(
        "--read-seconds",
        type=float,
        default=1.0,
        help="seconds to read ESP32 response after sending, default: 1.0",
    )
    return parser.parse_args()


def read_response(ser, seconds):
    deadline = time.monotonic() + seconds
    chunks = []

    while time.monotonic() < deadline:
        data = ser.read(ser.in_waiting or 1)
        if data:
            chunks.append(data)
        else:
            time.sleep(0.02)

    return b"".join(chunks)


def main():
    args = parse_args()
    command = MODE_COMMANDS[args.command.lower()]

    try:
        with serial.Serial(args.port, args.baud, timeout=0, write_timeout=1) as ser:
            ser.reset_input_buffer()
            ser.write(command.encode("ascii"))
            ser.flush()
            response = read_response(ser, args.read_seconds)
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        return 1

    if response:
        print(response.decode("utf-8", errors="replace"), end="")
    else:
        print("No response received.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
