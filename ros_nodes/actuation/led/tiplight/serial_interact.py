#!/usr/bin/env python3
import argparse
import select
import signal
import sys
import termios
import time
import tty

import serial


DEFAULT_PORT = "/dev/gameuav_tiplight"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read from and write to an Espressif USB serial device."
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
        "--send",
        help="send one message, read briefly, then exit",
    )
    parser.add_argument(
        "--newline",
        choices=("none", "lf", "crlf"),
        default="lf",
        help="line ending appended to --send and Enter in interactive mode",
    )
    parser.add_argument(
        "--read-seconds",
        type=float,
        default=1.0,
        help="seconds to keep reading after --send, default: 1.0",
    )
    return parser.parse_args()


def encode_newline(kind):
    if kind == "lf":
        return b"\n"
    if kind == "crlf":
        return b"\r\n"
    return b""


def open_port(path, baud):
    return serial.Serial(
        port=path,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0,
        write_timeout=1,
    )


def read_for(ser, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        data = ser.read(ser.in_waiting or 1)
        if data:
            sys.stdout.buffer.write(data)
            sys.stdout.buffer.flush()
        else:
            time.sleep(0.01)


def send_once(ser, message, newline, read_seconds):
    ser.write(message.encode("utf-8") + encode_newline(newline))
    ser.flush()
    read_for(ser, read_seconds)


def interactive(ser, newline):
    stdin_fd = sys.stdin.fileno()
    old_stdin_attrs = termios.tcgetattr(stdin_fd)
    line_ending = encode_newline(newline)
    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    print("Connected. Type to send. Ctrl-C exits.", file=sys.stderr)
    try:
        tty.setraw(stdin_fd)
        while running:
            readable, _, _ = select.select([stdin_fd], [], [], 0.02)

            data = ser.read(ser.in_waiting or 1)
            if data:
                sys.stdout.buffer.write(data)
                sys.stdout.buffer.flush()

            if stdin_fd in readable:
                typed = sys.stdin.buffer.read1(1024)
                if not typed or typed == b"\x03":
                    break
                if typed in (b"\r", b"\n"):
                    ser.write(line_ending)
                    sys.stdout.buffer.write(b"\r\n")
                else:
                    ser.write(typed)
                ser.flush()
    finally:
        termios.tcsetattr(stdin_fd, termios.TCSANOW, old_stdin_attrs)
        print("\nDisconnected.", file=sys.stderr)


def main():
    args = parse_args()

    try:
        with open_port(args.port, args.baud) as ser:
            if args.send is not None:
                send_once(ser, args.send, args.newline, args.read_seconds)
            else:
                interactive(ser, args.newline)
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        return 1
    except OSError as exc:
        print(f"Terminal error: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
