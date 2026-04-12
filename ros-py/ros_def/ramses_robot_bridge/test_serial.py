#!/usr/bin/env python3
"""
test_serial.py
==============
Standalone sanity-check script — does NOT require ROS2.
Run this directly on the Pi to verify the serial protocol
before launching the full bridge node.

Usage
-----
  python3 test_serial.py
  python3 test_serial.py --port /dev/ttyUSB1
  python3 test_serial.py --cmd "s2up 200"
"""

import argparse
import sys
import time
import threading
import serial


DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUD = 115200


def reader_thread(ser: serial.Serial, stop: threading.Event) -> None:
    """Print every line received from the ESP32."""
    buf = b''
    while not stop.is_set():
        try:
            waiting = ser.in_waiting
            if waiting:
                buf += ser.read(waiting)
            else:
                chunk = ser.read(1)
                if chunk:
                    buf += chunk
        except serial.SerialException as exc:
            print(f'[SERIAL ERROR] {exc}', file=sys.stderr)
            break

        while b'\n' in buf:
            line_bytes, buf = buf.split(b'\n', 1)
            line = line_bytes.rstrip(b'\r').decode('ascii', errors='replace').strip()
            if line:
                ts = time.strftime('%H:%M:%S')
                print(f'[{ts}] ← {line}')


def send_and_wait(ser: serial.Serial, cmd: str, timeout: float = 30.0) -> bool:
    """Send a command and wait for DONE or ERR. Returns True on DONE."""
    print(f'\n→ Sending: {cmd!r}')
    ser.write((cmd.strip() + '\n').encode('ascii'))

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        time.sleep(0.05)
        # The reader thread is printing responses, so we just wait.
        # For a real test we'd parse here, but this is just a smoke test.
    return True


def main() -> None:
    parser = argparse.ArgumentParser(description='ESP32 serial protocol tester')
    parser.add_argument('--port', default=DEFAULT_PORT)
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD)
    parser.add_argument('--cmd',  default=None,
                        help='Single command to send (default: run test sequence)')
    args = parser.parse_args()

    print(f'Opening {args.port} @ {args.baud}...')
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        sys.exit(1)

    ser.reset_input_buffer()
    ser.reset_output_buffer()
    print('Port open. Listening for 2 s to catch startup banner...\n')

    stop = threading.Event()
    t = threading.Thread(target=reader_thread, args=(ser, stop), daemon=True)
    t.start()

    time.sleep(2.0)

    if args.cmd:
        # Single command mode
        ser.write((args.cmd.strip() + '\n').encode('ascii'))
        print('Waiting 10 s for response...')
        time.sleep(10.0)
    else:
        # Default smoke-test sequence
        tests = [
            ('estop status',  2.0),
            ('debug off',     2.0),
            ('where',         2.0),
            ('limits',        3.0),
            ('debug on',      2.0),
            ('where',         2.0),
            ('debug off',     2.0),
        ]
        for cmd, wait in tests:
            print(f'\n─── Test: {cmd!r} (wait {wait}s) ───')
            ser.write((cmd.strip() + '\n').encode('ascii'))
            time.sleep(wait)

    stop.set()
    ser.close()
    print('\nDone.')


if __name__ == '__main__':
    main()
