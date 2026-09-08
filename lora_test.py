"""Test two UART LoRa modules connected to the same PC (Python 3.9+)."""

import argparse
from contextlib import ExitStack
import logging
import math
import secrets
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    raise SystemExit("Missing pyserial. Run: python -m pip install -r requirements.txt")


def positive_int(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be greater than 0")
    return number


def positive_seconds(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be a finite number greater than 0")
    return number


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port-a", default="COM11")
    parser.add_argument("--port-b", default="COM27")
    parser.add_argument("--baud", type=positive_int, default=9600,
                        help="UART baud rate on both ports (default: 9600)")
    parser.add_argument("--parity", choices=("N", "E", "O"), default="N")
    parser.add_argument("--count", type=positive_int, default=5,
                        help="packets per direction (default: 5)")
    parser.add_argument("--timeout", type=positive_seconds, default=3.0,
                        help="receive timeout in seconds (default: 3)")
    parser.add_argument("--interval", type=positive_seconds, default=1.0,
                        help="pause after each transmission (default: 1)")
    parser.add_argument("--log", help="also save a UTF-8 log to this file")
    parser.add_argument("--list-ports", action="store_true")
    args = parser.parse_args()
    if not args.list_ports and args.port_a.strip().upper() == args.port_b.strip().upper():
        parser.error("port-a and port-b must be different")
    return args


def open_port(name, baud, parity):
    # Configure before opening; no RTS/CTS, DSR/DTR or XON/XOFF flow control.
    # M0/M1 are set on the hardware, not controlled by this script.
    port = serial.Serial(port=None, baudrate=baud, bytesize=serial.EIGHTBITS,
                         parity=parity, stopbits=serial.STOPBITS_ONE,
                         timeout=0, write_timeout=2,
                         xonxoff=False, rtscts=False, dsrdtr=False)
    port.dtr = False
    port.rts = False
    port.port = name
    port.open()
    return port


def transfer(source, destination, payload, timeout):
    """Match the complete unique payload across arbitrary UART read chunks."""
    destination.reset_input_buffer()
    logging.info("TX %s -> %s: %r", source.port, destination.port, payload)
    started = time.monotonic()
    written = source.write(payload)
    if written != len(payload):
        raise serial.SerialTimeoutException("incomplete serial write")
    # Do not flush(): some USB drivers can block indefinitely while draining.
    # Timing includes host/USB/UART buffering as well as the radio link.
    deadline = started + timeout
    received = bytearray()
    preview = bytearray()
    total_bytes = 0
    while time.monotonic() < deadline:
        chunk = destination.read(min(destination.in_waiting, 4096))
        if chunk:
            total_bytes += len(chunk)
            preview.extend(chunk[:max(0, 96 - len(preview))])
            received.extend(chunk)
            if payload in received:
                elapsed_ms = (time.monotonic() - started) * 1000
                logging.info("PASS %s -> %s: matched %d bytes, %.1f ms",
                             source.port, destination.port, len(payload), elapsed_ms)
                return elapsed_ms
            # Retain enough tail for a payload split across two reads.
            received = received[-len(payload):]
        else:
            time.sleep(0.01)
    logging.warning("FAIL %s -> %s: timeout %.1f s; received %d bytes; preview=%r; hex=%s",
                    source.port, destination.port, timeout, total_bytes,
                    bytes(preview), preview.hex(" ") or "(empty)")
    return None


def main():
    args = arguments()
    if args.list_ports:
        for port in sorted(list_ports.comports()):
            print(f"{port.device}: {port.description}")
        return 0

    handlers = [logging.StreamHandler(sys.stdout)]
    try:
        if args.log:
            handlers.append(logging.FileHandler(args.log, encoding="utf-8"))
    except OSError as error:
        print(f"Cannot open log file: {error}", file=sys.stderr)
        return 2
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s",
                        datefmt="%Y-%m-%d %H:%M:%S", handlers=handlers)
    logging.info("E22 test: A=%s B=%s, UART=%d 8%s1, %d packets/direction",
                 args.port_a, args.port_b, args.baud, args.parity, args.count)
    logging.info("Requires compatible radio settings and transparent mode on both modules.")
    session = secrets.token_hex(4)
    results = {"A->B": [], "B->A": []}

    try:
        with ExitStack() as stack:
            a = stack.enter_context(open_port(args.port_a, args.baud, args.parity))
            b = stack.enter_context(open_port(args.port_b, args.baud, args.parity))
            time.sleep(1.0)
            a.reset_input_buffer()
            b.reset_input_buffer()
            for sequence in range(1, args.count + 1):
                # Half duplex: finish one direction before transmitting back.
                for source, destination, direction in ((a, b, "A->B"), (b, a, "B->A")):
                    # A short probe fits the minimum 32-byte radio packet size
                    # for ordinary test counts, even if packet RSSI is enabled.
                    payload = f"E22|{session}|{direction}|{sequence:04d}\r\n".encode("ascii")
                    elapsed = transfer(source, destination, payload, args.timeout)
                    results[direction].append(elapsed)
                    time.sleep(args.interval)
    except (serial.SerialException, OSError) as error:
        logging.error("Serial error: %s", error)
        logging.error("Check USB connections and close other programs using these ports.")
        return 2
    except KeyboardInterrupt:
        logging.info("Stopped by user; ports closed.")
        return 130

    all_passed = True
    for direction, samples in results.items():
        successful = [sample for sample in samples if sample is not None]
        lost = len(samples) - len(successful)
        all_passed = all_passed and lost == 0
        average = f"{sum(successful) / len(successful):.1f} ms" if successful else "n/a"
        logging.info("SUMMARY %s: %d/%d passed, loss=%.1f%%, average=%s",
                     direction, len(successful), len(samples),
                     100 * lost / len(samples), average)
    logging.info("RESULT: %s", "PASS - both directions passed all packets" if all_passed
                 else "FAIL - some packets were not received intact before the timeout")
    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
