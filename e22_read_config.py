"""Read E22 UART registers locally. Hardware must be in mode 2: M1=1, M0=0."""

import argparse
import logging
import sys
import time

from lora_test import open_port, positive_seconds, serial


def read_registers(port, start, length, timeout):
    """Send C1 only; accept a complete matching response, including split reads."""
    command = bytes((0xC1, start, length))
    port.reset_input_buffer()
    logging.info("%s TX: %s", port.port, command.hex(" "))
    if port.write(command) != len(command):
        raise serial.SerialTimeoutException("incomplete serial write")
    deadline = time.monotonic() + timeout
    buffer = bytearray()
    preview = bytearray()
    while time.monotonic() < deadline:
        chunk = port.read(min(port.in_waiting, 4096))
        if chunk:
            preview.extend(chunk[:max(0, 96 - len(preview))])
            buffer.extend(chunk)
            position = buffer.find(command)
            if position >= 0 and len(buffer) >= position + 3 + length:
                response = bytes(buffer[position:position + 3 + length])
                logging.info("%s RX: %s", port.port, response.hex(" "))
                return response[3:]
            buffer = buffer[-(3 + length):]
        else:
            time.sleep(0.01)
    raise TimeoutError(f"{port.port}: no complete C1 response; RX={preview.hex(' ') or '(empty)'}")


def decode_config(data):
    if len(data) != 7:
        raise ValueError("expected registers 00 through 06 (7 bytes)")
    speed, options, channel, features = data[3:7]
    return {
        "address": f"0x{int.from_bytes(data[:2], 'big'):04X}",
        "network_id": data[2],
        "uart_baud": (1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200)[speed >> 5],
        "uart_format": ("8N1", "8O1", "8E1", "8N1")[(speed >> 3) & 3],
        # Low air-rate code mappings differ across firmware generations.
        "air_rate_code": speed & 7,
        "packet_bytes": (240, 128, 64, 32)[options >> 6],
        "power_code": options & 3,
        "channel": channel,
        "frequency_MHz_for_E22_400": f"{410.125 + channel:.3f}",
        "transmission": "fixed/addressed" if features & 0x40 else "transparent",
        "packet_rssi": bool(features & 0x80),
        "relay": bool(features & 0x20),
        "lbt": bool(features & 0x10),
        "wor_transmitter": bool(features & 8),
        "wor_period_ms": ((features & 7) + 1) * 500,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ports", nargs="+", default=["COM11", "COM27"])
    parser.add_argument("--timeout", type=positive_seconds, default=3.0)
    parser.add_argument("--log", help="append a UTF-8 diagnostic log")
    args = parser.parse_args()
    if len(set(port.upper() for port in args.ports)) != len(args.ports):
        parser.error("duplicate ports")
    handlers = [logging.StreamHandler(sys.stdout)]
    try:
        if args.log:
            handlers.append(logging.FileHandler(args.log, encoding="utf-8"))
    except OSError as error:
        print(f"Cannot open log file: {error}", file=sys.stderr)
        return 2
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s",
                        datefmt="%Y-%m-%d %H:%M:%S", handlers=handlers)
    logging.info("READ ONLY. Requires M1=1, M0=0. Configuration UART: 9600 8N1.")
    logging.info("This checks local PC-to-module UART, not the radio link.")
    configs = {}
    status = 0
    try:
        for name in args.ports:
            try:
                with open_port(name, 9600, "N") as port:
                    time.sleep(1.0)
                    data = read_registers(port, 0, 7, args.timeout)
                    config = decode_config(data)
                    configs[name] = config
                    for field, value in config.items():
                        logging.info("%s %-30s %s", name, field, value)
                    try:
                        product = read_registers(port, 0x80, 7, args.timeout)
                        logging.info("%s product/version raw: %s", name, product.hex(" "))
                    except TimeoutError as error:
                        logging.warning("Optional product information unavailable: %s", error)
            except (serial.SerialException, OSError, TimeoutError) as error:
                status = 1
                logging.error("%s", error)
                logging.error("Check mode 2 jumpers, VCC and USB/module connections; close RF_Setting.")
        if len(configs) > 1:
            first_name, first = next(iter(configs.items()))
            for name, config in list(configs.items())[1:]:
                differences = [key for key in first if first[key] != config[key]]
                if differences:
                    for key in differences:
                        logging.warning("DIFF %s: %s=%s, %s=%s", key, first_name, first[key], name, config[key])
                else:
                    logging.info("Readable settings match: %s and %s", first_name, name)
        logging.info("Air-rate codes need the matching firmware manual; default code 2 is 2.4 kbps.")
        logging.info("Encryption keys are write-only; identical readable settings do not prove keys match.")
        logging.info("Exit 0 means requested configuration reads succeeded, not radio communication success.")
        logging.info("Restore M1=0, M0=0 on both modules before running lora_test.py.")
    except KeyboardInterrupt:
        logging.info("Stopped; serial ports closed.")
        return 130
    return status


if __name__ == "__main__":
    sys.exit(main())
