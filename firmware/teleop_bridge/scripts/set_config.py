#!/usr/bin/env python
import argparse
import getpass
import json
import struct
import time
from typing import Protocol, cast

import serial
from bw_tools.structs.teleop_bridge.config import Config
from bw_tools.structs.teleop_bridge.header import MAX_PACKET_SIZE
from bw_tools.structs.teleop_bridge.serial_packet import SerialHeader


class CommandLineArgs(Protocol):
    config: str
    device: str


def wait_for_response(device: serial.Serial) -> bytes:
    received_length = 0
    while received_length == 0:
        time.sleep(0.001)
        c0 = device.read(1)
        if c0 == SerialHeader.C0:
            c1 = device.read(1)
            if c1 == SerialHeader.C1:
                received_length = struct.unpack("<H", device.read(2))[0]
                if received_length > MAX_PACKET_SIZE:
                    print(f"Received length ({received_length}) is greater than max packet size ({MAX_PACKET_SIZE})")
                    break
            else:
                print(c0.decode(), end="")
                print(c1.decode(), end="")
        else:
            print(c0.decode(), end="")
    if received_length == 0:
        raise ValueError("Failed to get response")
    received_data = device.read(received_length)
    return received_data


def main() -> None:
    parser = argparse.ArgumentParser(description="Configure a Mini Bot")
    parser.add_argument("config", default="./config.json", nargs="?", help="JSON configuration file")
    parser.add_argument("-d", "--device", default="/dev/ttyACM0", help="Serial device")
    args = cast(CommandLineArgs, parser.parse_args())

    baud = 115200

    with open(args.config, "r") as f:
        config = Config.from_dict(json.load(f))

    password = None
    while password is None:
        password = getpass.getpass(f"Enter password for {config.ssid}: ")
        config.password = password
    data = config.to_serial_bytes()
    device = serial.Serial(args.device, baud)

    device.write(data)
    print(f"Sent {len(data)} bytes to {args.device}. Waiting for response.")
    received_data = wait_for_response(device)

    try:
        received_config = Config.from_bytes(received_data)
    except ValueError as e:
        print(f"Failed to parse received data: {e}. Received data: {received_data}")
        while True:
            print(device.read(1).decode(), end="")

    if received_config.ssid != config.ssid:
        print(f"Received SSID ({received_config.ssid}) does not match sent SSID ({config.ssid})")
        received_dict = received_config.to_dict()
        config_dict = config.to_dict()
        for key in config_dict:
            if config_dict.get(key, None) != received_dict.get(key, None):
                print(f"\t{key}: {received_dict[key]} != {config_dict[key]}")
    print("Configuration complete.")


if __name__ == "__main__":
    main()
