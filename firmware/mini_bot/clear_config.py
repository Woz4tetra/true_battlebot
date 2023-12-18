import argparse
import time
from typing import Protocol

import serial


class CommandLineArgs(Protocol):
    device: str


def main() -> None:
    parser = argparse.ArgumentParser(description="Configure a Mini Bot")
    parser.add_argument("config", default="./config.json", nargs="?", help="JSON configuration file")
    parser.add_argument("-d", "--device", default="/dev/ttyACM0", help="Serial device")
    args: CommandLineArgs = parser.parse_args()  # type: ignore

    baud = 115200
    device = serial.Serial(args.device, baudrate=baud)

    print("Clearing device config.")
    device.write(b"clear\n")
    time.sleep(0.1)  # Ensure buffer is sent
    device.close()
    time.sleep(1.0)
    while True:
        try:
            device = serial.Serial(args.device, baudrate=baud)
            break
        except (serial.SerialException, OSError):
            time.sleep(0.1)
    print("Device rebooted. Press Ctrl-C to exit.")
    while True:
        print(device.read(1).decode(), end="")


if __name__ == "__main__":
    main()
