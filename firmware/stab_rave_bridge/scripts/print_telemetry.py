#!/usr/bin/env python
import argparse
import json
import socket


def make_udp_server(ip: str, port: int) -> socket.socket:
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind((ip, port))
    server.setblocking(False)
    return server


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=4176)
    args = parser.parse_args()
    udp = make_udp_server("0.0.0.0", args.port)

    try:
        while True:
            try:
                packet, _ = udp.recvfrom(1024)
            except BlockingIOError:
                continue
            decoded_packet = packet.decode("utf-8")
            data = json.loads(decoded_packet)
            left_command = data.get("left_command", 0.0)
            gyro_z = data.get("gyro", {}).get("z")
            print(left_command, gyro_z)

    finally:
        udp.close()


if __name__ == "__main__":
    main()
