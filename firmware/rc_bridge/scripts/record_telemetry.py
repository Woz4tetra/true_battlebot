import argparse
import json
import socket
import time
from datetime import datetime
from typing import List


def make_udp_server(ip: str, port: int) -> socket.socket:
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind((ip, port))
    server.setblocking(False)
    return server


def main() -> None:
    now = datetime.now()
    timestamp = now.strftime("%Y-%m-%d--%H-%M-%S")
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--output", type=str, default=f"{timestamp}.jsonl")
    parser.add_argument("--port", type=int, default=4176)
    args = parser.parse_args()
    udp = make_udp_server("0.0.0.0", args.port)
    out_path = args.output
    write_frequency = 20
    packets: List[bytes] = []

    try:
        while True:
            while True:
                try:
                    packet, _ = udp.recvfrom(1024)
                    packets.append(packet)
                except BlockingIOError:
                    break
            if len(packets) > write_frequency:
                with open(out_path, "a") as file:
                    for packet in packets:
                        decoded_packet = packet.decode("utf-8")
                        try:
                            data = json.loads(decoded_packet)
                        except json.JSONDecodeError:
                            print(f"Failed to decode: {decoded_packet}")
                            continue
                        data["local_time"] = time.time()
                        file.write(json.dumps(data) + "\n")
                        print(decoded_packet)
                packets.clear()

    finally:
        udp.close()


if __name__ == "__main__":
    main()
