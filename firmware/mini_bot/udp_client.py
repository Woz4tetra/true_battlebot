import socket
import time

from bw_tools.structs.teleop_bridge.config import Config
from bw_tools.structs.teleop_bridge.motor_command import MotorCommand
from bw_tools.structs.teleop_bridge.motor_description import MotorDescription

sock = socket.socket(type=socket.SOCK_DGRAM)
# sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.bind(("0.0.0.0", 4176))


def motor_demo():
    command = 0
    prev_delta = time.perf_counter()
    while True:
        packet = MotorDescription(1, [MotorCommand(1, command), MotorCommand(1, command)]).as_bytes()
        if time.perf_counter() - prev_delta > 0.5:
            prev_delta = time.perf_counter()
            command = 255 if command != 255 else 0
        sock.sendto(packet, ("192.168.8.187", 4176))
        # sock.sendto(packet, ("192.168.8.255", 4176))
        time.sleep(0.02)


def config_demo():
    config = Config(ssid="HavocNet", port=4176, device_id=1, password="something")
    packet = config.as_bytes()
    sock.sendto(packet, ("192.168.8.187", 4176))
    time.sleep(0.1)


if __name__ == "__main__":
    config_demo()
    # motor_demo()
