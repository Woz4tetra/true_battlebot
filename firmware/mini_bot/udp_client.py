import socket
import time

from bw_teleop.structs import MotorCommand, MotorDescription

sock = socket.socket(type=socket.SOCK_DGRAM)
# sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.bind(("0.0.0.0", 4176))
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
