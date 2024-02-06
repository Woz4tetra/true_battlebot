#!/usr/bin/env python
import socket
import struct
import time
from typing import Callable, Dict, List, Optional, Type, TypeVar

import rospy
from bw_tools.structs.teleop_bridge.header import Header
from bw_tools.structs.teleop_bridge.motor_command import MotorCommand
from bw_tools.structs.teleop_bridge.motor_description import MotorDescription
from bw_tools.structs.teleop_bridge.packet import Packet
from bw_tools.structs.teleop_bridge.ping_info import PingInfo

BUFFER_SIZE = 512

T = TypeVar("T", bound=Packet)


def is_loopback(host: str):
    loopback_checker = {
        socket.AF_INET: lambda x: struct.unpack("!I", socket.inet_aton(x))[0] >> (32 - 8) == 127,
        socket.AF_INET6: lambda x: x == "::1",
    }
    for family in (socket.AF_INET, socket.AF_INET6):
        try:
            r = socket.getaddrinfo(host, None, family, socket.SOCK_STREAM)
        except socket.gaierror:
            return False
        for family, _, _, _, sockaddr in r:
            if not loopback_checker[family](sockaddr[0]):
                return False
    return True


class BridgeInterface:
    def __init__(
        self,
        broadcast_address: str,
        port: int,
        device_id: int,
        packet_callbacks: Dict[Type[T], Callable[[T], None]],
    ) -> None:
        self.port = port
        self.device_id = device_id
        self.broadcast_address = broadcast_address
        self.packet_type_to_class = {packet_cls.TYPE: packet_cls for packet_cls in packet_callbacks.keys()}
        self.packet_callbacks = packet_callbacks

        self.destination = ""
        self.blacklist_ips = set()
        self.whitelist_ips = set()

        self.socket = socket.socket(type=socket.SOCK_DGRAM)
        self.socket.bind(("0.0.0.0", self.port))
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.socket.setblocking(False)

        self.start_time = time.perf_counter()

    def send_packet(self, packet: bytes) -> None:
        if self.destination:
            rospy.logdebug(f"Sending packet to {self.destination}:{self.port}. {len(packet)} bytes. {packet}")
            try:
                self.socket.sendto(packet, (self.destination, self.port))
            except BlockingIOError as e:
                rospy.logwarn(f"Failed to send packet. {e}")

    def broadcast_packet(self, packet: bytes) -> None:
        rospy.logdebug(f"Broadcasting packet. {len(packet)} bytes. {packet}")
        try:
            self.socket.sendto(packet, (self.broadcast_address, self.port))
        except BlockingIOError as e:
            rospy.logwarn(f"Failed to broadcast packet. {e}")

    def send_command(self, commands: List[MotorCommand]) -> None:
        packet = MotorDescription.from_values(self.device_id, commands).to_bytes()
        self.send_packet(packet)

    def get_ping_time(self) -> float:
        return time.perf_counter() - self.start_time

    def send_ping(self) -> None:
        timestamp = self.get_ping_time()
        # Timer will loop at ~2.38 hours
        microseconds = int(timestamp * 1e6) & ((2 << 31) - 1)
        self.broadcast_packet(PingInfo.from_values(0, microseconds).to_bytes())

    def compute_latency(self, ping_info: PingInfo) -> float:
        timestamp = self.get_ping_time()
        return timestamp - ping_info.timestamp * 1e-6

    def is_address_ok(self, address: str, port: int) -> bool:
        if address in self.blacklist_ips:
            return False
        if address not in self.whitelist_ips:
            if is_loopback(address):
                self.blacklist_ips.add(address)
                return False
            else:
                self.whitelist_ips.add(address)

        if port != self.port:
            rospy.logwarn(f"Received packet from an unknown address: {address}:{port}")
            return False

        return True

    def parse_header(self, packet: bytes) -> Optional[Header]:
        read_size = len(packet)
        if read_size == 0:
            return None
        if read_size < Header.sizeof():
            rospy.logwarn("Received packet is too small to be valid")
            return None
        header = Header.from_bytes(packet)
        if header.device_id != self.device_id:
            rospy.logdebug(
                f"Received packet for a different device. Received {header.device_id} != expected {self.device_id}"
            )
            return None

        if header.size != read_size:
            rospy.logwarn(f"Received packet size does not match header. Expected: {header.size}. Read: {read_size}")
            return None

        return header

    def receive(self) -> None:
        try:
            (packet, (address, port)) = self.socket.recvfrom(BUFFER_SIZE)
        except BlockingIOError:
            return

        if not self.is_address_ok(address, port):
            return

        rospy.logdebug(f"Received packet from {address}:{port}. {len(packet)} bytes. {packet}")

        header = self.parse_header(packet)
        if not header:
            return
        self.destination = address

        packet_cls = self.packet_type_to_class[header.type]
        self.packet_callbacks[packet_cls](packet_cls.from_bytes(packet))
