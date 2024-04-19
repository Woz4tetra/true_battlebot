import struct


class SerialHeader:
    C0 = b"b"
    C1 = b"w"


def to_serial_packet(packet: bytes) -> bytes:
    serial_length = struct.pack("<H", len(packet))
    return SerialHeader.C0 + SerialHeader.C1 + serial_length + packet
