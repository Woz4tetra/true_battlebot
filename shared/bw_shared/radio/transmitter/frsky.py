from typing import Optional

import serial
from bw_shared.radio.crsf.crsf_packet import CrsfPacket
from bw_shared.radio.crsf.crsf_parser import CrsfParser
from serial.tools.list_ports import comports


def find_transmitter() -> serial.Serial:
    for port in comports():
        if port.pid == 22336 and port.vid == 1155:
            return serial.Serial(port.device, 115200)
    raise RuntimeError("Transmitter not found")


class FrSkyTransmitter:
    def __init__(self) -> None:
        self.device: Optional[serial.Serial] = None
        self.max_command = 1000
        self.min_command = -1000
        self.command = self._make_command(0.0, 0.0)
        self.parser = CrsfParser()

    def open(self) -> None:
        self.device = find_transmitter()

    def _write(self, data: bytes) -> None:
        if self.device is None:
            raise RuntimeError("Device not connected. Call connect() first.")
        self.device.write(data)

    def set_telemetry(self, telemetry: bool) -> None:
        if telemetry:
            self._write(b"telemetry on\r\n")
        else:
            self._write(b"telemetry off\r\n")

    def _make_command(self, linear_x: float, angular_z: float) -> tuple[bytes, bytes]:
        linear_value = int(self.max_command * linear_x)
        angular_value = int(self.max_command * angular_z)
        linear_value = max(self.min_command, min(self.max_command, linear_value))
        angular_value = max(self.min_command, min(self.max_command, angular_value))
        linear_command = f"trainer 3 {linear_value}\r\n"
        rotate_command = f"trainer 0 {angular_value}\r\n"
        return linear_command.encode(), rotate_command.encode()

    def set_command(self, linear_x: float, angular_z: float) -> None:
        self.command = self._make_command(linear_x, angular_z)

    def read(self) -> list[tuple[CrsfPacket, str]]:
        if self.device is None:
            raise RuntimeError("Device not connected. Call connect() first.")
        response = self.device.read_all()
        if not response:
            return []

        return self.parser.parse(response)

    def write(self) -> None:
        for command in self.command:
            if command:
                self._write(command)

    def close(self) -> None:
        if self.device is not None:
            self.device.close()
            self.device = None
