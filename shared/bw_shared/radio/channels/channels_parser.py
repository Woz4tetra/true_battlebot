import struct
from typing import Optional


class ChannelStreamingParser:
    def __init__(self):
        self.sync_sequence = bytes([0xA3, 0xA4, 0xA5])
        self.max_channels = 32
        self.channels_per_packet = 16
        self.buffer = bytearray()
        self.last_channels = [0] * self.max_channels
        self.partial_channels = [None, None]  # Store partial channel sets

    def process_data(self, data: bytes) -> list[list[int]]:
        """
        Process incoming serial data and return list of channel arrays. Return any excess bytes that do not form a
        complete packet.
        """
        self.buffer.extend(data)

        channel_updates = []

        while len(self.buffer) >= len(self.sync_sequence) + 2:  # Minimum packet size
            # Look for sync sequence (0xA3 0xA4 0xA5)
            sync_index = -1
            for i in range(len(self.buffer) - len(self.sync_sequence) + 1):
                if self.buffer[i : i + len(self.sync_sequence)] == self.sync_sequence:
                    sync_index = i
                    break

            if sync_index == -1:
                # No sync sequence found
                self.buffer = bytearray()
                return channel_updates

            # Remove data before sync sequence
            if sync_index > 0:
                self.buffer = self.buffer[sync_index:]

            # Check if we have enough data for phase and length bytes
            if len(self.buffer) < len(self.sync_sequence) + 2:
                return channel_updates

            phase = self.buffer[len(self.sync_sequence)]
            packet_length = self.buffer[len(self.sync_sequence) + 1]

            # Check if we have the complete packet
            total_packet_size = len(self.sync_sequence) + 2 + packet_length
            if len(self.buffer) < total_packet_size:
                return channel_updates

            # Extract packet data (phase + length + data + checksum)
            packet_data = self.buffer[len(self.sync_sequence) : total_packet_size]

            # Remove processed packet from buffer
            self.buffer = self.buffer[total_packet_size:]

            # Parse the channel packet
            channels = self._parse_channel_packet_phase(packet_data)
            if channels is not None:
                # Check if we have a complete set of channels
                complete_channels = self._update_channels_from_phase(phase, channels)
                if complete_channels is not None:
                    channel_updates.append(complete_channels)

        return channel_updates

    def _parse_channel_packet_phase(self, packet_data: bytes) -> Optional[list[int]]:
        """Parse phase-based channel data packet"""
        if len(packet_data) < 4:  # phase + length + at least 2 bytes data
            return None

        phase = packet_data[0]
        length = packet_data[1]

        # Expected: 16 channels * 2 bytes + 1 checksum = 33 bytes
        expected_length = (self.channels_per_packet * 2) + 1
        if length != expected_length:
            print(f"Invalid packet length for phase {phase}: got {length}, expected {expected_length}")
            return None

        if len(packet_data) < 2 + length:
            return None

        # Calculate expected checksum (phase XOR length XOR all data bytes)
        expected_checksum = phase ^ length
        for i in range(2, len(packet_data) - 1):
            expected_checksum ^= packet_data[i]

        received_checksum = packet_data[-1]

        if expected_checksum != received_checksum:
            print(
                f"Checksum mismatch in phase {phase}: expected 0x{expected_checksum:02X}, got 0x{received_checksum:02X}"
            )
            return None

        # Extract channel data (skip phase, length, and checksum)
        channel_data = packet_data[2:-1]

        if len(channel_data) != self.channels_per_packet * 2:
            print(
                f"Invalid channel data length in phase {phase}: {len(channel_data)}, "
                f"expected {self.channels_per_packet * 2}"
            )
            return None

        # Unpack 16-bit little-endian signed integers
        channels = []
        for i in range(0, len(channel_data), 2):
            value = struct.unpack("<h", channel_data[i : i + 2])[0]
            channels.append(value)

        return channels

    def _update_channels_from_phase(self, phase: int, channels: list[int]) -> Optional[list[int]]:
        """Update channel array based on phase and return complete set if both phases received"""
        if phase == 0:
            self.partial_channels[0] = channels.copy()
        elif phase == 1:
            self.partial_channels[1] = channels.copy()
        else:
            print(f"Invalid phase: {phase}")
            return None

        # Check if we have both phases
        if self.partial_channels[0] is not None and self.partial_channels[1] is not None:
            complete_channels = self.partial_channels[0] + self.partial_channels[1]
            # Reset partial channels for next cycle
            self.partial_channels = [None, None]
            return complete_channels

        return None
