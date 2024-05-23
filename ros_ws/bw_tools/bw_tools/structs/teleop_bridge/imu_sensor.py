from __future__ import annotations

from ctypes import c_float

from bw_shared.geometry.xyz import XYZ
from sensor_msgs.msg import Imu
from tf_conversions import transformations

from bw_tools.structs.teleop_bridge.header import Header
from bw_tools.structs.teleop_bridge.packet import Packet, build_struct
from bw_tools.structs.teleop_bridge.packet_type import PacketType

Vector3DStruct = build_struct(
    [
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
    ]
)


class ImuSensor(Packet):
    TYPE = PacketType.IMU
    STRUCT = build_struct(
        [
            ("header", Header.STRUCT),
            ("accel", Vector3DStruct),
            ("gyro", Vector3DStruct),
            ("orientation", Vector3DStruct),
        ]
    )

    @classmethod
    def from_values(cls, device_id: int, accel: XYZ, gyro: XYZ, orientation: XYZ) -> ImuSensor:
        return cls(
            cls.STRUCT(
                Header.from_values(cls.sizeof(), cls.TYPE, device_id).struct,
                Vector3DStruct(accel.x, accel.y, accel.z),
                Vector3DStruct(gyro.x, gyro.y, gyro.z),
                Vector3DStruct(orientation.x, orientation.y, orientation.z),
            )
        )

    @property
    def header(self) -> Header:
        return self.struct.header

    @property
    def accel(self) -> XYZ:
        return XYZ(self.struct.accel.x, self.struct.accel.y, self.struct.accel.z)

    @property
    def gyro(self) -> XYZ:
        return XYZ(self.struct.gyro.x, self.struct.gyro.y, self.struct.gyro.z)

    @property
    def orientation(self) -> XYZ:
        return XYZ(self.struct.orientation.x, self.struct.orientation.y, self.struct.orientation.z)

    def to_msg(self) -> Imu:
        msg = Imu()
        msg.linear_acceleration.x = self.accel.x
        msg.linear_acceleration.y = self.accel.y
        msg.linear_acceleration.z = self.accel.z
        msg.angular_velocity.x = self.gyro.x
        msg.angular_velocity.y = self.gyro.y
        msg.angular_velocity.z = self.gyro.z

        quaternion = transformations.quaternion_from_euler(self.orientation.x, self.orientation.y, self.orientation.z)
        msg.orientation.x = quaternion[0]
        msg.orientation.y = quaternion[1]
        msg.orientation.z = quaternion[2]
        msg.orientation.w = quaternion[3]

        return msg
