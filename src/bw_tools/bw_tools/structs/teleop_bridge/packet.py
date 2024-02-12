from abc import ABC, abstractmethod
from ctypes import (
    Array,
    BigEndianStructure,
    LittleEndianStructure,
    Structure,
    c_double,
    c_float,
    c_int8,
    c_int16,
    c_int32,
    c_int64,
    c_uint8,
    c_uint16,
    c_uint32,
    c_uint64,
    sizeof,
)
from typing import List, Literal, Tuple, Type, TypeVar, Union

from bw_tools.structs.teleop_bridge.packet_type import PacketType

T = TypeVar("T", bound="Packet")

SupportedTypes = Type[
    Union[c_int8, c_uint8, c_uint16, c_int16, c_uint32, c_int32, c_uint64, c_int64, c_float, c_double, Array, Structure]
]


def build_struct(
    fields: List[Tuple[str, SupportedTypes]], byteorder: Literal["big", "little"] = "little"
) -> Type[Structure]:
    class LittleStruct(LittleEndianStructure):
        _pack_ = 1
        _fields_ = fields

    class BigStruct(BigEndianStructure):
        _pack_ = 1
        _fields_ = fields

    return LittleStruct if byteorder == "little" else BigStruct


class Packet(ABC):
    TYPE: PacketType
    STRUCT: type[Structure]

    def __init__(self, struct: Structure) -> None:
        self.struct = struct

    def to_bytes(self) -> bytes:
        return bytes(self.struct)

    @classmethod
    def from_bytes(cls: Type[T], data: bytes) -> T:
        return cls(cls.STRUCT.from_buffer_copy(data))

    @classmethod
    def sizeof(cls) -> int:
        return sizeof(cls.STRUCT)

    @classmethod
    @abstractmethod
    def from_values(cls: Type[T], *args) -> T:
        pass
