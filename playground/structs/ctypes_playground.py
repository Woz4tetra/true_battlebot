import copy
import ctypes
from ctypes import Structure, c_uint8, c_uint16, sizeof
from enum import IntEnum
from typing import Any, Callable, Optional, Protocol, Type, TypeVar, Union


class HeaderType(IntEnum):
    MOTOR = 1
    PING = 2
    CONFIG = 3


T = TypeVar("T", bound="Structified")


class Structified(Protocol):
    _struct: Structure


def from_bytes(cls: Type[T], packet: bytes) -> T:
    struct = cls._struct.__class__.from_buffer_copy(packet[: sizeof(cls._struct)])
    for name, _ in cls._struct._fields_:
        setattr(cls, name, getattr(struct, name))
    return cls()


def to_bytes(cls: T) -> bytes:
    struct = copy.deepcopy(cls._struct)
    for name, _ in struct._fields_:
        setattr(struct, name, getattr(cls, name))
    return bytes(struct)


def is_ctypes_type(obj) -> bool:
    # noinspection PyProtectedMember
    return issubclass(obj, (ctypes._SimpleCData, ctypes.Structure, ctypes.Union, ctypes.Array))


def _process_class(cls):
    cls_annotations = cls.__dict__.get("__annotations__", {})

    class Struct(Structure):
        _fields_ = [(name, value) for name, value in cls_annotations.items() if is_ctypes_type(value)]

    cls._struct = Struct()

    cls.from_bytes = classmethod(from_bytes)
    cls.to_bytes = to_bytes

    return cls


S = TypeVar("S")


def cstruct(cls: Optional[Type[S]] = None) -> Union[S, Callable[[Type[S]], S]]:
    def wrap(cls):
        return _process_class(cls)

    # See if we're being called as @cstruct or @cstruct().
    if cls is None:
        # We're called with parens.
        return wrap

    # We're called as @cstruct without parens.
    return wrap(cls)


@cstruct
class Header:
    device_id: c_uint8
    type: c_uint8
    size: c_uint16


@cstruct
class MotorCommand:
    channel: c_uint8
    speed: c_uint8


header = from_bytes(Header, b"1\x0134")
print(header.device_id)
print(header.type)
print(header.size)
print(header.to_bytes())

header = Header.from_bytes(b"1\x0135")
print(header.to_bytes())

motor = MotorCommand.from_bytes(b"56")
print(motor.channel)
print(motor.speed)
print(motor.to_bytes())
