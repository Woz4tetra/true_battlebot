from dataclasses import asdict
from enum import Enum
from typing import Type, TypeVar

from dacite import Config
from dacite import from_dict as from_dict_dacite

T = TypeVar("T")


def from_dict(cls: Type[T], data: dict) -> T:
    return from_dict_dacite(
        data_class=cls,
        data=data,
        config=Config(
            strict=True,
            cast=[Enum],
        ),
    )


def _asdict_factory(data):
    def convert_value(obj):
        if isinstance(obj, Enum):
            return obj.value
        return obj

    return dict((k, convert_value(v)) for k, v in data)


def to_dict(obj) -> dict:
    return asdict(obj, dict_factory=_asdict_factory)
