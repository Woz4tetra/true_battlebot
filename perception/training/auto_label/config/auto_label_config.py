from __future__ import annotations

from dataclasses import dataclass

from bw_shared.messages.dataclass_utils import from_dict, to_dict


@dataclass
class AutoLabelConfig:
    min_size: tuple[int, int] = (800, 600)
    max_size: tuple[int, int] = (3200, 2400)
    default_size: tuple[int, int] | None = None

    @classmethod
    def from_dict(cls, config_dict: dict) -> AutoLabelConfig:
        return from_dict(cls, config_dict)

    def to_dict(self) -> dict:
        return to_dict(self)
