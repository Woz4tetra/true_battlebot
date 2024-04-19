from enum import Enum, auto  # noqa: F401
from typing import Any, List


class EnumAutoLowerStr(Enum):
    @staticmethod
    def _generate_next_value_(name: str, start: int, count: int, last_values: List[Any]) -> str:
        return name.lower()
