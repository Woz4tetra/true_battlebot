from typing import Union

from .crash_avoid_front_selector import CrashAvoidFrontSelector
from .crash_selector import CrashSelector
from .push_from_behind_selector import PushFromBehindSelector
from .sacrificial_selector import SacrificialSelector

SelectorAlgorithm = Union[SacrificialSelector, PushFromBehindSelector, CrashSelector, CrashAvoidFrontSelector]

__all__ = [
    "CrashAvoidFrontSelector",
    "CrashSelector",
    "PushFromBehindSelector",
    "SacrificialSelector",
]