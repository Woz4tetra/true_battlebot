from abc import ABC, abstractmethod

from bw_navigation.selector_algorithms.match_state import MatchState, SelectionResult


class BaseSelector(ABC):
    def __init__(self) -> None:
        pass

    @abstractmethod
    def get_target(self, match_state: MatchState) -> SelectionResult:
        pass
