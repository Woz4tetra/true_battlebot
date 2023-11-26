from bw_navigation.selector_algorithms.match_state import MatchState, SelectionResult

from .base_selector import BaseSelector


class PushFromBehindSelector(BaseSelector):
    """
    If the controlled robot is not in line with the guidance bot and opponent, select goal that's behind the
    opponent pointing towards the guidance bot.
    Otherwise, if the controlled bot is in position, select goal that's just in front the guidance bot
    (push the opponent to the guidance bot).
    """

    def __init__(self) -> None:
        pass

    def get_target(self, match_state: MatchState) -> SelectionResult:
        pass
