from abc import ABC, abstractmethod

from game.data.game_container import GameContainer
from game.data.game_state import GameState
from game.frontends.user_input_event import UserInputEvent


class SnakeFrontend(ABC):
    def __init__(self) -> None:
        pass

    @abstractmethod
    def draw(self, game: GameContainer, state: GameState, is_paused: bool) -> UserInputEvent:
        pass

    def close(self) -> None:
        pass
