from abc import ABC, abstractmethod

from game.data.direction import Direction
from game.data.game_container import GameContainer


class Player(ABC):
    @abstractmethod
    def get_direction(self, game: GameContainer) -> Direction: ...
