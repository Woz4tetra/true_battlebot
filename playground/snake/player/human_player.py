from game.data.direction import Direction
from game.data.game_container import GameContainer
from game.frontends.frontend_runner import FrontendRunner

from player.player import Player


class HumanPlayer(Player):
    def __init__(self, frontend_runner: FrontendRunner) -> None:
        self.frontend_runner = frontend_runner

    def get_direction(self, game: GameContainer) -> Direction:
        return self.frontend_runner.get_direction()
