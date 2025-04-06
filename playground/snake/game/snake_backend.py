from game.actions.advance_snake import advance_snake
from game.actions.initialize_game import initialize_game
from game.actions.set_snake_direction import set_snake_direction
from game.data.direction import Direction
from game.data.game_state import GameState


class SnakeBackend:
    def __init__(self, width: int, height: int) -> None:
        self.game = initialize_game(width, height)

    def step(self, direction: Direction) -> GameState:
        if direction != Direction.NONE:
            set_snake_direction(self.game.snake, direction)
        state = advance_snake(self.game)
        return state

    def get_score(self) -> int:
        return self.game.score
