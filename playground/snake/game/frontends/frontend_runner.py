import time

from game.data.direction import Direction
from game.data.game_container import GameContainer
from game.data.game_state import GameState
from game.frontends.frontend_tick_event import FrontendTickEvent
from game.frontends.snake_frontend import SnakeFrontend
from game.frontends.user_input_event import UserInputEvent


class FrontendRunner:
    def __init__(self, frontend: SnakeFrontend, sleep_delay: float, step_interval: float) -> None:
        self.event_mapping = {
            UserInputEvent.MOVE_UP: Direction.UP,
            UserInputEvent.MOVE_DOWN: Direction.DOWN,
            UserInputEvent.MOVE_LEFT: Direction.LEFT,
            UserInputEvent.MOVE_RIGHT: Direction.RIGHT,
        }
        self.snake_frontend = frontend
        self.is_paused = False
        self.sleep_delay = sleep_delay
        self.step_interval = step_interval
        self.direction = Direction.NONE
        self.buffered_inputs = []
        self.prev_step_time = 0.0

    def step(self, game: GameContainer, state: GameState) -> FrontendTickEvent:
        event = self.snake_frontend.draw(game, state, self.is_paused)
        time.sleep(self.sleep_delay)
        if event == UserInputEvent.QUIT:
            return FrontendTickEvent.QUIT
        if event == UserInputEvent.TOGGLE_PAUSE:
            self.is_paused = not self.is_paused
        if self.is_paused:
            return FrontendTickEvent.PAUSED
        if state == GameState.GAME_OVER:
            return FrontendTickEvent.DONE
        elif state == GameState.GAME_WIN:
            return FrontendTickEvent.DONE
        next_direction = self.event_mapping.get(event, None)
        if next_direction is not None:
            if len(self.buffered_inputs) < 2:
                self.buffered_inputs.append(next_direction)

        now = time.monotonic()
        if now - self.prev_step_time >= self.step_interval:
            self.prev_step_time = now
            self.direction = self.buffered_inputs.pop(0) if self.buffered_inputs else Direction.NONE
            return FrontendTickEvent.APPLY_INPUT
        return FrontendTickEvent.NONE

    def get_direction(self) -> Direction:
        return self.direction
