import curses
import time

from game.data.direction import Direction
from game.data.game_state import GameState
from game.data.user_input_event import UserInputEvent
from game.snake_backend import SnakeBackend
from game.text_snake_game.text_game import TextGame


def main(screen: curses.window) -> None:
    event_mapping = {
        UserInputEvent.MOVE_UP: Direction.UP,
        UserInputEvent.MOVE_DOWN: Direction.DOWN,
        UserInputEvent.MOVE_LEFT: Direction.LEFT,
        UserInputEvent.MOVE_RIGHT: Direction.RIGHT,
    }
    snake_backend = SnakeBackend(width=20, height=10)
    snake_frontend = TextGame(screen)
    is_paused = False
    state = GameState.RUNNING
    starting_step_interval = 0.5
    prev_step_time = time.monotonic()
    direction = Direction.NONE
    buffered_inputs = []
    try:
        while True:
            event = snake_frontend.draw(snake_backend.game, state, is_paused)
            time.sleep(0.05)
            if event == UserInputEvent.QUIT:
                break
            if event == UserInputEvent.TOGGLE_PAUSE:
                is_paused = not is_paused
            if is_paused:
                continue
            if state == GameState.GAME_OVER:
                print("Game Over!")
                continue
            elif state == GameState.GAME_WIN:
                print("You Win!")
                continue
            next_direction = event_mapping.get(event, None)
            if next_direction is not None:
                if len(buffered_inputs) < 2:
                    buffered_inputs.append(next_direction)

            step_interval = starting_step_interval - snake_backend.get_score() * 0.01

            now = time.monotonic()
            if now - prev_step_time >= step_interval:
                prev_step_time = now
                direction = buffered_inputs.pop(0) if buffered_inputs else Direction.NONE
                state = snake_backend.step(direction)
    finally:
        snake_frontend.close()


if __name__ == "__main__":
    curses.wrapper(main)
