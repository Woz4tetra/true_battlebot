import argparse
import time

from game.data.direction import Direction
from game.data.game_state import GameState
from game.data.user_input_event import UserInputEvent
from game.frontends.snake_frontend import SnakeFrontend
from game.snake_backend import SnakeBackend


def run(snake_frontend: SnakeFrontend, width: int, height: int) -> None:
    event_mapping = {
        UserInputEvent.MOVE_UP: Direction.UP,
        UserInputEvent.MOVE_DOWN: Direction.DOWN,
        UserInputEvent.MOVE_LEFT: Direction.LEFT,
        UserInputEvent.MOVE_RIGHT: Direction.RIGHT,
    }
    snake_backend = SnakeBackend(width=width, height=height)
    is_paused = False
    state = GameState.RUNNING
    step_interval = 0.2
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
                continue
            elif state == GameState.GAME_WIN:
                continue
            next_direction = event_mapping.get(event, None)
            if next_direction is not None:
                if len(buffered_inputs) < 2:
                    buffered_inputs.append(next_direction)

            now = time.monotonic()
            if now - prev_step_time >= step_interval:
                prev_step_time = now
                direction = buffered_inputs.pop(0) if buffered_inputs else Direction.NONE
                state = snake_backend.step(direction)
    finally:
        snake_frontend.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Snake Game")
    parser.add_argument("--width", type=int, default=20, help="Width of the game grid")
    parser.add_argument("--height", type=int, default=20, help="Height of the game grid")
    parser.add_argument("--mode", type=str, default="text", choices=["text", "gui"], help="Game mode")
    args = parser.parse_args()

    if args.mode == "text":
        import curses

        from game.frontends.text_frontend import TextFrontend

        def text_main(screen: curses.window) -> None:
            frontend = TextFrontend(screen)
            run(frontend, args.width, args.height)

        curses.wrapper(text_main)
    else:
        from game.frontends.graphic_frontend import GraphicFrontend

        frontend = GraphicFrontend()
        run(frontend, args.width, args.height)


if __name__ == "__main__":
    main()
