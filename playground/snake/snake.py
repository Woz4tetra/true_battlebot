import argparse
from enum import Enum

from game.data.direction import Direction
from game.data.game_state import GameState
from game.frontends.frontend_runner import FrontendRunner
from game.frontends.frontend_tick_event import FrontendTickEvent
from game.frontends.snake_frontend import SnakeFrontend
from game.snake_backend import SnakeBackend


class InputMethod(Enum):
    USER = "user"
    AI = "ai"


def run(snake_frontend: SnakeFrontend, width: int, height: int, input_method: InputMethod, sleep_delay: float) -> None:
    snake_backend = SnakeBackend(width=width, height=height)
    if input_method == InputMethod.AI:
        step_interval = 0.0
    else:
        step_interval = 0.2
        sleep_delay = 0.05
    frontend_runner = FrontendRunner(snake_frontend, sleep_delay=sleep_delay, step_interval=step_interval)
    state = GameState.RUNNING
    try:
        while True:
            event = frontend_runner.step(snake_backend.game, state)
            if event == FrontendTickEvent.QUIT:
                break
            elif event == FrontendTickEvent.PAUSED or event == FrontendTickEvent.DONE:
                continue
            elif event == FrontendTickEvent.APPLY_INPUT:
                direction = frontend_runner.get_direction()
                if input_method == InputMethod.AI:
                    # AI logic to determine the next direction
                    # For now, just set it to a random direction
                    import random

                    direction = random.choice(list(Direction))

                state = snake_backend.step(direction)
    finally:
        snake_frontend.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Snake Game")
    parser.add_argument("-wd", "--width", type=int, default=20, help="Width of the game grid")
    parser.add_argument("-ht", "--height", type=int, default=20, help="Height of the game grid")
    parser.add_argument("-m", "--mode", type=str, default="gui", choices=["text", "gui"], help="Game mode")
    parser.add_argument("-i", "--input", type=str, default="user", choices=["user", "ai"], help="Input method")
    args = parser.parse_args()

    input_method = InputMethod(args.input)

    if args.mode == "text":
        import curses

        from game.frontends.text_frontend import TextFrontend

        def text_main(screen: curses.window) -> None:
            frontend = TextFrontend(screen)
            run(frontend, args.width, args.height, input_method=input_method, sleep_delay=0.05)

        curses.wrapper(text_main)
    else:
        from game.frontends.graphic_frontend import GraphicFrontend

        frontend = GraphicFrontend()
        run(
            frontend,
            args.width,
            args.height,
            input_method=input_method,
            sleep_delay=0.001 if input_method == InputMethod.AI else 0.05,
        )


if __name__ == "__main__":
    main()
