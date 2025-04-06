import curses

from game.data.game_container import GameContainer
from game.data.game_state import GameState
from game.data.user_input_event import UserInputEvent
from game.frontends.snake_frontend import SnakeFrontend
from game.utils.get_snake_coords import get_snake_coords


def get_block(y: int) -> str:
    """
    Get the block character for a given y-coordinate.

    Args:
        y (int): The y-coordinate.

    Returns:
        str: The block character.
    """
    if y % 2 == 0:
        return "▀"
    else:
        return "▄"


class TextFrontend(SnakeFrontend):
    def __init__(self, screen: curses.window) -> None:
        self.screen = screen
        self.screen.nodelay(True)
        self.screen.addstr(0, 0, "Press 'q' to quit")
        curses.curs_set(False)
        self.game_offset = (2, 2)

        curses.init_pair(2, curses.COLOR_GREEN, curses.COLOR_BLACK)
        self.snake_color = curses.color_pair(2)
        curses.init_pair(3, curses.COLOR_RED, curses.COLOR_BLACK)
        self.apple_color = curses.color_pair(3)
        curses.init_pair(4, curses.COLOR_RED, curses.COLOR_GREEN)
        self.apple_snake_color = curses.color_pair(4)
        curses.init_pair(5, curses.COLOR_GREEN, curses.COLOR_RED)
        self.snake_apple_color = curses.color_pair(5)
        self.screen.refresh()

    def _get_key(self) -> int | None:
        """
        Get a key press from the user.

        Args:
            screen (curses.window): The curses window to read input from.

        Returns:
            int: The key pressed by the user.
        """
        try:
            return self.screen.getch()
        except curses.error:
            return None

    def _get_user_event(self, key: str, key_raw: int) -> UserInputEvent:
        event = UserInputEvent.NONE
        if key == "q":
            event = UserInputEvent.QUIT
        elif key_raw == curses.KEY_UP:
            event = UserInputEvent.MOVE_UP
        elif key_raw == curses.KEY_DOWN:
            event = UserInputEvent.MOVE_DOWN
        elif key_raw == curses.KEY_LEFT:
            event = UserInputEvent.MOVE_LEFT
        elif key_raw == curses.KEY_RIGHT:
            event = UserInputEvent.MOVE_RIGHT
        elif key == "p":
            event = UserInputEvent.TOGGLE_PAUSE
        return event

    def _draw_game_container(self, game: GameContainer) -> None:
        snake = game.snake
        apples = game.apples

        # Draw the snake
        snake_drawn_coordinates = set()
        for segment in get_snake_coords(snake):
            y = segment.y // 2
            x = segment.x
            if (y, x) in snake_drawn_coordinates:
                block = "█"
            else:
                block = get_block(segment.y)
            snake_drawn_coordinates.add((y, x))
            self.screen.addstr(y + self.game_offset[1], x + self.game_offset[0], block, self.snake_color)

        # Draw the apples
        apple_drawn_coordinates = set()
        for apple in apples:
            y = apple.coord.y // 2
            x = apple.coord.x
            if (y, x) in apple_drawn_coordinates:
                block = "█"
            else:
                block = get_block(apple.coord.y)
            if (y, x) in snake_drawn_coordinates:
                color = self.apple_snake_color
            else:
                color = self.apple_color
            apple_drawn_coordinates.add((y, x))
            self.screen.addstr(y + self.game_offset[1], x + self.game_offset[0], block, color)

        # Clear the screen and display the grid
        self.screen.refresh()

    def _draw_game_state(self, state: GameState) -> None:
        state_str = "Game Over!" if state == GameState.GAME_OVER else "You Win!"
        self.screen.addstr(1, 0, state_str)

    def _draw_paused(self) -> None:
        self.screen.addstr(1, 0, "Game Paused")

    def _draw_score(self, score: int) -> None:
        self.screen.addstr(0, 0, f"Score: {score}")

    def _draw_border(self, width: int, height: int) -> None:
        height //= 2
        gx, gy = self.game_offset
        for x in range(width):
            self.screen.addstr(gy - 1, x + gx, "─")
            self.screen.addstr(height + gy, x + gx, "─")
        for y in range(height):
            self.screen.addstr(y + gy, 1, "│")
            self.screen.addstr(y + gy, width + 2, "│")
        self.screen.addstr(gy - 1, gx - 1, "┌")
        self.screen.addstr(gy - 1, width + gx, "┐")
        self.screen.addstr(height + gy, gx - 1, "└")
        self.screen.addstr(height + gy, width + gx, "┘")

    def draw(self, game: GameContainer, state: GameState, is_paused: bool) -> UserInputEvent:
        key_raw = self._get_key()
        if key_raw is not None:
            key = chr(key_raw & 0xFF)
            event = self._get_user_event(key, key_raw)
        else:
            event = UserInputEvent.NONE
        self.screen.clear()
        self._draw_border(game.grid.width, game.grid.height)
        self._draw_score(game.score)
        if is_paused:
            self._draw_paused()
            return event

        self._draw_game_container(game)
        if state == GameState.GAME_OVER or state == GameState.GAME_WIN:
            self._draw_game_state(state)
        self.screen.refresh()
        return event

    def close(self) -> None:
        """
        Close the curses window and restore terminal settings.
        """
        curses.curs_set(True)
