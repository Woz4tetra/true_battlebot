import curses

from game.data.game_container import GameContainer
from game.data.game_state import GameState
from game.data.user_input_event import UserInputEvent
from game.utils.get_snake_coords import get_snake_coords


class TextGame:
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
        for segment in get_snake_coords(snake):
            self.screen.addstr(segment.y + self.game_offset[1], segment.x + self.game_offset[0], "█", self.snake_color)

        # Draw the apples
        for apple in apples:
            self.screen.addstr(
                apple.coord.y + self.game_offset[1], apple.coord.x + self.game_offset[0], "█", self.apple_color
            )

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
