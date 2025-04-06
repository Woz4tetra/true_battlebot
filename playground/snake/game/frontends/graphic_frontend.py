import pygame

from game.data.game_container import GameContainer
from game.data.game_state import GameState
from game.data.grid import Grid
from game.data.user_input_event import UserInputEvent
from game.frontends.snake_frontend import SnakeFrontend
from game.utils.get_snake_coords import get_snake_coords


class GraphicFrontend(SnakeFrontend):
    def __init__(self) -> None:
        pygame.init()

        self.screen_size = (640, 480)
        self.screen_rect = pygame.Rect(0, 0, self.screen_size[0], self.screen_size[1])

        # Set the display mode
        winstyle = 0  # |FULLSCREEN
        bestdepth = pygame.display.mode_ok(self.screen_rect.size, winstyle, 32)
        self.screen = pygame.display.set_mode(self.screen_rect.size, winstyle, bestdepth)
        pygame.display.set_caption("Snake Game")

        self.game_offset = (20, 40)

        self.screen.fill((0, 0, 0))

    def _get_user_event(self) -> UserInputEvent:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return UserInputEvent.QUIT
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    return UserInputEvent.MOVE_UP
                elif event.key == pygame.K_DOWN:
                    return UserInputEvent.MOVE_DOWN
                elif event.key == pygame.K_LEFT:
                    return UserInputEvent.MOVE_LEFT
                elif event.key == pygame.K_RIGHT:
                    return UserInputEvent.MOVE_RIGHT
                elif event.key == pygame.K_ESCAPE or event.key == pygame.K_p:
                    return UserInputEvent.TOGGLE_PAUSE
                elif event.key == pygame.K_q:
                    return UserInputEvent.QUIT
        return UserInputEvent.NONE

    def _get_square_size(self, grid: Grid) -> int:
        screen_min = min(self.screen_size[0] - self.game_offset[0], self.screen_size[1] - self.game_offset[1]) - 10
        grid_min = min(grid.width, grid.height)
        return screen_min // grid_min

    def _draw_grid_square(self, x: int, y: int, color: tuple[int, int, int], grid: Grid) -> None:
        square_size = self._get_square_size(grid)
        pygame.draw.rect(
            self.screen,
            color,
            (
                x * square_size + self.game_offset[0],
                y * square_size + self.game_offset[1],
                square_size,
                square_size,
            ),
        )

    def _draw_score(self, score: int) -> None:
        font = pygame.font.SysFont("Arial", 24)
        text_surface = font.render(f"Score: {score}", True, (255, 255, 255))
        self.screen.blit(text_surface, (10, 10))

    def _draw_paused(self) -> None:
        font = pygame.font.SysFont("Arial", 24)
        text_surface = font.render("Game Paused", True, (255, 255, 255))
        self.screen.blit(
            text_surface, (self.screen_size[0] // 2 - text_surface.get_width() // 2, self.screen_size[1] // 2)
        )

    def _draw_game_state(self, state: GameState) -> None:
        state_str = "Game Over!" if state == GameState.GAME_OVER else "You Win!"
        font = pygame.font.SysFont("Arial", 24)
        text_surface = font.render(state_str, True, (255, 255, 255))
        self.screen.blit(
            text_surface, (self.screen_size[0] // 2 - text_surface.get_width() // 2, self.screen_size[1] // 2)
        )

    def _draw_border(self, width: int, height: int) -> None:
        pygame.draw.rect(
            self.screen,
            (255, 255, 255),
            (
                self.game_offset[0] - 1,
                self.game_offset[1] - 1,
                width * self._get_square_size(Grid(width, height)) + 2,
                height * self._get_square_size(Grid(width, height)) + 2,
            ),
            1,
        )

    def _draw_game_container(self, game: GameContainer) -> None:
        grid = game.grid
        snake = game.snake
        apples = game.apples

        # Draw the snake
        for segment in get_snake_coords(snake):
            self._draw_grid_square(segment.x, segment.y, (0, 255, 0), grid)

        # Draw the apples
        for apple in apples:
            self._draw_grid_square(apple.coord.x, apple.coord.y, (255, 0, 0), grid)

    def draw(self, game: GameContainer, state: GameState, is_paused: bool) -> UserInputEvent:
        event = self._get_user_event()

        self.screen.fill((0, 0, 0))
        self._draw_score(game.score)
        self._draw_border(game.grid.width, game.grid.height)
        if is_paused:
            self._draw_paused()
            pygame.display.flip()
            return event

        self._draw_game_container(game)
        if state == GameState.GAME_OVER or state == GameState.GAME_WIN:
            self._draw_game_state(state)
        pygame.display.flip()
        return event

    def close(self) -> None:
        pygame.quit()
