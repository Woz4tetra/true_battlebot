from game.actions.check_apple_collision import check_apple_collision
from game.data.apple_collision_event import AppleCollisionEvent
from game.data.game_container import GameContainer
from game.data.game_state import GameState
from game.data.snake import SnakePart
from game.utils.get_neighbor import get_neighbor
from game.utils.get_snake_tail import get_snake_tail_parent
from game.utils.is_in_bounds import is_in_bounds
from game.utils.is_touching_coordinate import is_snake_touching_coordinate


def advance_snake(game_state: GameContainer) -> GameState:
    snake = game_state.snake
    grid = game_state.grid
    next_coord = get_neighbor(snake.coord, snake.direction)

    # Check if the next coordinate is within bounds
    if not is_in_bounds(next_coord, grid):
        return GameState.GAME_OVER

    # Check if the snake is touching itself
    if is_snake_touching_coordinate(snake, next_coord):
        return GameState.GAME_OVER

    # Move the snake
    collision_event = check_apple_collision(next_coord, game_state)
    if collision_event == AppleCollisionEvent.NO_MORE_ROOM:
        return GameState.GAME_WIN
    # Add a new segment to the snake where the head currently is
    new_segment = SnakePart(coord=snake.coord, prev_segment=snake.prev_segment)
    snake.prev_segment = new_segment
    tail_parent_segment = get_snake_tail_parent(snake)
    if collision_event == AppleCollisionEvent.NONE and tail_parent_segment is not None:
        # delete the tail segment
        tail_parent_segment.prev_segment = None
    snake.coord = next_coord

    return GameState.RUNNING
