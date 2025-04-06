from game.data.direction import Direction
from game.data.snake import SnakeHead
from game.utils.get_neighbor import get_neighbor


def set_snake_direction(snake: SnakeHead, direction: Direction) -> None:
    next_coord = get_neighbor(snake.coord, direction)
    prev_segment = snake.prev_segment
    if prev_segment is not None and next_coord == prev_segment.coord:
        return
    snake.direction = direction
