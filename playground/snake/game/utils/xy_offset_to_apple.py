from game.data.apple import Apple
from game.data.snake import SnakeHead


def xy_offset_to_apple(snake: SnakeHead, apples: list[Apple]) -> tuple[int, int] | None:
    """
    Get the x-y offset from the snake head to the nearest apple.
    If no apple is found, return None.

    Args:
        snake (SnakeHead): The head of the snake.
        apples (list[Apple]): A list of apples in the game.

    Returns:
        tuple[int, int] | None: The x-y offset to the nearest apple or None if no apple is found.
    """
    if not apples:
        return None

    min_distance = float("inf")
    x_offset = 0
    y_offset = 0
    for apple in apples:
        distance = snake.coord.manhatten_distance(apple.coord)
        if distance < min_distance:
            min_distance = distance
            x_offset = apple.coord.x - snake.coord.x
            y_offset = apple.coord.y - snake.coord.y

    return (x_offset, y_offset)
