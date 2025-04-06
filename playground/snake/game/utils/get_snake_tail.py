from game.data.snake import SnakeHead, SnakePart


def get_snake_tail_parent(head: SnakeHead) -> SnakePart | None:
    """
    Get the parent segment of the tail segment of the snake.

    Args:
        head (SnakeHead): The head of the snake.

    Returns:
        SnakePart: The tail segment of the snake.
    """

    parent_segment: SnakePart | None = None
    current_segment: SnakePart = head
    while current_segment.prev_segment is not None:
        parent_segment = current_segment
        current_segment = current_segment.prev_segment
    return parent_segment
