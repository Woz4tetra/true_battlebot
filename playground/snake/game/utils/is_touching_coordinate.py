from game.data.coordinate import Coordinate
from game.data.snake import SnakeHead
from game.utils.get_snake_coords import get_snake_coords


def is_touching_coordinate(check_coords: list[Coordinate], coord: Coordinate) -> bool:
    for check_coord in check_coords:
        if check_coord == coord:
            return True
    return False


def is_snake_touching_coordinate(snake: SnakeHead, coord: Coordinate) -> bool:
    return is_touching_coordinate(get_snake_coords(snake), coord)
