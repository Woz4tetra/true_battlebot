from game.actions.get_next_apple import get_next_apple
from game.data.apple_collision_event import AppleCollisionEvent
from game.data.coordinate import Coordinate
from game.data.game_container import GameContainer


def check_apple_collision(next_coord: Coordinate, game_state: GameContainer) -> AppleCollisionEvent:
    apples = game_state.apples
    snake = game_state.snake
    grid = game_state.grid
    for apple in apples:
        if apple.coord != next_coord:
            continue
        game_state.score += 1
        next_apple = get_next_apple(grid, snake, apples)
        if next_apple is None:
            return AppleCollisionEvent.NO_MORE_ROOM
        apples.remove(apple)
        apples.append(next_apple)
        break
    else:
        return AppleCollisionEvent.NONE
    return AppleCollisionEvent.COLLISION
