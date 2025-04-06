from game.data.game_container import GameContainer
from game.data.game_state import GameState


def tick_score(game: GameContainer) -> GameState:
    """
    Decrease the life of the snake and increase the score.
    If the life reaches zero, the game is over.

    Args:
        game (GameContainer): The current game state.

    Returns:
        GameState: The updated game state.
    """
    if game.life > 0:
        game.life -= 1
    game.score += 1
    return GameState.RUNNING if game.life > 0 else GameState.GAME_OVER
