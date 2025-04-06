import pickle
from pathlib import Path

import neat.config
from game.data.direction import Direction
from game.data.game_container import GameContainer
from game.utils.get_lookaround_grid import get_lookaround_grid
from game.utils.xy_offset_to_apple import xy_offset_to_apple

from player.neural_network_input import NeuralNetworkInput
from player.player import Player


def neural_network_output_to_direction(output: list[float]) -> Direction:
    """
    Convert the neural network output to a direction.

    Args:
        output (list): A list of floats representing the neural network output.

    Returns:
        Direction: The direction corresponding to the highest output value.
    """
    return Direction(output.index(max(output)))


class AiPlayer(Player):
    def __init__(self, net: neat.nn.FeedForwardNetwork, lookaround_width: int = 5, lookaround_height: int = 5) -> None:
        self.direction = Direction.NONE
        self.lookaround_width = lookaround_width
        self.lookaround_height = lookaround_height
        self.net = net

    def get_direction(self, game: GameContainer) -> Direction:
        apple_offset = xy_offset_to_apple(game.snake, game.apples)
        if apple_offset is None:
            raise ValueError("No apple offset found.")
        lookaround_grid = get_lookaround_grid(game, self.lookaround_width, self.lookaround_height)

        nn_input = NeuralNetworkInput(lookaround_grid, apple_offset)
        nn_output = self.net.activate(nn_input.to_list())
        direction = neural_network_output_to_direction(nn_output)
        return direction


def load_ai_player_from_file(genome_file: Path, config_file: Path) -> AiPlayer:
    config = neat.config.Config(
        neat.DefaultGenome, neat.DefaultReproduction, neat.DefaultSpeciesSet, neat.DefaultStagnation, str(config_file)
    )
    with open(genome_file, "rb") as file:
        genome = pickle.load(file)
    net = neat.nn.FeedForwardNetwork.create(genome, config)
    return AiPlayer(net)
