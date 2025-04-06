import pickle
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import neat.config
from game.data.game_state import GameState
from game.snake_backend import SnakeBackend
from player.ai_player import AiPlayer


@dataclass
class TrainingRun:
    player: AiPlayer
    genome: Any
    snake_backend: SnakeBackend
    state: GameState = GameState.RUNNING


def all_running(runs: dict[int, TrainingRun]) -> bool:
    """
    Check if all training runs are still running.

    Args:
        runs (dict): A dictionary of training runs.

    Returns:
        bool: True if all runs are running, False otherwise.
    """
    return all(run.state == GameState.RUNNING for run in runs.values())


def eval_genomes(genomes: list, config: neat.config.Config):
    runs: dict[int, TrainingRun] = {}

    for genome_id, genome in genomes:
        width = random.randint(10, 30)
        height = random.randint(10, 30)

        # Set the genome fitness to 0
        genome.fitness = 0

        # Create a neural network from the genome
        net = neat.nn.FeedForwardNetwork.create(genome, config)

        # Create a SnakeBackend instance
        snake_backend = SnakeBackend(width=width, height=height)

        runs[genome_id] = TrainingRun(AiPlayer(net), genome, snake_backend)

    while all_running(runs):
        for genome_id, run in runs.items():
            snake_backend = run.snake_backend
            game = snake_backend.game

            direction = run.player.get_direction(game)
            state = snake_backend.step(direction)

            run.genome.fitness = game.score
            run.state = state


def train(config_file: Path):
    config = neat.config.Config(
        neat.DefaultGenome, neat.DefaultReproduction, neat.DefaultSpeciesSet, neat.DefaultStagnation, str(config_file)
    )

    # Create the population, which is the top-level object for a NEAT run.
    population = neat.Population(config)

    # Add a stdout reporter to show progress in the terminal.
    population.add_reporter(neat.StdOutReporter(True))
    stats = neat.StatisticsReporter()
    population.add_reporter(stats)
    # p.add_reporter(neat.Checkpointer(5))

    # Run for up to 50 generations.
    winner = population.run(eval_genomes, 50)

    # show final stats
    print("\nBest genome:\n{!s}".format(winner))

    # Save the winner.
    with open("winner.pkl", "wb") as file:
        pickle.dump(winner, file)
        print("Winner genome saved to winner.pkl")


if __name__ == "__main__":
    # Determine path to configuration file. This path manipulation is
    # here so that the script will run successfully regardless of the
    # current working directory.
    local_dir = Path(__file__).resolve().parent
    config_path = local_dir / "neat.conf"

    # Run the NEAT algorithm.
    train(config_path)
