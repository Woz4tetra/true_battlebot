import random
from pathlib import Path

import neat.config
from game.snake_backend import SnakeBackend


def eval_genomes(genomes: list, config: neat.config.Config):
    runs = []

    for genome_id, genome in genomes:
        width = random.randint(10, 30)
        height = random.randint(10, 30)

        # Set the genome fitness to 0
        genome.fitness = 0

        # Create a neural network from the genome
        net = neat.nn.FeedForwardNetwork.create(genome, config)

        # Create a SnakeBackend instance
        snake_backend = SnakeBackend(width=width, height=height)

        runs.append((net, genome, snake_backend))


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


if __name__ == "__main__":
    # Determine path to configuration file. This path manipulation is
    # here so that the script will run successfully regardless of the
    # current working directory.
    local_dir = Path(__file__).resolve().parent
    config_path = local_dir / "neat.conf"

    # Run the NEAT algorithm.
    train(config_path)
