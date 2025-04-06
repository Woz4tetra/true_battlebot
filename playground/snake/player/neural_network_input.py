from dataclasses import dataclass

from game.data.grid_cell_type import GridCellType


@dataclass
class NeuralNetworkInput:
    lookaround_grid: list[list[GridCellType]]
    apple_offset: tuple[int, int]

    def to_list(self) -> list[float]:
        """
        Convert the NeuralNetworkInput to a list of floats.

        Returns:
            list[float]: A list of floats representing the input.
        """
        return [float(item) for sublist in self.lookaround_grid for item in sublist] + [
            float(self.apple_offset[0]),
            float(self.apple_offset[1]),
        ]
