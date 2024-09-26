from typing import Optional

from bw_shared.configs.maps_config import MapsConfig
from bw_shared.configs.size import Size
from bw_shared.enums.cage_model import CAGE_MODEL_MAPPING, CageModel


def load_cage_model_sizes(maps: MapsConfig, cages: Optional[list[CageModel]] = None) -> dict[CageModel, Size]:
    cage_model_sizes = {}
    if cages is None:
        cages = list(CageModel)
    for cage_model in cages:
        field_type = CAGE_MODEL_MAPPING[cage_model]
        cage_model_sizes[cage_model] = maps.maps_map[field_type].size
    return cage_model_sizes
