from typing import Union

from .ransac_field_filter_config import RansacFieldFilterConfig
from .simulated_field_filter_config import SimulatedFieldFilterConfig

FieldFilterConfig = Union[RansacFieldFilterConfig, SimulatedFieldFilterConfig]