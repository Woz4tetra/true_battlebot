from bw_shared.configs.maps import MapConfig
from perception_tools.rosbridge.ros_factory import RosFactory

from app.container import Container
from perception.detection.app.config.field_filter_config.field_filter_types import FieldFilterConfig
from perception.detection.app.config.field_filter_config.ransac_field_filter_config import RansacFieldFilterConfig
from perception.detection.app.config.field_filter_config.simulated_field_filter_config import SimulatedFieldFilterConfig
from perception.detection.app.field_filter.field_filter_interface import FieldFilterInterface
from perception.detection.app.field_filter.ransac_field_filter import RansacFieldFilter
from perception.detection.app.field_filter.simulated_field_filter import SimulatedFieldFilter


def load_field_filter(
    map_config: MapConfig, field_filter: FieldFilterConfig, container: Container
) -> FieldFilterInterface:
    if isinstance(field_filter, RansacFieldFilterConfig):
        return RansacFieldFilter(map_config, field_filter)
    elif isinstance(field_filter, SimulatedFieldFilterConfig):
        ros_factory = container.resolve(RosFactory)
        return SimulatedFieldFilter(map_config, field_filter)
    else:
        raise ValueError("Invalid field filter type")
