from bw_shared.configs.maps import MapConfig
from perception_tools.messages.field_result import FieldResult
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from roslibpy import Ros

from app.config.field_filter_config.field_filter_types import FieldFilterConfig
from app.config.field_filter_config.ransac_field_filter_config import RansacFieldFilterConfig
from app.config.field_filter_config.simulated_field_filter_config import SimulatedFieldFilterConfig
from app.container import Container
from app.field_filter.field_filter_interface import FieldFilterInterface
from app.field_filter.ransac_field_filter import RansacFieldFilter
from app.field_filter.simulated_field_filter import SimulatedFieldFilter


def load_field_filter(
    map_config: MapConfig, field_filter: FieldFilterConfig, container: Container
) -> FieldFilterInterface:
    if isinstance(field_filter, RansacFieldFilterConfig):
        return RansacFieldFilter(map_config, field_filter)
    elif isinstance(field_filter, SimulatedFieldFilterConfig):
        ros = container.resolve(Ros)
        simulated_field_result_sub = RosPollSubscriber(
            ros, field_filter.namespace + "/simulated_field_result", FieldResult
        )
        return SimulatedFieldFilter(map_config, field_filter, simulated_field_result_sub)
    else:
        raise ValueError("Invalid field filter type")
