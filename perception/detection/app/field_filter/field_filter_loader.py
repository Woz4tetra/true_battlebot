import bw_interfaces.msg as bw_interfaces
from app.config.config import Config
from app.config.field_filter.field_filter_types import FieldFilterConfig
from app.config.field_filter.global_field_manager_config import GlobalFieldManagerConfig
from app.config.field_filter.point_cloud_field_filter_config import (
    LeastSquaresPlaneSolverConfig,
    PointCloudFieldFilterConfig,
    RansacPlaneSolverConfig,
)
from app.config.field_filter.simulated_field_filter_config import SimulatedFieldFilterConfig
from app.container import Container
from app.field_filter.field_filter_interface import FieldFilterInterface
from app.field_filter.global_field_transformer import GlobalFieldTransformer
from app.field_filter.point_cloud_field_filter import PointCloudFieldFilter
from app.field_filter.simulated_field_filter import SimulatedFieldFilter
from app.field_filter.solvers import LeastSquaresSolver, RansacPlaneSolver
from app.field_filter.solvers.base_plane_solver import BasePlaneSolver
from bw_shared.configs.maps_config import MapConfig
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from perception_tools.rosbridge.ros_publisher import RosPublisher
from perception_tools.rosbridge.transform_broadcaster_bridge import TransformBroadcasterBridge
from visualization_msgs.msg import MarkerArray


def make_solver(field_filter_config: PointCloudFieldFilterConfig) -> BasePlaneSolver:
    solver_config = field_filter_config.solver
    if isinstance(solver_config, RansacPlaneSolverConfig):
        return RansacPlaneSolver(solver_config)
    elif isinstance(solver_config, LeastSquaresPlaneSolverConfig):
        return LeastSquaresSolver(solver_config)
    else:
        raise ValueError(f"Invalid solver type: {type(solver_config)}")


def load_field_filter(field_filter_config: FieldFilterConfig, container: Container) -> FieldFilterInterface:
    if isinstance(field_filter_config, PointCloudFieldFilterConfig):
        solver = make_solver(field_filter_config)
        return PointCloudFieldFilter(solver, field_filter_config)
    elif isinstance(field_filter_config, SimulatedFieldFilterConfig):
        config = container.resolve(Config)
        simulated_field_result_sub = RosPollSubscriber(
            config.camera_topic.namespace + "/simulated_field_result", bw_interfaces.EstimatedObject
        )
        return SimulatedFieldFilter(field_filter_config, simulated_field_result_sub)
    else:
        raise ValueError(f"Invalid field filter type: {type(field_filter_config)}")


def load_global_field_transformer(config: GlobalFieldManagerConfig, container: Container) -> GlobalFieldTransformer:
    map_config = container.resolve(MapConfig)
    set_corner_side_sub = RosPollSubscriber("/set_cage_corner", bw_interfaces.CageCorner)
    estimated_field_pub = RosPublisher("/filter/field", bw_interfaces.EstimatedObject, latch=True)
    tf_broadcaster = container.resolve(TransformBroadcasterBridge)
    estimated_field_marker_pub = RosPublisher("/filter/field/marker", MarkerArray, latch=True)
    latched_corner_pub = RosPublisher("/cage_corner", bw_interfaces.CageCorner, latch=True)
    return GlobalFieldTransformer(
        config,
        map_config=map_config,
        set_corner_side_sub=set_corner_side_sub,
        estimated_field_pub=estimated_field_pub,
        estimated_field_marker_pub=estimated_field_marker_pub,
        tf_broadcaster=tf_broadcaster,
        latched_corner_pub=latched_corner_pub,
    )
