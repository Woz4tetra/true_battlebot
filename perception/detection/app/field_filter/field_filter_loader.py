from app.config.config import Config
from app.config.field_filter_config.field_filter_types import FieldFilterConfig
from app.config.field_filter_config.point_cloud_field_filter_config import (
    LeastSquaresPlaneSolverConfig,
    PointCloudFieldFilterConfig,
    RansacPlaneSolverConfig,
)
from app.config.field_filter_config.simulated_field_filter_config import SimulatedFieldFilterConfig
from app.container import Container
from app.field_filter.field_filter_interface import FieldFilterInterface
from app.field_filter.point_cloud_field_filter import PointCloudFieldFilter
from app.field_filter.simulated_field_filter import SimulatedFieldFilter
from app.field_filter.solvers import LeastSquaresSolver, RansacPlaneSolver
from app.field_filter.solvers.base_plane_solver import BasePlaneSolver
from bw_interfaces.msg import EstimatedObject
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber


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
            config.camera_topic.namespace + "/simulated_field_result", EstimatedObject
        )
        return SimulatedFieldFilter(field_filter_config, simulated_field_result_sub)
    else:
        raise ValueError(f"Invalid field filter type: {type(field_filter_config)}")
