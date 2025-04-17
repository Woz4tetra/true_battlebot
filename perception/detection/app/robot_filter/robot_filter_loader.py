import tf2_ros
from app.config.robot_filter.robot_filter_config import RobotFilterConfig
from app.container import Container
from app.robot_filter.robot_filter import RobotFilter
from bw_interfaces.msg import EstimatedObjectArray, RobotFleetConfigMsg
from bw_shared.configs.shared_config import SharedConfig
from bw_shared.enums.robot_team import RobotTeam
from geometry_msgs.msg import Twist
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from perception_tools.rosbridge.ros_publisher import RosPublisher
from std_msgs.msg import Empty


def load_robot_filter(container: Container, config: RobotFilterConfig, shared_config: SharedConfig) -> RobotFilter:
    robots_config = shared_config.robots
    opponent_fleet_sub = RosPollSubscriber("/opponent_fleet", RobotFleetConfigMsg)
    cmd_vel_subs = {
        robot.name: RosPollSubscriber(f"/{robot.name}/cmd_vel", Twist, queue_size=1)
        for robot in robots_config.robots
        if robot.team == RobotTeam.OUR_TEAM
    }
    reset_filters_sub = RosPollSubscriber("/reset_filters", Empty, queue_size=1)
    filter_state_array_pub = RosPublisher("/filtered_states", EstimatedObjectArray, queue_size=1)
    tf_broadcaster = container.resolve(tf2_ros.TransformBroadcaster)
    return RobotFilter(
        config=config,
        robots_config=robots_config,
        opponent_fleet_sub=opponent_fleet_sub,
        cmd_vel_subs=cmd_vel_subs,
        reset_filters_sub=reset_filters_sub,
        filter_state_array_pub=filter_state_array_pub,
        tf_broadcaster=tf_broadcaster,
    )
