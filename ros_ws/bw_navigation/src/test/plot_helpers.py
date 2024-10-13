import math

from bw_interfaces.msg import EstimatedObject
from bw_shared.geometry.pose2d import Pose2D
from matplotlib import patches
from matplotlib.axes import Axes


def draw_arrow(ax: Axes, pose: Pose2D, **kwargs) -> None:
    if "width" not in kwargs:
        kwargs["width"] = 0.5
    if "length" not in kwargs:
        kwargs["length"] = 0.2
    length = kwargs.pop("length")
    arrow = patches.Arrow(
        pose.x,
        pose.y,
        length * math.cos(pose.theta),
        length * math.sin(pose.theta),
        **kwargs,
    )
    ax.add_patch(arrow)


def draw_circle(ax: Axes, pose: Pose2D, radius: float, **kwargs) -> None:
    circle = patches.Circle((pose.x, pose.y), radius, **kwargs)
    ax.add_patch(circle)


def draw_robot(ax: Axes, robot: EstimatedObject, circle_args: dict, arrow_args: dict) -> None:
    pose = Pose2D.from_msg(robot.pose.pose)
    draw_circle(ax, pose, max(robot.size.x, robot.size.y), **circle_args)
    draw_arrow(ax, pose, **arrow_args)


def plot_robots(
    ax: Axes, controlled_robot_state: EstimatedObject, friendly_robot_states: list[EstimatedObject]
) -> None:
    draw_robot(
        ax,
        controlled_robot_state,
        circle_args=dict(color="blue", fill=False),
        arrow_args=dict(color="blue"),
    )
    for robot in friendly_robot_states:
        draw_robot(ax, robot, circle_args=dict(color="orange", fill=False), arrow_args=dict(color="orange"))
