import argparse
import time
from typing import Any

import numpy as np
from bw_shared.geometry.rpy import RPY
from bw_shared.geometry.transform3d import Transform3D
from bw_shared.geometry.twist2d import Twist2D
from geometry_msgs.msg import Vector3
from matplotlib import pyplot as plt
from pyphysx_render.pyrender import PyPhysxViewer
from pyphysx_utils.rate import Rate
from pyphysx_utils.transformations import pose_to_transformation_matrix
from pyphysx_utils.urdf_robot_parser import URDFRobot

from pyphysx import (
    BroadPhaseType,
    Material,
    Physics,
    RigidDynamic,
    RigidStatic,
    Scene,
    SceneFlag,
    cast_transformation,
)


def tf_to_physx_tf(transform: Transform3D) -> tuple[np.ndarray, np.ndarray]:
    return cast_transformation(transform.position_array.tolist() + transform.quaternion_np.tolist())


def physx_tf_to_tf(physx_tf: tuple[np.ndarray, np.ndarray]) -> Transform3D:
    return Transform3D(pose_to_transformation_matrix(physx_tf))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("-r", "--render", action="store_true")
    args = parser.parse_args()
    should_render = args.render

    xs = []
    thetas = []
    times = []
    Physics.init_gpu()

    for _ in range(100):
        t0 = time.perf_counter()

        wheel_joint_properties: dict[str, Any] = dict(
            stiffness=1e25,
            damping=0,
            force_limit=3.402823e38,
            is_acceleration=False,
        )
        left_joints = [
            "wheel_front_left_joint",
            "wheel_back_left_joint",
        ]
        right_joints = [
            "wheel_back_right_joint",
            "wheel_front_right_joint",
        ]
        scene = Scene()

        robot = URDFRobot("main_bot.urdf", kinematic=False)
        joints = left_joints + right_joints
        for joint_name in joints:
            robot.movable_joints[joint_name].configure_drive(**wheel_joint_properties)
        for link_name, link in robot.links.items():
            actor: RigidDynamic = link.actor
            for shape in actor.get_attached_shapes():
                for material in shape.get_materials():
                    material.set_static_friction(1.0)
                    material.set_dynamic_friction(1.0)
                    material.set_restitution(0.0)

        root_actor: RigidDynamic = robot.root_node.actor
        root_actor.set_global_pose(
            tf_to_physx_tf(
                Transform3D.from_position_and_rpy(Vector3(0.0, 0.0, 0.03), RPY.from_degrees((0.0, 0.0, 0.0)))
            )
        )
        robot_agg = robot.get_aggregate()
        scene.add_aggregate(robot_agg)

        plane = RigidStatic.create_plane(material=Material(static_friction=1.0, dynamic_friction=1.0, restitution=0.0))
        scene.add_actor(plane)

        rate = Rate(120)
        period = rate.period()

        twist = Twist2D(x=1.0, y=0.0, theta=2 * np.pi)
        wheel_radius = 0.02285709
        base_radius = 0.09205103

        if should_render:
            render = PyPhysxViewer()
            render.add_physx_scene(scene)
        for joint_name in left_joints:
            robot.movable_joints[joint_name].set_joint_velocity(0.0)
        root_actor.set_global_pose(
            tf_to_physx_tf(
                Transform3D.from_position_and_rpy(Vector3(0.0, 0.0, 0.03), RPY.from_degrees((0.0, 0.0, 0.0)))
            )
        )
        robot.update(period)

        current_time = 0.0

        t1 = time.perf_counter()
        while current_time < 0.8:
            if 0.2 < current_time < 0.7:
                for joint_name in left_joints:
                    wheel_velocity = (twist.x - twist.theta * base_radius) / wheel_radius
                    robot.movable_joints[joint_name].set_joint_velocity(-1 * wheel_velocity)
                for joint_name in right_joints:
                    wheel_velocity = (twist.x + twist.theta * base_radius) / wheel_radius
                    robot.movable_joints[joint_name].set_joint_velocity(-1 * wheel_velocity)
            else:
                for joint_name in joints:
                    robot.movable_joints[joint_name].set_joint_velocity(0.0)

            robot.update(period)
            scene.simulate(period)

            current_time += period
            pose = physx_tf_to_tf(root_actor.get_global_pose())
            xs.append(pose.position.x)
            thetas.append(pose.rpy[2])
            times.append(current_time)
            if should_render:
                render.update()
                rate.sleep()
        t2 = time.perf_counter()

        print("load", t1 - t0)
        print("render", t2 - t1)

        if should_render:
            render.close()
        del scene
        del robot

    fig, subplots = plt.subplots(2, 1, sharex=True)
    subplots[0].plot(times, xs, ".")
    subplots[0].set_title("X position of the robot")
    subplots[0].set_ylabel("X position (m)")
    subplots[0].set_xlabel("Time (s)")
    subplots[0].grid()
    subplots[1].plot(times, thetas, ".", label="Angle")
    subplots[1].set_title("Theta position of the robot")
    subplots[1].set_ylabel("Theta position (rad)")
    subplots[1].set_xlabel("Time (s)")
    subplots[1].grid()

    plt.show()


if __name__ == "__main__":
    main()
