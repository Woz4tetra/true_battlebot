import argparse
import time

import numpy as np
from bw_shared.geometry.rpy import RPY
from bw_shared.geometry.transform3d import Transform3D
from geometry_msgs.msg import Vector3
from matplotlib import pyplot as plt
from pyphysx_render.pyrender import PyPhysxViewer
from pyphysx_utils.rate import Rate

from pyphysx import Material, RigidDynamic, RigidStatic, Scene, Shape, cast_transformation  # type: ignore


def tf_to_physx_tf(transform: Transform3D) -> tuple[np.ndarray, np.ndarray]:
    return cast_transformation(transform.position_array.tolist() + transform.quaternion_np.tolist())


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("-r", "--render", action="store_true")
    args = parser.parse_args()

    cube_height = 0.01
    scene_tf = Transform3D.from_position_and_rpy(Vector3(), RPY.from_degrees((0.0, 45.0, 0.0)))
    relative_plane_tf = Transform3D.from_position_and_rpy(Vector3(), RPY.from_degrees((0.0, 90.0, 0.0)))
    relative_cube_tf = Transform3D.from_position_and_rpy(Vector3(0.0, 0.0, cube_height / 2))

    plane_tf = relative_plane_tf.transform_by(scene_tf)
    cube_tf = relative_cube_tf.transform_by(scene_tf)

    should_render = args.render

    scene = Scene()

    plane = RigidStatic.create_plane(material=Material(static_friction=1.0, dynamic_friction=0.6, restitution=0.0))
    plane.set_global_pose(tf_to_physx_tf(plane_tf))
    scene.add_actor(plane)

    actor = RigidDynamic()
    actor.attach_shape(
        Shape.create_box([1.0, 1.0, cube_height], Material(static_friction=0.15, dynamic_friction=0.1, restitution=0.0))
    )
    actor.set_global_pose(tf_to_physx_tf(cube_tf))
    actor.set_mass(1.0)
    scene.add_actor(actor)

    if should_render:
        render = PyPhysxViewer()
        render.add_physx_scene(scene)

    zs = []
    times = []
    rate = Rate(240)
    current_time = 0.0
    t0 = time.perf_counter()
    while current_time < 3.0:
        period = rate.period()
        scene.simulate(period)
        current_time += period
        pose = actor.get_global_pose()
        zs.append(pose[0][2])
        times.append(current_time)
        if should_render:
            render.update()
            rate.sleep()
    if should_render:
        render.close()

    t1 = time.perf_counter()
    print(t1 - t0)

    plt.plot(times, zs)
    plt.show()


if __name__ == "__main__":
    main()
