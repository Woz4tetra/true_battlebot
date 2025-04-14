import time

from matplotlib import pyplot as plt
from pyphysx_utils.rate import Rate

from pyphysx import Material, RigidDynamic, RigidStatic, Scene, Shape  # type: ignore

scene = Scene()
scene.add_actor(RigidStatic.create_plane(material=Material(static_friction=1.0, dynamic_friction=0.5, restitution=0.0)))

actor = RigidDynamic()
actor.attach_shape(Shape.create_box([0.2] * 3, Material(restitution=1.0)))
actor.set_global_pose([0.5, 0.5, 1.0])
actor.set_mass(1.0)
scene.add_actor(actor)

zs = []
times = []
rate = Rate(400)
current_time = 0.0
t0 = time.perf_counter()
while current_time < 3.0:
    period = rate.period()
    scene.simulate(period)
    current_time += period
    pose = actor.get_global_pose()
    zs.append(pose[0][2])
    times.append(current_time)
t1 = time.perf_counter()
print(t1 - t0)

plt.plot(times, zs)
plt.show()
