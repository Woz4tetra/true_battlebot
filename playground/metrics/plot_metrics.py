import csv

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

# path = "/data/videos/Cage-2-Overhead-High-2024-10-26_15-30-33.608_trim_images_world_points.csv"
path = "/data/videos/Cage-5-Overhead-High-2024-10-26_20-15-49.226_trim_images_world_points.csv"

object_data = []
with open(path) as file:
    csv_reader = csv.DictReader(file)
    for row in csv_reader:
        timestamp = float(row["timestamp"])
        object_id = int(row["object_id"])
        x = float(row["x"])
        y = float(row["y"])
        if object_id == 0:
            object_data.append((timestamp, x, y))
object_path = np.array(object_data)
xy = object_path[:, 1:]
times = object_path[:, 0]
distances = np.linalg.norm(np.diff(xy, axis=0), axis=1)
dt = np.diff(times)
speeds = distances / dt
average_dt = np.mean(dt)

figure_1 = plt.figure(1)
plt.hist(speeds, bins=100)

# Animate the object path
figure_2 = plt.figure(2)
plt.xlim(-1.5, 1.5)
plt.ylim(-1.5, 1.5)

(line,) = plt.plot([], [], lw=2)
plt.plot(xy[:, 0], xy[:, 1], lw=2, color="red")
plt.scatter(object_path[0, 1], object_path[0, 2], color="green")
plt.scatter(object_path[-1, 1], object_path[-1, 2], color="red")


def update(frame):
    line.set_data(object_path[:frame, 1], object_path[:frame, 2])
    return (line,)


ani = FuncAnimation(figure_2, update, frames=len(object_path), blit=True, interval=average_dt * 1000)


plt.show()
