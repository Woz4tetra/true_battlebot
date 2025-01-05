import csv

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

path = "/data/videos/Cage-2-Overhead-High-2024-10-26_15-30-33.608_trim_images_world_points.csv"

object_data = []
with open(path) as file:
    csv_reader = csv.DictReader(file)
    for row in csv_reader:
        timestamp = float(row["timestamp"])
        object_id = int(row["object_id"])
        x = float(row["x"])
        y = float(row["y"])
        if object_id == 1:
            object_data.append((timestamp, x, y))
object_path = np.array(object_data)

distances = np.linalg.norm(np.diff(object_path[:, 1:], axis=0), axis=1)
speeds = distances / np.diff(object_path[:, 0])
figure_1 = plt.figure(1)
plt.hist(speeds, bins=100)

# Animate the object path
figure_2 = plt.figure(2)
plt.xlim(-1.5, 1.5)
plt.ylim(-1.5, 1.5)

(line,) = plt.plot([], [], lw=2)
plt.plot(object_path[:, 1], object_path[:, 2], lw=2, color="red")
plt.scatter(object_path[0, 1], object_path[0, 2], color="green")
plt.scatter(object_path[-1, 1], object_path[-1, 2], color="red")


def update(frame):
    line.set_data(object_path[:frame, 1], object_path[:frame, 2])
    return (line,)


ani = FuncAnimation(figure_2, update, frames=len(object_path), blit=True, interval=30)


plt.show()
