import csv
import warnings

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection
from PIL import Image
from scipy.signal import savgol_filter

path = "/data/videos/Cage-2-Overhead-High-2024-10-26_15-30-33.608_trim_images_map_points.csv"
# path = "/data/videos/Cage-5-Overhead-High-2024-10-26_20-15-49.226_trim_images_map_points.csv"

background_image_path = "/data/videos/Cage-2-Overhead-High-2024-10-26_15-30-33.608_trim_images_background.jpg"
# background_image_path = "/data/videos/Cage-5-Overhead-High-2024-10-26_20-15-49.226_trim_images_background.jpg"


def colored_line(x, y, c, ax, **lc_kwargs):
    """
    Plot a line with a color specified along the line by a third value.

    It does this by creating a collection of line segments. Each line segment is
    made up of two straight lines each connecting the current (x, y) point to the
    midpoints of the lines connecting the current point with its two neighbors.
    This creates a smooth line with no gaps between the line segments.

    Parameters
    ----------
    x, y : array-like
        The horizontal and vertical coordinates of the data points.
    c : array-like
        The color values, which should be the same size as x and y.
    ax : Axes
        Axis object on which to plot the colored line.
    **lc_kwargs
        Any additional arguments to pass to matplotlib.collections.LineCollection
        constructor. This should not include the array keyword argument because
        that is set to the color argument. If provided, it will be overridden.

    Returns
    -------
    matplotlib.collections.LineCollection
        The generated line collection representing the colored line.
    """
    if "array" in lc_kwargs:
        warnings.warn('The provided "array" keyword argument will be overridden')

    # Default the capstyle to butt so that the line segments smoothly line up
    default_kwargs = {"capstyle": "butt"}
    default_kwargs.update(lc_kwargs)

    # Compute the midpoints of the line segments. Include the first and last points
    # twice so we don't need any special syntax later to handle them.
    x = np.asarray(x)
    y = np.asarray(y)
    x_midpts = np.hstack((x[0], 0.5 * (x[1:] + x[:-1]), x[-1]))
    y_midpts = np.hstack((y[0], 0.5 * (y[1:] + y[:-1]), y[-1]))

    # Determine the start, middle, and end coordinate pair of each line segment.
    # Use the reshape to add an extra dimension so each pair of points is in its
    # own list. Then concatenate them to create:
    # [
    #   [(x1_start, y1_start), (x1_mid, y1_mid), (x1_end, y1_end)],
    #   [(x2_start, y2_start), (x2_mid, y2_mid), (x2_end, y2_end)],
    #   ...
    # ]
    coord_start = np.column_stack((x_midpts[:-1], y_midpts[:-1]))[:, np.newaxis, :]
    coord_mid = np.column_stack((x, y))[:, np.newaxis, :]
    coord_end = np.column_stack((x_midpts[1:], y_midpts[1:]))[:, np.newaxis, :]
    segments = np.concatenate((coord_start, coord_mid, coord_end), axis=1)

    lc = LineCollection(segments, **default_kwargs)
    lc.set_array(c)  # set the colors of each segment

    return ax.add_collection(lc)


object_data_map = []
object_data_camera = []
with open(path) as file:
    csv_reader = csv.DictReader(file)
    for row in csv_reader:
        timestamp = float(row["timestamp"])
        object_id = int(row["object_id"])
        map_x = float(row["map_x"])
        map_y = float(row["map_y"])
        camera_x = float(row["camera_x"])
        camera_y = float(row["camera_y"])
        if object_id == 1:
            object_data_map.append((timestamp, map_x, map_y))
            object_data_camera.append((timestamp, camera_x, camera_y))
object_path_map = np.array(object_data_map)
object_path_camera = np.array(object_data_camera)
xy_map = object_path_map[:, 1:]
xy_camera = object_path_camera[:, 1:]
times = object_path_map[:, 0]
xy_map = savgol_filter(xy_map, 10, 3, axis=0)
distances = np.linalg.norm(np.diff(xy_map, axis=0), axis=1)

dt = np.diff(times)
speeds = distances / dt
average_dt = np.mean(dt)

background_image = np.array(Image.open(background_image_path))

figure_1 = plt.figure(1)

gradient_color = np.linspace(0, 1, len(xy_map))
cmap = plt.get_cmap("viridis")

axes_1 = figure_1.gca()
axes_1.imshow(background_image)
colored_line(xy_camera[:, 0], xy_camera[:, 1], gradient_color, axes_1)
(ani1_line,) = axes_1.plot([], [], lw=2)

# Animate the object path
figure_2 = plt.figure(2)

axes_2 = figure_2.subplots(2, 1)
axes_2[0].set_xlim(-1.5, 1.5)
axes_2[0].set_ylim(-1.5, 1.5)

(ani2_line,) = axes_2[0].plot([], [], lw=2)
colored_line(xy_map[:, 0], xy_map[:, 1], gradient_color, axes_2[0])
axes_2[0].scatter(xy_map[0, 0], xy_map[0, 1], color="green")
axes_2[0].scatter(xy_map[-1, 0], xy_map[-1, 1], color="red")
axes_2[0].set_xlabel("X (m)")
axes_2[0].set_ylabel("Y (m)")
axes_2[0].set_title("Object Path")
axes_2[0].set_aspect("equal")

axes_2[1].hist(speeds, bins=100)
axes_2[1].set_xlabel("Speed (m/s)")
axes_2[1].set_ylabel("Count")
axes_2[1].set_title("Speed Histogram")

# def update_map(frame):
#     ani2_line.set_data(object_path_map[:frame, 1], object_path_map[:frame, 2])
#     return (ani2_line,)


# def update_camera(frame):
#     ani1_line.set_data(object_path_camera[:frame, 1], object_path_camera[:frame, 2])
#     return (ani1_line,)


# ani1 = FuncAnimation(figure_1, update_camera, frames=len(object_path_camera), blit=True, interval=average_dt * 1000)
# ani2 = FuncAnimation(figure_2, update_map, frames=len(object_path_map), blit=True, interval=average_dt * 1000)


plt.show()
