import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import CubicSpline

# Sample data: time, position, and velocity
t = np.array([0, 1, 2, 3, 4])
x = np.random.rand(5)  # Random data for 2D
y = np.random.rand(5)  # Random data for 2D
vx = np.random.rand(3)  # Random velocity data
vy = np.random.rand(3)  # Random velocity data

# Ensure velocity data aligns with position data; extrapolate or interpolate if needed
# For simplicity, assuming velocities are at t[1:-1] and extrapolating start/end
vx_start = (x[1] - x[0]) / (t[1] - t[0])
vx_end = (x[-1] - x[-2]) / (t[-1] - t[-2])
vx_full = np.insert(vx, [0, len(vx)], [vx_start, vx_end])

vy_start = (y[1] - y[0]) / (t[1] - t[0])
vy_end = (y[-1] - y[-2]) / (t[-1] - t[-2])
vy_full = np.insert(vy, [0, len(vy)], [vy_start, vy_end])

pos = np.column_stack((x, y))
v_full = np.column_stack((vx_full, vy_full))

# Fit the cubic spline with specified boundary conditions (first derivatives)
cs = CubicSpline(t, pos, bc_type=((1, v_full[0]), (1, v_full[-1])))

# Generate a denser time array for plotting the spline
t_dense = np.linspace(t.min(), 4.1, 100)


# Example of evaluating the spline and its derivative
time_eval = 4.1
position_eval = cs(time_eval)
velocity_eval = cs(time_eval, 1)  # 1 for first derivative
print(f"Position at t={time_eval}: {position_eval}")
print(f"Velocity at t={time_eval}: {velocity_eval}")

# Plot the results in 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
ax.plot(t_dense, cs(t_dense)[:, 0], cs(t_dense)[:, 1], "-", label="Cubic Spline Fit")
ax.plot(t, x, y, "x", label="Position Points")
ax.plot([time_eval], [position_eval[0]], [position_eval[1]], "s", label=f"Position at t={time_eval}")
ax.plot([time_eval], [velocity_eval[0]], [velocity_eval[1]], "s", label=f"Velocity at t={time_eval}")
ax.set_xlabel("Time")
ax.set_ylabel("X Position")
ax.set_zlabel("Y Position")
ax.set_title("Cubic Spline Fit to Position and Velocity Data")
ax.legend()
plt.show()
