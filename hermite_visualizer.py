import numpy as np
import matplotlib.pyplot as plt

# Hermite spline point generator
def get_hermite_point(t, p_start, angle_start, p_end, angle_end, magnitude):
    v_start_x = np.sin(np.radians(angle_start)) * magnitude
    v_start_y = np.cos(np.radians(angle_start)) * magnitude
    v_end_x = np.sin(np.radians(angle_end)) * magnitude
    v_end_y = np.cos(np.radians(angle_end)) * magnitude

    t2 = t * t
    t3 = t2 * t
    h00 = 2 * t3 - 3 * t2 + 1
    h10 = t3 - 2 * t2 + t
    h01 = -2 * t3 + 3 * t2
    h11 = t3 - t2

    x = h00 * p_start[0] + h10 * v_start_x + h01 * p_end[0] + h11 * v_end_x
    y = h00 * p_start[1] + h10 * v_start_y + h01 * p_end[1] + h11 * v_end_y
    return x, y

# Parameters (edit these for your bot)
p_start = (0, 0)
angle_start = 0      # degrees, 0 = facing up
p_end = (5, 0)
angle_end = 180       # degrees, 90 = facing right
tightness = 10       # higher = wider curve

# Generate curve points
ts = np.linspace(0, 1, 100)
curve = [get_hermite_point(t, p_start, angle_start, p_end, angle_end, tightness) for t in ts]
xs, ys = zip(*curve)

# Plot
plt.figure(figsize=(8, 8))
plt.plot(xs, ys, label='Hermite Spline Curve')
plt.scatter([p_start[0], p_end[0]], [p_start[1], p_end[1]], color='red', label='Start/End')

# Draw direction vectors
plt.arrow(p_start[0], p_start[1], np.sin(np.radians(angle_start))*tightness, np.cos(np.radians(angle_start))*tightness,
          head_width=2, head_length=3, color='green', label='Start Heading')
plt.arrow(p_end[0], p_end[1], np.sin(np.radians(angle_end))*tightness, np.cos(np.radians(angle_end))*tightness,
          head_width=2, head_length=3, color='blue', label='End Heading')

plt.title('Hermite Spline Path Visualization')
plt.xlabel('X (inches)')
plt.ylabel('Y (inches)')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()
