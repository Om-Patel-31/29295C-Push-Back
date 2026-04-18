import numpy as np
import matplotlib.pyplot as plt

# ===============================
# Robot-style Hermite Spline
# 0° = facing +Y
# X right, Y forward
# ===============================

def get_hermite_point(t, p_start, h_start_deg, p_end, h_end_deg):
    # Distance-based tangent magnitude (robot realistic)
    dx = p_end[0] - p_start[0]
    dy = p_end[1] - p_start[1]
    distance = np.hypot(dx, dy)

    magnitude = distance  # auto-scale for natural curves

    # Robot heading convention (0° = +Y)
    v_start_x = np.sin(np.radians(h_start_deg)) * magnitude
    v_start_y = np.cos(np.radians(h_start_deg)) * magnitude
    v_end_x   = np.sin(np.radians(h_end_deg))   * magnitude
    v_end_y   = np.cos(np.radians(h_end_deg))   * magnitude

    t2 = t * t
    t3 = t2 * t

    h00 = 2*t3 - 3*t2 + 1
    h10 = t3 - 2*t2 + t
    h01 = -2*t3 + 3*t2
    h11 = t3 - t2

    x = h00*p_start[0] + h10*v_start_x + h01*p_end[0] + h11*v_end_x
    y = h00*p_start[1] + h10*v_start_y + h01*p_end[1] + h11*v_end_y

    return x, y


# ====== CHANGE THESE LIKE YOUR ROBOT ======

p_start = (0, 0)
heading_start = 0        # Facing forward
p_end = (-20, 20)         # 24in right, 24in forward
heading_end = -50        # End facing right

# ==========================================

ts = np.linspace(0, 1, 100)
curve = [get_hermite_point(t, p_start, heading_start, p_end, heading_end) for t in ts]
xs, ys = zip(*curve)

plt.figure(figsize=(7,7))
plt.plot(xs, ys, label="Robot Path")
plt.scatter([p_start[0], p_end[0]],
            [p_start[1], p_end[1]],
            color='red')

# Draw heading arrows
arrow_scale = 10
plt.arrow(p_start[0], p_start[1],
          np.sin(np.radians(heading_start))*arrow_scale,
          np.cos(np.radians(heading_start))*arrow_scale,
          head_width=2, color='green')

plt.arrow(p_end[0], p_end[1],
          np.sin(np.radians(heading_end))*arrow_scale,
          np.cos(np.radians(heading_end))*arrow_scale,
          head_width=2, color='blue')

plt.title("VEX Robot Hermite Path")
plt.xlabel("X (inches)")
plt.ylabel("Y (inches)")
plt.axis("equal")
plt.grid(True)
plt.show()