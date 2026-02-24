import matplotlib
matplotlib.use("TkAgg")

import math
import time
import matplotlib.pyplot as plt

# =====================================================
# STATE DEFINITIONS (INDUSTRY STYLE)
# =====================================================
TARGET_TRACKING   = "TARGET_TRACKING (CAMERA)"
AVOID_OBSTACLE    = "AVOID_OBSTACLE (LIDAR)"
RETURN_TO_TARGET  = "RETURN_TO_TARGET"

# =====================================================
# ROBOT & ENVIRONMENT
# =====================================================
x, y, theta = 0.0, 0.0, 0.0
target_x, target_y = 10.0, 4.0

obs_x, obs_y = 5.0, 2.0
obs_r = 1.2

safe_r   = obs_r + 0.5
orbit_r  = safe_r + 0.6

v = 0.8
dt = 0.05

# =====================================================
# PID CONTROLLER
# =====================================================
class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.prev = 0.0
        self.intg = 0.0

    def compute(self, err, dt):
        self.intg += err * dt
        d = (err - self.prev) / dt
        self.prev = err
        return self.kp * err + self.ki * self.intg + self.kd * d

pid = PID(2.5, 0.0, 1.0)

# =====================================================
# GEOMETRY FUNCTIONS
# =====================================================
def check_los(rx, ry, tx, ty, ox, oy, limit_r):
    dx, dy = tx - rx, ty - ry
    if dx == 0 and dy == 0:
        return True

    t = ((ox - rx) * dx + (oy - ry) * dy) / (dx*dx + dy*dy)
    t = max(0.0, min(1.0, t))

    cx = rx + t * dx
    cy = ry + t * dy

    return math.hypot(cx - ox, cy - oy) > limit_r


def calc_waypoints(rx, ry, ox, oy, r):
    dx, dy = ox - rx, oy - ry
    d = math.hypot(dx, dy)
    if d <= r:
        return None

    a = math.atan2(dy, dx)
    b = math.acos(r / d)

    w1 = (ox + r * math.cos(a + b), oy + r * math.sin(a + b))
    w2 = (ox + r * math.cos(a - b), oy + r * math.sin(a - b))
    return w1, w2


def choose_best_wp(w1, w2, tx, ty):
    d1 = math.hypot(w1[0] - tx, w1[1] - ty)
    d2 = math.hypot(w2[0] - tx, w2[1] - ty)
    return w1 if d1 < d2 else w2

# =====================================================
# VISUALIZATION
# =====================================================
plt.ion()
fig, ax = plt.subplots(figsize=(10, 6))
ax.set_xlim(-1, 12)
ax.set_ylim(-2, 8)
ax.set_aspect("equal")
ax.grid(True)

ax.add_patch(plt.Circle((obs_x, obs_y), obs_r, color="red", alpha=0.8))
ax.add_patch(plt.Circle((obs_x, obs_y), safe_r, fill=False, ls="--", color="red"))
ax.add_patch(plt.Circle((obs_x, obs_y), orbit_r, fill=False, ls=":", color="green"))

robot_dot, = ax.plot([], [], "co", ms=8)
robot_dir, = ax.plot([], [], "c", lw=2)
path_plot, = ax.plot([], [], "--", color="gray")
target_plot, = ax.plot(target_x, target_y, "mx", ms=12, mew=3)
wp_plot, = ax.plot([], [], "go", ms=8)

info = ax.text(
    0.02, 0.95, "", transform=ax.transAxes,
    fontsize=10, va="top",
    bbox=dict(facecolor="black", alpha=0.8),
    color="white"
)

# =====================================================
# STATE MACHINE VARIABLES
# =====================================================
state = TARGET_TRACKING
reason = "INITIAL"
active_wp = None

path_x, path_y = [], []

print("Simulation started...")

# =====================================================
# MAIN LOOP
# =====================================================
while True:

    dist_target = math.hypot(target_x - x, target_y - y)
    dist_obs = math.hypot(obs_x - x, obs_y - y)

    if dist_target < 0.3:
        print("TARGET REACHED")
        break

    los = check_los(x, y, target_x, target_y, obs_x, obs_y, safe_r)

    # =======================
    # STATE MACHINE LOGIC
    # =======================
    if state == TARGET_TRACKING:
        if not los:
            state = AVOID_OBSTACLE
            reason = "LOS BLOCKED"
            w1, w2 = calc_waypoints(x, y, obs_x, obs_y, orbit_r)
            active_wp = choose_best_wp(w1, w2, target_x, target_y)

    elif state == AVOID_OBSTACLE:
        if math.hypot(active_wp[0] - x, active_wp[1] - y) < 0.5 and dist_obs > orbit_r:
            state = RETURN_TO_TARGET
            reason = "WAYPOINT PASSED"

    elif state == RETURN_TO_TARGET:
        if los:
            state = TARGET_TRACKING
            reason = "LOS RESTORED"
            active_wp = None

    # =======================
    # CONTROL TARGET
    # =======================
    if state == AVOID_OBSTACLE:
        gx, gy = active_wp
        wp_plot.set_data([gx], [gy])
    else:
        gx, gy = target_x, target_y
        wp_plot.set_data([], [])

    desired = math.atan2(gy - y, gx - x)
    err = (desired - theta + math.pi) % (2*math.pi) - math.pi

    omega = pid.compute(err, dt)
    omega = max(min(omega, 3.0), -3.0)

    theta += omega * dt
    x += v * math.cos(theta) * dt
    y += v * math.sin(theta) * dt

    # =======================
    # DRAW
    # =======================
    path_x.append(x)
    path_y.append(y)

    robot_dot.set_data([x], [y])
    robot_dir.set_data([x, x + 0.6*math.cos(theta)],
                       [y, y + 0.6*math.sin(theta)])
    path_plot.set_data(path_x, path_y)

    info.set_text(
        f"ACTIVE STATE:\n{state}\n\n"
        f"TRANSITION:\n{reason}\n\n"
        f"LOGIC:\n"
        f"TARGET → CAMERA\n"
        f"OBSTACLE → LIDAR\n"
        f"WAYPOINT → RETURN"
    )

    fig.canvas.draw()
    fig.canvas.flush_events()
    time.sleep(0.02)

plt.ioff()
plt.show()
