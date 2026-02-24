import matplotlib
matplotlib.use('TkAgg')

import math
import matplotlib.pyplot as plt
import time

# ---------------- PID ----------------
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output


# ---------------- BAŞLANGIÇ ----------------
x, y, theta = 0.0, 0.0, 0.0
target_x, target_y = 8.0, 6.0
v = 1.0
dt = 0.1

pid = PID(2.0, 0.0, 0.4)

x_path = []
y_path = []

# ---------------- ENGEL ----------------
obstacle_x, obstacle_y = 4.0, 3.0
obstacle_radius = 0.7

escape_mode = False
escape_x, escape_y = 0.0, 0.0

# ---------------- GRAFİK ----------------
plt.ion()
fig, ax = plt.subplots()
ax.set_xlim(-1, 10)
ax.set_ylim(-1, 10)
ax.set_aspect("equal")
ax.grid()
ax.set_title("PID + GÖRÜNEN Local Waypoint + Engel Kaçış")

robot_dot, = ax.plot([], [], "bo", markersize=10, label="Robot")
path_line, = ax.plot([], [], "b--")
target_dot, = ax.plot(target_x, target_y, "rx", markersize=12, label="Ana Hedef")

# ✅ LOCAL WAYPOINT NOKTASI GÖRÜNECEK
local_wp_dot, = ax.plot([], [], "go", markersize=8, label="Local Waypoint")

obstacle_circle = plt.Circle((obstacle_x, obstacle_y), obstacle_radius, color="red", alpha=0.5)
ax.add_patch(obstacle_circle)

ax.legend()

print("Simülasyon başlıyor...")

# ---------------- DÖNGÜ ----------------
running = True
while running:
    # ANA HEDEF MESAFESİ
    dist_target = math.hypot(target_x - x, target_y - y)
    if dist_target < 0.2:
        print("✅ Ana Hedefe Ulaşıldı!")
        break

    # ENGEL MESAFESİ
    dist_obstacle = math.hypot(obstacle_x - x, obstacle_y - y)

    # ---------------- LOCAL WAYPOINT ÜRET ----------------
    if dist_obstacle < 1.2 and not escape_mode:
        escape_mode = True

        angle_to_target = math.atan2(target_y - y, target_x - x)

        # ✅ SADECE 30 DERECE SAP
        escape_angle = angle_to_target + math.radians(70)

        escape_x = x + 1.0 * math.cos(escape_angle)
        escape_y = y + 0.5 * math.sin(escape_angle)

        print("⚠️ Engel Algılandı → Local Waypoint Üretildi")

    # ---------------- HEDEF SEÇİMİ ----------------
    if escape_mode:
        goal_x, goal_y = escape_x, escape_y
        local_wp_dot.set_data([escape_x], [escape_y])  # ✅ LOCAL NOKTA GÖZÜKÜR

        if math.hypot(goal_x - x, goal_y - y) < 0.2:
            escape_mode = False
            local_wp_dot.set_data([], [])
            print("✅ Engel Geçildi → Ana Hedefe Dönülüyor")
    else:
        goal_x, goal_y = target_x, target_y

    # ---------------- PID ----------------
    desired_theta = math.atan2(goal_y - y, goal_x - x)
    error = desired_theta - theta
    error = math.atan2(math.sin(error), math.cos(error))

    omega = pid.compute(error, dt)

    theta += omega * dt
    x += v * math.cos(theta) * dt
    y += v * math.sin(theta) * dt

    x_path.append(x)
    y_path.append(y)

    # ---------------- ÇİZİM ----------------
    robot_dot.set_data([x], [y])
    path_line.set_data(x_path, y_path)

    fig.canvas.draw()
    fig.canvas.flush_events()

    time.sleep(0.05)

plt.ioff()
plt.show()
