import matplotlib
matplotlib.use('TkAgg')

import math
import matplotlib.pyplot as plt
import time

# ---------------- PID SINIFI ----------------
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


# ---------------- AYARLAR ----------------
x, y, theta = 0.0, 0.0, 0.0
v = 1.0
dt = 0.1
pid = PID(Kp=2.0, Ki=0.0, Kd=0.5)

# ✅ WAYPOINT LİSTESİ
waypoints = [
    (2, 1),
    (6, 2),
    (8, 6),
    (5, 9),
    (1, 7)
]

current_wp = 0

x_path = []
y_path = []

# ---------------- GRAFİK ----------------
plt.ion()
fig, ax = plt.subplots()
ax.set_xlim(-1, 10)
ax.set_ylim(-1, 10)
ax.set_aspect("equal")
ax.grid()
ax.set_title("Waypoint + PID Takibi")

robot_dot, = ax.plot([], [], "bo", markersize=10)
path_line, = ax.plot([], [], "b--", alpha=0.5)

# Waypointleri çiz
for wp in waypoints:
    ax.plot(wp[0], wp[1], "rx")

print("Simülasyon başlıyor...")

# ---------------- DÖNGÜ ----------------
running = True
while running:

    # ✅ TÜM WAYPOINTLER BİTTİ Mİ?
    if current_wp >= len(waypoints):
        print("✅ TÜM WAYPOINTLER TAMAMLANDI!")
        break

    target_x, target_y = waypoints[current_wp]

    dist = math.sqrt((target_x - x) ** 2 + (target_y - y) ** 2)

    # ✅ HEDEF NOKTAYA GELİNCE SONRAKİ WAYPOINT
    if dist < 0.2:
        print(f"✅ {current_wp + 1}. Waypoint geçildi")
        current_wp += 1
        continue

    # PID ve Hareket
    desired_theta = math.atan2(target_y - y, target_x - x)
    error = desired_theta - theta
    error = math.atan2(math.sin(error), math.cos(error))

    omega = pid.compute(error, dt)

    theta += omega * dt
    x += v * math.cos(theta) * dt
    y += v * math.sin(theta) * dt

    x_path.append(x)
    y_path.append(y)

    # Güncelle
    robot_dot.set_data([x], [y])
    path_line.set_data(x_path, y_path)

    fig.canvas.draw()
    fig.canvas.flush_events()

    time.sleep(0.05)

plt.ioff()
plt.show()
