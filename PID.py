import matplotlib
matplotlib.use('TkAgg')  # <--- BU SATIR PYCHARM İÇİN EKLENDİq
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
target_x, target_y = 8.0, 6.0
v = 1.0
dt = 0.1
pid = PID(Kp=2.0, Ki=0.0, Kd=0.5)

x_path = []
y_path = []

# ---------------- GRAFİK PENCERESİ ----------------
plt.ion()  # İnteraktif modu aç
fig, ax = plt.subplots()
ax.set_xlim(-1, 10)
ax.set_ylim(-1, 10)
ax.set_aspect("equal")
ax.grid()
ax.set_title("Canlı PID Takibi")

robot_dot, = ax.plot([], [], "bo", markersize=10, label="Robot")
path_line, = ax.plot([], [], "b--", alpha=0.5)
target_dot, = ax.plot(target_x, target_y, "rx", markersize=12, label="Hedef")
ax.legend()

print("Simülasyon başlıyor...")

# ---------------- DÖNGÜ ----------------
running = True
while running:
    # Mesafe kontrolü
    dist = math.sqrt((target_x - x) ** 2 + (target_y - y) ** 2)
    if dist < 0.1:
        print("Hedefe Ulaşıldı!")
        running = False
        continue

    # PID ve Hareket
    desired_theta = math.atan2(target_y - y, target_x - x)
    error = desired_theta - theta
    error = math.atan2(math.sin(error), math.cos(error))  # Açı normalizasyonu

    omega = pid.compute(error, dt)

    theta += omega * dt
    x += v * math.cos(theta) * dt
    y += v * math.sin(theta) * dt

    x_path.append(x)
    y_path.append(y)

    # Güncelleme
    robot_dot.set_data([x], [y])
    path_line.set_data(x_path, y_path)

    fig.canvas.draw()
    fig.canvas.flush_events()

    time.sleep(0.05)  # Hızı görmek için biraz yavaşlatıyoruz

plt.ioff()
plt.show()