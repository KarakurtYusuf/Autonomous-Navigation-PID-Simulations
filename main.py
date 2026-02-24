import matplotlib.pyplot as plt
import math

# Başlangıç
x, y, theta = 0, 0, 0
v = 1.0  # m/s
delta = math.radians(10)  # 10 derece
L = 0.5  # aks mesafesi
dt = 0.1  # zaman adımı

x_list, y_list = [x], [y]

for _ in range(50):
    x += v * math.cos(theta) * dt
    y += v * math.sin(theta) * dt
    theta += (v / L) * math.tan(delta) * dt
    x_list.append(x)
    y_list.append(y)

plt.plot(x_list, y_list)
plt.axis('equal')
plt.show()
