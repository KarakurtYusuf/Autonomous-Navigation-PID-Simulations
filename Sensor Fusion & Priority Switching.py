import matplotlib

matplotlib.use('TkAgg')

import math
import matplotlib.pyplot as plt
import time
import numpy as np


# ---------------- PID KONTROLCÜSÜ ----------------
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output


# ---------------- GEOMETRİ VE MATEMATİK ----------------

def check_line_of_sight(rx, ry, tx, ty, ox, oy, safe_r):
    """
    Robot ile Hedef arasında engel var mı?
    safe_r: Engelin güvenlik yarıçapı (robot bu yarıçapa girmemeli)
    """
    dx = tx - rx
    dy = ty - ry
    if dx == 0 and dy == 0: return True

    # Doğru parçasının engele en yakın noktası
    t = ((ox - rx) * dx + (oy - ry) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))  # Doğru parçası üzerinde sınırla

    closest_x = rx + t * dx
    closest_y = ry + t * dy

    dist = math.hypot(closest_x - ox, closest_y - oy)

    # Eğer en yakın nokta güvenlik yarıçapından büyükse görüş var demektir.
    return dist > safe_r


def calculate_avoidance_waypoint(rx, ry, ox, oy, target_r):
    """
    Robotun konumundan, engelin 'target_r' yarıçaplı sanal çemberine teğet noktalar bulur.
    target_r: Waypoint'in yerleştirileceği geniş çemberin yarıçapı.
    """
    dx = ox - rx
    dy = oy - ry
    d = math.hypot(dx, dy)

    # Robot zaten bu çemberin içindeyse hesaplanamaz (veya çok geç kalınmıştır)
    if d <= target_r:
        return None

    angle_to_obs = math.atan2(dy, dx)

    # Teğet açısı (Matematiksel Teorem)
    try:
        offset_angle = math.acos(target_r / d)
    except ValueError:
        return None

    # İki olası teğet noktası (Sağ ve Sol)
    a1 = angle_to_obs + offset_angle
    a2 = angle_to_obs - offset_angle

    p1 = (ox + target_r * math.cos(a1), oy + target_r * math.sin(a1))
    p2 = (ox + target_r * math.cos(a2), oy + target_r * math.sin(a2))

    return p1, p2


def pick_best_waypoint(p1, p2, tx, ty):
    """Hedefe en yakın olan waypoint'i seç."""
    d1 = math.hypot(p1[0] - tx, p1[1] - ty)
    d2 = math.hypot(p2[0] - tx, p2[1] - ty)
    return p1 if d1 < d2 else p2


# ---------------- SİMÜLASYON AYARLARI ----------------
x, y, theta = 0.0, 0.0, 0.0  # Robot Başlangıç
target_x, target_y = 10.0, 5.0  # Hedef

# Engel Tanımları
obs_x, obs_y = 4.0, 2.5
obs_r = 1.0  # Engelin Kırmızı Görünen Yarıçapı

# --- KRİTİK AYARLAR ---
# 1. Güvenlik Marjı: Robotun asla girmemesi gereken "Dashed Red Line" sınırı
safety_margin = 0.5
safe_zone_r = obs_r + safety_margin

# 2. Waypoint Yarıçapı: Yeşil noktanın konulacağı yer.
# PÜF NOKTA: Bunu güvenlik sınırından DAHA BÜYÜK yapıyoruz ki robot oraya giderken güvenlik sınırını kesmesin.
waypoint_calc_r = safe_zone_r + 0.8

# Algılama Mesafesi (Ne kadar uzaktan tepki versin?)
detection_dist = waypoint_calc_r + 2.0

v = 1.0
dt = 0.05
pid = PID(3.0, 0.0, 0.8)  # Dönüşler biraz sert olsun

# Durum Değişkenleri
escape_mode = False
wp_x, wp_y = None, None
x_path, y_path = [], []

# ---------------- GÖRSELLEŞTİRME ----------------
plt.ion()
fig, ax = plt.subplots(figsize=(10, 6))
fig.patch.set_facecolor('#111111')
ax.set_facecolor('#111111')
ax.set_xlim(-1, 12)
ax.set_ylim(-2, 8)
ax.set_aspect("equal")
ax.grid(True, linestyle=':', alpha=0.2, color='white')

# 1. Fiziksel Engel (Koyu Kırmızı)
obs_patch = plt.Circle((obs_x, obs_y), obs_r, color='#800000', alpha=0.9, zorder=3)
ax.add_patch(obs_patch)
ax.text(obs_x, obs_y, "ENGEL", color='white', ha='center', va='center', fontsize=8, fontweight='bold', zorder=4)

# 2. Güvenlik Alanı (Kesik Çizgili Kırmızı) - Robot buraya girmemeli
safe_patch = plt.Circle((obs_x, obs_y), safe_zone_r, color='red', fill=False, linestyle='--', linewidth=1.5, alpha=0.8,
                        zorder=3)
ax.add_patch(safe_patch)

# 3. Waypoint Yörüngesi (Sadece referans için silik gri çember)
calc_patch = plt.Circle((obs_x, obs_y), waypoint_calc_r, color='gray', fill=False, linestyle=':', alpha=0.3)
ax.add_patch(calc_patch)

# Çizim Elemanları
robot_plot, = ax.plot([], [], marker='o', color='#00ffff', markersize=8, zorder=5)
robot_dir, = ax.plot([], [], color='#00ffff', linewidth=2, zorder=5)
path_plot, = ax.plot([], [], color='white', linestyle='--', linewidth=1, alpha=0.5)
target_plot, = ax.plot(target_x, target_y, 'mX', markersize=12, label='Hedef')
wp_plot, = ax.plot([], [], 'o', color='#00ff00', markersize=8, zorder=6)  # YEŞİL WAYPOINT

# Bilgi Kutusu
status_text = ax.text(0.02, 0.95, "", transform=ax.transAxes, color='white', fontsize=10,
                      bbox=dict(facecolor='black', edgecolor='white', alpha=0.8), va='top')

print("Simülasyon Başlıyor...")

# ---------------- ANA DÖNGÜ ----------------
while True:
    # Mesafeler
    dist_to_target = math.hypot(target_x - x, target_y - y)
    dist_to_obs = math.hypot(obs_x - x, obs_y - y)

    # 1. Hedefe Ulaşıldı mı?
    if dist_to_target < 0.3:
        status_text.set_text(">> HEDEFE ULASILDI <<")
        status_text.set_color("#00ff00")
        fig.canvas.draw()
        print("Bitti.")
        break

    # 2. Görüş Hattı Kontrolü (Line of Sight)
    # Robotun güvenlik çemberine teğet geçip geçmediğine bakar
    has_los = check_line_of_sight(x, y, target_x, target_y, obs_x, obs_y, safe_zone_r)

    # Kamera Modu: Görüş açık VE engel kaçış modunda değiliz (veya moddan çıkabiliriz)
    # Ancak "Escape Mode" aktifse, waypoint'e ulaşana kadar kamerayı devre dışı bırakalım.

    current_target_x, current_target_y = target_x, target_y
    info_str = "MOD: NORMAL SEYIR"
    color_code = "white"

    # --- KAÇIŞ MANTIĞI ---

    # Eğer engele yaklaşılıyorsa (Detection Zone) VE Görüş Yoksa -> Hesapla
    if dist_to_obs < detection_dist and not has_los:
        if not escape_mode:
            # Waypoint Hesapla (Geniş Yarıçap Kullanarak)
            tangents = calculate_avoidance_waypoint(x, y, obs_x, obs_y, waypoint_calc_r)

            if tangents:
                escape_mode = True
                p1, p2 = tangents
                # Hedefe en mantıklı olanı seç
                wp_x, wp_y = pick_best_waypoint(p1, p2, target_x, target_y)
                print(f"Engel Algılandı! Yeni Rota: {wp_x:.2f}, {wp_y:.2f}")

    # Eğer Kaçış Modundaysak
    if escape_mode:
        info_str = "MOD: ENGELDEN KACIS (LIDAR)"
        color_code = "#ffaa00"  # Turuncu

        # Hedef artık Waypoint (Yeşil Nokta)
        current_target_x, current_target_y = wp_x, wp_y
        wp_plot.set_data([wp_x], [wp_y])

        # Waypoint'e mesafe
        dist_to_wp = math.hypot(x - wp_x, y - wp_y)

        # Waypoint'e geldik mi? (Veya çok yaklaştık mı?)
        if dist_to_wp < 0.5:
            # Şimdi tekrar hedefe bak, yol açık mı?
            if check_line_of_sight(x, y, target_x, target_y, obs_x, obs_y, safe_zone_r):
                escape_mode = False
                wp_plot.set_data([], [])  # Noktayı gizle
                print("Waypoint geçildi, yol açık. Hedefe dönülüyor.")
            else:
                # Yol hala kapalıysa (çok nadir olur), escape modunda kal ve çıkışa devam et
                # Burada basitlik adına escape mode'u kapatıp PID'nin halletmesini bekleyebiliriz
                # veya ikinci bir waypoint hesaplatabiliriz.
                # Şimdilik: Modu kapat, döngü tekrar algılayıp yeni nokta koyacaktır.
                escape_mode = False

    else:
        # Normal Mod
        if has_los:
            info_str = "MOD: KAMERA KILITLEME"
            color_code = "#00ffff"  # Cyan
        wp_plot.set_data([], [])

    # --- HAREKET KONTROLÜ (PID) ---
    desired_theta = math.atan2(current_target_y - y, current_target_x - x)
    error = desired_theta - theta
    # Açıyı -pi, +pi arasına sıkıştır
    error = math.atan2(math.sin(error), math.cos(error))

    omega = pid.compute(error, dt)

    theta += omega * dt
    x += v * math.cos(theta) * dt
    y += v * math.sin(theta) * dt

    # --- ÇİZİM GÜNCELLEME ---
    x_path.append(x)
    y_path.append(y)

    path_plot.set_data(x_path, y_path)
    robot_plot.set_data([x], [y])
    robot_dir.set_data([x, x + 0.6 * math.cos(theta)], [y, y + 0.6 * math.sin(theta)])

    status_text.set_text(f"{info_str}\nWP Dist: {0.0 if not escape_mode else math.hypot(x - wp_x, y - wp_y):.2f}")
    status_text.set_color(color_code)
    if escape_mode:
        status_text.set_bbox(dict(facecolor='black', edgecolor='red', alpha=0.8))
    else:
        status_text.set_bbox(dict(facecolor='black', edgecolor='white', alpha=0.8))

    fig.canvas.draw()
    fig.canvas.flush_events()
    time.sleep(0.02)

plt.ioff()
plt.show()