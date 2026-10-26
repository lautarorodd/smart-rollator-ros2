import math
import numpy as np
import cv2
from camera_bearing import *

# 1. Pixel -> Winkel: Mitte = 0, rechts = negativ (links positiv)
fx, cx = 490.862, 320.0
assert abs(pixel_to_bearing_deg(320, fx, cx)) < 1e-9
assert pixel_to_bearing_deg(500, fx, cx) < 0 and pixel_to_bearing_deg(100, fx, cx) > 0
# Randwinkel: fx -> HFOV
u_edge = 640
print('Randwinkel bei u=640:', round(pixel_to_bearing_deg(u_edge, fx, cx), 1), 'Grad (rechts)')

# 2. Farbflaeche finden (gruen), Position kontrollieren
img = np.zeros((360, 640, 3), np.uint8)
img[:] = (90, 90, 90)
img[150:200, 400:440] = (0, 220, 0)          # RGB gruen, Mitte x ~ 419.5
res = find_color_centroid(img, (40, 100, 100), (85, 255, 255))
assert res is not None and abs(res[0] - 419.5) < 1.0, res
assert find_color_centroid(np.full((360, 640, 3), 90, np.uint8), (40, 100, 100), (85, 255, 255)) is None

# 3. LiDAR-Winkel: Objekt bei +20 Grad links, 2 m; front_deg = 90 (Sensorwinkel)
inc = math.radians(0.1)
n = 3600
angle_min = -math.pi
ranges = [4.0] * n
for i in range(n):
    a = math.degrees(angle_min + i * inc)
    rel = (a - 90 + 180) % 360 - 180
    if abs(rel - 20) < 2.0:
        ranges[i] = 2.0
out = nearest_cluster_bearing_deg(ranges, angle_min, inc, 90.0)
assert abs(out[0] - 20) < 0.2 and abs(out[1] - 2.0) < 1e-6, out

# 4. Anpassung: cam = 1.05 * lidar + 2 -> a, b zurueck
l = np.array([-30, -15, 0, 15, 30.])
c = 1.05 * l + 2.0
a, b, rms = fit_bearings(l, c)
assert abs(a - 1.05) < 1e-9 and abs(b - 2.0) < 1e-9 and rms < 1e-9
# 5. Tiefenversatz: Kamera 0,19 m hinter dem LiDAR. Objekt 1 m vorn, 0,5 m links
ang = math.degrees(math.atan2(0.5, 1.0)); d = math.hypot(1.0, 0.5)
a2, d2 = shift_to_camera(ang, d, -0.19, 0.0)
assert abs(a2 - math.degrees(math.atan2(0.5, 1.19))) < 1e-9, a2
assert abs((ang - a2) - 3.8) < 0.1, (ang, a2)           # ca. 3,8 Grad Unterschied
# ohne Versatz: unveraendert
a3, d3 = shift_to_camera(ang, d, 0.0, 0.0)
assert abs(a3 - ang) < 1e-9 and abs(d3 - d) < 1e-9
# Objekt genau vorn: Winkel bleibt 0
assert abs(shift_to_camera(0.0, 2.0, -0.19, 0.0)[0]) < 1e-9
print('Alle Tests bestanden')
