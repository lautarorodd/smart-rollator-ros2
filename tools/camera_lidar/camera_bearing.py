"""Reine Funktionen (ohne ROS) fuer den Winkeltest Kamera <-> LiDAR.

Konvention: Winkel in Grad, 0 = Gehrichtung, POSITIV = LINKS (wie ROS).
"""
import math

import numpy as np

# Kalibrierung der Gruppe F1 (C920, gueltig fuer 1280 x 720), halbiert fuer 640 x 360.
# fx, cx sind die einzigen Werte, die fuer den Blickwinkel noetig sind.
F1_FX_1280 = 981.724
F1_CX_1280 = 674.714
F1_W = 1280


def scaled_intrinsics(width, fx_ref=F1_FX_1280, cx_ref=F1_CX_1280, w_ref=F1_W):
    s = width / w_ref
    return fx_ref * s, cx_ref * s


def pixel_to_bearing_deg(u, fx, cx):
    """Pinhole-Formel. Bildspalte u (nach rechts waechst) -> Winkel (links positiv)."""
    return -math.degrees(math.atan2(u - cx, fx))


def find_color_centroid(rgb, hsv_lo, hsv_hi, min_area_px=150):
    """Schwerpunkt der groessten Farbflaeche. rgb: HxWx3 uint8.

    Gibt (u, v, area, mask) oder None zurueck. hsv_*: OpenCV-HSV (H 0-179).
    """
    import cv2
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, np.array(hsv_lo, np.uint8), np.array(hsv_hi, np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask)
    if n <= 1:
        return None
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    area = int(stats[i, cv2.CC_STAT_AREA])
    if area < min_area_px:
        return None
    return float(cents[i][0]), float(cents[i][1]), area, mask


def nearest_cluster_bearing_deg(ranges, angle_min, angle_increment, front_deg,
                                min_range=0.15, max_range=6.0,
                                half_fov_deg=50.0, window_m=0.15):
    """Winkel und Abstand des naechsten Objekts im vorderen Sektor.

    Nimmt den naechsten gueltigen Punkt und mittelt alle Punkte, deren
    Abstand sich um hoechstens window_m davon unterscheidet und die nahe
    beieinander liegen. Gibt (winkel_deg, abstand_m, n_punkte) oder None.
    """
    r = np.asarray([np.nan if v is None else v for v in ranges], float)
    ang = angle_min + np.arange(r.size) * angle_increment
    rel = np.rad2deg(np.arctan2(np.sin(ang - np.deg2rad(front_deg)),
                                np.cos(ang - np.deg2rad(front_deg))))
    ok = (np.isfinite(r) & (r >= min_range) & (r <= max_range)
          & (np.abs(rel) <= half_fov_deg))
    if not ok.any():
        return None
    r, rel = r[ok], rel[ok]
    i0 = int(np.argmin(r))
    sel = (np.abs(r - r[i0]) <= window_m) & (np.abs(rel - rel[i0]) <= 8.0)
    return float(np.mean(rel[sel])), float(np.mean(r[sel])), int(sel.sum())


def fit_bearings(lidar_deg, cam_deg):
    """Lineare Anpassung cam = a * lidar + b. Gibt (a, b, rms_rest_deg)."""
    x, y = np.asarray(lidar_deg, float), np.asarray(cam_deg, float)
    if x.size < 2:
        return None
    a, b = np.polyfit(x, y, 1)
    rest = y - (a * x + b)
    return float(a), float(b), float(np.sqrt(np.mean(rest ** 2)))


def shift_to_camera(angle_deg, dist_m, cam_x, cam_y):
    """Rechnet einen LiDAR-Punkt (Winkel links positiv, Abstand) in das System der Kamera um.

    cam_x: Lage der Kamera in Gehrichtung relativ zum LiDAR (negativ = hinter dem LiDAR).
    cam_y: seitliche Lage (links positiv).
    Gibt (winkel_deg, abstand_m) so zurueck, wie die Kamera das Objekt sieht.
    """
    a = math.radians(angle_deg)
    x = dist_m * math.cos(a) - cam_x
    y = dist_m * math.sin(a) - cam_y
    return math.degrees(math.atan2(y, x)), math.hypot(x, y)
