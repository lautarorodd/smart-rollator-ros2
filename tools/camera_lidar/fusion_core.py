"""Reine Fusionslogik Kamera-Detektion + LiDAR (ohne ROS).

Idee: Die Kamera sagt WAS (Klasse) und in welchem Winkelbereich (Boxkanten),
der LiDAR sagt WIE WEIT. Der Winkelbereich der Box waehlt die LiDAR-Punkte aus;
der Bodenkontakt der Box (Unterkante) liefert eine unabhaengige Abstandsschaetzung,
die hilft, das richtige Objekt (und nicht die Wand dahinter) zu waehlen.

Konvention: 0 Grad = Gehrichtung, positiv = links; x vorn, y links; Meter.
"""
import math
from dataclasses import dataclass

import numpy as np

# Kameraoptik: Kalibrierung Gruppe F1 (C920, 1280x720), halbiert fuer 640x360
BASE_W = 640.0
FX0, FY0, CX0, CY0 = 490.862, 491.128, 337.357, 177.672

GERMAN = {
    'chair': 'Stuhl', 'person': 'Person', 'couch': 'Sofa', 'dining table': 'Tisch',
    'bottle': 'Flasche', 'potted plant': 'Pflanze', 'tv': 'Fernseher', 'laptop': 'Laptop',
    'backpack': 'Rucksack', 'bench': 'Bank', 'bicycle': 'Fahrrad', 'car': 'Auto',
    'dog': 'Hund', 'cat': 'Katze', 'suitcase': 'Koffer', 'bed': 'Bett',
    'refrigerator': 'Kuehlschrank', 'umbrella': 'Regenschirm', 'cup': 'Becher',
    'book': 'Buch', 'handbag': 'Handtasche', 'door': 'Tuer',
}


# Klassen, deren Unterkante normalerweise den Boden beruehrt (Bodenschaetzung sinnvoll).
# Alles andere (Rucksack, Flasche, Tasse, Laptop ...) kann in der Hand, auf einem Tisch usw. sein.
FLOOR_CLASSES = {
    'person', 'chair', 'couch', 'dining table', 'bench', 'bed', 'refrigerator', 'toilet',
    'bicycle', 'motorcycle', 'car', 'bus', 'truck', 'dog', 'cat', 'horse', 'potted plant',
    'suitcase', 'oven', 'sink',
}


@dataclass
class FusionParams:
    front_angle_deg: float = 0.0
    cam_x: float = -0.19          # Kamera relativ zum LiDAR in Gehrichtung (hinter = negativ)
    cam_y: float = 0.0            # seitlich, links positiv
    cam_height: float = 0.46      # Hoehe der Linse ueber dem Boden
    cam_pitch_deg: float = 0.0    # Neigung nach unten
    margin_deg: float = 2.0       # Sicherheitsrand um die Boxkanten
    min_range: float = 0.15
    max_range: float = 6.0
    min_points: int = 3
    cluster_gap: float = 0.25
    use_floor: bool = True
    fx: float = None
    fy: float = None
    cx: float = None
    cy: float = None


def intrinsics(p, img_w):
    s = img_w / BASE_W
    return (p.fx or FX0 * s, p.fy or FY0 * s, p.cx or CX0 * s, p.cy or CY0 * s)


def bbox_bearing_interval_deg(x1, x2, fx, cx, margin_deg=2.0):
    """Winkelbereich (rechts, links) einer Box; kleinere Bildspalte = weiter links."""
    left = -math.degrees(math.atan2(x1 - cx, fx))
    right = -math.degrees(math.atan2(x2 - cx, fx))
    return right - margin_deg, left + margin_deg


def floor_distance_m(v_bottom, img_h, cam_height, fy, cy, pitch_deg=0.0):
    """Abstand (in Gehrichtung, ab Kamera) des Bodenkontakts der Boxunterkante.

    None, wenn die Box unten am Bildrand abgeschnitten ist (Boden nicht sichtbar).
    """
    if v_bottom >= img_h - max(3.0, 0.04 * img_h):    # Box reicht bis zum unteren Bildrand
        return None
    down = math.atan2(v_bottom - cy, fy) + math.radians(pitch_deg)
    if down <= math.radians(1.0):
        return None
    return cam_height / math.tan(down)


def lidar_xy(ranges, angle_min, inc, p):
    r = np.array([np.nan if v is None else v for v in ranges], float)
    ang = angle_min + np.arange(r.size) * inc
    rel = ang - math.radians(p.front_angle_deg)
    ok = np.isfinite(r) & (r >= p.min_range) & (r <= p.max_range)
    return r[ok] * np.cos(rel[ok]), r[ok] * np.sin(rel[ok])


def fuse_detection(det, ranges, angle_min, inc, p):
    """det: dict mit x1,y1,x2,y2,img_w,img_h. Gibt dict oder None zurueck."""
    fx, fy, cx, cy = intrinsics(p, det['img_w'])
    lo, hi = bbox_bearing_interval_deg(det['x1'], det['x2'], fx, cx, p.margin_deg)
    x, y = lidar_xy(ranges, angle_min, inc, p)
    xc, yc = x - p.cam_x, y - p.cam_y                      # Punkte im Kamerasystem
    bc = np.degrees(np.arctan2(yc, xc))
    sel = (xc > 0.05) & (bc >= lo) & (bc <= hi)
    cls = det.get('cls')
    on_floor_class = cls is None or cls in FLOOR_CLASSES
    d_floor = (floor_distance_m(det['y2'], det['img_h'], p.cam_height, fy, cy, p.cam_pitch_deg)
               if (p.use_floor and on_floor_class) else None)
    if not sel.any():
        return None
    xs, ys, xcs = x[sel], y[sel], xc[sel]
    order = np.argsort(xcs)
    xs, ys, xcs = xs[order], ys[order], xcs[order]
    groups, start = [], 0
    for i in range(1, len(xcs) + 1):
        if i == len(xcs) or xcs[i] - xcs[i - 1] > p.cluster_gap:
            groups.append(slice(start, i))
            start = i
    cands = []
    for g in groups:
        if g.stop - g.start < p.min_points:
            continue
        d = np.hypot(xs[g], ys[g])
        b = np.degrees(np.arctan2(ys[g], xs[g]))
        cands.append({'d_cam': float(np.median(xcs[g])), 'distance_m': float(np.median(d)),
                      'bearing_deg': float((b.min() + b.max()) / 2),
                      'n_points': int(g.stop - g.start)})
    if not cands:
        return None
    match = 'nearest' if on_floor_class else 'nearest_erhoeht'
    best = cands[0]
    if d_floor is not None:
        c = min(cands, key=lambda c: abs(c['d_cam'] - d_floor))
        if abs(c['d_cam'] - d_floor) <= max(0.5, 0.3 * d_floor):
            best, match = c, 'floor'
        else:
            match = 'nearest_unsicher'
    out = dict(best)
    out.update({'match': match, 'd_floor_m': d_floor, 'n_clusters': len(cands)})
    return out
