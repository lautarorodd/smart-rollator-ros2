import math
import numpy as np
from fusion_core import *


def seg_hit(theta, p1, p2):
    dx, dy = math.cos(theta), math.sin(theta)
    ex, ey = p2[0] - p1[0], p2[1] - p1[1]
    den = dx * ey - dy * ex
    if abs(den) < 1e-12:
        return None
    t = (p1[0] * ey - p1[1] * ex) / den
    u = (p1[0] * dy - p1[1] * dx) / den
    return t if t > 0 and 0 <= u <= 1 else None


def square(cx, cy, w=0.04):
    h = w / 2
    pts = [(cx - h, cy - h), (cx + h, cy - h), (cx + h, cy + h), (cx - h, cy + h)]
    return [(pts[i], pts[(i + 1) % 4]) for i in range(4)]


def make_scan(front_deg, chair_x, chair_y, wall_x=4.5):
    segs = [((wall_x, -5), (wall_x, 5))]
    if chair_x is not None:
        for dx in (0.0, 0.4):
            for dy in (-0.2, 0.2):
                segs += square(chair_x + dx, chair_y + dy)
    n, inc = 3600, 2 * math.pi / 3600
    out = []
    for i in range(n):
        a = -math.pi + i * inc
        rel = a - math.radians(front_deg)
        hits = [h for h in (seg_hit(rel, *s) for s in segs) if h is not None]
        out.append(round(min(hits), 4) if hits else None)
    return out, -math.pi, inc


def chair_bbox(chair_x, chair_y, p, w=640, h=360):
    fx, fy, cx, cy = intrinsics(p, w)
    bs = []
    for dx in (0.0, 0.4):
        for dy in (-0.225, 0.225):
            xc, yc = chair_x + dx - p.cam_x, chair_y + dy - p.cam_y
            bs.append(math.atan2(yc, xc))
    u1 = cx - fx * math.tan(max(bs))      # links = groesserer Winkel = kleinere Spalte
    u2 = cx - fx * math.tan(min(bs))
    v2 = cy + fy * p.cam_height / (chair_x - p.cam_x)
    return {'x1': u1, 'x2': u2, 'y1': 60.0, 'y2': v2, 'img_w': w, 'img_h': h}


p = FusionParams(front_angle_deg=90.0, cam_x=-0.19, cam_y=0.0, cam_height=0.46)

# 1. Stuhl 2,0 m vorn, 0,1 m links; Wand dahinter bei 4,5 m
ranges, amin, inc = make_scan(90.0, 2.0, 0.1)
det = chair_bbox(2.0, 0.1, p)
r = fuse_detection(det, ranges, amin, inc, p)
print(r)
assert r and r['match'] == 'floor' and abs(r['distance_m'] - 2.0) < 0.15, r
assert abs(r['bearing_deg'] - math.degrees(math.atan2(0.1, 2.0))) < 4.0, r
assert r['n_clusters'] >= 2           # Vorder- und Hinterbeine bzw. Wand

# 2. Ohne Bodenprior: naechster Cluster = Vorderbeine
p2 = FusionParams(front_angle_deg=90.0, use_floor=False)
r2 = fuse_detection(det, ranges, amin, inc, p2)
assert r2 and r2['match'] == 'nearest' and abs(r2['distance_m'] - 2.0) < 0.15, r2

# 3. Box unten abgeschnitten (Stuhl zu nah, Boden nicht sichtbar): kein Bodenwert, nimmt naechsten
det_cut = dict(det, y2=360.0)
r3 = fuse_detection(det_cut, ranges, amin, inc, p)
assert r3 and r3['d_floor_m'] is None and r3['match'] == 'nearest' and abs(r3['distance_m'] - 2.0) < 0.15, r3

# 4. Stuhl seitlich (-0.5 m, also rechts)
ranges4, a4, i4 = make_scan(90.0, 1.5, -0.5)
r4 = fuse_detection(chair_bbox(1.5, -0.5, p), ranges4, a4, i4, p)
assert r4 and r4['bearing_deg'] < -10 and abs(r4['distance_m'] - 1.5) < 0.2, r4

# 5. Detektion ohne Objekt im Winkelbereich (nur Wand): nur Wand-Cluster; Bodenwert passt nicht -> unsicher
ranges5, a5, i5 = make_scan(90.0, None, 0)
r5 = fuse_detection(chair_bbox(2.0, 0.1, p), ranges5, a5, i5, p)
assert r5 is None or r5['match'] == 'nearest_unsicher', r5

# 6. Intervall und Bodenabstand
lo, hi = bbox_bearing_interval_deg(300, 400, 490.862, 337.357, 2.0)
assert lo < 0 < hi or (lo < hi), (lo, hi)
assert abs(floor_distance_m(177.672 + 491.128 * 0.46 / 2.0, 360, 0.46, 491.128, 177.672) - 2.0) < 1e-6
# 7. Box knapp ueber dem unteren Bildrand (356 von 360) zaehlt als abgeschnitten
assert floor_distance_m(356.0, 360, 0.46, 491.128, 177.672) is None
assert floor_distance_m(300.0, 360, 0.46, 491.128, 177.672) is not None

# 8. Objekte, die nicht am Boden stehen (Rucksack): keine Bodenschaetzung, markiert als erhoeht
det_bp = dict(det, cls='backpack')
r8 = fuse_detection(det_bp, ranges, amin, inc, p)
assert r8 and r8['d_floor_m'] is None and r8['match'] == 'nearest_erhoeht', r8
# und ein Stuhl mit Klasse bleibt auf dem Bodenpfad
r9 = fuse_detection(dict(det, cls='chair'), ranges, amin, inc, p)
assert r9 and r9['match'] == 'floor', r9
print('Alle Fusionstests bestanden')
