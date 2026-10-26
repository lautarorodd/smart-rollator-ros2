"""Reine Erkennungslogik des Frontwarners (ohne ROS).

Dieselbe Funktion wird online (Knoten) und offline (evaluate.py) benutzt,
damit aufgezeichnete Scans mit beliebigen Parametern neu bewertet werden
koennen, ohne erneut zu messen.
"""
from dataclasses import dataclass

import numpy as np

ORDER = {'SAFE': 0, 'WARN': 1, 'STOP': 2}


@dataclass(frozen=True)
class Params:
    mode: str = 'corridor'            # 'corridor' (feste Breite) oder 'sector' (Winkel)
    front_angle_deg: float = 0.0      # Sensorwinkel der Gehrichtung
    corridor_half_width_m: float = 0.5
    half_width_deg: float = 20.0
    warn_dist_m: float = 1.5
    stop_dist_m: float = 0.8
    min_range_m: float = 0.15
    min_points: int = 3
    clear_scans: int = 3


def front_coordinates(ranges, angle_min, angle_increment, p, range_max=None):
    """Gibt (x_vorn, y_seitlich, r, rel_winkel) der gueltigen Punkte zurueck.

    y positiv = links (ROS-Konvention, gegen den Uhrzeigersinn).
    """
    r = np.array([np.nan if v is None else v for v in ranges], dtype=float)
    ang = angle_min + np.arange(r.size) * angle_increment
    front = np.deg2rad(p.front_angle_deg)
    rel = np.arctan2(np.sin(ang - front), np.cos(ang - front))
    ok = np.isfinite(r) & (r >= p.min_range_m)
    if range_max:
        ok &= r <= range_max
    r, rel = r[ok], rel[ok]
    return r * np.cos(rel), r * np.sin(rel), r, rel


def classify_scan(ranges, angle_min, angle_increment, p, range_max=None):
    x, y, r, rel = front_coordinates(ranges, angle_min, angle_increment, p, range_max)
    if p.mode == 'corridor':
        sel = (x > 0) & (np.abs(y) <= p.corridor_half_width_m)
        d = x[sel]                      # Abstand in Gehrichtung
    else:
        sel = np.abs(rel) <= np.deg2rad(p.half_width_deg)
        d = r[sel]
    n_stop = int(np.sum(d < p.stop_dist_m))
    n_warn = int(np.sum(d < p.warn_dist_m))
    if n_stop >= p.min_points:
        state = 'STOP'
    elif n_warn >= p.min_points:
        state = 'WARN'
    else:
        state = 'SAFE'
    # naechste Distanz, die von mindestens min_points Punkten gestuetzt wird
    k = max(1, p.min_points)
    nearest = float(np.partition(d, k - 1)[k - 1]) if d.size >= k else None
    return {'state': state, 'nearest_m': nearest, 'n_stop': n_stop,
            'n_warn': n_warn, 'n_points': int(d.size)}


class Hysteresis:
    """Verschaerfung sofort, Entspannung erst nach clear_scans Scans."""

    def __init__(self, clear_scans=3):
        self.clear_scans = clear_scans
        self.state = 'SAFE'
        self._count = 0

    def update(self, raw):
        if ORDER[raw] >= ORDER[self.state]:
            self.state = raw
            self._count = 0
        else:
            self._count += 1
            if self._count >= self.clear_scans:
                self.state = raw
                self._count = 0
        return self.state
