#!/usr/bin/env python3
"""Findet die Gehrichtung eines LiDAR per Differenzmessung (Szene leer vs. mit Objekt).

  python3 scan_diff.py /high/scan [--n 30] [--drop 0.3]

Ablauf: 1) Szene leer lassen, ENTER. 2) Person/Objekt ca. 1,5 m genau vor den Rollator stellen, ENTER.
Alles, was sich nicht aendert (Wand, Rahmen, Halterung, Kabel), faellt heraus.
Winkel = LaserScan-Winkel in Grad (-180..180) = Wert fuer HIGH_FRONT bzw. FRONT.
Die Auswertung steckt in profile_from_ranges() und changed_groups() (ohne ROS, siehe test_scan_diff.py).
"""
import argparse
import math
import statistics
import time

BIN = 2.0
NB = int(360 / BIN)


def norm(a):
    return (a + 180.0) % 360.0 - 180.0


def profile_from_ranges(scans):
    """scans: Liste von (ranges, angle_min, angle_increment, range_min, range_max).
    Gibt je 2-Grad-Sektor den Median der gueltigen Abstaende zurueck (99.0 = kein Treffer)."""
    vals = [[] for _ in range(NB)]
    for ranges, angle_min, inc, rmin, rmax in scans:
        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r < rmin or r > rmax:
                continue
            ang = norm(math.degrees(angle_min + i * inc))
            vals[min(int((ang + 180.0) / BIN), NB - 1)].append(r)
    return [statistics.median(v) if v else 99.0 for v in vals]


def changed_groups(base, obj, drop=0.3):
    """Zusammenhaengende Sektoren, in denen das Objekt den Abstand um mehr als `drop` verkuerzt.
    Gibt Liste von (mitte_grad, breite_grad, abstand_mit_objekt, abstand_leer) zurueck, nach Abstand sortiert.
    Leere Liste, wenn nichts oder alles veraendert ist."""
    changed = [(base[b] - obj[b]) > drop for b in range(NB)]
    if not any(changed) or all(changed):
        return []
    start = next(b for b in range(NB) if not changed[b])
    groups, cur = [], []
    for k in range(1, NB + 1):
        b = (start + k) % NB
        if changed[b]:
            cur.append(b)
        elif cur:
            groups.append(cur)
            cur = []
    if cur:
        groups.append(cur)
    out = []
    for g in groups:
        angs = [b * BIN - 180.0 + BIN / 2 for b in g]
        c = math.degrees(math.atan2(sum(math.sin(math.radians(t)) for t in angs),
                                    sum(math.cos(math.radians(t)) for t in angs)))
        out.append((c, len(g) * BIN, statistics.median([obj[b] for b in g]),
                    statistics.median([base[b] for b in g])))
    return sorted(out, key=lambda t: t[2])


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    ap = argparse.ArgumentParser()
    ap.add_argument('topic')
    ap.add_argument('--n', type=int, default=30)
    ap.add_argument('--drop', type=float, default=0.3, help='Mindest-Abnahme der Distanz (m)')
    a = ap.parse_args()

    rclpy.init()
    node = Node('scan_diff')
    buf = []
    node.create_subscription(LaserScan, a.topic, buf.append, qos_profile_sensor_data)

    def capture(n):
        buf.clear()
        t0 = time.time()
        while rclpy.ok() and len(buf) < n and time.time() - t0 < 30:
            rclpy.spin_once(node, timeout_sec=0.5)
        return [(list(s.ranges), s.angle_min, s.angle_increment, s.range_min, s.range_max) for s in buf]

    input('1) Szene LEER lassen (niemand vor dem Rollator), dann ENTER ... ')
    s1 = capture(a.n)
    input('2) Person/Objekt ca. 1,5 m genau vor den Rollator stellen, dann ENTER ... ')
    s2 = capture(a.n)
    if len(s1) < 5 or len(s2) < 5:
        print('Zu wenige Scans. Laeuft der LiDAR? Topic:', a.topic)
        return

    groups = changed_groups(profile_from_ranges(s1), profile_from_ranges(s2), a.drop)
    if not groups:
        print('Keine eindeutige Aenderung gefunden. Objekt naeher/breiter oder --drop kleiner probieren.')
        return
    print('\nVeraenderte Bereiche (Winkel = LaserScan-Winkel):')
    for c, w, d_obj, d_leer in groups:
        print('  Mitte %7.1f Grad  Breite %4.0f Grad  mit Objekt ca. %.2f m (leer: %.2f m)' % (c, w, d_obj, d_leer))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
