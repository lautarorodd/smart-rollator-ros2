#!/usr/bin/env python3
"""Profil eines LiDAR-Scans: feste Naehe-Treffer (Eigenechos) und naechstes Objekt.

  python3 scan_profile.py /high/scan [--near 0.6] [--min-range 0.4] [--n 50]

Eigenecho = Sektor, in dem in >= 90 % der Scans etwas naeher als --near liegt (Rahmen, Halterung, Kabel,
aber auch eine Wand dicht hinter dem Rollator). Winkel = LaserScan-Winkel in Grad (-180..180).
Achtung: Der Abschnitt "Naechster Punkt" zeigt auch Waende und Eigenechos; fuer den Frontwinkel scan_diff.py nehmen.
Vorlaeufer mit Sektortabelle: scan_summary.py.
"""
import argparse
import math
import statistics
import time


def norm(a):
    return (a + 180.0) % 360.0 - 180.0


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    ap = argparse.ArgumentParser()
    ap.add_argument('topic')
    ap.add_argument('--n', type=int, default=50, help='Anzahl Scans')
    ap.add_argument('--near', type=float, default=0.6, help='Naehe-Schwelle (m) fuer Eigenechos')
    ap.add_argument('--min-range', type=float, default=0.0, help='Treffer darunter bei der Objektsuche ignorieren')
    ap.add_argument('--bin', type=float, default=2.0)
    a = ap.parse_args()

    rclpy.init()
    node = Node('scan_profile')
    scans = []
    node.create_subscription(LaserScan, a.topic, scans.append, qos_profile_sensor_data)
    t0 = time.time()
    while rclpy.ok() and len(scans) < a.n and time.time() - t0 < 30:
        rclpy.spin_once(node, timeout_sec=0.5)
    if len(scans) < 5:
        print('Zu wenige Scans (%d). Laeuft der LiDAR? Topic: %s' % (len(scans), a.topic))
        return

    nb = int(360 / a.bin)
    near_cnt = [0] * nb
    near_vals = [[] for _ in range(nb)]
    obj_ang, obj_dist = [], []
    for s in scans:
        best = None
        seen = set()
        for i, r in enumerate(s.ranges):
            if not math.isfinite(r) or r < s.range_min or r > s.range_max:
                continue
            ang = norm(math.degrees(s.angle_min + i * s.angle_increment))
            b = min(int((ang + 180.0) / a.bin), nb - 1)
            if r < a.near:
                near_vals[b].append(r)
                seen.add(b)
            if r >= a.min_range and (best is None or r < best[0]):
                best = (r, ang)
        for b in seen:
            near_cnt[b] += 1
        if best:
            obj_dist.append(best[0])
            obj_ang.append(best[1])

    n = len(scans)
    static = [near_cnt[b] >= 0.9 * n for b in range(nb)]
    print('\n%d Scans von %s. Feste Treffer naeher als %.2f m (Eigenechos):' % (n, a.topic, a.near))
    b, found = 0, False
    while b < nb:
        if static[b]:
            e = b
            while e + 1 < nb and static[e + 1]:
                e += 1
            vals = [v for k in range(b, e + 1) for v in near_vals[k]]
            print('  %7.1f .. %7.1f Grad   Abstand min %.2f m, Median %.2f m'
                  % (b * a.bin - 180, (e + 1) * a.bin - 180, min(vals), statistics.median(vals)))
            found = True
            b = e + 1
        else:
            b += 1
    if not found:
        print('  keine')

    if obj_ang:
        sx = sum(math.cos(math.radians(t)) for t in obj_ang)
        sy = sum(math.sin(math.radians(t)) for t in obj_ang)
        mean = math.degrees(math.atan2(sy, sx))
        spread = statistics.pstdev([norm(t - mean) for t in obj_ang])
        print('\nNaechster Punkt (ab %.2f m): Winkel ca. %.1f Grad (Streuung %.1f), Abstand ca. %.2f m'
              % (a.min_range, mean, spread, statistics.median(obj_dist)))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
