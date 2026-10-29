#!/usr/bin/env python3
"""Textuelles "Radar": kleinster Abstand je Winkelsektor eines LiDAR-Scans (ohne Monitor/RViz).

  python3 scan_summary.py --topic /high/scan            (obere Ebene)
  python3 scan_summary.py --topic /scan --bin 10 --count 5

Zeigt pro Sektor (Sensorwinkel, 0 = Sensor-x-Achse, links positiv) den kleinsten gueltigen Abstand.
Sektoren mit Abstaenden unter --near (Standard 0,5 m) sind markiert: dort ist etwas dicht am Sensor
(oft Halterung, Rahmen, Kabel oder Raspberry Pi des Rollators selbst).
Handtest: Hand 30-40 cm vor den Sensor halten und sehen, welcher Sektor reagiert (zeigt auch, wo vorn ist).
"""
import argparse
import math
import time


def summarize(ranges, angle_min, angle_increment, bin_deg=15.0, min_valid=0.05):
    """Gibt Liste (start_deg, min_range oder None, n_punkte, winkel_des_minimums) je Sektor zurueck."""
    n_bins = int(round(360.0 / bin_deg))
    best = [None] * n_bins
    count = [0] * n_bins
    where = [None] * n_bins
    for i, r in enumerate(ranges):
        if r is None or r != r or r in (float('inf'), float('-inf')) or r < min_valid:
            continue
        ang = math.degrees(angle_min + i * angle_increment)
        ang = (ang + 180.0) % 360.0 - 180.0
        b = int((ang + 180.0) // bin_deg) % n_bins
        count[b] += 1
        if best[b] is None or r < best[b]:
            best[b], where[b] = r, ang
    return [(-180.0 + b * bin_deg, best[b], count[b], where[b]) for b in range(n_bins)]


def nearest_points(ranges, angle_min, angle_increment, k=5, min_valid=0.05):
    pts = []
    for i, r in enumerate(ranges):
        if r is None or r != r or r in (float('inf'), float('-inf')) or r < min_valid:
            continue
        ang = (math.degrees(angle_min + i * angle_increment) + 180.0) % 360.0 - 180.0
        pts.append((r, ang))
    pts.sort()
    return pts[:k]


def format_report(ranges, angle_min, inc, bin_deg, near):
    rows = summarize(ranges, angle_min, inc, bin_deg)
    lines = [f"{'Sektor (Grad)':>16}  {'min. Abstand':>12}  {'Punkte':>6}"]
    for start, r, n, ang in rows:
        mark = '  <-- nah' if (r is not None and r < near) else ''
        txt = '       -' if r is None else f'{r:>7.2f} m'
        lines.append(f'{start:>7.0f}..{start + bin_deg:<5.0f}  {txt:>12}  {n:>6}{mark}')
    lines.append('')
    lines.append('Naechste 5 Punkte (Abstand m, Winkel Grad): ' +
                 ', '.join(f'{r:.2f}@{a:.0f}' for r, a in nearest_points(ranges, angle_min, inc)))
    return lines


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/high/scan')
    ap.add_argument('--bin', type=float, default=15.0)
    ap.add_argument('--near', type=float, default=0.5)
    ap.add_argument('--count', type=int, default=1, help='Anzahl Ausgaben (1 pro Sekunde)')
    a, _ = ap.parse_known_args()

    rclpy.init()
    node = rclpy.create_node('scan_summary')
    last = {'msg': None}
    node.create_subscription(LaserScan, a.topic, lambda m: last.__setitem__('msg', m), qos_profile_sensor_data)
    t0 = time.time()
    while last['msg'] is None and time.time() - t0 < 5:
        rclpy.spin_once(node, timeout_sec=0.2)
    if last['msg'] is None:
        print(f'FEHLER: Keine Daten auf {a.topic}. Laeuft der LiDAR?')
        return
    for k in range(a.count):
        t1 = time.time()
        while time.time() - t1 < 1.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        m = last['msg']
        rng = [None if (r != r or r in (float('inf'), float('-inf'))) else float(r) for r in m.ranges]
        print(f'--- {a.topic}, Ausgabe {k + 1} von {a.count} ---')
        print('\n'.join(format_report(rng, m.angle_min, m.angle_increment, a.bin, a.near)))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
