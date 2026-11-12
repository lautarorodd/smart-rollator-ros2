#!/usr/bin/env python3
"""Findet die Gehrichtung eines LiDAR per Differenzmessung (Szene leer vs. mit Objekt).

  python3 scan_diff.py /high/scan [--n 30] [--drop 0.3]
  python3 scan_diff.py /scan /high/scan --delay 15     # beide LiDAR in einem Durchgang, allein messbar

Ablauf: 1) Szene leer lassen, ENTER. 2) Person/Objekt ca. 1 bis 1,5 m genau vor den Rollator stellen, ENTER.
Alles, was sich nicht aendert (Wand, Rahmen, Halterung, Kabel), faellt heraus.
Mit --delay N wartet Schritt 2 nach dem ENTER noch N Sekunden (Piepton am Anfang und am Ende der Aufnahme),
damit die bedienende Person selbst nach vorn gehen kann. Wer hinter dem Rollator steht und nach vorn geht,
stoert nicht: gezaehlt wird nur, wo die Distanz abnimmt.
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


def countdown(sec, sleep=time.sleep, out=print):
    """Zaehlt sec Sekunden herunter (Piepton am Anfang, Ausgabe alle 5 s und in den letzten 3 s)."""
    out('\a%d s bis zur Aufnahme ...' % sec)
    for rest in range(int(sec), 0, -1):
        if rest != sec and (rest % 5 == 0 or rest <= 3):
            out('  %d' % rest)
        sleep(1.0)


def format_groups(topic, groups):
    """Ergebniszeilen fuer ein Topic (Liste von Strings)."""
    if not groups:
        return ['%s: keine eindeutige Aenderung gefunden. Objekt naeher/breiter oder --drop kleiner probieren.' % topic]
    lines = ['%s: veraenderte Bereiche (Winkel = LaserScan-Winkel):' % topic]
    for c, w, d_obj, d_leer in groups:
        lines.append('  Mitte %7.1f Grad  Breite %4.0f Grad  mit Objekt ca. %.2f m (leer: %.2f m)' % (c, w, d_obj, d_leer))
    return lines


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    ap = argparse.ArgumentParser()
    ap.add_argument('topics', nargs='+', help='ein oder mehrere LaserScan-Topics, z. B. /scan /high/scan')
    ap.add_argument('--n', type=int, default=30, help='Scans je Phase und Topic')
    ap.add_argument('--drop', type=float, default=0.3, help='Mindest-Abnahme der Distanz (m)')
    ap.add_argument('--delay', type=float, default=0.0,
                    help='Sekunden Wartezeit vor der Aufnahme mit Person (zum Hinstellen, wenn man allein misst)')
    a = ap.parse_args()

    rclpy.init()
    node = Node('scan_diff')
    bufs = {t: [] for t in a.topics}
    for t in a.topics:
        node.create_subscription(LaserScan, t, bufs[t].append, qos_profile_sensor_data)

    def capture(n):
        """Nimmt gleichzeitig von allen Topics je n Scans auf (hoechstens 30 s)."""
        for b in bufs.values():
            b.clear()
        t0 = time.time()
        while rclpy.ok() and min(len(b) for b in bufs.values()) < n and time.time() - t0 < 30:
            rclpy.spin_once(node, timeout_sec=0.5)
        return {t: [(list(s.ranges), s.angle_min, s.angle_increment, s.range_min, s.range_max) for s in b[:n]]
                for t, b in bufs.items()}

    input('1) Szene LEER lassen (niemand vor dem Rollator), dann ENTER ... ')
    s1 = capture(a.n)
    print('   leer aufgenommen.')
    input('2) Person/Objekt ca. 1 m genau vor den Rollator stellen, dann ENTER ... ')
    if a.delay > 0:
        countdown(int(round(a.delay)))
    print('   Aufnahme laeuft, nicht bewegen ...')
    s2 = capture(a.n)
    print('\a\a   fertig.')

    for t in a.topics:
        if len(s1[t]) < 5 or len(s2[t]) < 5:
            print('%s: zu wenige Scans (leer %d, Objekt %d). Laeuft der LiDAR?' % (t, len(s1[t]), len(s2[t])))
            continue
        groups = changed_groups(profile_from_ranges(s1[t]), profile_from_ranges(s2[t]), a.drop)
        print()
        print('\n'.join(format_groups(t, groups)))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
