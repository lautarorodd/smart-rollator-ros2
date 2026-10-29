#!/usr/bin/env python3
"""Rechnet vor dem Umbauen, was eine geneigte LiDAR-Ebene erfasst (ohne ROS, ohne Hardware).

Die geneigte Ebene ist eine ansteigende Rampe: In horizontalem Abstand d vom Sensor liegt sie in der Hoehe
  h(d) = h0 + d * tan(Neigung).
Ein haengendes Objekt mit der Hoehe [unten, oben] wird nur gesehen, solange die Rampe in dieser Hoehe liegt.
Faehrt der Rollator darauf zu, sinkt die Rampe am Objekt und laeuft irgendwann unter ihm durch.

Beispiele:
  python3 plane_coverage.py
  python3 plane_coverage.py --h0 0.65 --offset 0.22 --tilts 5 10 15 20
  python3 plane_coverage.py --target-height 1.5 --target-dist 2.0     (welche Neigung wuerde dort treffen?)
"""
import argparse
import math

DEFAULT_OBJECTS = [
    ('Tischplatte 0,75 m (5 cm dick)', 0.725, 0.775),
    ('Regal/Fensterbrett 1,0-1,1 m', 1.0, 1.1),
    ('Schild 1,4-1,6 m', 1.4, 1.6),
    ('Ast 1,7-1,8 m', 1.7, 1.8),
]


def ramp_height(x_front, h0, offset, tilt_deg):
    """Hoehe der Ebene in x_front Metern vor LiDAR 1 (offset: Sensor liegt so viel hinter LiDAR 1)."""
    return h0 + (x_front + offset) * math.tan(math.radians(tilt_deg))


def detection_window(lo, hi, h0, offset, tilt_deg):
    """Abstand (von, bis) ab LiDAR 1, in dem ein Objekt der Hoehe [lo, hi] von der Rampe getroffen wird."""
    t = math.tan(math.radians(tilt_deg))
    if t <= 0:
        return (0.0, 99.0) if lo <= h0 <= hi else None
    x1 = (lo - h0) / t - offset
    x2 = (hi - h0) / t - offset
    if x2 < 0:
        return None
    return max(0.0, x1), x2


def tilt_for(target_height, target_dist, h0, offset):
    return math.degrees(math.atan((target_height - h0) / (target_dist + offset)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--h0', type=float, default=0.65, help='Hoehe des Sensors (m)')
    ap.add_argument('--offset', type=float, default=0.22, help='Sensor liegt so weit hinter LiDAR 1 (m)')
    ap.add_argument('--tilts', type=float, nargs='*', default=[5, 10, 15, 20, 25])
    ap.add_argument('--target-height', type=float, default=None)
    ap.add_argument('--target-dist', type=float, default=None)
    a = ap.parse_args()

    dists = (0.8, 1.0, 1.5, 2.0, 3.0, 4.0)
    print(f'Hoehe der Ebene (m) in x m Abstand vor LiDAR 1  (Sensor {a.h0:.2f} m hoch, {a.offset:.2f} m dahinter)')
    print('Neigung | ' + '  '.join(f'x={d:<4}' for d in dists))
    for t in a.tilts:
        print(f'{t:>5.0f}°  | ' + '  '.join(f'{ramp_height(d, a.h0, a.offset, t):<6.2f}' for d in dists))
    print('\nObjekt wird gesehen, solange es x m vor LiDAR 1 liegt (von-bis):')
    print(f"{'Objekt':<34}" + ' | '.join(f'{t:.0f}°' .ljust(11) for t in a.tilts))
    for name, lo, hi in DEFAULT_OBJECTS:
        cells = []
        for t in a.tilts:
            w = detection_window(lo, hi, a.h0, a.offset, t)
            cells.append(('nie' if w is None else f'{w[0]:.1f}-{w[1]:.1f} m').ljust(11))
        print(f'{name:<34}' + ' | '.join(cells))
    print('\nLesehilfe: Liegt das Fenster weit ueber dem Warnabstand (z. B. 1,5 m), sieht der Sensor das Objekt zu frueh,\n'
          'um zu warnen, und verliert es danach wieder. Liegt es unter 0,8 m, ist es zu spaet.')
    if a.target_height and a.target_dist:
        t = tilt_for(a.target_height, a.target_dist, a.h0, a.offset)
        print(f'\nUm in {a.target_dist:.1f} m Abstand die Hoehe {a.target_height:.2f} m zu treffen, braucht es ca. {t:.0f}° Neigung.')


if __name__ == '__main__':
    main()
