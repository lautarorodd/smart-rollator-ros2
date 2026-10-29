#!/usr/bin/env python3
"""Hoehen-Abstands-Test: Welche LiDAR-Ebene erkennt ein Hindernis in welcher Hoehe UND in welchem Abstand?

Eine flache Platte (z. B. Karton 20 cm hoch, 40 cm breit) wird mit der UNTERKANTE auf verschiedene Hoehen und in
verschiedene Abstaende vor den LiDAR gestellt (mittig). Pro Aufstellung wird 2 s gemessen, ob die untere
(rollator/hazard/front) und die obere Ebene (rollator/hazard/high) WARN oder STOP melden.

  python3 hoehen_test.py                       (Standard: 7 Hoehen x 3 Abstaende)
  python3 hoehen_test.py --heights 0.75 1.2 1.5 --dists 1.0 2.0 3.0
WICHTIG: front_obstacle der oberen Ebene mit grossem Warnabstand starten (z. B. warn_dist_m 4.5), sonst
meldet er nichts, obwohl der LiDAR die Platte sieht. Abstaende ab LiDAR 1 (vorn) messen.
"""
import argparse
import csv
import math
import os
import threading
import time

DEFAULT_HEIGHTS = [0.45, 0.60, 0.75, 0.90, 1.05, 1.20, 1.50]
DEFAULT_DISTS = [1.0, 2.0, 3.0]


def verdict(nonsafe, total, threshold=0.8):
    if total == 0:
        return 'keine Daten'
    return 'ja' if nonsafe / total >= threshold else 'nein'


def matrix(rows, plane, heights, dists):
    """rows: (hoehe, abstand, urteil_unten, urteil_oben). Gibt Textzeilen einer Hoehe-x-Abstand-Tabelle zurueck."""
    idx = 2 if plane == 'low' else 3
    lines = [f"{'Unterkante':>11} | " + ' '.join(f'{d:>5.1f} m' for d in dists)]
    for h in heights:
        cells = []
        for d in dists:
            v = [r[idx] for r in rows if abs(r[0] - h) < 1e-6 and abs(r[1] - d) < 1e-6]
            cells.append({'ja': 'X', 'nein': '.', 'keine Daten': '?'}.get(v[0], '-') if v else '-')
        lines.append(f'{h:>9.2f} m | ' + ' '.join(f'{c:>7}' for c in cells))
    return lines


def predict_high(h, d, h0, offset, tilt_deg, panel):
    """Modell: trifft die ansteigende Rampe der oberen Ebene eine Platte [h, h+panel] im Abstand d vor LiDAR 1?"""
    ramp = h0 + (d + offset) * math.tan(math.radians(tilt_deg))
    return h <= ramp <= h + panel


def predict_low(h, low_h, panel):
    """Untere Ebene waagerecht in Hoehe low_h."""
    return h <= low_h <= h + panel


def compare(rows, h0, offset, tilt_deg, panel, low_h):
    """Vergleicht Messung und Modell. Gibt (uebereinstimmungen, gesamt, abweichungen) zurueck."""
    ok, total, diffs = 0, 0, []
    for h, d, lo, hi in rows:
        for plane, got, pred in (('untere', lo, predict_low(h, low_h, panel)),
                                 ('obere', hi, predict_high(h, d, h0, offset, tilt_deg, panel))):
            if got not in ('ja', 'nein'):
                continue
            total += 1
            if (got == 'ja') == pred:
                ok += 1
            else:
                diffs.append((plane, h, d, 'gemessen ' + got, 'Modell ' + ('ja' if pred else 'nein')))
    return ok, total, diffs


def fit_tilt(rows, h0, offset, panel):
    """Welche Neigung der oberen Ebene passt am besten zu den Messungen? Gibt (score, [neigungen])."""
    best, best_score, t = [], -1, 0.0
    while t <= 30.01:
        score = sum(1 for h, d, lo, hi in rows
                    if hi in ('ja', 'nein') and predict_high(h, d, h0, offset, t, panel) == (hi == 'ja'))
        if score > best_score:
            best_score, best = score, [t]
        elif score == best_score:
            best.append(t)
        t += 0.5
    return best_score, best


def main():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    ap = argparse.ArgumentParser()
    ap.add_argument('--heights', nargs='*', type=float, default=DEFAULT_HEIGHTS)
    ap.add_argument('--dists', nargs='*', type=float, default=DEFAULT_DISTS)
    ap.add_argument('--seconds', type=float, default=2.0)
    ap.add_argument('--out', default=os.path.expanduser('~/hoehentest.csv'))
    ap.add_argument('--tilt', type=float, default=5.0, help='Neigung der oberen Ebene (Grad, nach oben positiv)')
    ap.add_argument('--h0', type=float, default=0.65, help='Hoehe der oberen Ebene am Sensor (m)')
    ap.add_argument('--offset', type=float, default=0.22, help='oberer LiDAR liegt so weit hinter LiDAR 1 (m)')
    ap.add_argument('--panel', type=float, default=0.20, help='Hoehe der Platte (m)')
    ap.add_argument('--low-h', type=float, default=0.28, help='Hoehe der unteren Ebene (m)')
    a, _ = ap.parse_known_args()

    class Listener(Node):
        def __init__(self):
            super().__init__('hoehen_test')
            self.count = {'low': [0, 0], 'high': [0, 0]}
            self.create_subscription(String, 'rollator/hazard/front', lambda m: self.add('low', m.data), 10)
            self.create_subscription(String, 'rollator/hazard/high', lambda m: self.add('high', m.data), 10)

        def add(self, plane, state):
            c = self.count[plane]
            c[1] += 1
            if state != 'SAFE':
                c[0] += 1

    rclpy.init()
    node = Listener()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    time.sleep(1.5)
    rows, done = [], False
    print(__doc__)
    for h in a.heights:
        for d in a.dists:
            ans = input(f'\nPlatte mit der UNTERKANTE auf {h:.2f} m, {d:.1f} m vor LiDAR 1, mittig. '
                        'ENTER = messen, s = ueberspringen, q = Ende > ').strip().lower()
            if ans == 'q':
                done = True
                break
            if ans == 's':
                continue
            node.count = {'low': [0, 0], 'high': [0, 0]}
            time.sleep(a.seconds)
            lo, hi = verdict(*node.count['low']), verdict(*node.count['high'])
            print(f'   untere Ebene: {lo}   obere Ebene: {hi}   (Nachrichten: {node.count["low"][1]} / {node.count["high"][1]})')
            if 'keine Daten' in (lo, hi):
                print('   ACHTUNG: eine Ebene sendet nichts. Laufen beide front_obstacle-Knoten?')
            rows.append((h, d, lo, hi))
        if done:
            break
    print('\n=== Untere Ebene (X = erkannt, . = nicht erkannt) ===')
    print('\n'.join(matrix(rows, 'low', a.heights, a.dists)))
    print('\n=== Obere Ebene ===')
    print('\n'.join(matrix(rows, 'high', a.heights, a.dists)))
    ok, total, diffs = compare(rows, a.h0, a.offset, a.tilt, a.panel, a.low_h)
    if total:
        print(f'\n=== Modell-Vergleich (obere Ebene {a.tilt:.1f} Grad, {a.h0:.2f} m, {a.offset:.2f} m hinten) ===')
        print(f'Messung und Modell stimmen in {ok} von {total} Faellen ueberein.')
        for plane, h, d, got, pred in diffs:
            print(f'  Abweichung {plane} Ebene, Unterkante {h:.2f} m, {d:.1f} m: {got}, {pred}')
        score, tilts = fit_tilt(rows, a.h0, a.offset, a.panel)
        print(f'Beste Uebereinstimmung der oberen Ebene ({score} Faelle) bei einer Neigung von '
              f'{min(tilts):.1f} bis {max(tilts):.1f} Grad.')
    miss = [(h, d) for h, d, lo, hi in rows if lo == 'nein' and hi == 'nein']
    print(f'\nVon keiner Ebene erkannt: {len(miss)} von {len(rows)} Aufstellungen')
    with open(a.out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['unterkante_m', 'abstand_m', 'untere_ebene', 'obere_ebene'])
        w.writerows(rows)
    print('Gespeichert:', a.out)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
