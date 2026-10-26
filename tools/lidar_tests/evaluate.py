#!/usr/bin/env python3
"""Offline-Auswertung aufgezeichneter Versuche (kein ROS noetig).

  python3 evaluate.py ~/lidar_trials --warn 1.5 --stop 0.8
Optional: --csv ergebnis.csv   --no-sweep
"""
import argparse
import csv
import glob
import json
import math
import os
import statistics
from collections import Counter
from dataclasses import replace

from detector_core import ORDER, Hysteresis, Params, classify_scan

MARGIN_M = 0.10   # Versuche nahe an einer Grenze zaehlen als "Grenzbereich"


def load(path):
    with open(path) as f:
        lines = [json.loads(line) for line in f if line.strip()]
    return lines[0]['meta'], lines[1:]


def wilson(k, n, z=1.96):
    if n == 0:
        return 0.0, 1.0
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return max(0.0, c - h), min(1.0, c + h)


def expected_state(meta, p):
    d, lat = meta.get('dist_m'), meta.get('lateral_m') or 0.0
    if d is None or abs(lat) > p.corridor_half_width_m:
        return 'SAFE'
    return 'STOP' if d < p.stop_dist_m else 'WARN' if d < p.warn_dist_m else 'SAFE'


def borderline(meta, p):
    d, lat = meta.get('dist_m'), meta.get('lateral_m') or 0.0
    if d is None:
        return False
    near_lat = abs(abs(lat) - p.corridor_half_width_m) < MARGIN_M
    near_d = (abs(d - p.stop_dist_m) < MARGIN_M) or (abs(d - p.warn_dist_m) < MARGIN_M)
    return near_lat or near_d


def run_scans(scans, p):
    raw, stable, nearest = [], [], []
    hyst = Hysteresis(p.clear_scans)
    for s in scans:
        res = classify_scan(s['ranges'], s['angle_min'], s['angle_increment'],
                            p, s.get('range_max'))
        raw.append(res['state'])
        stable.append(hyst.update(res['state']))
        nearest.append(res['nearest_m'])
    return raw, stable, nearest


def majority(states):
    c = Counter(states)
    top = max(c.values())
    return max((s for s, n in c.items() if n == top), key=lambda s: ORDER[s])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('folder')
    ap.add_argument('--warn', type=float, default=1.5)
    ap.add_argument('--stop', type=float, default=0.8)
    ap.add_argument('--mode', default='corridor', choices=['corridor', 'sector'])
    ap.add_argument('--corridor', type=float, default=0.5)
    ap.add_argument('--half-width-deg', type=float, default=20.0)
    ap.add_argument('--min-points', type=int, default=3)
    ap.add_argument('--csv', default=None)
    ap.add_argument('--no-sweep', action='store_true')
    a = ap.parse_args()

    files = sorted(glob.glob(os.path.join(os.path.expanduser(a.folder), '*.jsonl')))
    data = [load(f) for f in files]
    trials = [(m, s) for m, s in data if m.get('kind', 'trial') == 'trial' and s]
    walks = [(m, s) for m, s in data if m.get('kind') == 'walk' and s]
    print(f'{len(trials)} Versuche, {len(walks)} Gaenge in {a.folder}\n')

    def base(meta):
        return Params(mode=a.mode, front_angle_deg=meta.get('front_angle_deg', 0.0),
                      corridor_half_width_m=a.corridor, half_width_deg=a.half_width_deg,
                      warn_dist_m=a.warn, stop_dist_m=a.stop, min_points=a.min_points)

    # ---- 1. Einzelversuche ----
    print('=== 1. Einzelversuche (Parameter: %s, Warn %.2f m, Stopp %.2f m, min %d Punkte) ===' %
          (a.mode, a.warn, a.stop, a.min_points))
    print(f"{'Label':<26}{'Objekt':<12}{'Abst.':>6}{'seitl.':>7}{'gemess.':>9}{'Fehler':>8}  {'Verteilung':<24}{'erw.':<6}Ergebnis")
    rows, outcomes = [], []
    for meta, scans in trials:
        p = base(meta)
        raw, stable, nearest = run_scans(scans, p)
        got = majority(raw)
        exp = expected_state(meta, p)
        border = borderline(meta, p)
        valid = [n for n in nearest if n is not None]
        med = statistics.median(valid) if valid else None
        err = (med - meta['dist_m']) * 100 if (med is not None and meta.get('dist_m') is not None
                                               and abs(meta.get('lateral_m') or 0) <= p.corridor_half_width_m) else None
        if got == exp:
            res = 'ok'
        elif border:
            res = 'Grenzbereich'
        elif ORDER[got] < ORDER[exp]:
            res = 'VERPASST'
        else:
            res = 'FEHLALARM'
        dist = Counter(raw)
        dtxt = ' '.join(f'{k}:{v}' for k, v in sorted(dist.items(), key=lambda kv: -ORDER[kv[0]]))
        print(f"{meta['label'][:25]:<26}{meta.get('object','')[:11]:<12}"
              f"{(meta['dist_m'] if meta.get('dist_m') is not None else float('nan')):>6.2f}"
              f"{(meta.get('lateral_m') or 0):>+7.2f}"
              f"{(med if (med is not None and meta.get('dist_m') is not None) else float('nan')):>9.2f}"
              f"{(err if err is not None else float('nan')):>7.1f}c  {dtxt:<24}{exp:<6}{res}")
        outcomes.append((meta, res))
        rows.append({'datei': meta.get('recorded_at', ''), 'label': meta['label'],
                     'objekt': meta.get('object', ''), 'dist_m': meta.get('dist_m'),
                     'lateral_m': meta.get('lateral_m'), 'gemessen_m': med,
                     'fehler_cm': err, 'erwartet': exp, 'ergebnis_mehrheit': got,
                     'bewertung': res})

    # ---- 2. Zusammenfassung ----
    strict = [r for _, r in outcomes if r != 'Grenzbereich']
    n = len(strict)
    missed = sum(r == 'VERPASST' for r in strict)
    false = sum(r == 'FEHLALARM' for r in strict)
    ok = sum(r == 'ok' for r in strict)
    print('\n=== 2. Zusammenfassung (ohne Grenzbereich +-%.0f cm) ===' % (MARGIN_M * 100))
    lo, hi = wilson(ok, n)
    print(f'Richtig:    {ok}/{n}  ({(ok / n * 100 if n else 0):.0f} %, 95%-KI {lo * 100:.0f}-{hi * 100:.0f} %)')
    print(f'Verpasst:   {missed}  <- sicherheitskritisch')
    print(f'Fehlalarm:  {false}')
    print(f'Grenzbereich (nicht gewertet): {len(outcomes) - n}')
    if missed == 0 and n:
        print(f'Hinweis: 0 verpasste von {n} Versuchen belegt nur, dass die wahre Verpasst-Rate '
              f'unter ca. {min(1, 3 / n) * 100:.0f} % liegt (Regel der drei).')

    # ---- 3. Messfehler nach Objekt ----
    print('\n=== 3. Messfehler in Gehrichtung (cm, gemessen - wahr) je Objekt ===')
    per_obj = {}
    for r in rows:
        if r['fehler_cm'] is not None:
            per_obj.setdefault(r['objekt'], []).append(r['fehler_cm'])
    for o, v in sorted(per_obj.items()):
        sd = statistics.stdev(v) if len(v) > 1 else 0.0
        print(f'{o:<16} n={len(v):<3} Mittel {statistics.mean(v):+6.1f}  sigma {sd:5.1f}  min {min(v):+6.1f}  max {max(v):+6.1f}')
    missing = [r for r in rows if r['dist_m'] is not None and r['gemessen_m'] is None
               and abs(r['lateral_m'] or 0) <= a.corridor]
    if missing:
        print('Objekte im Korridor, die der Sensor NICHT gesehen hat:')
        for r in missing:
            print(f"  - {r['label']} ({r['objekt']}, {r['dist_m']} m)")

    # ---- 4. Gaenge ----
    if walks:
        print('\n=== 4. Gaenge ===')
        for meta, scans in walks:
            p = base(meta)
            raw, stable, nearest = run_scans(scans, p)
            ts = [s['t'] for s in scans]
            dur = (ts[-1] - ts[0]) if len(ts) > 1 else 0
            non_safe = sum(1 for x in stable if x != 'SAFE')
            changes = sum(1 for i in range(1, len(stable)) if stable[i] != stable[i - 1])
            line = f"{meta['label']}: {len(scans)} Scans, {dur:.0f} s, nicht-SAFE {non_safe} Scans, Zustandswechsel {changes}"
            if dur > 0:
                line += f" ({changes / dur * 60:.1f} pro Minute)"
            print(line)
            for tgt in ('WARN', 'STOP'):
                for i, st in enumerate(stable):
                    if st == tgt and nearest[i] is not None:
                        print(f'   erstes {tgt} bei gemessener Distanz {nearest[i]:.2f} m')
                        break

    # ---- 5. Variantenvergleich ----
    if not a.no_sweep and trials:
        print('\n=== 5. Variantenvergleich (gleiche Rohdaten, andere Parameter) ===')
        variants = []
        for mp in (1, 3, 5):
            for cw in (0.4, 0.5, 0.6):
                variants.append(('corridor', cw, 20.0, mp))
            for hw in (15.0, 20.0, 30.0):
                variants.append(('sector', 0.5, hw, mp))
        results = []
        for mode, cw, hw, mp in variants:
            miss = fa = tot = 0
            for meta, scans in trials:
                p = replace(base(meta), mode=mode, corridor_half_width_m=cw,
                            half_width_deg=hw, min_points=mp)
                ref = replace(p, mode='corridor', corridor_half_width_m=a.corridor)
                if borderline(meta, ref):
                    continue
                exp = expected_state(meta, ref)
                got = majority(run_scans(scans, p)[0])
                tot += 1
                if ORDER[got] < ORDER[exp]:
                    miss += 1
                elif ORDER[got] > ORDER[exp]:
                    fa += 1
            results.append((miss, fa, mode, cw, hw, mp, tot))
        results.sort()
        print(f"{'Modus':<10}{'Breite':>10}{'min Pkt':>9}{'verpasst':>10}{'Fehlalarm':>11}{'Versuche':>10}")
        for miss, fa, mode, cw, hw, mp, tot in results:
            width = f'{cw:.1f} m' if mode == 'corridor' else f'+-{hw:.0f} Grad'
            print(f'{mode:<10}{width:>10}{mp:>9}{miss:>10}{fa:>11}{tot:>10}')
        print('(Die "Wahrheit" ist immer der Korridor von %.2f m aus --corridor; verpasst zaehlt schwerer als Fehlalarm.)' % a.corridor)

    if a.csv:
        with open(a.csv, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print('\nCSV geschrieben:', a.csv)


if __name__ == '__main__':
    main()
