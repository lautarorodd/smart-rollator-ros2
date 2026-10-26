#!/usr/bin/env python3
"""Erzeugt synthetische Versuche zum Testen der Werkzeuge (kein Sensor noetig).

  python3 make_synthetic.py /tmp/syn --front-angle 90
  python3 evaluate.py /tmp/syn
"""
import argparse
import json
import math
import os
import random

N = 3600                      # 0,1 Grad Aufloesung
INC = 2 * math.pi / N


def ray_segment(theta, p1, p2):
    """Abstand entlang Strahl (ab Ursprung) bis Segment p1-p2, sonst None."""
    dx, dy = math.cos(theta), math.sin(theta)
    ex, ey = p2[0] - p1[0], p2[1] - p1[1]
    den = dx * ey - dy * ex
    if abs(den) < 1e-12:
        return None
    t = (p1[0] * ey - p1[1] * ex) / den
    u = (p1[0] * dy - p1[1] * dx) / den
    return t if t > 0 and 0 <= u <= 1 else None


def make_scan(front_deg, dist, lateral, width, glass=False, noise=0.01):
    walls = [((-6, -6), (6, -6)), ((6, -6), (6, 6)), ((6, 6), (-6, 6)), ((-6, 6), (-6, -6))]
    obj = None
    if dist is not None and not glass:
        obj = ((dist, lateral - width / 2), (dist, lateral + width / 2))
    out = []
    for i in range(N):
        a = -math.pi + i * INC                       # Sensorwinkel
        rel = a - math.radians(front_deg)            # relativ zur Gehrichtung
        cands = [ray_segment(rel, *w) for w in walls]
        if obj:
            cands.append(ray_segment(rel, *obj))
        cands = [c for c in cands if c is not None]
        r = min(cands) + random.gauss(0, noise) if cands else None
        out.append(round(r, 4) if r is not None and r > 0.05 else None)
    return {'t': 0.0, 'angle_min': -math.pi, 'angle_increment': INC,
            'range_max': 18.0, 'ranges': out}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('outdir')
    ap.add_argument('--front-angle', type=float, default=0.0)
    a = ap.parse_args()
    random.seed(1)
    os.makedirs(a.outdir, exist_ok=True)
    cases = [('frei', 'keins', None, 0, 0.3), ('k0.5', 'Karton', 0.5, 0, 0.3),
             ('k1.0', 'Karton', 1.0, 0, 0.3), ('k1.6', 'Karton', 1.6, 0, 0.3),
             ('k3.0', 'Karton', 3.0, 0, 0.3), ('seit0.8', 'Karton', 1.0, 0.8, 0.3),
             ('flasche3', 'Flasche', 3.0, 0, 0.08), ('glas', 'Glasscheibe', 1.0, 0, 0.3)]
    for k, (label, obj, d, lat, w) in enumerate(cases):
        for rep in range(2):
            scans = [make_scan(a.front_angle, d, lat, w, glass=(obj == 'Glasscheibe'))
                     for _ in range(10)]
            for j, s in enumerate(scans):
                s['t'] = 100.0 + j * 0.1
            meta = {'label': label, 'kind': 'trial', 'object': obj, 'dist_m': d,
                    'lateral_m': lat, 'front_angle_deg': a.front_angle,
                    'n_scans': len(scans), 'recorded_at': f'synthetic-{k}-{rep}'}
            with open(os.path.join(a.outdir, f'syn_{k:02d}_{rep}_{label}.jsonl'), 'w') as f:
                f.write(json.dumps({'meta': meta}) + '\n')
                for s in scans:
                    f.write(json.dumps(s) + '\n')
    print('fertig:', a.outdir)


if __name__ == '__main__':
    main()
