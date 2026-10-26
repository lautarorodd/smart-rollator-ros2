#!/usr/bin/env python3
"""Fuehrt einen Messplan (CSV) Versuch fuer Versuch aus.

CSV-Spalten: label,object,dist,lateral,repeats   (dist leer = freier Korridor)
Jede Wiederholung wird neu aufgebaut: ENTER = messen, s = ueberspringen, q = Ende.

  python3 run_plan.py plans/standard.csv --front-angle 90 --camera
"""
import argparse
import csv
import os

import rclpy

from record_trial import collect, make_stem, write_trial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('plan')
    ap.add_argument('--front-angle', type=float, required=True)
    ap.add_argument('--scans', type=int, default=30)
    ap.add_argument('--camera', action='store_true', help='pro Versuch 3 Kamerabilder (image_raw) mitspeichern')
    ap.add_argument('--outdir', default=os.path.expanduser('~/lidar_trials'))
    a = ap.parse_args()

    rows = list(csv.DictReader(open(a.plan, newline='')))
    jobs = []
    for r in rows:
        for k in range(1, int(r['repeats']) + 1):
            jobs.append((r, k))

    rclpy.init()
    try:
        for i, (r, k) in enumerate(jobs, 1):
            dist = float(r['dist']) if r['dist'].strip() else None
            lat = float(r['lateral']) if r['lateral'].strip() else 0.0
            txt = (f"[{i}/{len(jobs)}] {r['label']} | Objekt: {r['object']} | "
                   f"Abstand: {dist if dist is not None else '-'} m | "
                   f"seitlich: {lat:+.2f} m | Wdh {k}/{r['repeats']}")
            ans = input(txt + '\n   Objekt neu aufstellen, dann ENTER (s=skip, q=Ende) ')
            if ans.strip().lower() == 'q':
                break
            if ans.strip().lower() == 's':
                continue
            stem = make_stem(r['label'])
            images = None
            if a.camera:
                scans, images = collect(a.scans, image_dir=os.path.join(a.outdir, 'images'), stem=stem)
            else:
                scans = collect(a.scans)
            meta = {'label': r['label'], 'kind': 'trial', 'object': r['object'],
                    'dist_m': dist, 'lateral_m': lat,
                    'front_angle_deg': a.front_angle, 'notes': f'wdh {k}'}
            print('   gespeichert:', write_trial(a.outdir, meta, scans, stem=stem, images=images))
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
