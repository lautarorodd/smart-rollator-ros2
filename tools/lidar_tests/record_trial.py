#!/usr/bin/env python3
"""Zeichnet rohe LiDAR-Scans mit Metadaten als JSONL auf (ein Versuch = eine Datei).

Beispiele:
  python3 record_trial.py --label karton_1.0m --object Karton --dist 1.0 --lateral 0
  python3 record_trial.py --label frei --object keins
  python3 record_trial.py --kind walk --label gang_frei --scans 600
  python3 record_trial.py --label karton_1.0m --object Karton --dist 1.0 --camera   (mit Kamerabildern)
"""
import argparse
import json
import os
import re
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan


def image_msg_to_bgr(msg):
    h, w = msg.height, msg.width
    arr = np.frombuffer(msg.data, np.uint8)
    if msg.encoding in ('rgb8', 'bgr8'):
        a = arr.reshape(h, msg.step)[:, :w * 3].reshape(h, w, 3)
        return a[:, :, ::-1].copy() if msg.encoding == 'rgb8' else a.copy()
    raise ValueError(f'Encoding nicht unterstuetzt: {msg.encoding}')


def make_stem(label):
    safe = re.sub(r'[^A-Za-z0-9._-]+', '_', label)
    return f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_{safe}"


def collect(n_scans, timeout_s=None, image_dir=None, stem=None, n_images=3):
    """Sammelt n_scans LiDAR-Scans; mit image_dir zusaetzlich n_images Kamerabilder
    (gleichmaessig ueber die Aufnahme verteilt, Topic image_raw)."""
    node = rclpy.create_node('trial_recorder')
    scans, images = [], []
    state = {'img': None}
    marks = {max(1, int(n_scans * (k + 1) / (n_images + 1))) for k in range(n_images)} if image_dir else set()

    def on_img(msg):
        state['img'] = msg

    def cb(msg):
        scans.append({
            't': round(time.time(), 3),
            'angle_min': msg.angle_min,
            'angle_increment': msg.angle_increment,
            'range_max': msg.range_max,
            'ranges': [round(float(v), 4) if v == v and abs(v) != float('inf')
                       else None for v in msg.ranges],
        })
        if len(scans) in marks and state['img'] is not None:
            import cv2
            os.makedirs(image_dir, exist_ok=True)
            name = f'{stem}_img{len(images) + 1}.jpg'
            cv2.imwrite(os.path.join(image_dir, name), image_msg_to_bgr(state['img']))
            images.append(os.path.join('images', name))

    node.create_subscription(LaserScan, 'scan', cb, qos_profile_sensor_data)
    if image_dir:
        node.create_subscription(Image, 'image_raw', on_img, qos_profile_sensor_data)
    limit = timeout_s or (n_scans / 5.0 + 10)
    t0 = time.time()
    while rclpy.ok() and len(scans) < n_scans and time.time() - t0 < limit:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    if image_dir:
        return scans, images
    return scans


def write_trial(outdir, meta, scans, stem=None, images=None):
    os.makedirs(outdir, exist_ok=True)
    stem = stem or make_stem(meta['label'])
    path = os.path.join(outdir, f'{stem}.jsonl')
    meta = dict(meta, n_scans=len(scans),
                recorded_at=datetime.now().isoformat(timespec='seconds'))
    if images is not None:
        meta['images'] = images
    with open(path, 'w') as f:
        f.write(json.dumps({'meta': meta}) + '\n')
        for s in scans:
            f.write(json.dumps(s) + '\n')
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--label', required=True)
    ap.add_argument('--kind', default='trial', choices=['trial', 'walk'])
    ap.add_argument('--object', default='keins')
    ap.add_argument('--dist', type=float, default=None, help='wahrer Abstand in Gehrichtung (m)')
    ap.add_argument('--lateral', type=float, default=0.0, help='seitlicher Versatz (m), links positiv')
    ap.add_argument('--front-angle', type=float, default=0.0, help='Sensorwinkel der Gehrichtung (Grad)')
    ap.add_argument('--scans', type=int, default=30)
    ap.add_argument('--notes', default='')
    ap.add_argument('--camera', action='store_true', help='zusaetzlich Kamerabilder (image_raw) speichern')
    ap.add_argument('--outdir', default=os.path.expanduser('~/lidar_trials'))
    a = ap.parse_args()

    rclpy.init()
    stem = make_stem(a.label)
    images = None
    if a.camera:
        scans, images = collect(a.scans, image_dir=os.path.join(a.outdir, 'images'), stem=stem)
        if not images:
            print('ACHTUNG: keine Kamerabilder erhalten. Laeuft die Kamera (/image_raw)?')
    else:
        scans = collect(a.scans)
    rclpy.shutdown()
    meta = {'label': a.label, 'kind': a.kind, 'object': a.object, 'dist_m': a.dist,
            'lateral_m': a.lateral, 'front_angle_deg': a.front_angle, 'notes': a.notes}
    path = write_trial(a.outdir, meta, scans, stem=stem, images=images)
    print(f'{len(scans)} Scans gespeichert: {path}')
    if len(scans) < a.scans:
        print('ACHTUNG: weniger Scans als erwartet. Laeuft der Treiber?')


if __name__ == '__main__':
    main()
