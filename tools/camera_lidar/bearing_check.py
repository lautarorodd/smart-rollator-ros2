#!/usr/bin/env python3
"""Winkeltest Kamera <-> LiDAR (interaktiv).

Ein farbiges Objekt wird nacheinander an mehrere Positionen gestellt. Pro Position
werden der Winkel aus dem Kamerabild (Farbflaeche) und der Winkel aus dem LiDAR
(naechstes Objekt) gemessen und verglichen. Am Ende: lineare Anpassung
cam = a * lidar + b  (ideal a = 1, b = 0).

  python3 bearing_check.py --front-angle 90 --cam-x -0.19 --cam-y 0 --color green

Voraussetzung: Treiber fuer LiDAR (/scan) und Kamera (/image_raw) laufen.
--cam-x / --cam-y: Lage der Kamera relativ zum LiDAR; der LiDAR-Punkt wird damit in das
System der Kamera umgerechnet, bevor die Winkel verglichen werden.
Winkel: 0 = Gehrichtung, positiv = links.
"""
import argparse
import csv
import os
import statistics
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan

from camera_bearing import (find_color_centroid, fit_bearings,
                            nearest_cluster_bearing_deg, pixel_to_bearing_deg,
                            scaled_intrinsics, shift_to_camera)

PRESETS = {  # OpenCV-HSV (H 0-179)
    'green': ((40, 100, 80), (85, 255, 255)),
    'orange': ((5, 150, 120), (20, 255, 255)),
    'pink': ((140, 80, 100), (175, 255, 255)),
    'blue': ((95, 120, 70), (125, 255, 255)),
}


def image_to_rgb(msg):
    h, w = msg.height, msg.width
    arr = np.frombuffer(msg.data, np.uint8)
    if msg.encoding in ('rgb8', 'bgr8'):
        a = arr.reshape(h, msg.step)[:, :w * 3].reshape(h, w, 3)
        return a.copy() if msg.encoding == 'rgb8' else a[:, :, ::-1].copy()
    raise ValueError(f'Encoding nicht unterstuetzt: {msg.encoding} (erwartet rgb8 oder bgr8)')


class Listener(Node):
    def __init__(self):
        super().__init__('bearing_check')
        self.img = None
        self.scan = None
        self.create_subscription(Image, 'image_raw', self.on_img, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)

    def on_img(self, msg):
        self.img = msg

    def on_scan(self, msg):
        self.scan = msg


def measure(node, a, lo, hi, n=20):
    cam_f1, cam_ctr, lid, dist = [], [], [], []
    last_dbg = None
    for _ in range(n):
        time.sleep(0.1)
        img, scan = node.img, node.scan
        if img is None or scan is None:
            continue
        rgb = image_to_rgb(img)
        hit = find_color_centroid(rgb, lo, hi, a.min_area)
        L = nearest_cluster_bearing_deg(scan.ranges, scan.angle_min, scan.angle_increment,
                                        a.front_angle, max_range=a.max_range)
        if hit is None or L is None:
            continue
        u, v, area, mask = hit
        # LiDAR-Punkt in das System der Kamera umrechnen (Tiefen-/Seitenversatz)
        L = shift_to_camera(L[0], L[1], a.cam_x, a.cam_y) + (L[2],)
        w = img.width
        fx, cx = (a.fx or scaled_intrinsics(w)[0]), (a.cx or scaled_intrinsics(w)[1])
        cam_f1.append(pixel_to_bearing_deg(u, fx, cx))
        cam_ctr.append(pixel_to_bearing_deg(u, fx, w / 2))
        lid.append(L[0])
        dist.append(L[1])
        last_dbg = (rgb, u, v, mask)
    if len(lid) < n // 2:
        return None, last_dbg
    return (statistics.median(lid), statistics.median(cam_f1),
            statistics.median(cam_ctr), statistics.median(dist), len(lid)), last_dbg


def save_debug(dbg, path):
    import cv2
    rgb, u, v, mask = dbg
    out = rgb.copy()
    out[mask > 0] = (0.5 * out[mask > 0] + 0.5 * np.array([255, 0, 255])).astype(np.uint8)
    cv2.circle(out, (int(u), int(v)), 8, (255, 255, 255), 2)
    cv2.imwrite(path, out[:, :, ::-1])


def summary(samples, out_csv):
    if len(samples) < 3:
        print('Zu wenige Messpunkte (mindestens 3, besser 5 bis 7).')
        return
    L = [s[0] for s in samples]
    print(f"\n{'LiDAR':>8}{'Kamera(F1)':>12}{'Diff':>8}{'Kamera(Mitte)':>15}{'Diff':>8}{'Abst.':>8}")
    for lid, c1, c2, d in samples:
        print(f'{lid:>8.1f}{c1:>12.1f}{c1 - lid:>8.1f}{c2:>15.1f}{c2 - lid:>8.1f}{d:>8.2f}')
    for name, idx in (('Kalibrierung F1 (halbiert)', 1), ('Bildmitte als Zentrum', 2)):
        res = fit_bearings(L, [s[idx] for s in samples])
        a, b, rms = res
        maxd = max(abs(s[idx] - s[0]) for s in samples)
        print(f'\n{name}: Kamera = {a:.3f} * LiDAR {b:+.2f} Grad | Restfehler RMS {rms:.2f} Grad | groesste Abweichung {maxd:.1f} Grad')
    print('\nLesehilfe: a nahe 1,00 = Blickwinkel stimmt; b = konstanter Versatz (Kamera gedreht oder Bildzentrum falsch).')
    print('Ziel: groesste Abweichung unter 2 Grad.')
    with open(out_csv, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['lidar_deg', 'cam_f1_deg', 'cam_center_deg', 'dist_m'])
        w.writerows(samples)
    print('Gespeichert:', out_csv)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--front-angle', type=float, required=True, help='Sensorwinkel der Gehrichtung (Grad)')
    ap.add_argument('--color', default='green', choices=list(PRESETS))
    ap.add_argument('--hsv-lo', default=None, help='z. B. 40,100,80')
    ap.add_argument('--hsv-hi', default=None, help='z. B. 85,255,255')
    ap.add_argument('--cam-x', type=float, default=0.0, help='Kamera in Gehrichtung relativ zum LiDAR (m); hinter dem LiDAR = NEGATIV, z. B. -0.19')
    ap.add_argument('--cam-y', type=float, default=0.0, help='Kamera seitlich relativ zum LiDAR (m), links positiv')
    ap.add_argument('--fx', type=float, default=None)
    ap.add_argument('--cx', type=float, default=None)
    ap.add_argument('--min-area', type=int, default=150)
    ap.add_argument('--max-range', type=float, default=4.0)
    ap.add_argument('--out', default=os.path.expanduser('~/bearing_samples.csv'))
    ap.add_argument('--debug-img', default=os.path.expanduser('~/bearing_debug.jpg'))
    a, _ = ap.parse_known_args()
    lo, hi = PRESETS[a.color]
    if a.hsv_lo:
        lo = tuple(int(x) for x in a.hsv_lo.split(','))
    if a.hsv_hi:
        hi = tuple(int(x) for x in a.hsv_hi.split(','))

    rclpy.init()
    node = Listener()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    print('Warte auf /image_raw und /scan ...')
    t0 = time.time()
    while (node.img is None or node.scan is None) and time.time() - t0 < 10:
        time.sleep(0.2)
    if node.img is None or node.scan is None:
        print('FEHLER: Kein Bild oder kein Scan. Laufen Kamera und LiDAR?')
        return

    samples = []
    print(__doc__)
    print('Das farbige Objekt muss das NAECHSTE Objekt im vorderen Bereich des LiDAR sein (nichts Naeheres davor).')
    while True:
        cmd = input(f'\nPosition {len(samples) + 1}: Objekt aufstellen, ENTER = messen | f = auswerten | q = Ende > ').strip().lower()
        if cmd == 'q':
            break
        if cmd == 'f':
            summary(samples, a.out)
            continue
        res, dbg = measure(node, a, lo, hi)
        if dbg is not None:
            save_debug(dbg, a.debug_img)
        if res is None:
            print('Keine stabile Messung: Objekt im Bild? Farbe passt? Naechstes Objekt im LiDAR? '
                  f'(Debugbild: {a.debug_img})')
            continue
        lid, c1, c2, d, n = res
        samples.append((lid, c1, c2, d))
        print(f'LiDAR {lid:+.1f} Grad | Kamera(F1) {c1:+.1f} (Diff {c1 - lid:+.1f}) | '
              f'Kamera(Mitte) {c2:+.1f} (Diff {c2 - lid:+.1f}) | Abstand {d:.2f} m | {n} Messungen')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
