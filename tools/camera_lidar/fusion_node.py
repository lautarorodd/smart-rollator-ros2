#!/usr/bin/env python3
"""Erste Sensorfusion: Kamera-Detektionen + LiDAR-Scan -> Objekt mit Klasse und Abstand.

Liest   /scan                         (LiDAR)
        rollator/object/detections    (camera_detector.py)
Schreibt rollator/object/fused        (JSON-Text)

  python3 ~/camera_lidar/fusion_node.py --front-angle FRONT --cam-x -0.19 --cam-y 0 --cam-height 0.46
"""
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from fusion_core import GERMAN, FusionParams, fuse_detection


class Fusion(Node):
    def __init__(self, p):
        super().__init__('fusion_node')
        self.p = p
        self.scan, self.scan_t = None, 0.0
        self.last_empty_log = 0.0
        self.pub = self.create_publisher(String, 'rollator/object/fused', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(String, 'rollator/object/detections', self.on_det, 10)
        self.get_logger().info('fusion_node bereit')

    def on_scan(self, msg):
        self.scan, self.scan_t = msg, time.time()

    def on_det(self, msg):
        d = json.loads(msg.data)
        if self.scan is None or time.time() - self.scan_t > 1.0:
            self.get_logger().warn('Kein aktueller LiDAR-Scan', throttle_duration_sec=3.0)
            return
        objs = []
        for det in d['detections']:
            det = dict(det, img_w=d['img_w'], img_h=d['img_h'])
            res = fuse_detection(det, self.scan.ranges, self.scan.angle_min,
                                 self.scan.angle_increment, self.p)
            entry = {'cls': det['cls'], 'cls_de': GERMAN.get(det['cls'], det['cls']),
                     'conf': det['conf'], 'lidar': res is not None}
            if res:
                entry.update({k: (round(v, 3) if isinstance(v, float) else v)
                              for k, v in res.items() if k != 'd_cam'})
            objs.append(entry)
        out = String()
        out.data = json.dumps({'t': time.time(), 'inference_ms': d.get('inference_ms'), 'objects': objs})
        self.pub.publish(out)
        if not objs:
            if time.time() - self.last_empty_log > 3.0:
                self.get_logger().info('keine Objekte im Bild erkannt')
                self.last_empty_log = time.time()
            return
        for o in objs:
            if not o['lidar']:
                self.get_logger().info(f"{o['cls_de']} ({o['conf']:.2f}): Kamera ja, LiDAR kein Punkt im Winkelbereich")
                continue
            side = 'links' if o['bearing_deg'] > 0 else 'rechts'
            fl = f", Bodenschaetzung {o['d_floor_m']:.2f} m" if o.get('d_floor_m') else ', Boden nicht sichtbar'
            note = ''
            if o['match'] == 'nearest_erhoeht':
                note = '  [Abstand unsicher: Objekt vermutlich nicht am Boden, LiDAR-Ebene liegt tiefer]'
            elif o['match'] == 'nearest_unsicher':
                note = '  [Abstand unsicher: Bodenschaetzung passt nicht zum LiDAR]'
            self.get_logger().info(
                f"{o['cls_de'].upper()} ({o['conf']:.2f}): LiDAR {o['distance_m']:.2f} m, "
                f"{abs(o['bearing_deg']):.0f} Grad {side}{fl} | {o['n_points']} Punkte | {o['match']}{note}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--front-angle', type=float, required=True)
    ap.add_argument('--cam-x', type=float, default=-0.19, help='hinter dem LiDAR = negativ')
    ap.add_argument('--cam-y', type=float, default=0.0)
    ap.add_argument('--cam-height', type=float, default=0.46, help='Hoehe der Linse ueber dem Boden (m)')
    ap.add_argument('--cam-pitch', type=float, default=0.0, help='Neigung nach unten (Grad)')
    ap.add_argument('--margin-deg', type=float, default=2.0)
    ap.add_argument('--max-range', type=float, default=6.0)
    ap.add_argument('--min-points', type=int, default=3)
    ap.add_argument('--no-floor', action='store_true', help='Bodenschaetzung abschalten')
    a, _ = ap.parse_known_args()
    p = FusionParams(front_angle_deg=a.front_angle, cam_x=a.cam_x, cam_y=a.cam_y,
                     cam_height=a.cam_height, cam_pitch_deg=a.cam_pitch, margin_deg=a.margin_deg,
                     max_range=a.max_range, min_points=a.min_points, use_floor=not a.no_floor)
    rclpy.init()
    node = Fusion(p)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
