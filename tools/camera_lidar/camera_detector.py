#!/usr/bin/env python3
"""Objekterkennung auf dem Kamerabild (YOLO) -> JSON auf rollator/object/detections.

Muss mit dem Python der virtuellen Umgebung gestartet werden (dort liegt ultralytics),
nachdem ROS geladen wurde:

  ~/yolo-venv/bin/python ~/camera_lidar/camera_detector.py

Optionen: --imgsz 320 (schneller) / 640 (genauer), --conf 0.35,
          --classes chair person  (nur diese Klassen; auch kommagetrennt: "chair,dining table"), --no-image,
          --device auto|cpu|cuda:0  (auto = GPU, wenn torch CUDA sieht, sonst CPU; Pi: cpu, Jetson: cuda:0),
          --model yolo11n.pt  (relativer Name: zuerst neben diesem Skript gesucht, z. B. auch yolo11n.engine)
Die reine Logik (pick_device, resolve_model) kommt ohne ROS aus, Test: test_camera_detector.py.
"""
import argparse
import json
import os
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def pick_device(requested, cuda_available):
    """'auto' -> 'cuda:0' wenn CUDA verfuegbar, sonst 'cpu'. Explizites 'cuda...' ohne CUDA -> ValueError
    (lieber klar abbrechen als still auf der CPU mit 2 Bildern/s laufen)."""
    req = (requested or 'auto').strip().lower()
    if req == 'auto':
        return 'cuda:0' if cuda_available else 'cpu'
    if req.startswith('cuda') or req.isdigit():
        if not cuda_available:
            raise ValueError(f'--device {requested}: torch sieht keine CUDA-GPU (falsche torch-Version im venv?)')
        return req if req.startswith('cuda') else f'cuda:{req}'
    if req == 'cpu':
        return 'cpu'
    raise ValueError(f'--device {requested}: erwartet auto, cpu oder cuda:N')


def parse_classes(items):
    """--classes als Liste (person chair) oder kommagetrennt in einem Argument ("person,dining table,chair",
    so steht es in rollator.env, weil Klassennamen Leerzeichen enthalten). Leer/None -> None (alle Klassen)."""
    if not items:
        return None
    out = []
    for it in items:
        out += [c.strip() for c in it.split(',')] if ',' in it else [it.strip()]
    out = [c for c in out if c]
    return out or None


def resolve_model(model, here=HERE, exists=os.path.exists):
    """Relativer Modellname ohne Verzeichnis: zuerst neben diesem Skript suchen (dort liegt yolo11n.pt),
    sonst unveraendert zurueckgeben (ultralytics laedt Standardmodelle dann selbst herunter)."""
    if os.path.isabs(model) or os.path.dirname(model):
        return os.path.expanduser(model)
    cand = os.path.join(here, model)
    return cand if exists(cand) else model


def image_to_rgb(msg):
    h, w = msg.height, msg.width
    arr = np.frombuffer(msg.data, np.uint8)
    if msg.encoding in ('rgb8', 'bgr8'):
        a = arr.reshape(h, msg.step)[:, :w * 3].reshape(h, w, 3)
        return a.copy() if msg.encoding == 'rgb8' else a[:, :, ::-1].copy()
    raise ValueError(f'Encoding nicht unterstuetzt: {msg.encoding}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='yolo11n.pt')
    ap.add_argument('--imgsz', type=int, default=320)
    ap.add_argument('--conf', type=float, default=0.35)
    ap.add_argument('--classes', nargs='*', default=None)
    ap.add_argument('--no-image', action='store_true')
    ap.add_argument('--device', default='auto', help='auto (GPU falls vorhanden), cpu oder cuda:0')
    a, _ = ap.parse_known_args()

    import torch
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    try:
        a.device = pick_device(a.device, torch.cuda.is_available())
    except ValueError as e:
        raise SystemExit(f'FEHLER: {e}')
    a.model = resolve_model(a.model)
    a.classes = parse_classes(a.classes)

    class Detector(Node):
        def __init__(self, a):
            super().__init__('camera_detector')
            from ultralytics import YOLO
            self.a = a
            self.get_logger().info(f'Lade Modell {a.model} auf {a.device} ...')
            self.model = YOLO(a.model)
            self.class_ids = None
            if a.classes:
                names = {v: k for k, v in self.model.names.items()}
                missing = [c for c in a.classes if c not in names]
                if missing:
                    raise SystemExit(f'Unbekannte Klassen: {missing}')
                self.class_ids = [names[c] for c in a.classes]
            # Aufwaermlauf (der erste Durchlauf ist langsam)
            self.model.predict(np.zeros((360, 640, 3), np.uint8), imgsz=a.imgsz, verbose=False, device=a.device)
            self.pub = self.create_publisher(String, 'rollator/object/detections', 10)
            self.pub_img = None if a.no_image else self.create_publisher(Image, 'detections/image', 1)
            self.latest, self.seq, self.done = None, 0, 0
            self.n_frames, self.t_stat, self.ms_sum = 0, time.time(), 0.0
            self.create_subscription(Image, 'image_raw', self.on_img, qos_profile_sensor_data)
            self.create_timer(0.02, self.process)
            self.get_logger().info('camera_detector bereit, warte auf /image_raw')

        def on_img(self, msg):
            self.latest = msg
            self.seq += 1

        def process(self):
            if self.latest is None or self.seq == self.done:
                return
            msg, self.done = self.latest, self.seq
            rgb = image_to_rgb(msg)
            bgr = np.ascontiguousarray(rgb[:, :, ::-1])
            t0 = time.time()
            res = self.model.predict(bgr, imgsz=self.a.imgsz, conf=self.a.conf,
                                     classes=self.class_ids, verbose=False, device=self.a.device)[0]
            ms = (time.time() - t0) * 1000
            dets = []
            for b in res.boxes:
                x1, y1, x2, y2 = (float(v) for v in b.xyxy[0].tolist())
                dets.append({'cls': self.model.names[int(b.cls[0])], 'conf': round(float(b.conf[0]), 3),
                             'x1': round(x1, 1), 'y1': round(y1, 1), 'x2': round(x2, 1), 'y2': round(y2, 1)})
            out = String()
            out.data = json.dumps({'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                                   'recv': time.time(), 'img_w': msg.width, 'img_h': msg.height,
                                   'inference_ms': round(ms, 1), 'detections': dets})
            self.pub.publish(out)
            if self.pub_img is not None:
                plot = np.ascontiguousarray(res.plot()[:, :, ::-1])      # BGR -> RGB
                im = Image()
                im.header, im.height, im.width = msg.header, plot.shape[0], plot.shape[1]
                im.encoding, im.is_bigendian, im.step = 'rgb8', 0, plot.shape[1] * 3
                im.data = plot.tobytes()
                self.pub_img.publish(im)
            self.n_frames += 1
            self.ms_sum += ms
            if time.time() - self.t_stat >= 5.0:
                dt = time.time() - self.t_stat
                self.get_logger().info(f'{self.n_frames / dt:.1f} Bilder/s, Inferenz {self.ms_sum / max(1, self.n_frames):.0f} ms, '
                                       f'letzte Erkennungen: {[d["cls"] for d in dets]}')
                self.n_frames, self.ms_sum, self.t_stat = 0, 0.0, time.time()

    rclpy.init()
    node = Detector(a)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
