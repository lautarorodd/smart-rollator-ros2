"""Sprachausgabe fuer den Frontwarner.

WARN (Objekt unter warn_dist_m)  -> "Achtung, <Objekt> vorne." wenn die Kamera das
                                    Objekt sicher erkennt, sonst "Achtung, Hindernis vorne."
STOP (Objekt unter stop_dist_m)  -> "Stopp!" (bewusst kurz)

Die Warnung selbst kommt immer vom LiDAR (rollator/hazard/front). Die Kamera liefert
nur den Namen, und nur dann, wenn ein erkanntes Objekt zum Hindernis passt:
- Konfidenz mindestens min_conf
- vom LiDAR bestaetigt, nahe genug (max_dist_m) und im Frontbereich (max_bearing_deg)
- in den letzten max_age_s Sekunden mindestens min_hits Mal gesehen
"""
import json
import os
import subprocess
import time
from collections import Counter, deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

ORDER = {'SAFE': 0, 'WARN': 1, 'STOP': 2}


def slug(cls):
    return cls.replace(' ', '_')


def pick_label(entries, now, max_age_s, min_hits):
    """entries: Folge von (zeit, klasse). Gibt die haeufigste aktuelle Klasse zurueck
    oder None, wenn keine oft genug gesehen wurde."""
    recent = [c for t, c in entries if now - t <= max_age_s]
    if not recent:
        return None
    cls, hits = Counter(recent).most_common(1)[0]
    return cls if hits >= min_hits else None


def object_qualifies(o, min_conf, max_dist_m, max_bearing_deg):
    return (o.get('lidar') and o.get('conf', 0.0) >= min_conf
            and o.get('match') != 'nearest_erhoeht'
            and o.get('distance_m', 99.0) <= max_dist_m
            and abs(o.get('bearing_deg', 99.0)) <= max_bearing_deg)


class HazardSpeaker(Node):
    def __init__(self):
        super().__init__('hazard_speaker')
        self.declare_parameter('sound_dir', os.path.expanduser('~/rollator_sounds'))
        self.declare_parameter('repeat_warn_s', 4.0)
        self.declare_parameter('repeat_stop_s', 2.0)
        self.declare_parameter('name_objects', True)
        self.declare_parameter('min_conf', 0.6)
        self.declare_parameter('min_hits', 2)
        self.declare_parameter('max_age_s', 1.5)
        self.declare_parameter('max_dist_m', 1.8)
        self.declare_parameter('max_bearing_deg', 25.0)

        self.create_subscription(String, 'rollator/hazard/front', self.on_state, 10)
        self.create_subscription(String, 'rollator/object/fused', self.on_fused, 10)

        self.seen = deque(maxlen=200)      # (zeit, klasse) passender Objekte
        self.proc = None
        self.last_state = 'SAFE'
        self.last_play = 0.0
        self.get_logger().info('hazard_speaker gestartet (mit Objektnamen)')

    def p(self, name):
        return self.get_parameter(name).value

    def on_fused(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        now = time.monotonic()
        for o in data.get('objects', []):
            if object_qualifies(o, self.p('min_conf'), self.p('max_dist_m'),
                                self.p('max_bearing_deg')):
                self.seen.append((now, o['cls']))

    def busy(self):
        return self.proc is not None and self.proc.poll() is None

    def choose_file(self, state):
        if state == 'STOP':
            return 'stop.wav', None
        label = None
        if self.p('name_objects'):
            label = pick_label(self.seen, time.monotonic(), self.p('max_age_s'), self.p('min_hits'))
        if label:
            name = f'warn_{slug(label)}.wav'
            if os.path.isfile(os.path.join(self.p('sound_dir'), name)):
                return name, label
            self.get_logger().warn(f'Keine Ansage fuer "{label}" vorhanden, nehme "Hindernis"',
                                   throttle_duration_sec=10.0)
        return 'warn.wav', None

    def play(self, state):
        name, label = self.choose_file(state)
        path = os.path.join(self.p('sound_dir'), name)
        if not os.path.isfile(path):
            self.get_logger().error(f'Audiodatei fehlt: {path}', throttle_duration_sec=5.0)
            return
        if self.busy():
            self.proc.terminate()
        self.proc = subprocess.Popen(['aplay', '-q', path])
        self.last_play = time.monotonic()
        self.get_logger().info(f'spiele {state}' + (f' ({label})' if label else ''))

    def on_state(self, msg):
        state = msg.data
        if state not in ORDER:
            return
        if state != 'SAFE':
            now = time.monotonic()
            escalated = ORDER[state] > ORDER[self.last_state]
            interval = self.p('repeat_stop_s' if state == 'STOP' else 'repeat_warn_s')
            if escalated or (not self.busy() and now - self.last_play >= interval):
                self.play(state)
        self.last_state = state


def main():
    rclpy.init()
    node = HazardSpeaker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
