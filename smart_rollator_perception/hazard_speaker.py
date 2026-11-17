"""Sprachausgabe fuer den Frontwarner mit ein oder zwei LiDAR-Ebenen.

Eingaenge: rollator/hazard/front (untere Ebene), rollator/hazard/high (obere Ebene, optional),
           rollator/object/fused und rollator/object/fused_high (Objektnamen der Kameras)
Der staerkere Zustand beider Ebenen gilt.

  STOP                      -> "Stopp!" (bewusst kurz)
  WARN nur obere Ebene      -> "Achtung, Hindernis oben."
  WARN (untere Ebene)       -> "Achtung, <Objekt> vorne." bei sicherer Erkennung, sonst "Hindernis vorne"
  nur Seite (beide Ebenen)  -> "Vorsicht, links." / "Vorsicht, rechts." / "Vorsicht, links und rechts."
                               (rollator/hazard/side, rollator/hazard/high_side; schwaechste Stufe, sofort bei neuer
                               Seite, sonst alle repeat_side_s Sekunden)
  Hinweis (info_classes)    -> "Tuer vorne, drei Meter." einmal je Objekt, 2 bis 4 m (speaker_logic.py);
                               nur wenn keine Warnung und kein Seitenhinweis ansteht

Der Zustand einer Ebene gilt nur state_timeout_s Sekunden; verstummt eine Ebene, wird sie als SAFE
gewertet und es erscheint eine Warnung im Log (kein stilles Weiterwarnen mit altem Zustand).
"""
import json
import os
import subprocess
import time
from collections import Counter, deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from smart_rollator_perception.front_logic import SIDE_SOUNDS, combine_side
from smart_rollator_perception.speaker_logic import InfoTracker, info_candidate, info_sound

ORDER = {'SAFE': 0, 'WARN': 1, 'STOP': 2}


def slug(cls):
    return cls.replace(' ', '_')


def pick_label(entries, now, max_age_s, min_hits):
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
        self.declare_parameter('state_timeout_s', 1.0)
        self.declare_parameter('repeat_side_s', 8.0)
        # Hinweisansagen (Orientierung): Klassen kommagetrennt, leer = aus
        self.declare_parameter('info_classes', 'door')
        self.declare_parameter('info_min_conf', 0.2)   # YOLO-World: Tuer stabil, aber nur ~0,23 (12.11.2026)
        self.declare_parameter('info_min_dist_m', 1.8)
        self.declare_parameter('info_max_dist_m', 4.5)
        self.declare_parameter('info_max_bearing_deg', 25.0)

        self.create_subscription(String, 'rollator/hazard/front', self.on_low, 10)
        self.create_subscription(String, 'rollator/hazard/high', self.on_high, 10)
        self.create_subscription(String, 'rollator/object/fused', self.on_fused, 10)
        self.create_subscription(String, 'rollator/object/fused_high', self.on_fused, 10)
        self.create_subscription(String, 'rollator/hazard/side', lambda m: self.set_side('low', m.data), 10)
        self.create_subscription(String, 'rollator/hazard/high_side', lambda m: self.set_side('high', m.data), 10)

        self.state = {'low': 'SAFE', 'high': 'SAFE'}
        self.stamp = {'low': 0.0, 'high': 0.0}
        self.seen = deque(maxlen=200)
        self.proc = None
        self.last_state = 'SAFE'
        self.last_play = 0.0
        self.side = {'low': None, 'high': None}
        self.side_stamp = {'low': 0.0, 'high': 0.0}
        self.last_side = None
        self.last_side_play = 0.0
        self.info = InfoTracker()
        self.info_classes = {c.strip() for c in str(self.p('info_classes')).split(',') if c.strip()}
        self.get_logger().info('hazard_speaker gestartet (zwei Ebenen, Objektnamen)')

    def p(self, name):
        return self.get_parameter(name).value

    def on_fused(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        now = time.monotonic()
        for o in data.get('objects', []):
            if object_qualifies(o, self.p('min_conf'), self.p('max_dist_m'), self.p('max_bearing_deg')):
                self.seen.append((now, o['cls']))
            cand = info_candidate(o, self.info_classes, self.p('info_min_conf'), self.p('info_min_dist_m'),
                                  self.p('info_max_dist_m'), self.p('info_max_bearing_deg'))
            if cand:
                self.info.seen(cand[0], cand[1], now)

    def on_low(self, msg):
        self.set_plane('low', msg.data)

    def on_high(self, msg):
        self.set_plane('high', msg.data)

    def set_plane(self, plane, state):
        if state not in ORDER:
            return
        self.state[plane] = state
        self.stamp[plane] = time.monotonic()
        self.update()

    def set_side(self, plane, value):
        self.side[plane] = value if value in SIDE_SOUNDS else None
        self.side_stamp[plane] = time.monotonic()
        self.update()

    def current_side(self):
        now = time.monotonic()
        timeout = self.p('state_timeout_s')
        fresh = [self.side[pl] if now - self.side_stamp[pl] <= timeout else None for pl in ('low', 'high')]
        return combine_side(*fresh)

    def play_side(self, side):
        path = os.path.join(self.p('sound_dir'), SIDE_SOUNDS[side])
        if not os.path.isfile(path):
            self.get_logger().error(f'Audiodatei fehlt: {SIDE_SOUNDS[side]} (make_sounds.sh)', throttle_duration_sec=10.0)
            return False
        if self.busy():
            return False                            # laufende Warnung nicht fuer einen Seitenhinweis abbrechen
        self.proc = subprocess.Popen(['aplay', '-q', path])
        self.last_side_play = time.monotonic()
        self.get_logger().info(f'spiele Seite {side}')
        return True

    def play_info(self, cls, meters):
        name = info_sound(cls, meters)
        path = os.path.join(self.p('sound_dir'), name)
        self.info.announced(cls, time.monotonic())        # auch ohne Datei nicht dauernd neu versuchen
        if not os.path.isfile(path):
            self.get_logger().error(f'Audiodatei fehlt: {name} (make_sounds.sh)', throttle_duration_sec=10.0)
            return
        self.proc = subprocess.Popen(['aplay', '-q', path])
        self.get_logger().info(f'spiele Hinweis {cls} {meters} m')

    def current(self):
        now = time.monotonic()
        timeout = self.p('state_timeout_s')
        cur = {}
        for plane in ('low', 'high'):
            cur[plane] = self.state[plane] if now - self.stamp[plane] <= timeout else 'SAFE'
        if self.stamp['high'] > 0 and now - self.stamp['high'] > 3.0:
            self.get_logger().warn('Obere Ebene sendet nicht mehr (nicht ueberwacht)',
                                   throttle_duration_sec=10.0)
        return cur

    def busy(self):
        return self.proc is not None and self.proc.poll() is None

    def choose_files(self, eff, cur):
        if eff == 'STOP':
            return ['stop.wav'], None
        if cur['low'] == 'SAFE' and cur['high'] != 'SAFE':
            return ['warn_oben.wav', 'warn.wav'], 'oben'
        label = None
        if self.p('name_objects'):
            label = pick_label(self.seen, time.monotonic(), self.p('max_age_s'), self.p('min_hits'))
        if label:
            return [f'warn_{slug(label)}.wav', 'warn.wav'], label
        return ['warn.wav'], None

    def play(self, eff, cur):
        candidates, label = self.choose_files(eff, cur)
        path = None
        for i, name in enumerate(candidates):
            cand = os.path.join(self.p('sound_dir'), name)
            if os.path.isfile(cand):
                path = cand
                if i > 0:
                    self.get_logger().warn(f'Ansage {candidates[0]} fehlt, nehme {name}',
                                           throttle_duration_sec=10.0)
                    label = None
                break
        if path is None:
            self.get_logger().error(f'Audiodatei fehlt: {candidates[-1]}', throttle_duration_sec=5.0)
            return
        if self.busy():
            self.proc.terminate()
        self.proc = subprocess.Popen(['aplay', '-q', path])
        self.last_play = time.monotonic()
        self.get_logger().info(f'spiele {eff} ({label})' if label else f'spiele {eff}')

    def update(self):
        cur = self.current()
        eff = max(cur.values(), key=lambda s: ORDER[s])
        if eff != 'SAFE':
            now = time.monotonic()
            escalated = ORDER[eff] > ORDER[self.last_state]
            interval = self.p('repeat_stop_s' if eff == 'STOP' else 'repeat_warn_s')
            if escalated or (not self.busy() and now - self.last_play >= interval):
                self.play(eff, cur)
        else:
            side = self.current_side()
            now = time.monotonic()
            if not side:
                self.last_side = None
            elif side != self.last_side or now - self.last_side_play >= self.p('repeat_side_s'):
                if self.play_side(side):
                    self.last_side = side
            if not side and not self.busy():
                due = self.info.due(now)
                if due:
                    self.play_info(*due)
        if eff != 'SAFE':
            self.last_side = None                   # nach einer Warnung die Seite wieder neu ansagen
        self.last_state = eff


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
