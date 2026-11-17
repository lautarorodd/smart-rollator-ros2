"""Test fuer speaker_logic.py (ohne ROS). Aufruf aus dem Repo: python3 test/test_speaker_logic.py"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from smart_rollator_perception.speaker_logic import InfoTracker, info_candidate, info_sound  # noqa: E402

ARGS = dict(classes={'door'}, min_conf=0.4, min_dist=1.8, max_dist=4.5, max_bearing_deg=25.0)


def door(d, conf=0.7, bearing=3.0, lidar=True, cls='door'):
    return {'cls': cls, 'conf': conf, 'distance_m': d, 'bearing_deg': bearing, 'lidar': lidar}


# --- Dateiname ---
assert info_sound('door', 3) == 'info_door_3.wav'
assert info_sound('traffic light', 2) == 'info_traffic_light_2.wav'

# --- Kandidat: Klasse, Abstand 1,8..4,5 m (gerundet 2..4), Richtung, Konfidenz, LiDAR-Abstand noetig ---
assert info_candidate(door(3.1), **ARGS) == ('door', 3)
assert info_candidate(door(2.6), **ARGS) == ('door', 3)
assert info_candidate(door(1.9), **ARGS) == ('door', 2)
assert info_candidate(door(4.4), **ARGS) == ('door', 4)
assert info_candidate(door(1.5), **ARGS) is None          # zu nah: das ist Sache der Warnung
assert info_candidate(door(5.0), **ARGS) is None          # zu weit
assert info_candidate(door(3.0, bearing=40.0), **ARGS) is None
assert info_candidate(door(3.0, conf=0.2), **ARGS) is None
assert info_candidate(door(3.0, lidar=False), **ARGS) is None
assert info_candidate(door(3.0, cls='chair'), **ARGS) is None
assert info_candidate({'cls': 'door', 'conf': 0.9, 'lidar': True}, **ARGS) is None   # ohne Abstand

# --- Tracker: erst nach min_hits Sichtungen, dann genau einmal ---
tr = InfoTracker(min_hits=3, window_s=1.5, forget_s=10.0)
tr.seen('door', 3, 0.0)
tr.seen('door', 3, 0.1)
assert tr.due(0.2) is None
tr.seen('door', 3, 0.2)
assert tr.due(0.2) == ('door', 3)
tr.announced('door', 0.3)
for t in (0.4, 0.5, 0.6, 5.0, 9.0):          # Tuer bleibt sichtbar: keine Wiederholung
    tr.seen('door', 2, t)
    assert tr.due(t) is None, t
# 12 s nicht gesehen -> gilt als neue Tuer
for t in (21.0, 21.1, 21.2):
    tr.seen('door', 4, t)
assert tr.due(21.2) == ('door', 4)

# Vereinzelte Sichtungen (Fehlerkennung) loesen nichts aus: Abstand groesser als window_s
tr = InfoTracker(min_hits=3, window_s=1.5, forget_s=10.0)
for t in (0.0, 2.0, 4.0, 6.0):
    tr.seen('door', 3, t)
    assert tr.due(t) is None

# Median der Meter bei schwankendem Abstand
tr = InfoTracker(min_hits=3)
for t, m in ((0.0, 3), (0.1, 4), (0.2, 3)):
    tr.seen('door', m, t)
assert tr.due(0.2) == ('door', 3)

print('test_speaker_logic: alle Tests bestanden')
