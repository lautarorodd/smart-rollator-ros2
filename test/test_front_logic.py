"""Test fuer front_logic.py (ohne ROS). Aufruf aus dem Repo: python3 test/test_front_logic.py"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from smart_rollator_perception.front_logic import (  # noqa: E402
    combine_side, evaluate_scan, in_sectors, next_side, next_state, parse_sectors, sectors_overlap_front,
    status_line, wrap_deg)

FRONT = -178.0
ARGS = dict(front_deg=FRONT, half_width_deg=20.0, warn_dist=1.5, stop_dist=0.8, min_range=0.15, min_points=3)


def make_scan(objects=(), free=5.0):
    """360 Strahlen zu 1 Grad, Sensorwinkel -180..179. objects: (mitte_grad, breite_grad, abstand_m)."""
    rng = []
    for i in range(360):
        ang = i - 180.0
        r = free
        for c, w, d in objects:
            if abs(float(wrap_deg(ang - c))) <= w / 2:
                r = min(r, d)
        rng.append(r)
    return rng, math.radians(-180.0), math.radians(1.0), 18.0


def ev(scan, ignore=()):
    rng, amin, inc, rmax = scan
    return evaluate_scan(rng, amin, inc, rmax, ignore=ignore, **ARGS)


# --- parse_sectors ---
assert parse_sectors('') == [] and parse_sectors(None) == [] and parse_sectors('  ') == []
assert parse_sectors('70..90, -80..-70') == [(70.0, 90.0), (-80.0, -70.0)]
assert parse_sectors('70..90;-80..-70') == parse_sectors('70..90 -80..-70') == [(70.0, 90.0), (-80.0, -70.0)]
assert parse_sectors('170..190') == [(170.0, -170.0)]            # 190 = -170
for bad in ('70', '70..', 'a..b', '1..2..3'):
    try:
        parse_sectors(bad)
        raise AssertionError(f'kein Fehler bei {bad!r}')
    except ValueError:
        pass

# --- in_sectors: Grenzen eingeschlossen, auch ueber +-180 ---
m = in_sectors([69.0, 70.0, 80.0, 90.0, 91.0], [(70.0, 90.0)])
assert list(m) == [False, True, True, True, False], m
m = in_sectors([165.0, 175.0, 180.0, -180.0, -175.0, -165.0], [(170.0, -170.0)])
assert list(m) == [False, True, True, True, True, False], m
assert not in_sectors([0.0], []).any()

# --- Frontsektor-Ueberlappung (Warnung im Knoten) ---
assert not sectors_overlap_front([(70.0, 90.0)], FRONT, 20.0)
assert sectors_overlap_front([(150.0, 165.0)], FRONT, 20.0)       # 162..180 gehoert zum Frontsektor um -178
assert sectors_overlap_front([(-160.0, -150.0)], FRONT, 20.0)     # Grenze -158 liegt drin
assert not sectors_overlap_front([], FRONT, 20.0)

# --- Leere Szene mit Eigenechos 0,15 m (wie auf der Jetson gemessen) ---
echos = [(16.0, 3, 0.15), (-76.0, 3, 0.15), (80.0, 3, 0.15)]
r = ev(make_scan(echos))
assert r['raw'] == 'SAFE'
assert r['front'] is not None and abs(r['front'][0] - 5.0) < 1e-9     # vorne nur freie Flaeche
assert abs(r['nearest'][0] - 0.15) < 1e-9                              # global bleibt das Eigenecho sichtbar
assert r['ignored'] == 0

# --- Person vorn: Zustand und Logwerte zeigen die Person, nicht das Eigenecho ---
r = ev(make_scan(echos + [(-177.0, 15, 0.6)]))
assert r['raw'] == 'STOP', r
assert abs(r['front'][0] - 0.6) < 1e-9 and abs(float(wrap_deg(r['front'][1] - (-177.0)))) <= 8, r['front']
assert abs(r['nearest'][0] - 0.15) < 1e-9
r = ev(make_scan(echos + [(-177.0, 15, 1.2)]))
assert r['raw'] == 'WARN', r
r = ev(make_scan(echos + [(0.0, 30, 0.5)]))          # Person hinten (Sensorwinkel 0): kein Alarm
assert r['raw'] == 'SAFE', r

# --- min_points: ein einzelner Ausreisser loest nichts aus ---
r = ev(make_scan([(-178.0, 1, 0.5)]))
assert r['raw'] == 'SAFE', r

# --- Ignorierte Sektoren: Eigenechos fallen aus global und Zaehlung, Front unberuehrt ---
ign = parse_sectors('14..18, -78..-74, 78..82')
r = ev(make_scan(echos), ignore=ign)
assert r['ignored'] == 15, r['ignored']     # 3 Sektoren zu je 5 Strahlen (Echo + freie Flaeche)
assert abs(r['nearest'][0] - 5.0) < 1e-9, r['nearest']
r = ev(make_scan(echos + [(-177.0, 15, 0.6)]), ignore=ign)
assert r['raw'] == 'STOP' and abs(r['nearest'][0] - 0.6) < 1e-9

# Ein Echo direkt im Frontsektor (z. B. Halterung bei -170) wuerde STOP ausloesen; ignoriert wird es nicht gezaehlt
r = ev(make_scan([(-170.0, 4, 0.3)]))
assert r['raw'] == 'STOP'
r = ev(make_scan([(-170.0, 4, 0.3)]), ignore=parse_sectors('-173..-167'))
assert r['raw'] == 'SAFE'

# --- Ungueltige Werte (inf, nan, zu nah) werden verworfen ---
rng, amin, inc, rmax = make_scan()
rng[2] = float('inf')
rng[3] = float('nan')
rng[4] = 0.05
r = evaluate_scan(rng, amin, inc, rmax, **ARGS)
assert r['raw'] == 'SAFE' and r['front'][0] == 5.0
r = evaluate_scan([float('inf')] * 360, amin, inc, rmax, **ARGS)
assert r['front'] is None and r['nearest'] is None

# --- Hysterese (wie bisher im Knoten) ---
assert next_state('SAFE', 'STOP', 0, 3) == ('STOP', 0, True)
assert next_state('STOP', 'STOP', 0, 3) == ('STOP', 0, False)
assert next_state('STOP', 'SAFE', 0, 3) == ('STOP', 1, False)
assert next_state('STOP', 'SAFE', 1, 3) == ('STOP', 2, False)
assert next_state('STOP', 'SAFE', 2, 3) == ('SAFE', 0, True)
assert next_state('STOP', 'WARN', 1, 3) == ('STOP', 2, False)

# --- Logzeile ---
line = status_line(ev(make_scan(echos + [(-177.0, 15, 0.6)])), 'STOP', FRONT, 20.0)
assert line.startswith('Front (-178 +-20 Grad): 0.60 m bei ') and 'global: 0.15 m bei ' in line, line
assert 'Zustand STOP' in line and 'ignoriert' not in line
line = status_line(ev(make_scan(echos), ignore=ign), 'SAFE', FRONT, 20.0)
assert 'ignoriert: 15 Punkte' in line, line
line = status_line(evaluate_scan([float('inf')] * 360, amin, inc, rmax, **ARGS), 'SAFE', FRONT, 20.0)
assert 'Front (-178 +-20 Grad): frei | global: kein Punkt' in line, line

# --- Gang-Modus: Rollator 57 cm + 2 x 10 cm -> halbe Gangbreite 0,385 m; Seitenstreifen 0,35 m breit, 0,8 m lang ---
GANG = dict(corridor_half_m=0.385, side_width_m=0.35, side_length_m=0.8)


def scan_xy(points, free=5.0, front=FRONT, step=0.25):
    """Scan mit Punkten (x, y, breite_grad) im Rollator (x vorn, y links), LiDAR mittig; 4 Strahlen je Grad."""
    n = int(360 / step)
    rng = [free] * n
    for x, y, w in points:
        c = math.degrees(math.atan2(y, x)) + front
        d = math.hypot(x, y)
        for i in range(n):
            ang = -180.0 + i * step
            if abs(float(wrap_deg(ang - c))) <= w / 2:
                rng[i] = min(rng[i], d)
    return rng, math.radians(-180.0), math.radians(step), 18.0


def evg(scan, ignore=(), **kw):
    rng, amin, inc, rmax = scan
    return evaluate_scan(rng, amin, inc, rmax, ignore=ignore, **dict(ARGS, **dict(GANG, **kw)))


# Leere Szene: SAFE, keine Seite
r = evg(scan_xy([]))
assert r['raw'] == 'SAFE' and r['side'] is None and r['side_near'] is None, r
# Hindernis 0,3 m vor der rechten Rad-Linie (y = -0.28): der Kegel +-20 Grad sieht es nicht, der Gang schon
assert ev(make_scan([(math.degrees(math.atan2(-0.28, 0.30)) + FRONT, 3, math.hypot(0.30, 0.28))]))['raw'] == 'SAFE'
r = evg(scan_xy([(0.30, -0.28, 3)]))
assert r['raw'] == 'STOP' and r['side'] is None, r
assert abs(r['front'][0] - 0.30) < 0.02, r['front']                 # Abstand = Strecke in Gehrichtung
# Person 1,2 m vorn mittig: WARN; 1,4 m vorn, aber 0,48 m seitlich (19 Grad: im Kegel, ausserhalb des Gangs):
# Gang SAFE, Kegel WARN (der Kegel warnt weit vorn vor Dingen neben dem Weg)
assert evg(scan_xy([(1.2, 0.0, 15)]))['raw'] == 'WARN'
assert evg(scan_xy([(1.4, -0.48, 3)]))['raw'] == 'SAFE'
assert ev(make_scan([(math.degrees(math.atan2(-0.48, 1.4)) + FRONT, 3, math.hypot(1.4, 0.48))]))['raw'] == 'WARN'
# Person 0,5 m vorn, 0,55 m rechts der Mitte: kein Hindernis, aber Seite rechts
r = evg(scan_xy([(0.5, -0.55, 10)]))
assert r['raw'] == 'SAFE' and r['side'] == 'rechts', r
assert r['side_near'] is not None and abs(r['side_near'][0] - math.hypot(0.5, 0.55)) < 0.03, r['side_near']
r = evg(scan_xy([(0.5, 0.55, 10)]))
assert r['side'] == 'links', r
r = evg(scan_xy([(0.5, 0.55, 10), (0.4, -0.5, 10)]))
assert r['side'] == 'beide', r
# Seitenstreifen endet bei 0,8 m Laenge und 0,735 m Abstand von der Mitte
assert evg(scan_xy([(1.0, -0.55, 8)]))['side'] is None
assert evg(scan_xy([(0.5, -0.9, 8)]))['side'] is None
# Hinter dem LiDAR (x < 0) zaehlt nichts, auch nicht als Seite
r = evg(scan_xy([(-0.2, -0.5, 10)]))
assert r['raw'] == 'SAFE' and r['side'] is None, r
# Seitlicher Versatz des LiDAR (oben: 9,6 cm rechts): Punkt 0,34 m rechts vom LiDAR liegt 0,436 m rechts der
# Rollatormitte -> ausserhalb des Gangs (Seite); ohne Versatz waere er im Gang (STOP)
assert evg(scan_xy([(0.5, -0.34, 3)]))['raw'] == 'STOP'
r = evg(scan_xy([(0.5, -0.34, 3)]), lidar_y_m=-0.096)
assert r['raw'] == 'SAFE' and r['side'] == 'rechts', r
# Ignorierte Sektoren gelten auch im Gang-Modus
c = math.degrees(math.atan2(-0.28, 0.30)) + FRONT
assert evg(scan_xy([(0.30, -0.28, 3)]), ignore=[(c - 3, c + 3)])['raw'] == 'SAFE'
# Sektor-Modus liefert nie eine Seite
assert ev(make_scan([(-178.0 - 45, 10, 0.7)]))['side'] is None

# min_x_m: Bremszug direkt neben dem oberen LiDAR (Sensor -89 Grad, 0,29 m -> x = 0,01 m) loeste dauernd STOP aus
bz = (0.29 * math.cos(math.radians(88.0)), 0.29 * math.sin(math.radians(88.0)), 4)
assert evg(scan_xy([bz]))['raw'] == 'STOP'
r = evg(scan_xy([bz]), min_x_m=0.05)
assert r['raw'] == 'SAFE' and r['side'] is None, r
# ... ein echtes Hindernis knapp davor (x = 0,15 m, r = 0,16 m > min_range) bleibt sichtbar
assert evg(scan_xy([(0.15, 0.05, 6)]), min_x_m=0.05)["raw"] == "STOP"
# ... und der Sektor-Modus kennt min_x_m nicht (unveraendert)
assert ev(make_scan([(-177.0, 15, 0.6)]))['raw'] == 'STOP'

# --- Hysterese der Seite ---
assert next_side(None, 'rechts', 0, 3) == ('rechts', 0, True)
assert next_side('rechts', 'rechts', 0, 3) == ('rechts', 0, False)
assert next_side('rechts', 'links', 0, 3) == ('links', 0, True)
assert next_side('rechts', None, 0, 3) == ('rechts', 1, False)
assert next_side('rechts', None, 2, 3) == (None, 0, True)
assert next_side(None, None, 0, 3) == (None, 0, False)

# --- Seiten beider Ebenen ---
assert combine_side(None, None) is None
assert combine_side('links', None) == 'links' and combine_side(None, 'rechts') == 'rechts'
assert combine_side('links', 'rechts') == 'beide' and combine_side('beide', None) == 'beide'
assert combine_side('rechts', 'rechts') == 'rechts'

# --- Logzeile im Gang-Modus ---
line = status_line(evg(scan_xy([(0.5, -0.55, 10)])), 'SAFE', FRONT, 20.0, corridor_half_m=0.385)
assert line.startswith('Gang (+-0.39 m): ') and '| Seite rechts: 0.7' in line, line

print('test_front_logic: alle Tests bestanden')
