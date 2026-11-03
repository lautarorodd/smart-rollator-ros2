import math
from scan_diff import profile_from_ranges, changed_groups, BIN

# Synthetischer Scan: 360 Strahlen (1 Grad), Wand dicht bei Sensorwinkel 0 (hinten), sonst frei ab 3,5 m
def make_scan(person_center=None, person_width=15, person_dist=1.85):
    rng = []
    for i in range(360):
        ang = i - 180.0                                  # -180 .. 179
        r = 3.5
        if abs(ang) <= 40:
            r = 0.45                                     # Wand hinten (Eigenecho, aendert sich nie)
        if person_center is not None:
            d = (ang - person_center + 180.0) % 360.0 - 180.0
            if abs(d) <= person_width / 2:
                r = person_dist
        rng.append(r)
    return (rng, math.radians(-180.0), math.radians(1.0), 0.05, 18.0)

base = profile_from_ranges([make_scan() for _ in range(10)])
obj = profile_from_ranges([make_scan(person_center=-177.0) for _ in range(10)])   # Person ueber den +-180-Sprung hinweg
g = changed_groups(base, obj)
assert len(g) == 1, g
mitte, breite, d_obj, d_leer = g[0]
assert abs(((mitte + 180.0) % 360.0 - 180.0) - (-177.0)) < 2.0, mitte
assert 12 <= breite <= 18, breite
assert abs(d_obj - 1.85) < 0.01 and abs(d_leer - 3.5) < 0.01

# Die Wand (in beiden Messungen gleich) darf nie als Aenderung auftauchen
assert all(abs(x[0]) > 60 for x in g), g

# Keine Aenderung -> leere Liste
assert changed_groups(base, base) == []
# Ungueltige Werte (inf, nan) werden ignoriert
bad = ([float('inf')] * 360, math.radians(-180.0), math.radians(1.0), 0.05, 18.0)
assert profile_from_ranges([bad]) == [99.0] * int(360 / BIN)
print('test_scan_diff ok')
