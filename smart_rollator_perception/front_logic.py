"""Reine Logik fuer front_obstacle (ohne ROS, nur numpy). Test: python3 test/test_front_logic.py

Winkel sind Sensorwinkel der LaserScan-Nachricht in Grad (-180..180, links positiv).
Zu ignorierende Sektoren (Eigenechos der Montage: Rahmen, Halterung, Kabel) werden als Text angegeben:
  "70..90, -80..-70"   von..bis gegen den Uhrzeigersinn, Grenzen eingeschlossen; "170..-170" geht ueber +-180.

Zwei Arten, den Bereich vor dem Rollator abzugrenzen:
  Sektor (corridor_half_m = 0, wie bisher): Kegel front_deg +- half_width_deg, Abstand = Entfernung zum LiDAR.
  Gang (corridor_half_m > 0): Rechteck vor dem LiDAR, |y| <= corridor_half_m quer zur Gehrichtung (y im Rollator
    gemessen, lidar_y_m = seitlicher Versatz des LiDAR, links positiv), Abstand = Strecke x in Gehrichtung.
    Daneben links und rechts je ein Streifen side_width_m breit und side_length_m lang: Dinge dort sind kein
    Hindernis im Weg, aber sehr nah ("Vorsicht, rechts"). Ergebnis im Feld 'side'.
    Punkte mit x <= min_x_m (direkt neben dem LiDAR) zaehlen im Gang-Modus nicht: dort ist der Rollator selbst.
  Grund fuer den Gang: der Kegel ist dicht am Rollator schmaler als der Rollator selbst (+-20 Grad: 22 cm breit
  bei 0,3 m) und weit vorn breiter als der Weg (1,1 m bei 1,5 m).
"""
import numpy as np

ORDER = {'SAFE': 0, 'WARN': 1, 'STOP': 2}


def wrap_deg(a):
    """Winkel in Grad auf -180..180 abbilden (Skalar oder Array)."""
    return (np.asarray(a, dtype=float) + 180.0) % 360.0 - 180.0


def parse_sectors(spec):
    """'70..90, -80..-70' -> [(70.0, 90.0), (-80.0, -70.0)]. Leer oder None -> [].
    Trennzeichen zwischen Sektoren: Komma, Semikolon oder Leerzeichen. Fehlerhafte Eintraege -> ValueError."""
    if spec is None:
        return []
    sectors = []
    for item in str(spec).replace(';', ',').replace(' ', ',').split(','):
        item = item.strip()
        if not item:
            continue
        parts = item.split('..')
        if len(parts) != 2:
            raise ValueError(f'Sektor "{item}": erwartet von..bis in Grad, z. B. 70..90')
        try:
            lo, hi = float(parts[0]), float(parts[1])
        except ValueError:
            raise ValueError(f'Sektor "{item}": von und bis muessen Zahlen sein') from None
        sectors.append((float(wrap_deg(lo)), float(wrap_deg(hi))))
    return sectors


def in_sectors(angles_deg, sectors):
    """Bool-Maske: Winkel liegt in einem der Sektoren (gegen den Uhrzeigersinn von lo bis hi, inkl. Grenzen)."""
    a = np.asarray(angles_deg, dtype=float)
    mask = np.zeros(a.shape, dtype=bool)
    for lo, hi in sectors:
        width = (hi - lo) % 360.0
        mask |= ((a - lo) % 360.0) <= width + 1e-9
    return mask


def sectors_overlap_front(sectors, front_deg, half_width_deg, step_deg=0.5):
    """True, wenn ein ignorierter Sektor in den Frontsektor reicht (dort wuerden Hindernisse uebersehen)."""
    if not sectors:
        return False
    probe = front_deg + np.arange(-half_width_deg, half_width_deg + step_deg / 2, step_deg)
    return bool(in_sectors(wrap_deg(probe), sectors).any())


def evaluate_scan(ranges, angle_min, angle_increment, range_max, front_deg, half_width_deg,
                  warn_dist, stop_dist, min_range, min_points, ignore=(),
                  corridor_half_m=0.0, side_width_m=0.0, side_length_m=0.0, lidar_y_m=0.0, min_x_m=0.0):
    """Bewertet einen Scan. Gibt ein dict zurueck:
      raw       'SAFE' | 'WARN' | 'STOP' (ohne Hysterese)
      front     (abstand_m, sensorwinkel_grad) des naechsten Punkts im Frontsektor oder None
      nearest   (abstand_m, sensorwinkel_grad) des naechsten Punkts insgesamt oder None
      ignored   Anzahl gueltiger Punkte, die in ignorierten Sektoren lagen
      side      None | 'links' | 'rechts' | 'beide' (nur im Gang-Modus, sonst immer None)
      side_near (abstand_m, sensorwinkel_grad) des naechsten Punkts in den Seitenstreifen oder None
    Punkte in ignorierten Sektoren zaehlen weder fuer den Zustand noch fuer front/nearest/side.
    Im Gang-Modus ist 'front' der Punkt mit der kleinsten Strecke x in Gehrichtung (Abstand = x)."""
    r = np.asarray(ranges, dtype=float)
    ang = wrap_deg(np.degrees(angle_min + np.arange(len(r)) * angle_increment))
    rel = wrap_deg(ang - front_deg)

    valid = np.isfinite(r) & (r >= min_range) & (r <= range_max)
    skip = valid & in_sectors(ang, ignore)
    use = valid & ~skip

    side, side_near = None, None
    if corridor_half_m > 0:
        rr = np.where(valid, r, 0.0)
        x = rr * np.cos(np.radians(rel))                      # Strecke in Gehrichtung ab LiDAR
        y = rr * np.sin(np.radians(rel)) + lidar_y_m          # quer, im Rollator gemessen (links positiv)
        # Punkte direkt neben dem LiDAR (x <= min_x_m) gehoeren zum Rollator selbst (Rohre, Bremszug, Kabel):
        # dort steht der Rollator schon, ein Hindernis dort waere bereits beruehrt. Zaehlen weder als Gang noch als Seite.
        ahead = use & (x > max(min_x_m, 0.0))
        in_front = ahead & (np.abs(y) <= corridor_half_m)
        dist = x
        outer = corridor_half_m + side_width_m
        near = ahead & (x <= side_length_m)
        left = near & (y > corridor_half_m) & (y <= outer)
        right = near & (y < -corridor_half_m) & (y >= -outer)
        has_l, has_r = int(left.sum()) >= min_points, int(right.sum()) >= min_points
        side = 'beide' if has_l and has_r else 'links' if has_l else 'rechts' if has_r else None
        side_mask = (left if has_l else np.zeros_like(left)) | (right if has_r else np.zeros_like(right))
    else:
        in_front = use & (np.abs(rel) <= half_width_deg)
        dist = r
        side_mask = None

    d = dist[in_front]
    if int(np.sum(d < stop_dist)) >= min_points:
        raw = 'STOP'
    elif int(np.sum(d < warn_dist)) >= min_points:
        raw = 'WARN'
    else:
        raw = 'SAFE'

    def closest(mask, values=r):
        if not mask.any():
            return None
        i = np.where(mask)[0][np.argmin(values[mask])]
        return float(values[i]), float(ang[i])

    if side_mask is not None:
        side_near = closest(side_mask)
    return {'raw': raw, 'front': closest(in_front, dist), 'nearest': closest(use), 'ignored': int(skip.sum()),
            'side': side, 'side_near': side_near}


def next_side(side, raw_side, clear_count, clear_scans):
    """Hysterese fuer die Seitenmeldung: neue oder andere Seite sofort, Verschwinden erst nach `clear_scans`
    Scans ohne Treffer in Folge. Gibt (neue_seite, neuer_zaehler, geaendert) zurueck."""
    if raw_side is not None:
        return raw_side, 0, raw_side != side
    if side is None:
        return None, 0, False
    clear_count += 1
    if clear_count >= clear_scans:
        return None, 0, True
    return side, clear_count, False


def combine_side(a, b):
    """Seitenmeldungen zweier Ebenen zusammenfassen (None, 'links', 'rechts', 'beide')."""
    sides = {s for s in (a, b) if s}
    if not sides:
        return None
    if 'beide' in sides or sides == {'links', 'rechts'}:
        return 'beide'
    return sides.pop()


SIDE_SOUNDS = {'links': 'seite_links.wav', 'rechts': 'seite_rechts.wav', 'beide': 'seite_beide.wav'}


def next_state(state, raw, clear_count, clear_scans):
    """Hysterese: schlechter werden sofort, besser erst nach `clear_scans` Scans in Folge.
    Gibt (neuer_zustand, neuer_zaehler, geaendert) zurueck."""
    if ORDER[raw] >= ORDER[state]:
        return raw, 0, raw != state
    clear_count += 1
    if clear_count >= clear_scans:
        return raw, 0, True
    return state, clear_count, False


def status_line(result, state, front_deg, half_width_deg, corridor_half_m=0.0):
    """Logzeile: naechster Punkt im Frontsektor (bzw. Gang) und getrennt das globale Minimum."""
    def fmt(p):
        return 'kein Punkt' if p is None else f'{p[0]:.2f} m bei {p[1]:.0f} Grad'
    front = 'frei' if result['front'] is None else fmt(result['front'])
    zone = (f'Gang (+-{corridor_half_m:.2f} m)' if corridor_half_m > 0
            else f'Front ({front_deg:.0f} +-{half_width_deg:.0f} Grad)')
    line = (f'{zone}: {front} | '
            f'global: {fmt(result["nearest"])} | Zustand {state}')
    if result.get('side'):
        line += f' | Seite {result["side"]}: {fmt(result["side_near"])}'
    if result['ignored']:
        line += f' | ignoriert: {result["ignored"]} Punkte'
    return line
