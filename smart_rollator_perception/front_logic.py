"""Reine Logik fuer front_obstacle (ohne ROS, nur numpy). Test: python3 test/test_front_logic.py

Winkel sind Sensorwinkel der LaserScan-Nachricht in Grad (-180..180, links positiv).
Zu ignorierende Sektoren (Eigenechos der Montage: Rahmen, Halterung, Kabel) werden als Text angegeben:
  "70..90, -80..-70"   von..bis gegen den Uhrzeigersinn, Grenzen eingeschlossen; "170..-170" geht ueber +-180.
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
                  warn_dist, stop_dist, min_range, min_points, ignore=()):
    """Bewertet einen Scan. Gibt ein dict zurueck:
      raw       'SAFE' | 'WARN' | 'STOP' (ohne Hysterese)
      front     (abstand_m, sensorwinkel_grad) des naechsten Punkts im Frontsektor oder None
      nearest   (abstand_m, sensorwinkel_grad) des naechsten Punkts insgesamt oder None
      ignored   Anzahl gueltiger Punkte, die in ignorierten Sektoren lagen
    Punkte in ignorierten Sektoren zaehlen weder fuer den Zustand noch fuer front/nearest."""
    r = np.asarray(ranges, dtype=float)
    ang = wrap_deg(np.degrees(angle_min + np.arange(len(r)) * angle_increment))
    rel = wrap_deg(ang - front_deg)

    valid = np.isfinite(r) & (r >= min_range) & (r <= range_max)
    skip = valid & in_sectors(ang, ignore)
    use = valid & ~skip
    in_front = use & (np.abs(rel) <= half_width_deg)

    d = r[in_front]
    if int(np.sum(d < stop_dist)) >= min_points:
        raw = 'STOP'
    elif int(np.sum(d < warn_dist)) >= min_points:
        raw = 'WARN'
    else:
        raw = 'SAFE'

    def closest(mask):
        if not mask.any():
            return None
        i = np.where(mask)[0][np.argmin(r[mask])]
        return float(r[i]), float(ang[i])

    return {'raw': raw, 'front': closest(in_front), 'nearest': closest(use), 'ignored': int(skip.sum())}


def next_state(state, raw, clear_count, clear_scans):
    """Hysterese: schlechter werden sofort, besser erst nach `clear_scans` Scans in Folge.
    Gibt (neuer_zustand, neuer_zaehler, geaendert) zurueck."""
    if ORDER[raw] >= ORDER[state]:
        return raw, 0, raw != state
    clear_count += 1
    if clear_count >= clear_scans:
        return raw, 0, True
    return state, clear_count, False


def status_line(result, state, front_deg, half_width_deg):
    """Logzeile: naechster Punkt im Frontsektor und getrennt das globale Minimum."""
    def fmt(p):
        return 'kein Punkt' if p is None else f'{p[0]:.2f} m bei {p[1]:.0f} Grad'
    front = 'frei' if result['front'] is None else fmt(result['front'])
    line = (f'Front ({front_deg:.0f} +-{half_width_deg:.0f} Grad): {front} | '
            f'global: {fmt(result["nearest"])} | Zustand {state}')
    if result['ignored']:
        line += f' | ignoriert: {result["ignored"]} Punkte'
    return line
