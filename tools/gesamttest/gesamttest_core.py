"""Reine Logik des Gesamttests (ohne ROS), getestet mit test_gesamttest_core.py.

Der Plan speichert nur die physikalische Wahrheit (Objekt, Lage x/y ab unterem LiDAR, welche Ebene es
treffen kann). Die Erwartung (SAFE/WARN/STOP je Ebene, erwartete Ansage) wird hier aus den Werten in
rollator.env berechnet. Liegt ein Objekt nahe an einer Schwelle (Abstand +-grenz_m, Winkel +-grenz_deg),
gelten beide Nachbarzustaende als richtig.

Nachgebildete Regeln (Stand des Codes am 03.11.2026):
- front_obstacle: Frontsektor +-half_width_deg um FRONT, STOP < stop_dist, WARN < warn_dist.
- hazard_speaker: staerkerer Zustand beider Ebenen gilt; STOP -> "Stopp!"; WARN nur oben -> "oben";
  WARN unten -> Objektname, wenn die Fusion es sicher meldet, sonst "Hindernis vorne".
  Objektname nur fuer Klassen am Boden (FLOOR_CLASSES), Abstand <= 1,8 m, Winkel <= 25 Grad.
"""
import csv
import io
import math
import os
import sys
from collections import Counter
from dataclasses import dataclass

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'camera_lidar'))
from fusion_core import FLOOR_CLASSES  # noqa: E402

ORDER = {'SAFE': 0, 'WARN': 1, 'STOP': 2}
STATES = ('SAFE', 'WARN', 'STOP')
KEINE = 'KEINE'          # keine Ansage erwartet


@dataclass
class Config:
    warn_dist: float = 1.5
    stop_dist: float = 0.8
    half_width_deg: float = 20.0      # Parameter von front_obstacle (Standardwert)
    high_x: float = 0.0               # oberer LiDAR gegenueber dem unteren (m)
    high_y: float = 0.0
    grenz_m: float = 0.10             # Toleranz an den Abstandsschwellen
    grenz_deg: float = 3.0            # Toleranz am Rand des Frontsektors
    name_max_dist: float = 1.8        # hazard_speaker max_dist_m
    name_max_bearing: float = 25.0    # hazard_speaker max_bearing_deg
    min_share: float = 0.8            # Anteil richtiger Zustaende im Messfenster
    min_detect: float = 0.5           # Anteil der Fusionsmeldungen mit der erwarteten Klasse
    max_false: float = 0.02           # erlaubter Anteil Nicht-SAFE bei freier Fahrt


def parse_env(text):
    """KEY=WERT aus rollator.env (Kommentare nach # werden entfernt)."""
    out = {}
    for line in text.splitlines():
        line = line.split('#', 1)[0].strip()
        if '=' not in line:
            continue
        k, v = line.split('=', 1)
        out[k.strip()] = v.strip().strip('"\'')
    return out


def config_from_env(env):
    def f(key, default):
        try:
            return float(env.get(key, default))
        except ValueError:
            return default
    return Config(warn_dist=f('WARN_DIST', 1.5), stop_dist=f('STOP_DIST', 0.8),
                  high_x=f('HIGH_LIDAR_X', 0.0), high_y=f('HIGH_LIDAR_Y', 0.0))


def load_plan(text):
    """Plan als CSV (Trennzeichen ;). Spalten: id;art;objekt;klasse;x_m;y_m;unten;oben;dauer_s;aufbau"""
    rows = []
    for r in csv.DictReader(io.StringIO(text), delimiter=';'):
        if not r.get('id') or r['id'].startswith('#'):
            continue
        r = {k: (v or '').strip() for k, v in r.items()}
        r['x_m'] = float(r['x_m']) if r['x_m'] else None
        r['y_m'] = float(r['y_m']) if r['y_m'] else 0.0
        r['dauer_s'] = float(r['dauer_s'] or 5)
        rows.append(r)
    return rows


def plane_polar(x, y, plane, cfg):
    """Abstand (m) und Winkel zur Gehrichtung (Grad, links positiv) aus Sicht der Ebene."""
    if plane == 'high':
        x, y = x - cfg.high_x, y - cfg.high_y
    return math.hypot(x, y), math.degrees(math.atan2(y, x))


def _states_for_range(r, cfg):
    out = set()
    for probe in (r - cfg.grenz_m, r, r + cfg.grenz_m):
        out.add('STOP' if probe < cfg.stop_dist else 'WARN' if probe < cfg.warn_dist else 'SAFE')
    return out


def expected_plane(step, plane, cfg):
    """Menge der richtigen Zustaende einer Ebene; None = keine Erwartung (unbekannt)."""
    vis = step['unten' if plane == 'low' else 'oben']
    if vis == '?':
        return None
    if vis != 'ja' or step['x_m'] is None:
        return {'SAFE'}
    r, ang = plane_polar(step['x_m'], step['y_m'], plane, cfg)
    if abs(ang) > cfg.half_width_deg + cfg.grenz_deg:
        return {'SAFE'}
    states = _states_for_range(r, cfg)
    if abs(ang) >= cfg.half_width_deg - cfg.grenz_deg:
        states.add('SAFE')
    return states


def name_expected(step, cfg):
    """Klasse, die hazard_speaker nennen sollte, oder None."""
    cls = step.get('klasse') or ''
    if not cls or cls not in FLOOR_CLASSES or step['x_m'] is None or step['unten'] != 'ja':
        return None
    r, ang = plane_polar(step['x_m'], step['y_m'], 'low', cfg)
    return cls if r <= cfg.name_max_dist and abs(ang) <= cfg.name_max_bearing else None


def announcement_for(low, high, name):
    eff = max((low, high), key=lambda s: ORDER[s])
    if eff == 'SAFE':
        return KEINE
    if eff == 'STOP':
        return 'STOP'
    if low == 'SAFE':
        return 'WARN:oben'
    return 'WARN:' + name if name else 'WARN'


def expected_announcements(exp_low, exp_high, step, cfg):
    """(richtig, teilweise): Mengen von Ansage-Schluesseln. Teilweise = Warnung ohne Objektnamen."""
    lows = exp_low if exp_low is not None else set(STATES)
    highs = exp_high if exp_high is not None else {'SAFE'}
    name = name_expected(step, cfg)
    ok, part = set(), set()
    for lo in lows:
        for hi in highs:
            a = announcement_for(lo, hi, name)
            ok.add(a)
            if name and a == 'WARN:' + name:
                part.add('WARN')
    return ok, part - ok


def parse_announcement(msg):
    """Logzeile von hazard_speaker -> Schluessel ('STOP', 'WARN', 'WARN:oben', 'WARN:person') oder None."""
    if not msg.startswith('spiele '):
        return None
    rest = msg[len('spiele '):].strip()
    if '(' in rest:
        state, label = rest.split('(', 1)
        return state.strip() + ':' + label.rstrip(')').strip()
    return rest


def shares(states):
    n = len(states)
    c = Counter(states)
    return {s: c[s] / n for s in STATES} if n else {}


def compress(seq):
    out = []
    for s in seq:
        if not out or out[-1] != s:
            out.append(s)
    return out


def contains_in_order(seq, wanted):
    it = iter(seq)
    return all(any(x == w for x in it) for w in wanted)


def fmt_shares(sh):
    if not sh:
        return 'keine Daten'
    return ' '.join('%s %d%%' % (s, round(100 * v)) for s, v in sh.items() if v > 0)


def object_summary(fused_msgs, klasse):
    """fused_msgs: Liste der 'objects'-Listen. Gibt (Text, Anteil der Meldungen mit klasse)."""
    if not fused_msgs:
        return 'keine Fusionsdaten', 0.0
    seen = Counter()
    dist, bear = [], []
    for objs in fused_msgs:
        classes = {o.get('cls') for o in objs if o.get('lidar')}
        seen.update(classes)
        for o in objs:
            if o.get('lidar') and o.get('cls') == klasse:
                dist.append(o.get('distance_m', 0.0))
                bear.append(o.get('bearing_deg', 0.0))
    n = len(fused_msgs)
    parts = []
    for cls, k in seen.most_common(3):
        parts.append('%s %d%%' % (cls, round(100 * k / n)))
    text = ', '.join(parts) if parts else 'nichts erkannt'
    if dist:
        dist.sort()
        bear.sort()
        text += ' (%s: %.2f m, %.0f Grad)' % (klasse, dist[len(dist) // 2], bear[len(bear) // 2])
    return text, (seen[klasse] / n if klasse else 0.0)


def _worst(a, b):
    rank = {'OK': 0, 'OFFEN': 1, 'TEILWEISE': 2, 'FEHLER': 3}
    return a if rank[a] >= rank[b] else b


def evaluate(step, obs, cfg, high_active=True):
    """obs: {'low': [Zustaende], 'high': [...], 'ann': [Schluessel], 'fused': [objects-Listen]}.
    Gibt dict mit erwartet/beobachtet je Teil, 'status' und 'gruende'."""
    art = step['art']
    res = {'gruende': [], 'status': 'OK'}

    def mark(status, grund):
        res['status'] = _worst(res['status'], status)
        res['gruende'].append(grund)

    planes = [('low', 'unten')] + ([('high', 'oben')] if high_active else [])
    if not high_active:
        res['high_exp'], res['high_obs'] = '-', 'obere Ebene nicht aktiv'
        if step['oben'] == 'ja':
            mark('OFFEN', 'obere Ebene nicht aktiv')

    for plane, name in planes:
        states = obs.get(plane, [])
        sh = shares(states)
        res[plane + '_obs'] = fmt_shares(sh)
        if art == 'statisch':
            exp = expected_plane(step, plane, cfg)
            res[plane + '_exp'] = '/'.join(s for s in STATES if s in exp) if exp else 'offen'
            if not states:
                mark('FEHLER', '%s: keine Daten' % name)
            elif exp is None and plane == 'low':
                mark('OFFEN', '%s: nur beobachtet' % name)
            elif exp is None:
                res['gruende'].append('%s: nur beobachtet' % name)   # aendert das Ergebnis nicht
            elif sum(sh.get(s, 0) for s in exp) < cfg.min_share:
                mark('FEHLER', '%s: Zustand falsch' % name)
        elif art == 'annaeherung':
            seq = compress(states)
            res[plane + '_obs'] = ' > '.join(seq) if seq else 'keine Daten'
            if step['unten' if plane == 'low' else 'oben'] == 'ja':
                res[plane + '_exp'] = 'SAFE > WARN > STOP'
                if not contains_in_order(seq, ['SAFE', 'WARN', 'STOP']):
                    mark('FEHLER', '%s: Folge SAFE > WARN > STOP fehlt' % name)
            else:
                res[plane + '_exp'] = 'SAFE'
                if sh and sh.get('SAFE', 0) < cfg.min_share:
                    mark('FEHLER', '%s: Fehlalarm' % name)
        else:  # frei
            res[plane + '_exp'] = 'SAFE'
            alarms = sum(1 for a, b in zip(['SAFE'] + states, states) if a == 'SAFE' and b != 'SAFE')
            res[plane + '_obs'] += ', %d Fehlalarm(e)' % alarms if states else ''
            if not states:
                mark('FEHLER', '%s: keine Daten' % name)
            elif 1.0 - sh.get('SAFE', 0) > cfg.max_false:
                mark('FEHLER', '%s: Fehlalarm' % name)

    anns = obs.get('ann', [])
    res['ann_obs'] = ', '.join(compress(anns)) if anns else KEINE
    if art == 'statisch':
        exp_low = expected_plane(step, 'low', cfg)
        exp_high = expected_plane(step, 'high', cfg) if high_active else {'SAFE'}
        ok, part = expected_announcements(exp_low, exp_high, step, cfg)
        res['ann_exp'] = ' oder '.join(sorted(ok))
        got = set(anns) or {KEINE}
        if exp_low is not None:
            wrong = got - ok - part
            if wrong:
                mark('FEHLER', 'Ansage falsch (%s)' % ', '.join(sorted(wrong)))
            elif got & part:
                mark('TEILWEISE', 'Ansage ohne Objektname')
    elif art == 'annaeherung':
        res['ann_exp'] = 'WARN..., dann STOP'
        seq = compress(['STOP' if a == 'STOP' else 'WARN' for a in anns])
        if not contains_in_order(seq, ['WARN', 'STOP']):
            mark('FEHLER', 'Ansagen WARN dann STOP fehlen')
    else:
        res['ann_exp'] = KEINE
        if anns:
            mark('FEHLER', 'Ansage bei freier Fahrt')

    klasse = step.get('klasse') or ''
    res['obj'], rate = object_summary(obs.get('fused', []), klasse)
    if klasse and art == 'statisch' and step['unten'] == 'ja' and rate < cfg.min_detect:
        r, ang = plane_polar(step['x_m'], step['y_m'], 'low', cfg)
        if abs(ang) <= cfg.name_max_bearing:
            mark('TEILWEISE', 'Kamera erkennt %s nur in %d%%' % (klasse, round(100 * rate)))
    return res


def report_markdown(meta, rows):
    """meta: dict mit Kopfangaben; rows: Liste (step, res, gehoert, bemerkung)."""
    lines = ['# Gesamttest Smart Rollator', '']
    for k, v in meta.items():
        lines.append('- %s: %s' % (k, v))
    cnt = Counter(res['status'] for _, res, _, _ in rows)
    lines += ['', 'Ergebnis: ' + ', '.join('%s %d' % (s, cnt[s]) for s in ('OK', 'TEILWEISE', 'FEHLER', 'OFFEN')
                                          if cnt[s]) + ' (von %d Schritten)' % len(rows), '',
              '| Nr | Aufbau | unten erw. | unten beob. | oben erw. | oben beob. | Kamera/Fusion | Ansage erw. '
              '| Ansage beob. | gehoert | Ergebnis | Bemerkung |',
              '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for step, res, heard, note in rows:
        why = res['status'] + (': ' + '; '.join(res['gruende']) if res['gruende'] else '')
        cells = [step['id'], step['aufbau'], res.get('low_exp', ''), res.get('low_obs', ''),
                 res.get('high_exp', ''), res.get('high_obs', ''), res.get('obj', ''),
                 res.get('ann_exp', ''), res.get('ann_obs', ''), heard, why, note]
        lines.append('| ' + ' | '.join(str(c).replace('|', '/') for c in cells) + ' |')
    return '\n'.join(lines) + '\n'
