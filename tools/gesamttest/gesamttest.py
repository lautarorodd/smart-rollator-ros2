#!/usr/bin/env python3
"""Gefuehrter Gesamttest: Frontwarner beider Ebenen, Kamera/Fusion und Sprachausgabe.

Voraussetzung: ~/start_all.sh und (fuer die obere Ebene) ~/start_high.sh laufen.

  python3 gesamttest.py                        # ganzer Plan plans/gesamttest.csv
  python3 gesamttest.py --only 3 4 12          # nur einzelne Schritte

Ablauf je Schritt: Aufbau wird angezeigt -> aufbauen -> ENTER -> das Skript zeichnet dauer_s Sekunden auf
(Zustaende /rollator/hazard/front und /high, Fusion, Ansagen von hazard_speaker ueber /rosout) und
bewertet sofort. Danach fragt es, ob die Ansage wirklich aus dem Lautsprecher kam.
Ergebnis: ~/gesamttest_ergebnisse/<Datum_Zeit>/protokoll.md (nach jedem Schritt gespeichert) und
rohdaten.jsonl (alle Nachrichten). Bewertungslogik: gesamttest_core.py (ohne ROS, test_gesamttest_core.py).
"""
import argparse
import json
import os
import subprocess
import sys
import threading
import time

from gesamttest_core import config_from_env, evaluate, load_plan, parse_announcement, parse_env, report_markdown

HERE = os.path.dirname(os.path.abspath(__file__))


def git_info():
    try:
        h = subprocess.run(['git', '-C', HERE, 'rev-parse', '--short', 'HEAD'],
                           capture_output=True, text=True).stdout.strip()
        dirty = subprocess.run(['git', '-C', HERE, 'status', '--porcelain'],
                               capture_output=True, text=True).stdout.strip()
        return h + (' (mit lokalen Aenderungen)' if dirty else '')
    except OSError:
        return 'unbekannt'


def ask(prompt):
    try:
        return input(prompt).strip()
    except EOFError:
        return 'q'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--plan', default=os.path.join(HERE, 'plans', 'gesamttest.csv'))
    ap.add_argument('--env', default=os.path.expanduser('~/rollator.env'))
    ap.add_argument('--out', default=os.path.expanduser('~/gesamttest_ergebnisse'))
    ap.add_argument('--settle', type=float, default=1.5, help='Wartezeit nach ENTER vor der Aufzeichnung (s)')
    ap.add_argument('--only', nargs='*', help='nur diese Schritt-Nummern')
    a = ap.parse_args()

    env_text = open(a.env).read()
    cfg = config_from_env(parse_env(env_text))
    plan = load_plan(open(a.plan).read())
    if a.only:
        plan = [s for s in plan if s['id'] in a.only]

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rcl_interfaces.msg import Log
    from std_msgs.msg import String

    stamp = time.strftime('%Y-%m-%d_%H%M')
    outdir = os.path.join(a.out, stamp)
    os.makedirs(outdir, exist_ok=True)
    raw = open(os.path.join(outdir, 'rohdaten.jsonl'), 'a')
    events, lock = [], threading.Lock()

    def record(kind, value):
        t = time.monotonic()
        with lock:
            events.append((t, kind, value))
            raw.write(json.dumps({'t': time.time(), 'kind': kind, 'value': value}) + '\n')

    rclpy.init()
    node = Node('gesamttest')
    node.create_subscription(String, '/rollator/hazard/front', lambda m: record('low', m.data), 50)
    node.create_subscription(String, '/rollator/hazard/high', lambda m: record('high', m.data), 50)

    def on_fused(m):
        try:
            record('fused', json.loads(m.data).get('objects', []))
        except ValueError:
            pass
    node.create_subscription(String, '/rollator/object/fused', on_fused, 50)

    def on_log(m):
        if m.name == 'hazard_speaker':
            key = parse_announcement(m.msg)
            if key:
                record('ann', key)
            elif 'fehlt' in m.msg or 'sendet nicht' in m.msg:
                record('speaker_warnung', m.msg)
    node.create_subscription(Log, '/rosout', on_log, 100)

    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    def window(t0, t1):
        with lock:
            sel = [(k, v) for t, k, v in events if t0 <= t <= t1]
        return {'low': [v for k, v in sel if k == 'low'], 'high': [v for k, v in sel if k == 'high'],
                'ann': [v for k, v in sel if k == 'ann'], 'fused': [v for k, v in sel if k == 'fused'],
                'warn': [v for k, v in sel if k == 'speaker_warnung']}

    print('Pruefe, was laeuft (3 s) ...')
    t0 = time.monotonic()
    time.sleep(3.0)
    w = window(t0, time.monotonic())
    low_ok, high_ok, fused_ok = bool(w['low']), bool(w['high']), bool(w['fused'])
    print('  untere Ebene  /rollator/hazard/front : %s' % ('OK (%d Nachr.)' % len(w['low']) if low_ok else 'FEHLT'))
    print('  obere Ebene   /rollator/hazard/high  : %s' % ('OK (%d Nachr.)' % len(w['high']) if high_ok else 'FEHLT'))
    print('  Fusion        /rollator/object/fused : %s' % ('OK' if fused_ok else 'FEHLT'))
    print('  Schwellen: WARN < %.2f m, STOP < %.2f m; oberer LiDAR x=%.3f y=%.3f'
          % (cfg.warn_dist, cfg.stop_dist, cfg.high_x, cfg.high_y))
    if not low_ok:
        print('Untere Ebene sendet nicht. Erst ~/start_all.sh starten (und ~/status.sh pruefen).')
        rclpy.shutdown()
        sys.exit(1)
    if not high_ok and ask('Obere Ebene laeuft nicht (~/start_high.sh). Trotzdem weiter? [j/N] ').lower() != 'j':
        rclpy.shutdown()
        sys.exit(1)

    meta = {'Datum': time.strftime('%d.%m.%Y %H:%M'), 'Repository-Stand': git_info(),
            'Plan': os.path.basename(a.plan), 'WARN_DIST / STOP_DIST': '%.2f / %.2f m' % (cfg.warn_dist, cfg.stop_dist),
            'Oberer LiDAR (x, y)': '%.3f / %.3f m' % (cfg.high_x, cfg.high_y),
            'Obere Ebene aktiv': 'ja' if high_ok else 'nein', 'Fusion aktiv': 'ja' if fused_ok else 'nein',
            'Messfenster': 'nach ENTER %.1f s warten, dann dauer_s Sekunden aufzeichnen' % a.settle}
    tester = ask('Name der Testperson(en) (optional): ')
    if tester:
        meta['Testpersonen'] = tester
    report = os.path.join(outdir, 'protokoll.md')
    rows = []

    for i, step in enumerate(plan, 1):
        print('\n' + '=' * 70)
        print('Schritt %s (%d/%d): %s' % (step['id'], i, len(plan), step['aufbau']))
        cmd = ask('Aufbauen, dann ENTER  (s = ueberspringen, q = beenden): ').lower()
        if cmd == 'q':
            break
        if cmd == 's':
            continue
        if step['art'] == 'annaeherung':
            print('  LOS: jetzt langsam naeher kommen ...')
        time.sleep(a.settle)
        t0 = time.monotonic()
        print('  zeichne %.0f s auf ...' % step['dauer_s'])
        time.sleep(step['dauer_s'])
        w = window(t0, time.monotonic())
        res = evaluate(step, w, cfg, high_active=high_ok)
        print('  unten : erwartet %-18s beobachtet %s' % (res.get('low_exp', ''), res.get('low_obs', '')))
        print('  oben  : erwartet %-18s beobachtet %s' % (res.get('high_exp', ''), res.get('high_obs', '')))
        print('  Kamera: %s' % res['obj'])
        print('  Ansage: erwartet %-18s beobachtet %s' % (res['ann_exp'], res['ann_obs']))
        for msg in sorted(set(w['warn'])):
            print('  hazard_speaker meldet: %s' % msg)
        print('  ==> %s %s' % (res['status'], ('(' + '; '.join(res['gruende']) + ')') if res['gruende'] else ''))
        heard = '-'
        if w['ann']:
            heard = ask('  Ansage im Lautsprecher gehoert? [J/n] ').lower() or 'j'
            if heard.startswith('n'):
                res['status'] = 'FEHLER'
                res['gruende'].append('Ansage geloggt, aber nicht gehoert')
        note = ask('  Bemerkung (ENTER = keine): ')
        rows.append((step, res, heard, note))
        with open(report, 'w') as f:
            f.write(report_markdown(meta, rows))

    raw.close()
    if rows:
        print('\nProtokoll: %s' % report)
        print(next(l for l in report_markdown(meta, rows).splitlines() if l.startswith('Ergebnis:')))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
