from gesamttest_core import (Config, parse_env, config_from_env, load_plan, expected_plane,
                             expected_announcements, parse_announcement, evaluate, compress,
                             contains_in_order, report_markdown, KEINE)

ENV = """FRONT=-178.0      # Kommentar
WARN_DIST=1.5        # Warnabstand
STOP_DIST=0.8
HIGH_LIDAR_X=-0.22
HIGH_LIDAR_Y=-0.096
HIGH_CAM_DEVICE=/dev/v4l/by-path/HIER_EINTRAGEN
"""
env = parse_env(ENV)
assert env['FRONT'] == '-178.0' and env['HIGH_CAM_DEVICE'].endswith('HIER_EINTRAGEN')
cfg = config_from_env(env)
assert (cfg.warn_dist, cfg.stop_dist, cfg.high_x, cfg.high_y) == (1.5, 0.8, -0.22, -0.096)

PLAN = """id;art;objekt;klasse;x_m;y_m;unten;oben;dauer_s;aufbau
# Kommentarzeile
1;statisch;Person;person;1.1;0;ja;ja;5;Person 1,1 m
2;statisch;Person;person;0.4;0;ja;ja;5;Person 0,4 m
3;statisch;Person;person;1.2;0.6;ja;ja;5;Person seitlich
4;statisch;Karton;;1.0;0;ja;nein;5;Karton
5;statisch;Brett;;1.0;0;nein;ja;5;haengend
6;statisch;nichts;;;;nein;nein;5;frei
7;statisch;Flasche;bottle;1.0;0;?;nein;5;Flasche
8;statisch;Person;person;0.85;0;ja;ja;5;Grenze
"""
plan = load_plan(PLAN)
assert [s['id'] for s in plan] == ['1', '2', '3', '4', '5', '6', '7', '8']
S = {s['id']: s for s in plan}

# Erwartete Zustaende je Ebene (oben: 22 cm weiter hinten)
assert expected_plane(S['1'], 'low', cfg) == {'WARN'}
assert expected_plane(S['1'], 'high', cfg) == {'WARN'}          # 1,32 m
assert expected_plane(S['2'], 'low', cfg) == {'STOP'}
assert expected_plane(S['2'], 'high', cfg) == {'STOP'}          # 0,63 m
assert expected_plane(S['3'], 'low', cfg) == {'SAFE'}           # 26,6 Grad -> ausserhalb +-20
assert expected_plane(S['4'], 'high', cfg) == {'SAFE'}
assert expected_plane(S['5'], 'low', cfg) == {'SAFE'}
assert expected_plane(S['6'], 'low', cfg) == {'SAFE'}
assert expected_plane(S['7'], 'low', cfg) is None
assert expected_plane(S['8'], 'low', cfg) == {'STOP', 'WARN'}   # 0,85 m an der Stoppschwelle
assert expected_plane(dict(S['1'], x_m=1.2), 'high', cfg) == {'WARN', 'SAFE'}   # 1,42 m: an der Warnschwelle

# Erwartete Ansagen
ok, part = expected_announcements({'WARN'}, {'WARN'}, S['1'], cfg)
assert ok == {'WARN:person'} and part == {'WARN'}
ok, part = expected_announcements({'SAFE'}, {'WARN'}, S['5'], cfg)
assert ok == {'WARN:oben'} and part == set()
ok, part = expected_announcements({'WARN'}, {'SAFE'}, S['4'], cfg)
assert ok == {'WARN'}
ok, _ = expected_announcements({'SAFE'}, {'SAFE'}, S['6'], cfg)
assert ok == {KEINE}
# Flasche ist keine Bodenklasse -> hazard_speaker nennt sie nicht
ok, _ = expected_announcements({'WARN'}, {'SAFE'}, dict(S['7'], unten='ja'), cfg)
assert ok == {'WARN'}

# Logzeilen von hazard_speaker
assert parse_announcement('spiele STOP') == 'STOP'
assert parse_announcement('spiele WARN') == 'WARN'
assert parse_announcement('spiele WARN (oben)') == 'WARN:oben'
assert parse_announcement('spiele WARN (dining table)') == 'WARN:dining table'
assert parse_announcement('hazard_speaker gestartet') is None

assert compress(['SAFE', 'SAFE', 'WARN', 'WARN', 'STOP']) == ['SAFE', 'WARN', 'STOP']
assert contains_in_order(['SAFE', 'WARN', 'SAFE', 'WARN', 'STOP'], ['SAFE', 'WARN', 'STOP'])
assert not contains_in_order(['SAFE', 'STOP', 'WARN'], ['SAFE', 'WARN', 'STOP'])

person = [{'cls': 'person', 'conf': 0.9, 'lidar': True, 'distance_m': 1.18, 'bearing_deg': 1.0}]

# Statisch, alles richtig
r = evaluate(S['1'], {'low': ['WARN'] * 50, 'high': ['WARN'] * 50, 'ann': ['WARN:person'],
                      'fused': [person] * 20}, cfg)
assert r['status'] == 'OK', r
assert 'person 100%' in r['obj']
# Ansage ohne Objektnamen -> TEILWEISE
r = evaluate(S['1'], {'low': ['WARN'] * 50, 'high': ['WARN'] * 50, 'ann': ['WARN'],
                      'fused': [person] * 20}, cfg)
assert r['status'] == 'TEILWEISE', r
# Kamera erkennt die Person kaum -> TEILWEISE
r = evaluate(S['1'], {'low': ['WARN'] * 50, 'high': ['WARN'] * 50, 'ann': ['WARN:person'],
                      'fused': [person] * 2 + [[]] * 18}, cfg)
assert r['status'] == 'TEILWEISE', r
# Falscher Zustand unten -> FEHLER
r = evaluate(S['1'], {'low': ['SAFE'] * 50, 'high': ['WARN'] * 50, 'ann': ['WARN:oben'],
                      'fused': [person] * 20}, cfg)
assert r['status'] == 'FEHLER' and any('unten' in g for g in r['gruende']), r
# Warnung erwartet, aber keine Ansage -> FEHLER
r = evaluate(S['4'], {'low': ['WARN'] * 50, 'high': ['SAFE'] * 50, 'ann': [], 'fused': []}, cfg)
assert r['status'] == 'FEHLER', r
# Keine Daten -> FEHLER
r = evaluate(S['6'], {'low': [], 'high': ['SAFE'] * 50, 'ann': [], 'fused': []}, cfg)
assert r['status'] == 'FEHLER', r
# Obere Ebene laeuft nicht -> OFFEN statt FEHLER
r = evaluate(S['5'], {'low': ['SAFE'] * 50, 'ann': [], 'fused': []}, cfg, high_active=False)
assert r['status'] == 'OFFEN', r
# Unbekannte Sichtbarkeit -> OFFEN
r = evaluate(S['7'], {'low': ['WARN'] * 50, 'high': ['SAFE'] * 50, 'ann': ['WARN'], 'fused': []}, cfg)
assert r['status'] == 'OFFEN', r

# Annaeherung
walk = {'id': '9', 'art': 'annaeherung', 'objekt': 'Person', 'klasse': 'person', 'x_m': None,
        'y_m': 0.0, 'unten': 'ja', 'oben': 'ja', 'dauer_s': 15, 'aufbau': 'Person kommt naeher'}
seq = ['SAFE'] * 30 + ['WARN'] * 40 + ['STOP'] * 30
r = evaluate(walk, {'low': seq, 'high': seq, 'ann': ['WARN:person', 'WARN:person', 'STOP'], 'fused': []}, cfg)
assert r['status'] == 'OK', r
r = evaluate(walk, {'low': ['SAFE'] * 30 + ['STOP'] * 30, 'high': seq, 'ann': ['STOP'], 'fused': []}, cfg)
assert r['status'] == 'FEHLER', r

# Freie Fahrt: ein kurzer Fehlalarm in 300 Scans (0,3 %) ist erlaubt, Ansage nicht
free = {'id': '10', 'art': 'frei', 'objekt': '', 'klasse': '', 'x_m': None, 'y_m': 0.0,
        'unten': 'nein', 'oben': 'nein', 'dauer_s': 30, 'aufbau': 'freie Fahrt'}
r = evaluate(free, {'low': ['SAFE'] * 299 + ['WARN'], 'high': ['SAFE'] * 300, 'ann': [], 'fused': []}, cfg)
assert r['status'] == 'OK' and '1 Fehlalarm' in r['low_obs'], r
r = evaluate(free, {'low': ['SAFE'] * 300, 'high': ['SAFE'] * 300, 'ann': ['WARN'], 'fused': []}, cfg)
assert r['status'] == 'FEHLER', r

# Obere Ebene unbekannt (?), Rest richtig -> bleibt OK, mit Hinweis
stuhl = dict(S['1'], klasse='chair', oben='?')
chair = [{'cls': 'chair', 'conf': 0.8, 'lidar': True, 'distance_m': 1.1, 'bearing_deg': 0.0}]
r = evaluate(stuhl, {'low': ['WARN'] * 50, 'high': ['SAFE'] * 50, 'ann': ['WARN:chair'], 'fused': [chair] * 20}, cfg)
assert r['status'] == 'OK' and 'oben: nur beobachtet' in r['gruende'], r

md = report_markdown({'Datum': '03.11.2026'}, [(S['1'], evaluate(
    S['1'], {'low': ['WARN'] * 5, 'high': ['WARN'] * 5, 'ann': ['WARN:person'], 'fused': [person]}, cfg), 'j', '')])
assert '| 1 | Person 1,1 m |' in md and 'OK 1' in md
print('test_gesamttest_core ok')
