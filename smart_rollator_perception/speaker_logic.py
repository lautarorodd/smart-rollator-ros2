"""Reine Logik fuer Hinweisansagen von hazard_speaker (ohne ROS). Test: python3 test/test_speaker_logic.py

Hinweis (Orientierung, keine Warnung): ein Objekt einer Hinweisklasse (z. B. Tuer) wird in 2 bis 4 m Entfernung
genau einmal angesagt ("Tuer vorne, drei Meter"). Erneut erst, wenn es forget_s Sekunden nicht mehr gesehen wurde.
Warnungen (STOP/WARN) und Seitenhinweise haben Vorrang; das entscheidet hazard_speaker.
"""

WORDS = {2: 'zwei', 3: 'drei', 4: 'vier'}


def info_sound(cls, meters):
    """Dateiname der Hinweisansage, z. B. info_door_3.wav."""
    return f"info_{cls.replace(' ', '_')}_{meters}.wav"


def info_candidate(o, classes, min_conf, min_dist, max_dist, max_bearing_deg):
    """Klasse und gerundete Meter (2..4) eines fusionierten Objekts, wenn es als Hinweis taugt, sonst None.
    Verlangt einen LiDAR-Abstand (keine reine Kameraschaetzung)."""
    cls = o.get('cls')
    if cls not in classes or not o.get('lidar') or o.get('conf', 0.0) < min_conf:
        return None
    d = o.get('distance_m')
    if d is None or not (min_dist <= d <= max_dist) or abs(o.get('bearing_deg', 99.0)) > max_bearing_deg:
        return None
    m = int(round(d))
    return (cls, min(max(m, min(WORDS)), max(WORDS)))


class InfoTracker:
    """Merkt sich Sichtungen je Klasse und entscheidet, wann ein Hinweis faellig ist.
    seen(cls, meters, now) bei jeder Sichtung; due(now) -> (cls, meters) oder None; announced(cls, now) nach dem
    Abspielen. Faellig: mindestens min_hits Sichtungen in den letzten window_s Sekunden und noch nicht angesagt
    (oder seit forget_s Sekunden nicht mehr gesehen, dann gilt es als neues Objekt)."""

    def __init__(self, min_hits=3, window_s=1.5, forget_s=10.0):
        self.min_hits, self.window_s, self.forget_s = min_hits, window_s, forget_s
        self.hits = {}          # cls -> [(t, meters), ...]
        self.last_seen = {}     # cls -> t
        self.done = set()       # bereits angesagte Klassen (bis forget_s ohne Sichtung)

    def seen(self, cls, meters, now):
        if cls in self.done and now - self.last_seen.get(cls, now) > self.forget_s:
            self.done.discard(cls)
        self.last_seen[cls] = now
        h = [x for x in self.hits.get(cls, []) if now - x[0] <= self.window_s]
        h.append((now, meters))
        self.hits[cls] = h

    def due(self, now):
        for cls, h in self.hits.items():
            if cls in self.done:
                if now - self.last_seen.get(cls, now) > self.forget_s:
                    self.done.discard(cls)
                else:
                    continue
            recent = [m for t, m in h if now - t <= self.window_s]
            if len(recent) >= self.min_hits:
                return cls, sorted(recent)[len(recent) // 2]      # Median der Meter
        return None

    def announced(self, cls, now):
        self.done.add(cls)
        self.hits[cls] = []
        self.last_seen[cls] = now
