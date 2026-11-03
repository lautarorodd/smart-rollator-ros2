# CLAUDE.md – Smart Rollator (ROS 2 Jazzy, Raspberry Pi 5)

Hindernis- und Absturzkantenwarnung fuer sehbehinderte Rollator-Nutzer. Studentisches Projekt an der EAH Jena.
LiDAR erkennt, Kamera benennt, Sprachausgabe (Piper, Deutsch) warnt. Alles laeuft lokal auf dem Pi, ohne Netz.
Details: README.md, tools/camera_lidar/README.md, tools/lidar_tests/README.md.

## Sprache und Stil
- Mit Lautaro auf **Spanisch** schreiben (Rioplatense, Voseo ist in Ordnung).
- Dokumentation, Kommentare, Log-Texte und Docstrings auf **Deutsch**. In Skripten und Launch-Dateien ohne Umlaute (ue, ae, oe, ss), wie im bestehenden Code.
- Tests im Repo sind einfache assert-Skripte (`tools/camera_lidar/test_*.py`), keine pytest-Abhaengigkeit.

## Wichtig: du laeufst vermutlich ohne Hardware
- In einer Cloud-Sitzung gibt es weder Pi, LiDAR, Kamera noch `dmesg`. Alles, was nur auf dem Geraet pruefbar ist, ist **ungetestet**,
  bis Lautaro es auf dem Pi ausgefuehrt und die Ausgabe gezeigt hat. Schreibe das im Ergebnis dazu und behaupte nie "funktioniert".
- Reine Logik (Winkel, Sektoren, Fusion) in Funktionen ohne ROS-Import auslagern und mit einem `test_*.py` pruefen. `rclpy` erst in `main()` importieren.
- `scripts/rollator.env` enthaelt gemessene Werte. Aenderungen daran zeigen und begruenden, nichts stillschweigend umschreiben.
- Auf dem Pi nichts starten (`start_all.sh` usw.), ohne zu fragen; `stop_all.sh` zuerst, wenn etwas laeuft.

## Aufbau
- `smart_rollator_perception/` – ROS-Paket: `front_obstacle.py` (SAFE/WARN/STOP aus /scan), `hazard_speaker.py` (Sprache, beide Ebenen)
- `scripts/` – Start/Stopp, `rollator.env`, `install_links.sh`. Auf dem Pi liegt der Klon unter `~/ros2_ws/src/smart_rollator_perception`;
  `~/start_all.sh` usw., `~/rollator.env` und `~/camera_lidar` sind Links auf `scripts/` bzw. `tools/camera_lidar/` (angelegt mit `scripts/install_links.sh`).
  Es gibt also nur eine Quelle: eine Aenderung im Repo wirkt sofort auf dem Pi. Das Paket ist mit `colcon build --symlink-install` gebaut.
- `tools/camera_lidar/` – Sensorstart (`sensors_launch.py`, `sensors_high_launch.py`), Detektor, Fusion, Messwerkzeuge (`scan_diff.py`, `scan_profile.py`, `scan_summary.py`)
- `tools/lidar_tests/` – Messkampagne (Aufzeichnung, Plan, Auswertung)
- `tools/gesamttest/` – gefuehrter Gesamttest (beide Ebenen, Fusion, Ansagen) mit Protokoll; Logik in `gesamttest_core.py`

## Konventionen
- `base_link`: x = Gehrichtung, y = links, z = oben. Winkel der LaserScan-Nachricht: -180..180 Grad, links positiv.
- `FRONT` / `HIGH_FRONT` = Sensorwinkel der Gehrichtung des jeweiligen LiDAR (unten -178, oben -177).
- Themen: untere Ebene `/scan`, `rollator/hazard/front`; obere Ebene `/high/scan`, `/rollator/hazard/high`; Fusion `rollator/object/fused` und `..._high`.
- Geraete **immer ueber by-id** ansprechen (`/dev/serial/by-id/...`, `/dev/v4l/by-id/...`), nie ueber `/dev/ttyUSBn` oder `/dev/videoN`; die Nummern wechseln.
  `usb_cam` loest by-id-Links nicht auf: vorher `readlink -f`.

## Hardware (Stand 03.11.2026)
- Raspberry Pi 5, Ubuntu 24.04, offizielles 27-W-Netzteil. LiDAR RPLIDAR S2L (1.000.000 Baud, 10 Hz), unten = Seriennummer f656..., oben = b665... .
- Logitech C920 (nur eine Kamera, unten). Obere Kamera nicht aufgebaut (`HIGH_CAM_ENABLE=0`).
- Oberer LiDAR gegenueber dem unteren: 22 cm hinten, 9,6 cm rechts, Hoehe (`HIGH_LIDAR_Z=0.37`) noch nachzumessen.
- Start: `~/start_all.sh` (untere Ebene), danach `~/start_high.sh` (obere Ebene). Stopp: `~/stop_all.sh`. Sprachausgabe ueber `aplay` (ALSA).
- Bedienung im Feld ohne Monitor: Tablet mit Termius per SSH ueber das eigene WLAN `RollatorPi` des Pi (10.42.0.1, kein Internet).

## Offene Aufgaben (Prioritaet von oben)
1. `front_obstacle.py`: Die Logzeile "naechster Punkt" soll den naechsten Punkt **im Frontsektor** zeigen (und getrennt das globale Minimum); heute ist sie wegen der Eigenechos (0,15 m bei +80 Grad) irrefuehrend. Mit Test.
2. Eigenechos der Montage als zu ignorierende Sektoren fuehren, bevor beide Scans fusioniert werden.
3. `HIGH_LIDAR_Z` und `HIGH_CAM_HEIGHT` in `rollator.env` stimmig machen (nachmessen).
4. Gesamttest mit Lautsprecher fuer beide Ebenen und Protokoll (nur mit Hardware): Werkzeug `tools/gesamttest/gesamttest.py`, Durchfuehrung offen.
5. Kamera erreicht bei Volllast nur 15 bis 19 Hz (Ziel 30) und die CPU 77 Grad C: Ursache klaeren (nur mit Hardware).

## Lizenz und Datenschutz
- Projektlizenz offen; ultralytics/YOLO steht unter AGPL-3.0. Vor Weitergabe klaeren.
- Keine Kamerabilder von Personen ins Repository.
