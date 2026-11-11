# CLAUDE.md – Smart Rollator (Raspberry Pi 5 / ROS 2 Jazzy, Migration auf Jetson Orin Nano / ROS 2 Humble)

Hindernis- und Absturzkantenwarnung fuer sehbehinderte Rollator-Nutzer. Studentisches Projekt an der EAH Jena.
LiDAR erkennt, Kamera benennt, Sprachausgabe (Piper, Deutsch) warnt. Alles laeuft lokal auf dem Rechner am Rollator, ohne Netz.
Bisher Raspberry Pi 5; die Migration auf eine Jetson Orin Nano laeuft (siehe Abschnitt "Migration auf Jetson Orin Nano").
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
- Auf der Jetson laeuft Claude Code direkt auf dem Geraet: Lesen (Geraete, Status, Logs) ist erlaubt, aber keine Knoten starten,
  kein `sudo` und keine Pakete (apt, pip) installieren, ohne Lautaro zu fragen. Auch hier gilt: ungetestet, bis die Ausgabe vorliegt.

## Aufbau
- `smart_rollator_perception/` – ROS-Paket: `front_obstacle.py` (SAFE/WARN/STOP aus /scan), `hazard_speaker.py` (Sprache, beide Ebenen)
- `scripts/` – Start/Stopp, `rollator.env`, `install_links.sh`. Auf dem Pi liegt der Klon unter `~/ros2_ws/src/smart_rollator_perception`;
  `~/start_all.sh` usw., `~/rollator.env` und `~/camera_lidar` sind Links auf `scripts/` bzw. `tools/camera_lidar/` (angelegt mit `scripts/install_links.sh`).
  Es gibt also nur eine Quelle: eine Aenderung im Repo wirkt sofort auf dem Pi. Das Paket ist mit `colcon build --symlink-install` gebaut.
  Auf der Jetson heisst der Klon `~/ros2_ws/src/smart-rollator-ros2` (Paketname bleibt `smart_rollator_perception`, `install_links.sh` funktioniert mit beiden Namen).
- `tools/camera_lidar/` – Sensorstart (`sensors_launch.py`, `sensors_high_launch.py`), Detektor, Fusion, Messwerkzeuge (`scan_diff.py`, `scan_profile.py`, `scan_summary.py`)
- `tools/lidar_tests/` – Messkampagne (Aufzeichnung, Plan, Auswertung)

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

## Migration auf Jetson Orin Nano (Stand 11.11.2026)
- Ziel: Jetson Orin Nano 8 GB, JetPack 6.2.3 (L4T R36), Ubuntu 22.04, ROS 2 Humble aus apt, CUDA 12.6, TensorRT 10.3. Arbeitszweig `jetson-migration`.
- Stand des Pi gesichert im Tag `pi5-jazzy-estable` (= `dc7cbe7`, main nach PR #1). Den Tag nicht veraendern.
- Die Skripte sollen auf beiden Plattformen laufen (Pi/Jazzy und Jetson/Humble), nicht nur auf der Jetson.
- Geprueft auf der Jetson: Python-Code nutzt keine Jazzy-only-APIs (`rclpy.try_shutdown`, `static_transform_publisher --x ...`,
  `PushRosNamespace`, `usb_cam` mit `yuyv2rgb` gibt es in Humble); die reinen Tests laufen mit Python 3.10 / numpy 1.21.
- Plattformabhaengig: `/opt/ros/jazzy` fest in den Startskripten, `wlan0` in `pi_remote_setup.sh` (Jetson: `wlP1p1s0`),
  `thermal_zone0` (Jetson: besser `tj-thermal`), Audio (Jetson ohne Klinke: USB-Lautsprecher noetig).
- Detektor: heute ultralytics `yolo11n.pt` mit `device='cpu'` fest im Code. Ziel: TensorRT-Engine (`.engine`, FP16), auf der Jetson selbst
  exportiert (nicht portabel). numpy < 2 im venv halten, sonst brechen `rclpy`/`cv_bridge`.
- Reihenfolge der Schritte und Entscheidungen: README.md, Abschnitt "Jetson Orin Nano".

## Offene Aufgaben (Prioritaet von oben)
1. `front_obstacle.py`: Die Logzeile "naechster Punkt" soll den naechsten Punkt **im Frontsektor** zeigen (und getrennt das globale Minimum); heute ist sie wegen der Eigenechos (0,15 m bei +80 Grad) irrefuehrend. Mit Test.
2. Eigenechos der Montage als zu ignorierende Sektoren fuehren, bevor beide Scans fusioniert werden.
3. `HIGH_LIDAR_Z` und `HIGH_CAM_HEIGHT` in `rollator.env` stimmig machen (nachmessen).
4. Gesamttest mit Lautsprecher fuer beide Ebenen und Protokoll (nur mit Hardware).
5. Kamera erreicht bei Volllast nur 15 bis 19 Hz (Ziel 30) und die CPU 77 Grad C: Ursache klaeren (nur mit Hardware).

## Lizenz und Datenschutz
- Projektlizenz offen; ultralytics/YOLO steht unter AGPL-3.0. Vor Weitergabe klaeren.
- Keine Kamerabilder von Personen ins Repository.
