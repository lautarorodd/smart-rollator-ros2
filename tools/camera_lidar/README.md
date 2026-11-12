# camera_lidar

Werkzeuge zur gemeinsamen Nutzung von RPLIDAR S2L und Logitech C920.

- `sensors_launch.py` – startet LiDAR, Kamera (YUYV 640 x 360) und statische Transformationen
- `bearing_check.py` – interaktiver Winkeltest Kamera gegen LiDAR
- `camera_detector.py` – YOLO-Objekterkennung auf dem Kamerabild (Python der venv `~/yolo-venv`)
- `fusion_node.py` – erste Sensorfusion: Klasse aus der Kamera, Abstand aus dem LiDAR
- `sensors_high_launch.py` – LiDAR (und optional Kamera) der oberen Ebene im Namensraum /high
- `scan_diff.py` – findet den Frontwinkel eines LiDAR: Szene leer gegen Szene mit Person (Wand und Rahmen fallen heraus); `test_scan_diff.py`.
  Mehrere Topics gleichzeitig und Wartezeit zum Hinstellen, wenn man allein misst: `python3 scan_diff.py /scan /high/scan --delay 15`
- `scan_profile.py` – feste Eigenechos (Rahmen, Halterung, Wand) eines Scans; `scan_summary.py` – Sektortabelle
- `camera_bearing.py`, `fusion_core.py` – reine Funktionen (ohne ROS), getestet mit `test_*.py`

Konvention: 0 Grad = Gehrichtung, positive Winkel = links; Kamera hinter dem LiDAR = negatives cam_x.

Fusionsprinzip: Die Boxkanten der Detektion ergeben einen Winkelbereich; die LiDAR-Punkte in diesem
Bereich werden zu Gruppen zusammengefasst. Die Unterkante der Box liefert eine zweite, unabhaengige
Abstandsschaetzung (Bodenkontakt, Kamerahoehe 0,46 m, Neigung 0 Grad); die Gruppe, die dazu passt, gilt als
das Objekt. Ohne sichtbaren Boden (Objekt zu nah) wird die naechste Gruppe genommen.

Startwerte der Kameraoptik: Kalibrierung der Gruppe F1, halbiert auf 640 Pixel Breite (nicht nachgemessen).
Lizenzhinweis: ultralytics/YOLO steht unter AGPL-3.0; fuer den Prototyp intern in Ordnung, vor einer Lizenzierung klaeren.
