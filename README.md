# Smart Rollator – ROS 2 Prototyp

Hinderniserkennung für Menschen mit Sehbehinderung am Rollator: LiDAR erkennt, die Kamera benennt,
die Sprachausgabe warnt. Alles läuft lokal auf einem Raspberry Pi 5 ohne Netzverbindung.

## Hardware
- Raspberry Pi 5, offizielles 27-W-Netzteil
- RPLIDAR S2L, vorn am Rollator, ca. 0,28 m über dem Boden
- Logitech C920, ca. 0,46 m über dem Boden, 0,19 m hinter der LiDAR-Achse
- Lautsprecher (Pi 5 hat keine Klinkenbuchse: HDMI, USB oder Bluetooth)

## Software
Ubuntu 24.04, ROS 2 Jazzy. Wichtigste Themen:
- /scan (LiDAR), /image_raw (Kamera, YUYV 640 x 360)
- rollator/hazard/front: SAFE, WARN oder STOP (Frontwarner)
- rollator/object/detections: Kamera-Detektionen (YOLO)
- rollator/object/fused: Detektion mit LiDAR-Abstand (erste Sensorfusion)

## Aufbau des Repositorys
- smart_rollator_perception/: ROS-Paket mit front_obstacle und hazard_speaker
- tools/camera_lidar/: Start von LiDAR und Kamera, Winkeltest, Detektor, Fusion
- tools/lidar_tests/: Messkampagne (Aufzeichnung, Messplan, Auswertung, PROTOKOLL.md)
- scripts/: Startskripte, Einrichtung von SSH und eigenem WLAN
- config/: RViz-Konfiguration

## Installation (Kurzfassung)
1. Ubuntu 24.04 und ros-jazzy-desktop, ros-jazzy-usb-cam installieren
2. Treiber des LiDAR: https://github.com/Slamtec/sllidar_ros2 nach ~/ros2_ws/src klonen und mit colcon bauen
3. Dieses Repository nach ~/ros2_ws/src klonen und bauen
4. Sprachausgabe: Piper in ~/piper-venv, Stimme de_DE-thorsten-medium, Ansagen als WAV in ~/rollator_sounds
5. Detektor: python3 -m venv --system-site-packages ~/yolo-venv und pip install ultralytics

## Start
    ~/start_all.sh
    ~/yolo-venv/bin/python ~/camera_lidar/camera_detector.py
    python3 ~/camera_lidar/fusion_node.py --front-angle <Winkel> --cam-x -0.19 --cam-y 0 --cam-height 0.46

## Stand und Grenzen
- Prototyp im Labor; Schwellenwerte des Frontwarners sind Startwerte (Messkampagne siehe tools/lidar_tests)
- Kameraoptik aus der Kalibrierung der Gruppe F1 übernommen, nicht nachgemessen; Kameraneigung noch zu bestimmen
- Das LiDAR sieht nur seine Ebene: Objekte über oder unter der Ebene und Glas werden nicht zuverlässig erkannt

## Lizenzen und Datenschutz
- Lizenz des Projekts noch nicht festgelegt. ultralytics/YOLO steht unter AGPL-3.0, Piper und die Stimme haben eigene Lizenzen: vor einer Weitergabe klären.
- Kamerabilder zeigen Personen: keine Bilder ohne Einwilligung speichern oder committen.

## Zweite LiDAR-Ebene und Messwerkzeuge (Stand 29.10.2026)

- `scripts/start_high.sh` startet die obere Ebene (zweiter LiDAR, optional Kamera) im Namensraum /high; `scripts/stop_all.sh` beendet beide Ebenen
- `hazard_speaker` wertet beide Ebenen aus (staerkerer Zustand gilt, "Hindernis oben") und nennt bei sicherer Erkennung das Objekt
- `tools/camera_lidar/hoehen_test.py` misst, welche Ebene welche Hoehe in welchem Abstand erkennt, und vergleicht mit dem Rampenmodell
- `tools/camera_lidar/plane_coverage.py` rechnet Erfassungsfenster einer geneigten LiDAR-Ebene vorab
- `tools/camera_lidar/scan_summary.py` zeigt als Text, in welchem Sektor etwas nah am Sensor liegt (Eigenechos)

Bekannte Punkte: Eine geneigte 2D-Ebene ist eine Rampe; haengende Objekte werden nur in einem Abstandsfenster gesehen, die Warnlogik braucht dafuer ein Gedaechtnis. Die obere Ebene ist noch nicht auf Eigenechos und Frontwinkel geprueft.
