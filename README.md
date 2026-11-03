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

## Einrichtung auf dem Pi (frischer Klon)

    cd ~/ros2_ws/src && git clone https://github.com/lautarorodd/smart-rollator-ros2.git smart_rollator_perception
    cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select smart_rollator_perception
    ~/ros2_ws/src/smart_rollator_perception/scripts/install_links.sh   # ~/start_all.sh usw. werden Links auf scripts/
    ~/make_sounds.sh                                                    # Ansagen erzeugen (Piper, einmalig)

Nicht im Repository (auf dem Pi vorhanden): `~/yolo-venv`, `~/piper-venv`, `~/piper-voices`, das YOLO-Modell `yolo11n.pt`,
der LiDAR-Treiber `~/ros2_ws/src/sllidar_ros2`.

## Start
    ~/start_all.sh          # untere Ebene: LiDAR, Kamera, Frontwarner, Sprache, Detektor, Fusion
    ~/start_high.sh         # obere Ebene (zweiter LiDAR), erst nach start_all.sh
    ~/stop_all.sh           # beendet beide Ebenen
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

Bekannte Punkte: Eine geneigte 2D-Ebene ist eine Rampe; haengende Objekte werden nur in einem Abstandsfenster gesehen, die Warnlogik braucht dafuer ein Gedaechtnis. Eigenechos und Frontwinkel der oberen Ebene: siehe Stand 03.11.2026.

## Stand 03.11.2026

Feste Geraete (rollator.env): Beide LiDAR und die Kamera werden ueber /dev/serial/by-id bzw. /dev/v4l/by-id angesprochen
(Seriennummer, unabhaengig vom USB-Port). Unteres LiDAR = Seriennummer f656..., oberes LiDAR = b665... .
usb_cam loest by-id-Links nicht auf: start_sensors.sh wandelt sie mit `readlink -f` in /dev/videoN um (die Nummer
wechselt zwischen Starts, z. B. video0 oder video4).

Obere Ebene:
- Frontwinkel des oberen LiDAR HIGH_FRONT = -177 Grad, gemessen mit `tools/camera_lidar/scan_diff.py` (Person 1,5 m vor dem
  Rollator, Szene leer gegen Szene mit Person; Mitte -177 Grad, Breite 15 Grad). Mit dem Platzhalter 0 Grad zeigte der
  Frontwarner nach hinten auf eine Wand in 0,44 m und meldete STOP bei freier Front.
- Lage gegenueber dem unteren LiDAR: HIGH_LIDAR_X = -0,22 m (hinten), HIGH_LIDAR_Y = -0,096 m (rechts); sensors_high_launch.py
  nimmt dafuer lidar_x und lidar_y entgegen.
- Mit dem richtigen Winkel meldet der obere Frontwarner SAFE bei freier Front und WARN, dann STOP, wenn sich eine Person nahert.
- Eigenechos des oberen LiDAR (scan_profile.py, leerer Frontbereich): 0,14 m bei +80 Grad und 0,33 m bei -83 Grad
  (vermutlich Rahmen oder Halterung, nicht geprueft). Der breite Block von -38 bis +50 Grad bei 0,44 bis 0,48 m ist
  vermutlich die Wand hinter dem Rollator im Testraum. Beide Gruppen liegen ausserhalb des Frontsektors (+-20 Grad um -177).

Messung zwei LiDAR und Kamera gleichzeitig: Beide LiDAR laufen mit 10 Hz (Abstaende 0,096 bis 0,104 s), keine over-current-Meldungen
in dmesg. Kamera nur 15 bis 19 Hz statt 30 Hz bei laufendem Gesamtsystem (mit Detektor und Fusion; ohne diese nicht gemessen); CPU-Temperatur 77 Grad C.
Ein "reset high-speed USB device" an usb 4-1 ist nicht zugeordnet (CP2102N-Adapter sind full-speed, vermutlich die Kamera).

Offene Punkte:
- Log von front_obstacle: "naechster Punkt" ist der Minimalpunkt des ganzen Scans, nicht des Frontsektors (zeigt immer das Eigenecho
  bei 0,15 m), waehrend der Zustand SAFE/WARN/STOP nur den Frontsektor auswertet. Das Log ist irrefuehrend.
- Kameraframerate und Temperatur (Kuehlung, Last von Detektor und Fusion) untersuchen.
- HIGH_LIDAR_Z = 0,37 m und HIGH_CAM_HEIGHT = 1,18 m ergeben zusammen keine stimmige Hoehe der oberen Ebene: nachmessen.
- Gesamttest mit Lautsprecher (Ansage bei WARN/STOP der oberen Ebene, Reihenfolge der Meldungen) noch nicht protokolliert.
- Obere Kamera (HIGH_CAM_ENABLE=0) ist nicht aufgebaut.
