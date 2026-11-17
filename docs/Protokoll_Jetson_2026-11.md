# Protokoll: Umzug auf die Jetson Orin Nano und erster Prototyp (11.–12.11.2026)

Zusammenfassung der Arbeitssitzungen mit Claude Code auf der Jetson. Ziel: festhalten, **was** geändert wurde,
**warum**, was gemessen wurde und was noch offen ist – auch das, was nicht im Repository steht (Einstellungen am
System der Jetson, Dateien im Home-Verzeichnis). Zweig: `jetson-migration`, letzter Code-Commit `f231f41`.

Alles „geprüft" gilt für den Raum im Gebäude mit stehendem bzw. langsam geschobenem Rollator. Im Freien und in Fahrt
ist das System **nicht validiert**.

---

## 1. Ergebnis in Kürze

| Bereich | Stand |
|---|---|
| Plattform | Jetson Orin Nano 8 GB, JetPack 6.2.3, Ubuntu 22.04, ROS 2 Humble. Skripte laufen weiter auch auf Pi/Jazzy |
| LiDAR unten + oben | beide 10 Hz, Frontwinkel bestätigt, Eigenechos gemessen und ausgeblendet |
| Warnbereich | **Gang** (Rechteck in Rollatorbreite) statt Kegel ±20°, dazu Seitenstreifen „Vorsicht, links/rechts" |
| Kamera + Erkennung | YOLO auf der **GPU**, ~20–21 Bilder/s (Pi: CPU, 15–19 Hz). Modell YOLO-World mit 20 COCO-Klassen + **Tür** |
| Ansagen | Piper (Deutsch) über **Bluetooth-Lautsprecher** (A2DP). „Stopp!", „Achtung, Person/Stuhl/… vorne", „Achtung, Hindernis oben", „Vorsicht, links/rechts", „Tür vorne, drei Meter" |
| Feldbetrieb | eigenes WLAN `RollatorJetson` (10.42.0.1), Betrieb ohne Monitor und ohne Anmeldung, Kaltstart getestet |
| Offen (wichtig) | Lautsprecher trennt sich alle 30–120 s; Dauer-STOP in der letzten Sitzung (Ursache unklar, Abschnitt 8) |

---

## 2. Ablauf (Commits)

| Datum | Commit | Inhalt |
|---|---|---|
| 11.11. | `3a6f6bf` | Doku: Migrationsplan (README, Abschnitt „Jetson Orin Nano") |
| 11.11. | `62eaa22` | Startskripte für Pi/Jazzy und Jetson/Humble (`plattform.sh`), Temperatur über Zonentyp |
| 11.11. | `a4285f1` | Schritte 3 und 4 geprüft: beide LiDAR, Kamera, tf auf der Jetson |
| 11.11. | `a3fa325` | `front_obstacle`: Logzeile trennt Frontsektor und globales Minimum |
| 11.11. | `6401b9b` | `IGNORE_SECTORS` / `HIGH_IGNORE_SECTORS` in den Startskripten |
| 12.11. | `bf81adb` | `scan_diff.py`: mehrere Topics gleichzeitig, `--delay` (Messen allein möglich) |
| 12.11. | `021c0cc` | Detektor auf der GPU (`--device auto`), Eigenechos gemessen, Gesamttest beider Ebenen |
| 12.11. | `0d8ddd9` | Feldbetrieb: Hotspot, linger, Bluetooth-Lautsprecher beim Start (`bt_speaker.sh`) |
| 12.11. | `8b15e27` | Gang statt Kegel, Seitenhinweis „Vorsicht, links/rechts" |
| 12.11. | `15694dd` | Gang: Punkte direkt neben dem LiDAR ignorieren (`corridor_min_x_m`) |
| 12.11. | `3230c61` | 20 Objektklassen, Hinweisansage „Tür vorne, N Meter" (`speaker_logic.py`) |
| 12.11. | `f231f41` | YOLO-World mit Tür, Schwellen, Türabstand in der Bildmitte |

Der Stand des Pi ist im Tag `pi5-jazzy-estable` (= `dc7cbe7`) gesichert.

---

## 3. Änderungen im Repository und warum

### 3.1 Messwerkzeuge
- **`tools/camera_lidar/scan_diff.py`**: `scan_diff.py /scan /high/scan --delay 15` misst beide LiDAR in einem
  Durchgang; nach dem zweiten ENTER bleiben 15 s Zeit (Piepton), um sich selbst vor den Rollator zu stellen.
  *Warum:* Lautaro misst allein, steht mit dem Rechner hinter dem Rollator. Wer hinten weggeht, stört nicht, weil
  nur Stellen zählen, an denen die Distanz abnimmt.

### 3.2 Warnlogik (`smart_rollator_perception/front_logic.py`, ohne ROS, Test `test/test_front_logic.py`)
- **Gang statt Kegel.** Der Kegel ±20° war dicht am Rollator schmaler als der Rollator (22 cm breit bei 0,3 m) und
  weit vorn breiter als der Weg (1,1 m bei 1,5 m). Neu: Rechteck vor dem LiDAR, halbe Breite
  `CORRIDOR_HALF_WIDTH = 0,385 m` (Rollator 57 cm Rad zu Rad außen / 2 + 10 cm Rand). WARN/STOP nach der Strecke
  in Gehrichtung (x), nicht nach der Schrägentfernung.
- **Seitenstreifen** links und rechts neben dem Gang (`SIDE_WIDTH` 0,35 m, `SIDE_LENGTH` 0,8 m): kein Hindernis im
  Weg, aber sehr nah → Thema `rollator/hazard/side` (`links`, `rechts`, `beide`, `NONE`).
- **Seitlicher Versatz des LiDAR** (`lidar_y_m`): der obere LiDAR sitzt 9,6 cm rechts; sein Gang wird auf die
  Rollatormitte bezogen (`HIGH_LIDAR_Y`).
- **`corridor_min_x_m` = 0,05 m**: Punkte direkt neben dem LiDAR zählen nicht. *Warum:* nach dem Hochstellen der
  Griffe löste ein loser Bremszug oben dauernd STOP aus (Sensor −89°, 0,29 m, x = 0,01 m). Er lag nur zeitweise im
  Strahl, daher fand `scan_profile.py` kein festes Eigenecho, und ein fester Sektor hätte nicht gereicht.
- Ohne `CORRIDOR_HALF_WIDTH` bleibt der alte Kegel → der Pi verhält sich unverändert.

### 3.3 Knoten
- **`front_obstacle.py`**: Parameter `corridor_half_width_m`, `side_width_m`, `side_length_m`, `lidar_y_m`,
  `corridor_min_x_m`; veröffentlicht zusätzlich `rollator/hazard/side`; Logzeile „Gang (±0.39 m): …".
- **`hazard_speaker.py`**: Vorrang **STOP > WARN > Seite > Hinweis**.
  - „Vorsicht, links/rechts/links und rechts": sofort bei neuer Seite, sonst alle 8 s, unterbricht keine Warnung.
  - Hinweis „Tür vorne, zwei/drei/vier Meter": einmal je Tür in 1,8–4,5 m, ±25°, nur mit LiDAR-Abstand, erneut
    erst nach 10 s ohne Sichtung. Logik in **`speaker_logic.py`** (ohne ROS, Test `test/test_speaker_logic.py`).
- **`tools/camera_lidar/camera_detector.py`**: `--device auto|cpu|cuda:0` (auto = GPU, falls torch CUDA sieht;
  Pi bleibt auf CPU), `--model` (Name neben dem Skript gesucht), `--classes` auch kommagetrennt. ROS-Importe in
  `main()`; Test `test_camera_detector.py`.
- **`tools/camera_lidar/fusion_core.py`**: für Türen (`WIDE_CLASSES`) Abstand in der **Bildmitte** statt nächster
  Gruppe. *Warum:* die Tür füllt das Bild, die nächste Gruppe war Rahmen oder Seitenwand (2,03 m bei 33° statt
  Tür bei 10°). Mitte mehr als 0,8 m hinter den Rändern → offene Tür → Abstand des Rahmens. Deutsche Namen für
  Bus, Lastwagen, Motorrad, Ampel, Hydrant, Stoppschild.

### 3.4 Skripte und Einstellungen
- **`scripts/plattform.sh`**: lädt ROS über `ROS_SETUP`/`ROS_DISTRO`/humble/jazzy statt fest `/opt/ros/jazzy`;
  Temperatur aus Zone `tj-thermal`.
- **`scripts/start_all.sh` / `start_high.sh`**: übergeben Gang-, Seiten- und Ignorier-Parameter sowie
  `DETECT_MODEL`, `DETECT_CLASSES`, `DETECT_CONF`, `DETECT_IMGSZ` aus `rollator.env`; `start_all.sh` ruft
  `bt_speaker.sh` auf, wenn `BT_SPEAKER` gesetzt ist; oben Remap `rollator/hazard/side` → `/rollator/hazard/high_side`.
- **`scripts/bt_speaker.sh`** (neu): startet zuerst PulseAudio, dann Verbindung, dann A2DP als Standardausgabe.
  *Warum:* nach Kaltstart verband sich der Lautsprecher nur mit dem Freisprechprofil, weil PulseAudio die
  A2DP-Endpunkte erst beim ersten Zugriff anmeldet.
- **`scripts/pi_remote_setup.sh`**: für Pi und Jetson (WLAN-Gerät automatisch, Name als Argument, 2,4 GHz Kanal 6,
  `loginctl enable-linger`).
- **`scripts/make_sounds.sh`**: Seitenhinweise, Türhinweise, weitere Klassen (30 Ansagen). „Tür" mit Umlaut im
  Ansagetext, weil Piper „Tuer" falsch ausspricht.
- **`scripts/rollator.env`** (Werte und Herkunft siehe Abschnitt 5).

---

## 4. Änderungen am System der Jetson (NICHT im Repository)

Diese Punkte gehen bei einer Neuinstallation verloren und müssen von Hand wiederholt werden.

| Was | Wo / Befehl | Warum |
|---|---|---|
| A2DP für Bluetooth freigeschaltet | `/etc/systemd/system/bluetooth.service.d/nv-bluetooth-service.conf` mit `[Service]`, `ExecStart=`, `ExecStart=/usr/lib/bluetooth/bluetoothd`; danach `daemon-reload`, `restart bluetooth` | JetPack startet `bluetoothd` mit `--noplugin=audio,a2dp,avrcp` → nur Freisprechprofil 8 kHz |
| linger | `sudo loginctl enable-linger lautaro` | PulseAudio läuft ohne Anmeldung und bleibt nach Trennen der SSH-Sitzung aktiv |
| PulseAudio beim Booten | `systemctl --user enable pulseaudio.service` | zusammen mit linger: Ton ohne Monitor/Anmeldung |
| Nur A2DP, kein Freisprechprofil | `~/.config/pulse/default.pa` (lädt `module-bluetooth-discover headset=ofono`) | Versuch gegen die Abbrüche des Lautsprechers (Abschnitt 8); rückgängig: Datei löschen, `systemctl --user restart pulseaudio` |
| apt-Paket `cuda-cupti-12-6` | `sudo apt install cuda-cupti-12-6` | torch braucht `libcupti.so.12` |
| Hotspot `RollatorJetson` | NetworkManager-Verbindung `RollatorHotspot`, 2,4 GHz Kanal 6, Autostart; `scripts/pi_remote_setup.sh RollatorJetson` | SSH im Feld ohne Netzkabel |
| `~/piper-venv` | `python3 -m venv --without-pip` + `get-pip.py`, `pip install piper-tts` (1.8) | Sprachsynthese; `python3-venv` fehlt auf der Jetson (sudo nötig) |
| `~/piper-voices/de_DE-thorsten-medium.onnx` (+ `.json`) | von huggingface.co/rhasspy/piper-voices | deutsche Stimme |
| `~/rollator_sounds/*.wav` (30) | `scripts/make_sounds.sh` | Ansagen offline |
| `~/yolo-venv` (`--system-site-packages`) | torch 2.8.0 + torchvision 0.23.0 aus `https://pypi.jetson-ai-lab.io/jp6/cu126`, ultralytics 8.4, numpy 1.26 (`-c` mit `numpy<2`), CLIP aus `git+https://github.com/ultralytics/CLIP.git` | GPU-Erkennung; torch ≥ 2.9 verlangt `libcudss.so.0`, das es für JetPack 6.2 nicht gibt; numpy 2 bricht `rclpy`/`cv_bridge` |
| `~/camera_lidar/yolo_world_rollator.pt` | aus `yolov8s-worldv2.pt` mit `set_classes(DETECT_CLASSES + door)` und `save()` erzeugt | YOLO-World mit festen Klassen; zur Laufzeit kein CLIP nötig. `.pt` wird von git ignoriert |
| git-Identität | `user.name` / `user.email` auf der Jetson gesetzt | Commits direkt auf der Jetson |

**Modell neu erzeugen** (mit Internet, CLIP installiert):
```
cd ~/camera_lidar && source ~/rollator.env && ~/yolo-venv/bin/python -c "
from ultralytics import YOLO
from camera_detector import parse_classes
m = YOLO('yolov8s-worldv2.pt'); m.set_classes(parse_classes(['$DETECT_CLASSES'])); m.save('yolo_world_rollator.pt')"
```
(`door` steht bereits in `DETECT_CLASSES`.)

---

## 5. Messwerte und Einstellungen (`scripts/rollator.env`)

| Wert | Einstellung | Herkunft |
|---|---|---|
| Frontwinkel unten | `FRONT=-178` (gemessen −177,5) | `scan_diff.py`, Person ~1 m vorn: zwei Gruppen = zwei Beine bei −172 und 177 |
| Frontwinkel oben | `HIGH_FRONT=-177` (gemessen −173, minus 3,7° Parallaxe durch 9,6 cm Versatz = −176,7) | `scan_diff.py` |
| Eigenechos unten | `IGNORE_SECTORS="-56..-46,-36..-16,30..40,52..60"` | `scan_profile.py` + Fotos: Korbknäufe links/rechts 0,27 m, Jetson/Kabel 0,22–0,43 m, Rahmen hinten rechts 0,46 m |
| Eigenechos oben | `HIGH_IGNORE_SECTORS="-86..-80,78..86"` | senkrechte Rahmenrohre links 0,33 m / rechts 0,13 m |
| Gang / Seite | `CORRIDOR_HALF_WIDTH=0.385`, `SIDE_WIDTH=0.35`, `SIDE_LENGTH=0.8` | Rollator 57 cm gemessen, Rand 10 cm; Seitenwerte Startwerte |
| Warnabstände | `WARN_DIST=1.5`, `STOP_DIST=0.8` | Startwerte (unverändert) |
| Erkennung | `DETECT_MODEL=yolo_world_rollator.pt`, `DETECT_CONF=0.2`, `DETECT_IMGSZ=640`, 20 COCO-Klassen + door | Tür in 20/20 Bildern mit nur ~0,23–0,34; bei 320 px gar nicht |
| Lautsprecher | `BT_SPEAKER=F4:2B:7D:67:7D:E7` (Anker SoundCore) | Bluetooth, weil alle USB-Ports belegt sind |

Weitere Messungen:
- GPU gegen CPU (yolo11n, 640×360): **39 ms** gegen 425 ms je Bild. YOLO-World v2 s: ebenso ~39–40 ms (Zeit geht
  in der Vorverarbeitung verloren, nicht im Netz). Im Betrieb ~20–21 Bilder/s, Inferenz 41–44 ms.
- Temperatur `tj-thermal` im Betrieb 45 °C (Pi: 77 °C).
- Kamera `/image_raw` 29,6 Hz bei 640×360 (`yuyv2rgb` auf der CPU).
- Gesamttest im Raum: Person 1,25 m → WARN, 0,52 m → STOP, seitlich weg → SAFE; „Person" und „Stuhl" benannt.
- Oberer LiDAR um **6° nach oben** geneigt (Angabe Lautaro). In `rollator.env`/tf noch nicht abgebildet.

---

## 6. Bedienung im Feld

1. Jetson an der Akkubank einschalten, ~1 min warten. Rechner mit WLAN **RollatorJetson** verbinden,
   `ssh lautaro@10.42.0.1`.
2. Uhrzeit setzen (Jetson hat keinen RTC-Puffer, das Hochschulnetz sperrt NTP). Unter **Windows/PowerShell**:
   ```
   ssh -t lautaro@10.42.0.1 "sudo date -s '$(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')'"
   ```
3. `~/start_all.sh` (muss „Lautsprecher bereit" melden), danach `~/start_high.sh`.
4. Lautsprecher auf volle Lautstärke; Handprobe vor jedem LiDAR.
5. Optional aufzeichnen (ohne Kamerabilder), in `tmux`:
   ```
   mkdir -p ~/bags && ros2 bag record -o ~/bags/feldtest_$(date +%H%M) /scan /high/scan /rollator/hazard/front \
     /rollator/hazard/high /rollator/hazard/side /rollator/hazard/high_side /rollator/object/fused /rollator/object/detections
   ```
6. Ende: `~/stop_all.sh` (stoppt auch die LiDAR-Motoren), dann `sudo poweroff`, LED aus abwarten.

Die Knoten laufen unabhängig von der SSH-Sitzung weiter; reißt das WLAN ab, warnt der Rollator trotzdem.

---

## 7. Stolpersteine und Lösungen

| Problem | Ursache | Lösung |
|---|---|---|
| `python3 -m venv` scheitert | `python3.10-venv` fehlt (sudo) | `venv --without-pip` + `get-pip.py` |
| torch lädt nicht (`libcupti.so.12`, `libcudss.so.0`) | torch 2.11 zu neu für JetPack 6.2; CUPTI nicht installiert | torch 2.8.0 / torchvision 0.23.0, `cuda-cupti-12-6` |
| Lautsprecher nur 8 kHz mono | NVIDIA schaltet A2DP in `bluetoothd` ab | Override in `/etc/systemd/system/bluetooth.service.d/` (Abschnitt 4) |
| Nach Kaltstart kein A2DP | PulseAudio startete erst nach dem Verbinden | `bt_speaker.sh`: PulseAudio zuerst; `systemctl --user enable pulseaudio`; linger |
| LiDAR-Motor dreht nach dem Beenden weiter | `sllidar_node` stürzt in Humble bei SIGTERM ab und stoppt den Motor nicht | mit `stop_all.sh` (SIGINT) beenden; Notfall: Befehle STOP `A5 25` und Motor 0 `A5 A8 02 00 00 0F` an den by-id-Port (1 000 000 Baud) |
| Uhr falsch nach Stromverlust, TLS/git push scheitern | kein RTC-Puffer, NTP im Hochschulnetz gesperrt | Zeit aus HTTP-Date-Header oder per PowerShell setzen; dauerhaft: RTC-Batterie oder NTP-Server der Hochschule |
| `date -Iseconds` unter Windows | PowerShell kennt das nicht (`Get-Date`) | Befehl in Abschnitt 6 |
| `argparse` hält `-56..-46` für eine Option | führendes Minus | `--ignore="$IGNORE_SECTORS"` (mit `=`) |
| Skript mit `set -u` bricht in `plattform.sh` ab | `plattform.sh` erwartet leere Variablen | kein `set -u` in Aufrufern |
| Tür wird mit 320 px nicht erkannt | zu kleine Auflösung | `DETECT_IMGSZ=640` (kaum langsamer) |
| Türabstand falsch (Seitenwand) | Fusion nahm die nächste Gruppe in der Box | Mitte der Box für `WIDE_CLASSES` |
| Dauer-STOP oben nach Hochstellen der Griffe | loser Bremszug direkt neben dem LiDAR | `corridor_min_x_m`; Bremszug mit Kabelbinder befestigen |
| `pip install git+https://…` von Claude Code abgelehnt | Sicherheitsfilter des Auto-Modus | Lautaro führt solche Befehle selbst aus |

---

## 8. Offene Punkte (Priorität von oben)

1. **Dauer-STOP in der letzten Sitzung** (Log mit falscher Uhr, ~30 min): beide Ebenen fast nur STOP (unten
   1564 von 1800 Logzeilen), dadurch kein Türhinweis, obwohl die Tür 526-mal erkannt wurde (398 × Mitte,
   128 × Rahmen = offene Tür, 470 davon im Ansagebereich).
   - unten: Punkt bei Sensor −98°, x = 0,05 m (genau an der Grenze `corridor_min_x_m`), ~0,3 m links neben dem LiDAR;
   - oben: Punkt bei Sensor −128°, 0,42 m, ~0,22 m links der Mitte, 0,28 m vor dem LiDAR;
   - gleichzeitig „Seite beide" (~0,63 m): vermutlich schmaler Gang oder Türrahmen, oder neue Eigenechos nach dem
     Umbau der Griffe (Bremszug, Griffhebel). Nächster Schritt: Fotos/Ort klären, `scan_profile.py` am selben Ort,
     ggf. `corridor_min_x_m` auf 0,10 m und Bremszüge fixieren. Frage: Soll ein Türrahmen beim Durchfahren
     (beidseitig ~0,4 m) STOP auslösen?
2. **Lautsprecher-Abbrüche – gelöst am 17.11.2026.** Die Abbrüche kamen immer ca. 2–3 min nach dem Verbinden, jedes
   Mal beim Wiederaufnehmen des ruhenden Streams (`avdtp Start: Connection timed out`, danach „Host is down"); Abstand
   spielte keine Rolle. Ursache: PulseAudio (`module-suspend-on-idle`) legt die Ausgabe in Pausen still, der Lautsprecher
   geht dann in einen Sparmodus und antwortet beim nächsten Hinweis nicht mehr. Abhilfe: Modul nicht laden (dauernder
   Stream mit Stille), Vorlage `scripts/pulse_default.pa`; `bt_speaker.sh` entlädt es zusätzlich zur Laufzeit. Test:
   6,5 min ohne Abbruch, danach Ansage einwandfrei. Außerdem trennt `bt_speaker.sh` jetzt eine Verbindung ohne
   A2DP-Karte (Lautsprecher hatte sich nach dem Booten vor PulseAudio verbunden) und verbindet neu.
   Nebenbefund: Hotspot auf 5 GHz geht nicht – der Treiber `rtl8822ce` kennt nur Ländercode `00`, alle 5-GHz-Kanäle
   `NO_IR`; `options rtl8822ce rtw_country_code=DE` (in `/etc/modprobe.d/rtl8822ce-country.conf`) wird ignoriert und
   kann gelöscht werden. Als Client in einem fremden 5-GHz-Netz wäre die Jetson möglich.
3. **Türhinweis** im Gebäude bestätigen (geschlossene Tür: „Tür vorne, zwei Meter" einmal gehört); offene Tür hängt
   an Punkt 1.
4. **Test im Freien** mit Aufzeichnung: Bordsteine/Rampen (Bodentreffer des unteren LiDAR → falsche WARN),
   Wände/Autos neben dem Weg (Seitenhinweis zu häufig?), Rechtzeitigkeit beim Gehen, YOLO bei Tageslicht.
5. **Offene Türen mit dem LiDAR** erkennen (Lücke ~0,8–1 m in einer Wand) – gewünscht.
6. **Bewegte Hindernisse**: heute nur Momentabstand (WARN < 1,5 m). Person kommt mit 1,4 m/s entgegen, Rollator
   1 m/s → nur ~0,6 s Vorwarnung. Nötig: Verfolgung über die Zeit, Geschwindigkeit, Warnung nach Zeit bis zum
   Zusammenstoß („Person kommt entgegen").
7. Fusion schreibt ~20 Logzeilen/s → drosseln.
8. `IGNORE_SECTORS` unten erweitern (`-36..-12`, `2..10`; nur Logkosmetik, liegen hinter dem LiDAR).
9. `HIGH_LIDAR_Z`/`HIGH_CAM_HEIGHT` stimmig machen; 6°-Neigung oben in tf/Doku.
10. TensorRT-Engine exportieren und vergleichen; RTC-Batterie oder NTP-Server; Lizenz klären (ultralytics/YOLO
    unter AGPL-3.0).

**Verloren gegangen**: die Hilfsskripte aus dem temporären Arbeitsordner von Claude Code (`medir_front.sh`,
WARN/STOP-Prüfung `ws_test.sh`/`ws_run.py`/`ws_eval.py`) wurden beim Neustart gelöscht. Die Messung lässt sich mit
`scan_profile.py` und `scan_diff.py /scan /high/scan --delay 15` wiederholen; die WARN/STOP-Prüfung an Marken
(0,6 / 0,7 / 0,9 / 1,2 / 1,4 / 1,6 / 2,0 m) müsste bei Bedarf neu ins Repository.
