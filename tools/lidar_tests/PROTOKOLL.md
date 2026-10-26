# Testprotokoll Frontwarner (ROS 2) – Messkampagne 1

Ziel: Die Erkennung im Frontsektor mit **echten Messungen** bewerten, Schwellenwerte aus Daten ableiten und die Schwächen des Vorgängerprojekts (Gruppe A, SoSe 2025) vermeiden.

## Lehren aus Gruppe A (aus dem Repository belegt)

| Befund bei Gruppe A | Folge für diese Kampagne |
| --- | --- |
| Der Scan wurde auf 360 Werte (1° je Wert) verkleinert; Mindestbreite ≈ 8 cm, Objekt bei 3,0 m wurde im ersten Lauf nicht erkannt (0 von 20), erst nach einer Korrektur 20 von 20 | Volle Winkelauflösung des Treibers verwenden; Objekte bei 3 m und schmale Objekte gezielt testen |
| Schwellen 1,5 m / 3,5 m stehen fest im Code; die geplante datenbasierte Herleitung (≥ 95 % Recall) ist im Repository nicht umgesetzt, die Analyse nutzte emulierte Bags | Rohdaten aufzeichnen und Schwellen offline aus Messungen ableiten |
| „Erwartetes Level“ im Test hing von den Schwellen ab (Test prüfte die eigene Einstellung) | Nur die physikalische Wahrheit speichern (Abstand, seitlicher Versatz); Erwartung offline berechnen |
| 20 Nachrichten pro Aufbau (ca. 2 s) sind kein unabhängiges Ergebnis | Jeden Aufbau mehrfach **neu** aufbauen (3 Wiederholungen) |
| Keine Messungen nahe an den Schwellen, keine Gänge, keine Fehlalarmrate | Grenzbereich ± 10 cm, Annäherungsgänge, Gang durch freien Korridor |
| Vorderkante-Versatz von 4 cm wurde nachträglich „korrigiert“ | Messen, wo genau die Referenz liegt (Vorderkante des Objekts) und im Protokoll festhalten |
| FRONT_CENTER_DEG an zwei Stellen unterschiedlich (355 / 284) | Gehrichtung einmal bestimmen, am Sensor markieren, in jeder Datei speichern |
| Glas: 0 % Erkennung | Glas erneut messen und als bekannte Grenze dokumentieren |

## Vorbereitung (10 Minuten)

1. Pi starten, Treiber starten: `ros2 launch sllidar_ros2 sllidar_s2_launch.py serial_port:=/dev/ttyUSB0`
2. **Auflösung prüfen** und notieren: `ros2 topic echo /scan --once --field angle_increment` und `ros2 topic echo /scan --once --field ranges | wc -w`
3. **Gehrichtung bestimmen:** Karton in 1 m vor den Rollator stellen, `front_obstacle` starten, im Log den Sensorwinkel des nächsten Punktes ablesen. Mit Klebeband am Sensor markieren. Wert = `FRONT`.
4. Boden markieren (Klebeband) bei 0,5 / 0,7 / 0,8 / 0,9 / 1,0 / 1,3 / 1,4 / 1,5 / 1,6 / 1,7 / 2,0 / 2,5 / 3,0 / 4,0 m **ab Sensormitte** und quer bei ±0,3 / ±0,45 / ±0,6 / ±0,8 m. Mit Maßband prüfen.
5. Objekte bereitlegen: Karton (ca. 30 × 30 cm), Flasche (≈ 8 cm), Stuhlbein, eine Person, Wand, Glasscheibe.
6. Ordner für Daten: `~/lidar_trials` (wird angelegt).

## Durchführung

**A. Standardplan (ca. 45–60 Minuten):**

    cd ~/lidar_tests
    python3 run_plan.py plans/standard.csv --front-angle FRONT

Jede Zeile des Plans wird dreimal gemessen, jedes Mal Objekt **neu aufstellen**. Abstand immer von der Sensormitte zur Vorderkante des Objekts.

**B. Gänge (je 60 s), mit `--kind walk`:**

    python3 record_trial.py --kind walk --label gang_frei --scans 600 --front-angle FRONT
    python3 record_trial.py --kind walk --label annaeherung_karton --object Karton --scans 300 --front-angle FRONT

- `gang_frei`: Rollator durch einen freien Flur schieben (Fehlalarme pro Minute).
- `annaeherung_karton`: Auf ein ruhendes Objekt in ca. 4 m zugehen, bis ca. 0,3 m davor (zeigt, bei welchem Abstand WARN und STOP wirklich auslösen).
- Optional: Person kreuzt den Weg; Person geht mit.

**C. Notizen:** Bei jedem Versuch Besonderheiten in `--notes` oder auf Papier festhalten (Licht, Bodenbelag, Bewegung).

## Auswertung

    python3 evaluate.py ~/lidar_trials --warn 1.5 --stop 0.8 --csv ergebnis.csv

Der Bericht enthält: Einzelversuche, Zusammenfassung mit 95-%-Konfidenzintervall, Messfehler je Objekt, Gänge (Fehlalarme pro Minute, Auslöseabstände) und einen Variantenvergleich (Korridor oder Winkelsektor, Breite, Mindestpunktzahl) auf denselben Rohdaten.

## Erfolgskriterien

1. **Verpasst = 0** bei allen gewerteten Versuchen im Korridor; Aussage immer mit Intervall („0 von n“ belegt nur, dass die Rate unter ca. 3/n liegt).
2. Schmales Objekt (≈ 8 cm) und Stuhlbein werden bei 3,0 m bzw. 1,5 m erkannt – oder die Grenze wird mit Zahl dokumentiert.
3. Fehlalarme im freien Gang: Anzahl pro Minute festhalten (Gruppe A hat dies nicht gemessen).
4. Auslöseabstände aus dem Annäherungsgang liegen nahe an den eingestellten Schwellen.
5. Glas: Ergebnis dokumentieren (Erwartung: nicht erkannt) und als Bedarf für einen zweiten Sensor (Ultraschall) festhalten.

## Offen für die nächste Kampagne

- Gesamtlatenz Hindernis → Ansage messen.
- Schwellen aus Marschgeschwindigkeit, Reaktionszeit und Latenz herleiten.
- Messungen mit Menschen aus der Zielgruppe (BSVT) und im Freien.
