# Gesamttest (beide Ebenen, Kamera, Sprachausgabe)

Gefuehrter Test des laufenden Systems mit Protokoll. Prueft je Schritt:

- Zustand der unteren Ebene (`/rollator/hazard/front`) und der oberen Ebene (`/rollator/hazard/high`): SAFE / WARN / STOP
- Kamera + Fusion (`/rollator/object/fused`): welche Klasse wie oft erkannt, Abstand und Winkel
- Ansage von `hazard_speaker` (aus `/rosout`, z. B. `spiele WARN (person)`) und ob sie wirklich zu hoeren war (Frage an die Testperson)

## Ablauf

    ~/start_all.sh
    ~/start_high.sh
    cd ~/ros2_ws/src/smart_rollator_perception/tools/gesamttest
    python3 gesamttest.py                 # ganzer Plan, ca. 20 Minuten
    python3 gesamttest.py --only 3 4 12   # einzelne Schritte wiederholen
    ~/stop_all.sh

Vorher Boden mit Klebeband markieren: 0,4 / 0,5 / 1,0 / 1,1 / 2,5 / 3,0 m **ab Mitte unterer LiDAR**, quer 0,3 m rechts
und 0,6 m links bei 1,1 m. Abstand immer bis zur Vorderkante des Objekts. Material: Person, Stuhl, Karton (mind. 40 cm hoch),
Flasche, ein Brett oder Karton fuer das haengende Hindernis (Hoehe der oberen Ebene vorher mit dem Zollstock pruefen),
ein freier Flur (mind. 1,4 m breit).

Ergebnis: `~/gesamttest_ergebnisse/<Datum_Zeit>/protokoll.md` (Tabelle, nach jedem Schritt gespeichert) und `rohdaten.jsonl`.
Das Protokoll enthaelt keine Bilder und darf ins Repository (z. B. nach `tools/gesamttest/protokolle/`).

## Bewertung

Der Plan (`plans/gesamttest.csv`) speichert nur den Aufbau: Objekt, Lage `x_m`/`y_m` ab dem unteren LiDAR und ob das Objekt
die untere/obere Ebene erreicht (`ja`/`nein`/`?`). Die Erwartung wird aus `~/rollator.env` berechnet (`WARN_DIST`, `STOP_DIST`,
`HIGH_LIDAR_X/Y`, Frontsektor +-20 Grad). Der obere LiDAR sieht ein Objekt 22 cm weiter entfernt als der untere.

- An den Schwellen (+-10 cm, +-3 Grad am Sektorrand) gelten beide Nachbarzustaende als richtig.
- **OK**: mindestens 80 % der Zustaende im Messfenster richtig, Ansage wie erwartet.
- **TEILWEISE**: Warnung richtig, aber ohne Objektnamen, oder die Kamera erkennt die erwartete Klasse in weniger als 50 % der Meldungen.
- **FEHLER**: falscher Zustand, falsche/fehlende Ansage, keine Daten, Ansage geloggt aber nicht gehoert, Fehlalarm bei freier Fahrt (> 2 %).
- **OFFEN**: keine Erwartung moeglich (`?` in der unteren Ebene, obere Ebene nicht gestartet).

Objektnamen sagt `hazard_speaker` nur fuer Klassen am Boden (`FLOOR_CLASSES` in `fusion_core.py`), bis 1,8 m und 25 Grad.
Eine Flasche wird deshalb nie genannt, auch wenn die Kamera sie erkennt.

Logik ohne ROS: `gesamttest_core.py`, Test: `python3 test_gesamttest_core.py`.
