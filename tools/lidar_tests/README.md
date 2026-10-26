# lidar_tests

Werkzeuge für die Messkampagne zum Frontwarner (siehe PROTOKOLL.md).

- `detector_core.py` – reine Erkennungslogik (Korridor oder Winkelsektor, Hysterese), ohne ROS
- `record_trial.py` – zeichnet rohe Scans mit Metadaten auf (braucht laufenden Treiber)
- `run_plan.py` – führt einen Messplan (`plans/*.csv`) Versuch für Versuch aus
- `evaluate.py` – bewertet alle Aufzeichnungen offline, mit beliebigen Parametern (ohne ROS)
- `make_synthetic.py` – erzeugt Testdaten, um die Werkzeuge ohne Sensor zu prüfen

Trockenlauf ohne Sensor:

    python3 make_synthetic.py /tmp/syn --front-angle 90
    python3 evaluate.py /tmp/syn
