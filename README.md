# Smart Rollator – ROS 2 Prototyp

Hinderniserkennung im Frontsektor mit Sprachausgabe, lokal auf dem Raspberry Pi 5.

## Umgebung

- Ubuntu 24.04 LTS, ROS 2 Jazzy
- RPLIDAR S2L mit dem Treiber `sllidar_ros2` (https://github.com/Slamtec/sllidar_ros2), in `~/ros2_ws/src` gebaut
- Piper (Python-Paket `piper-tts` in `~/piper-venv`) mit der Stimme `de_DE-thorsten-medium`
- Die Ansagen liegen als WAV-Dateien in `~/rollator_sounds` (`stop.wav`, `warn.wav`)

## Knoten

- `front_obstacle`: liest `/scan`, bewertet den Frontsektor und veröffentlicht `SAFE`, `WARN` oder `STOP` auf `rollator/hazard/front`
- `hazard_speaker`: spielt bei Verschärfung des Zustands die passende Ansage ab

## Start

Treiber:

    ros2 launch sllidar_ros2 sllidar_s2_launch.py serial_port:=/dev/ttyUSB0

Hinderniswarner (Winkel des Frontsektors am Sensor einsetzen):

    ros2 run smart_rollator_perception front_obstacle --ros-args -p front_angle_deg:=<Winkel>

Sprachausgabe:

    ros2 run smart_rollator_perception hazard_speaker

## Stand

Die Schwellenwerte (Warnung 1,5 m, Stopp 0,8 m) sind Startwerte und noch nicht aus Messungen abgeleitet.
