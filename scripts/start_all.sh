#!/bin/bash
# Startet alle Knoten im Hintergrund; Ausgaben in ~/logs. Ueberlebt das Trennen der SSH-Sitzung.
source "$(dirname "$(readlink -f "$0")")/plattform.sh"   # Pi/Jazzy oder Jetson/Humble
source_ros
source ~/rollator.env
export PYTHONUNBUFFERED=1
mkdir -p ~/logs

if ! [[ "$FRONT" =~ ^-?[0-9.]+$ ]]; then
  echo "FEHLER: FRONT in ~/rollator.env ist keine Zahl (aktuell: '$FRONT')"
  exit 1
fi
if pgrep -f sensors_launch.py > /dev/null; then
  echo "Laeuft schon. Erst ~/stop_all.sh ausfuehren."
  exit 1
fi

start() {   # start NAME BEFEHL...
  local name=$1; shift
  setsid nohup "$@" > ~/logs/$name.log 2>&1 &
  echo "gestartet: $name"
}

start sensors ~/start_sensors.sh
sleep 8
if ! pgrep -f sensors_launch.py > /dev/null; then
  echo "FEHLER: Sensoren (LiDAR unten + Kamera) laufen nicht. Siehe ~/logs/sensors.log"
  exit 1
fi
# Eigenechos der Montage (optional, IGNORE_SECTORS in ~/rollator.env, z. B. "10..35,-80..-72")
IGN_ARGS=()
[ -n "$IGNORE_SECTORS" ] && IGN_ARGS=(-p "ignore_sectors:=$IGNORE_SECTORS")
# Gang und Seitenstreifen (optional, CORRIDOR_HALF_WIDTH in ~/rollator.env; unterer LiDAR mittig: lidar_y_m 0)
[ -n "$CORRIDOR_HALF_WIDTH" ] && IGN_ARGS+=(-p corridor_half_width_m:=$CORRIDOR_HALF_WIDTH \
  -p side_width_m:=${SIDE_WIDTH:-0.35} -p side_length_m:=${SIDE_LENGTH:-0.8} -p lidar_y_m:=${LIDAR_Y:-0.0})
start front ros2 run smart_rollator_perception front_obstacle --ros-args -p front_angle_deg:=$FRONT -p warn_dist_m:=$WARN_DIST -p stop_dist_m:=$STOP_DIST "${IGN_ARGS[@]}"
# Bluetooth-Lautsprecher (optional, BT_SPEAKER in ~/rollator.env): verbinden und als A2DP-Standardausgabe setzen
if [ -n "$BT_SPEAKER" ]; then
  "$(dirname "$(readlink -f "$0")")/bt_speaker.sh" "$BT_SPEAKER" || echo "WARNUNG: ohne Lautsprecher weiter (siehe oben)"
fi
start speaker ros2 run smart_rollator_perception hazard_speaker
# Modell und Klassen aus ~/rollator.env (DETECT_MODEL, DETECT_CLASSES), sonst die bisherige Liste
start detector ~/yolo-venv/bin/python ~/camera_lidar/camera_detector.py --no-image --conf 0.35 \
  --model "${DETECT_MODEL:-yolo11n.pt}" \
  --classes "${DETECT_CLASSES:-person,chair,couch,dining table,bench,bicycle,dog,backpack,bottle}"
sleep 10
start fusion python3 ~/camera_lidar/fusion_node.py --front-angle $FRONT --cam-x $CAM_X --cam-y $CAM_Y --cam-height $CAM_HEIGHT --cam-pitch $CAM_PITCH
echo "Fertig. Pruefen mit ~/status.sh ; live ansehen mit: watch -n 1 ~/live.sh"
echo "Obere Ebene (zweiter LiDAR) danach mit: ~/start_high.sh"
