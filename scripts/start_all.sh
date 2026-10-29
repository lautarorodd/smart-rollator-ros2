#!/bin/bash
# Startet alle Knoten im Hintergrund; Ausgaben in ~/logs. Ueberlebt das Trennen der SSH-Sitzung.
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
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
start front ros2 run smart_rollator_perception front_obstacle --ros-args -p front_angle_deg:=$FRONT -p warn_dist_m:=$WARN_DIST -p stop_dist_m:=$STOP_DIST
start speaker ros2 run smart_rollator_perception hazard_speaker
start detector ~/yolo-venv/bin/python ~/camera_lidar/camera_detector.py --no-image --conf 0.35 --classes person chair couch "dining table" bench bicycle dog backpack bottle
sleep 10
start fusion python3 ~/camera_lidar/fusion_node.py --front-angle $FRONT --cam-x $CAM_X --cam-y $CAM_Y --cam-height $CAM_HEIGHT --cam-pitch $CAM_PITCH
echo "Fertig. Pruefen mit ~/status.sh ; live ansehen mit: watch -n 1 ~/live.sh"
