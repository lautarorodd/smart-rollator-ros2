#!/bin/bash
# Startet die OBERE Ebene im Hintergrund (Sensoren, Frontwarner, optional Erkennung und Fusion).
# Voraussetzung: ~/start_all.sh laeuft schon (untere Ebene, Sprachausgabe).
source "$(dirname "$(readlink -f "$0")")/plattform.sh"   # Pi/Jazzy oder Jetson/Humble
source_ros
source ~/rollator.env
export PYTHONUNBUFFERED=1
mkdir -p ~/logs

HIGH_CAM_ENABLE=${HIGH_CAM_ENABLE:-1}
if ! [[ "$HIGH_FRONT" =~ ^-?[0-9.]+$ ]] || [[ "$HIGH_LIDAR_PORT" == *HIER_EINTRAGEN* ]]; then
  echo "FEHLER: In ~/rollator.env fehlen HIGH_LIDAR_PORT oder HIGH_FRONT."
  exit 1
fi
if [ "$HIGH_CAM_ENABLE" = "1" ] && [[ "$HIGH_CAM_DEVICE" == *HIER_EINTRAGEN* ]]; then
  echo "FEHLER: HIGH_CAM_DEVICE fehlt in ~/rollator.env (oder HIGH_CAM_ENABLE=0 setzen, um ohne obere Kamera zu starten)."
  exit 1
fi
if [ "$HIGH_CAM_ENABLE" = "1" ]; then CAM_FLAG=true; else CAM_FLAG=false; HIGH_CAM_DEVICE=/dev/video0; fi
if pgrep -f sensors_high_launch.py > /dev/null; then
  echo "Obere Ebene laeuft schon. Erst ~/stop_all.sh."
  exit 1
fi

start() {
  local name=$1; shift
  setsid nohup "$@" > ~/logs/$name.log 2>&1 &
  echo "gestartet: $name"
}

start sensors_high ros2 launch ~/camera_lidar/sensors_high_launch.py \
  lidar_port:=$HIGH_LIDAR_PORT cam_device:=$HIGH_CAM_DEVICE cam_enable:=$CAM_FLAG lidar_front_deg:=$HIGH_FRONT \
  lidar_x:=${HIGH_LIDAR_X:-0.0} lidar_y:=${HIGH_LIDAR_Y:-0.0} lidar_z:=$HIGH_LIDAR_Z cam_x:=$HIGH_CAM_X cam_y:=$HIGH_CAM_Y cam_z:=$HIGH_CAM_Z cam_pitch_deg:=$HIGH_CAM_PITCH
sleep 8
# Eigenechos der Montage oben (optional, HIGH_IGNORE_SECTORS in ~/rollator.env)
IGN_ARGS=()
[ -n "$HIGH_IGNORE_SECTORS" ] && IGN_ARGS=(-p "ignore_sectors:=$HIGH_IGNORE_SECTORS")
# Gang und Seitenstreifen; der obere LiDAR sitzt HIGH_LIDAR_Y seitlich versetzt, der Gang bleibt auf der Rollatormitte
[ -n "$CORRIDOR_HALF_WIDTH" ] && IGN_ARGS+=(-p corridor_half_width_m:=$CORRIDOR_HALF_WIDTH \
  -p side_width_m:=${SIDE_WIDTH:-0.35} -p side_length_m:=${SIDE_LENGTH:-0.8} -p lidar_y_m:=${HIGH_LIDAR_Y:-0.0})
start front_high ros2 run smart_rollator_perception front_obstacle --ros-args \
  -r __node:=front_obstacle_high -r scan:=/high/scan -r rollator/hazard/front:=/rollator/hazard/high \
  -r rollator/hazard/side:=/rollator/hazard/high_side \
  -p front_angle_deg:=$HIGH_FRONT -p warn_dist_m:=$WARN_DIST -p stop_dist_m:=$STOP_DIST "${IGN_ARGS[@]}"

if [ "$HIGH_DETECTOR" = "1" ] && [ "$HIGH_CAM_ENABLE" = "1" ]; then
  start detector_high ~/yolo-venv/bin/python ~/camera_lidar/camera_detector.py --no-image --conf ${DETECT_CONF:-0.35} --imgsz ${DETECT_IMGSZ:-320} \
    --model "${DETECT_MODEL:-yolo11n.pt}" \
    --classes "${DETECT_CLASSES:-person,chair,couch,dining table,bench,bicycle,dog,backpack,bottle}" --ros-args \
    -r __node:=camera_detector_high -r image_raw:=/high/image_raw \
    -r rollator/object/detections:=/high/detections
  sleep 10
  start fusion_high python3 ~/camera_lidar/fusion_node.py --front-angle $HIGH_FRONT \
    --cam-x $HIGH_CAM_X --cam-y $HIGH_CAM_Y --cam-height $HIGH_CAM_HEIGHT --cam-pitch $HIGH_CAM_PITCH \
    --ros-args -r __node:=fusion_node_high -r scan:=/high/scan \
    -r rollator/object/detections:=/high/detections -r rollator/object/fused:=/rollator/object/fused_high
fi
echo "Fertig. Pruefen mit ~/status.sh"
