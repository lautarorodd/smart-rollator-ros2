#!/bin/bash
# LiDAR unten + Kamera + statische Transformationen. Werte kommen aus ~/rollator.env.
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/rollator.env

LIDAR_PORT=${LIDAR_PORT:-/dev/ttyUSB0}
CAM_LINK=${CAM_BY_ID:-/dev/video0}

# usb_cam loest by-id-Links nicht auf -> in das echte /dev/videoN umwandeln
CAM_DEV=$(readlink -f "$CAM_LINK")

[ -e "$LIDAR_PORT" ] || { echo "FEHLER: LiDAR unten nicht gefunden: $LIDAR_PORT"; exit 1; }
[ -e "$CAM_DEV" ]    || { echo "FEHLER: Kamera nicht gefunden: $CAM_LINK"; exit 1; }

exec ros2 launch ~/camera_lidar/sensors_launch.py lidar_port:=$LIDAR_PORT cam_device:=$CAM_DEV lidar_front_deg:=$FRONT cam_x:=$CAM_X cam_y:=$CAM_Y cam_z:=$CAM_Z cam_pitch_deg:=$CAM_PITCH
