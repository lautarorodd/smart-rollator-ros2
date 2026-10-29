#!/bin/bash
# LiDAR + Kamera + statische Transformationen. Werte kommen aus ~/rollator.env.
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/rollator.env
exec ros2 launch ~/camera_lidar/sensors_launch.py lidar_port:=${LIDAR_PORT:-/dev/ttyUSB0} lidar_front_deg:=$FRONT cam_x:=$CAM_X cam_y:=$CAM_Y cam_z:=$CAM_Z cam_pitch_deg:=$CAM_PITCH
