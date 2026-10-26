#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
exec ros2 launch ~/camera_lidar/sensors_launch.py lidar_front_deg:=-178.0 cam_x:=-0.19 cam_y:=0.0 cam_z:=0.18
