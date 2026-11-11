#!/bin/bash
source "$(dirname "$(readlink -f "$0")")/plattform.sh"   # Pi/Jazzy oder Jetson/Humble
source_ros
v4l2-ctl -d /dev/video0 -c focus_automatic_continuous=0 -c focus_absolute=0 -c exposure_dynamic_framerate=0
exec ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video0 -p image_width:=640 -p image_height:=360 \
  -p framerate:=30.0 -p pixel_format:=yuyv2rgb \
  -p camera_name:=c920 -p frame_id:=camera_link
