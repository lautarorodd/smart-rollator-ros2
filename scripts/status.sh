#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export PYTHONUNBUFFERED=1
echo "== Prozesse =="
for p in sllidar_node usb_cam_node_exe front_obstacle hazard_speaker camera_detector.py fusion_node.py; do
  n=$(pgrep -fc "$p")
  if [ "$n" -gt 0 ]; then echo "  OK    $p ($n)"; else echo "  FEHLT $p"; fi
done
echo "  (bei zwei Ebenen: sllidar_node, usb_cam_node_exe und front_obstacle je 2)"
echo "== Raten (je 4 s Messung) =="
echo "  /scan:       $(timeout 4 ros2 topic hz /scan 2>/dev/null | grep 'average rate' | tail -1)"
echo "  /image_raw:  $(timeout 4 ros2 topic hz /image_raw 2>/dev/null | grep 'average rate' | tail -1)"
if pgrep -f sensors_high_launch.py > /dev/null; then
  echo "  /high/scan:  $(timeout 4 ros2 topic hz /high/scan 2>/dev/null | grep 'average rate' | tail -1)"
  echo "  /high/image_raw: $(timeout 4 ros2 topic hz /high/image_raw 2>/dev/null | grep 'average rate' | tail -1)"
fi
echo "== Temperatur =="
echo "  $(( $(cat /sys/class/thermal/thermal_zone0/temp) / 1000 )) Grad C"
echo "== Uhrzeit =="
date
