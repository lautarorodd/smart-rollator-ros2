#!/bin/bash
source "$(dirname "$(readlink -f "$0")")/plattform.sh"
echo "$(date +%T)   Temp: $(temp_c) C"
echo "--- Frontwarner:"
tail -n 3 ~/logs/front.log 2>/dev/null | sed 's/^.*front_obstacle\]: //' | cut -c1-100
echo "--- Fusion:"
tail -n 4 ~/logs/fusion.log 2>/dev/null | sed 's/^.*fusion_node\]: //' | cut -c1-150
echo "--- Detektor:"
tail -n 1 ~/logs/detector.log 2>/dev/null | sed 's/^.*camera_detector\]: //' | cut -c1-120
