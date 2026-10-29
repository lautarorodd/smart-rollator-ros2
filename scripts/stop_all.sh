#!/bin/bash
# Stoppt alle Knoten beider Ebenen (untere und obere).
# Erst sanft (SIGINT, damit der LiDAR den Motor ausschaltet), dann SIGTERM, zuletzt SIGKILL.
PATTERNS="sensors_launch.py|sensors_high_launch.py|front_obstacle|hazard_speaker|camera_detector.py|fusion_node.py|sllidar_node|usb_cam_node_exe|static_transform_publisher"

# Prozesse, die zu den Mustern passen, ohne dieses Skript und seinen Aufrufer
targets() { pgrep -f "$PATTERNS" | grep -v -x -e "$$" -e "$PPID"; }
alive()   { [ -n "$(targets)" ]; }
send()    { local p; p=$(targets); [ -n "$p" ] && kill -"$1" $p 2>/dev/null; return 0; }

if ! alive; then
  echo "Nichts zu stoppen."
  exit 0
fi

send INT
for i in 1 2 3 4 5 6 7 8; do
  sleep 1
  alive || break
done

if alive; then
  echo "Sanftes Beenden hat nicht gereicht, sende SIGTERM ..."
  send TERM
  for i in 1 2 3; do sleep 1; alive || break; done
fi
if alive; then
  echo "Sende SIGKILL ..."
  send KILL
  sleep 1
fi

if alive; then
  echo "WARNUNG: Es laufen noch Prozesse:"
  for p in $(targets); do ps -o pid=,args= -p "$p" | cut -c1-120; done
  exit 1
fi
echo "Alles gestoppt (untere und obere Ebene)."
