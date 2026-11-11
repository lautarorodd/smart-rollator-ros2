#!/bin/bash
# Gemeinsame Hilfsfunktionen fuer Pi 5 (ROS 2 Jazzy) und Jetson Orin Nano (ROS 2 Humble).
# Wird von den Start- und Statusskripten mit "source" geladen, nicht direkt ausgefuehrt.
# ROS_ROOT und THERMAL_ROOT lassen sich fuer Tests umlenken (test_plattform.sh).

ROS_ROOT=${ROS_ROOT:-/opt/ros}
THERMAL_ROOT=${THERMAL_ROOT:-/sys/class/thermal}

# Pfad zu setup.bash der installierten ROS-2-Version.
# Vorrang: ROS_SETUP (manuell gesetzt) > ROS_DISTRO (falls installiert) > humble > jazzy.
# Auf jedem Rechner ist nur eine Version installiert, die Reihenfolge humble/jazzy ist daher egal.
ros_setup_path() {
  if [ -n "$ROS_SETUP" ]; then
    [ -f "$ROS_SETUP" ] && { echo "$ROS_SETUP"; return 0; }
    return 1
  fi
  local d
  for d in "$ROS_DISTRO" humble jazzy; do
    [ -n "$d" ] && [ -f "$ROS_ROOT/$d/setup.bash" ] && { echo "$ROS_ROOT/$d/setup.bash"; return 0; }
  done
  return 1
}

# ROS 2 und den Arbeitsbereich laden; bricht mit Meldung ab, wenn ROS fehlt.
source_ros() {
  local setup
  if ! setup=$(ros_setup_path); then
    echo "FEHLER: keine ROS-2-Installation gefunden (gesucht: ROS_SETUP, $ROS_ROOT/{\$ROS_DISTRO,humble,jazzy}/setup.bash)"
    exit 1
  fi
  source "$setup"
  [ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash
  return 0
}

# Datei mit der massgeblichen Temperatur. Zonen werden ueber ihren Typ gesucht, weil die Nummern
# je nach Geraet verschieden sind: Jetson "tj-thermal" (Sperrschicht, danach richtet sich die Drosselung),
# Pi und Jetson "cpu-thermal", sonst thermal_zone0.
thermal_file() {
  local want z
  for want in tj-thermal cpu-thermal; do
    for z in "$THERMAL_ROOT"/thermal_zone*; do
      [ "$(cat "$z/type" 2>/dev/null)" = "$want" ] && [ -r "$z/temp" ] && { echo "$z/temp"; return 0; }
    done
  done
  [ -r "$THERMAL_ROOT/thermal_zone0/temp" ] && { echo "$THERMAL_ROOT/thermal_zone0/temp"; return 0; }
  return 1
}

# Temperatur in ganzen Grad C, oder "?" wenn keine lesbare Zone existiert.
temp_c() {
  local f t
  f=$(thermal_file) && t=$(cat "$f" 2>/dev/null) && [ -n "$t" ] && { echo $(( t / 1000 )); return 0; }
  echo "?"
}
