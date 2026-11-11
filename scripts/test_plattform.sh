#!/bin/bash
# Test fuer plattform.sh mit nachgebauten Verzeichnissen (kein ROS, keine Hardware noetig).
#   bash scripts/test_plattform.sh
set -e
HIER="$(dirname "$(readlink -f "$0")")"
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
FEHLER=0

check() {   # check NAME ERWARTET ERHALTEN
  if [ "$2" = "$3" ]; then echo "  ok    $1"; else echo "  FEHLER $1: erwartet '$2', erhalten '$3'"; FEHLER=1; fi
}

zone() {    # zone NR TYP [TEMP]  (ohne TEMP: temp nicht lesbar, wie cv0-thermal auf der Jetson)
  mkdir -p "$1/thermal_zone$2"; echo "$3" > "$1/thermal_zone$2/type"
  if [ -n "$4" ]; then echo "$4" > "$1/thermal_zone$2/temp"; fi
}

lade() { unset ROS_SETUP ROS_DISTRO; source "$HIER/plattform.sh"; }

echo "== ROS-Version"
mkdir -p "$TMP/ros/humble" "$TMP/ros/jazzy"
lade; ROS_ROOT="$TMP/ros"
check "keine Installation" "fehlt" "$(ros_setup_path || echo fehlt)"
touch "$TMP/ros/jazzy/setup.bash"
check "nur jazzy (Pi)" "$TMP/ros/jazzy/setup.bash" "$(ros_setup_path)"
rm "$TMP/ros/jazzy/setup.bash"; touch "$TMP/ros/humble/setup.bash"
check "nur humble (Jetson)" "$TMP/ros/humble/setup.bash" "$(ros_setup_path)"
touch "$TMP/ros/jazzy/setup.bash"
check "beide, ohne ROS_DISTRO -> humble" "$TMP/ros/humble/setup.bash" "$(ros_setup_path)"
check "beide, ROS_DISTRO=jazzy" "$TMP/ros/jazzy/setup.bash" "$(ROS_DISTRO=jazzy ros_setup_path)"
check "ROS_DISTRO nicht installiert -> humble" "$TMP/ros/humble/setup.bash" "$(ROS_DISTRO=rolling ros_setup_path)"
check "ROS_SETUP hat Vorrang" "$TMP/ros/jazzy/setup.bash" "$(ROS_SETUP=$TMP/ros/jazzy/setup.bash ros_setup_path)"
check "ROS_SETUP fehlt -> Fehler" "fehlt" "$(ROS_SETUP=$TMP/gibtsnicht ros_setup_path || echo fehlt)"

echo "== Temperatur"
J="$TMP/jetson"   # Aufbau wie auf der Jetson Orin Nano (11.11.2026)
zone "$J" 0 cpu-thermal 47968; zone "$J" 1 gpu-thermal 48406; zone "$J" 2 cv0-thermal
zone "$J" 5 soc0-thermal 47968; zone "$J" 8 tj-thermal 52343
lade; THERMAL_ROOT="$J"
check "Jetson -> tj-thermal" "$J/thermal_zone8/temp" "$(thermal_file)"
check "Jetson Grad C" "52" "$(temp_c)"

P="$TMP/pi"       # Pi 5: nur cpu-thermal in Zone 0
zone "$P" 0 cpu-thermal 77150
THERMAL_ROOT="$P"
check "Pi -> cpu-thermal" "$P/thermal_zone0/temp" "$(thermal_file)"
check "Pi Grad C" "77" "$(temp_c)"

T="$TMP/tj_unlesbar"   # tj vorhanden, aber temp nicht lesbar -> cpu-thermal
zone "$T" 3 tj-thermal; zone "$T" 4 cpu-thermal 61000
THERMAL_ROOT="$T"
check "tj unlesbar -> cpu-thermal" "61" "$(temp_c)"

U="$TMP/unbekannt"     # unbekannte Typen -> thermal_zone0
zone "$U" 0 acpitz 45000
THERMAL_ROOT="$U"
check "unbekannter Typ -> Zone 0" "45" "$(temp_c)"

THERMAL_ROOT="$TMP/leer"
check "keine Zone -> ?" "?" "$(temp_c)"

if [ "$FEHLER" = 0 ]; then echo "Alle Tests bestanden"; else echo "Tests FEHLGESCHLAGEN"; exit 1; fi
