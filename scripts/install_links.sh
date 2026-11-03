#!/bin/bash
# Verknuepft die Dateien im Home-Verzeichnis mit diesem Repository, damit es nur EINE Quelle gibt.
# ~/start_all.sh usw., ~/rollator.env und ~/camera_lidar werden zu Links auf scripts/ bzw. tools/camera_lidar.
# Vorhandene echte Dateien oder Ordner werden nach ~/archiv/<Zeit>/home/ verschoben, nie geloescht.
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
case "$REPO" in
  "$HOME"/ros2_ws/src/*) ;;
  *) echo "FEHLER: nur aus dem Klon unter ~/ros2_ws/src/ ausfuehren (aktuell: $REPO)"; exit 1;;
esac
ARCH="$HOME/archiv/$(date +%Y-%m-%d_%H%M%S)/home"

link() {   # link LINKPFAD ZIEL
  local dst=$1 src=$2
  [ -e "$src" ] || { echo "FEHLER: $src fehlt"; exit 1; }
  if [ -e "$dst" ] && [ ! -L "$dst" ]; then
    mkdir -p "$ARCH"; mv "$dst" "$ARCH/"; echo "archiviert: $dst -> $ARCH/"
  fi
  ln -sfn "$src" "$dst"
  echo "Link: $dst -> $src"
}

for f in start_all.sh start_sensors.sh start_high.sh stop_all.sh status.sh live.sh make_sounds.sh rollator.env; do
  link "$HOME/$f" "$REPO/scripts/$f"
done
link "$HOME/camera_lidar" "$REPO/tools/camera_lidar"
chmod +x "$REPO"/scripts/*.sh
echo "Fertig. Werte aendern in ~/rollator.env (= $REPO/scripts/rollator.env) und mit git committen."
