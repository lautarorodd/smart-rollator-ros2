#!/bin/bash
# Verbindet den Bluetooth-Lautsprecher (BT_SPEAKER in ~/rollator.env, MAC-Adresse) und macht ihn mit A2DP zur
# Standardausgabe von PulseAudio, damit hazard_speaker (aplay) ueber ihn spielt. Wird von start_all.sh aufgerufen.
# Voraussetzung: einmal gekoppelt und vertraut (bluetoothctl pair/trust), A2DP nicht abgeschaltet (README, Jetson).
# Rueckgabe 0 = Lautsprecher bereit, 1 = nicht bereit (start_all.sh startet trotzdem, warnt aber).
MAC=${1:-$BT_SPEAKER}
[ -n "$MAC" ] || { echo "BT_SPEAKER nicht gesetzt"; exit 1; }
ID=${MAC//:/_}

connected() { bluetoothctl info "$MAC" 2>/dev/null | grep -q "Connected: yes"; }

# Zuerst PulseAudio: erst wenn es laeuft, meldet es bei bluetoothd die A2DP-Endpunkte an. Verbindet man vorher,
# kommt hoechstens das Freisprechprofil zustande (beobachtet nach Kaltstart der Jetson, 12.11.2026).
for i in $(seq 1 10); do
  pactl info > /dev/null 2>&1 && break   # startet PulseAudio bei Bedarf (Socket-Aktivierung)
  sleep 1
done
pactl info > /dev/null 2>&1 || { echo "WARNUNG: PulseAudio laeuft nicht (loginctl enable-linger gesetzt?)"; exit 1; }
sleep 2   # Endpunkte anmelden lassen

for i in 1 2 3; do
  connected && break
  bluetoothctl connect "$MAC" > /dev/null 2>&1
  sleep 3
done
if ! connected; then
  echo "WARNUNG: Lautsprecher $MAC nicht verbunden (eingeschaltet? an einem anderen Geraet gekoppelt?)"
  exit 1
fi

# PulseAudio braucht nach dem Verbinden einen Moment, bis die Karte erscheint
for i in $(seq 1 10); do
  pactl list short cards 2>/dev/null | grep -q "bluez_card.$ID" && break
  sleep 1
done
pactl set-card-profile "bluez_card.$ID" a2dp_sink 2>/dev/null
SINK="bluez_sink.$ID.a2dp_sink"
if ! pactl set-default-sink "$SINK" 2>/dev/null; then
  echo "WARNUNG: kein A2DP-Ausgang fuer $MAC (nur Freisprechprofil?). Siehe README, Abschnitt Jetson, Bluetooth."
  exit 1
fi
echo "Lautsprecher bereit: $SINK"
