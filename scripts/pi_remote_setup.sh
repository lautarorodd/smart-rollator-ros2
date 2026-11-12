#!/bin/bash
# Richtet SSH, das eigene WLAN (Hotspot) und den Betrieb ohne Anmeldung ein. Pi und Jetson.
#   Pi:     bash pi_remote_setup.sh                 (WLAN "RollatorPi")
#   Jetson: bash pi_remote_setup.sh RollatorJetson
# Mit Monitor oder ueber das Netzkabel ausfuehren, NICHT ueber eine WLAN-SSH-Sitzung (das WLAN wird umgestellt).
set -e
SSID=${1:-RollatorPi}
CON=RollatorHotspot

echo "== 1. SSH =="
if ! systemctl is-active --quiet ssh; then
    sudo apt install -y openssh-server
    sudo systemctl enable --now ssh
fi
echo "SSH-Dienst: $(systemctl is-active ssh)"
echo "Benutzername fuer die Anmeldung: $(whoami)"

echo
echo "== 2. WLAN-Adapter =="
nmcli device status
# Erstes verwaltetes WLAN-Geraet (Pi: wlan0, Jetson Orin Nano: wlP1p1s0)
WDEV=$(nmcli -t -f DEVICE,TYPE device | awk -F: '$2=="wifi"{print $1; exit}')
if [ -z "$WDEV" ]; then
    echo "FEHLER: kein verwaltetes WLAN-Geraet gefunden. Abbruch."
    exit 1
fi
echo "WLAN-Geraet: $WDEV, Name des eigenen WLAN: $SSID"

echo
echo "ACHTUNG: Das eigene WLAN ersetzt die WLAN-Verbindung dieses Rechners (Netzkabel bleibt unberuehrt)."
read -r -p "Fortfahren? (j/n) " ANS
[ "$ANS" = "j" ] || { echo "Abgebrochen."; exit 0; }

read -r -s -p "Passwort fuer das WLAN $SSID (mind. 8 Zeichen): " PW
echo
if [ "${#PW}" -lt 8 ]; then
    echo "Passwort zu kurz. Abbruch."
    exit 1
fi

sudo nmcli connection delete "$CON" 2>/dev/null || true
# 2,4 GHz, Kanal 6: auf der Jetson (Laendercode 00) sind nur die Kanaele 1-11 fuer einen Zugangspunkt frei
sudo nmcli device wifi hotspot ifname "$WDEV" con-name "$CON" ssid "$SSID" band bg channel 6 password "$PW"
sudo nmcli connection modify "$CON" connection.autoconnect yes connection.autoconnect-priority 100

echo
echo "== 3. Betrieb ohne Anmeldung (linger) =="
# Ohne angemeldeten Benutzer laeuft kein PulseAudio, und beim Trennen der SSH-Sitzung beendet systemd es.
# Mit linger startet der Benutzerdienst (PulseAudio, Bluetooth-Lautsprecher) beim Booten und bleibt aktiv.
sudo loginctl enable-linger "$(whoami)"
loginctl show-user "$(whoami)" -p Linger

echo
echo "== 4. Ergebnis =="
ip -4 addr show "$WDEV" | grep inet || echo "Keine IP auf $WDEV gefunden"
echo "Verbinden: WLAN $SSID, dann SSH an die oben gezeigte Adresse (normalerweise 10.42.0.1)."
echo "Rueckgaengig: sudo nmcli connection down $CON && sudo nmcli connection modify $CON connection.autoconnect no"
echo "              sudo loginctl disable-linger $(whoami)"
