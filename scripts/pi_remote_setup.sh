#!/bin/bash
# Richtet SSH und das eigene WLAN "RollatorPi" auf dem Pi ein.
# Mit Monitor und Tastatur am Pi ausfuehren, NICHT ueber eine WLAN-SSH-Sitzung.
set -e

echo "== 1. SSH =="
sudo apt install -y openssh-server
sudo systemctl enable --now ssh
echo "SSH-Dienst: $(systemctl is-active ssh)"
echo "Benutzername fuer die Anmeldung: $(whoami)"

echo
echo "== 2. WLAN-Adapter =="
nmcli device status
if ! nmcli -t -f DEVICE,TYPE device | grep -q '^wlan0:wifi'; then
    echo "FEHLER: kein verwaltetes WLAN-Geraet wlan0 gefunden. Abbruch."
    exit 1
fi

echo
echo "ACHTUNG: Das eigene WLAN ersetzt die WLAN-Verbindung des Pi (Kabel bleibt unberuehrt)."
read -r -p "Fortfahren? (j/n) " ANS
[ "$ANS" = "j" ] || { echo "Abgebrochen."; exit 0; }

read -r -s -p "Passwort fuer das WLAN RollatorPi (mind. 8 Zeichen): " PW
echo
if [ "${#PW}" -lt 8 ]; then
    echo "Passwort zu kurz. Abbruch."
    exit 1
fi

sudo nmcli connection delete RollatorHotspot 2>/dev/null || true
sudo nmcli device wifi hotspot ifname wlan0 con-name RollatorHotspot ssid RollatorPi password "$PW"
sudo nmcli connection modify RollatorHotspot connection.autoconnect yes connection.autoconnect-priority 100

echo
echo "== 3. Ergebnis =="
ip -4 addr show wlan0 | grep inet || echo "Keine IP auf wlan0 gefunden"
echo "Verbinden: WLAN RollatorPi, dann SSH an die oben gezeigte Adresse (normalerweise 10.42.0.1)."
echo "Rueckgaengig: sudo nmcli connection down RollatorHotspot && sudo nmcli connection modify RollatorHotspot connection.autoconnect no"

