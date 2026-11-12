#!/bin/bash
# Erzeugt die Ansagen fuer hazard_speaker mit Piper (einmalig; danach offline nutzbar).
set -e
OUT=~/rollator_sounds
MODEL=~/piper-voices/de_DE-thorsten-medium.onnx
PIPER=~/piper-venv/bin/piper
mkdir -p "$OUT"

say() {   # say DATEI "TEXT"
  if ! echo "$2" | "$PIPER" --model "$MODEL" --output_file "$OUT/$1" 2>/dev/null; then
    echo "$2" | "$PIPER" -m "$MODEL" -f "$OUT/$1"
  fi
  echo "  $1  <-  $2"
}

say warn.wav "Achtung, Hindernis vorne."
[ -f "$OUT/stop.wav" ] || say stop.wav "Stopp!"
say warn_oben.wav "Achtung, Hindernis oben."
say seite_links.wav "Vorsicht, links."
say seite_rechts.wav "Vorsicht, rechts."
say seite_beide.wav "Vorsicht, links und rechts."
# Hinweise (Orientierung), einmal je Objekt: info_<klasse>_<meter>.wav
for m in "2|zwei" "3|drei" "4|vier"; do
  say "info_door_${m%%|*}.wav" "Tür vorne, ${m##*|} Meter."
done

while IFS='|' read -r cls de; do
  [ -z "$cls" ] && continue
  say "warn_${cls// /_}.wav" "Achtung, $de vorne."
done << 'LISTE'
person|Person
chair|Stuhl
couch|Sofa
dining table|Tisch
bench|Bank
bicycle|Fahrrad
dog|Hund
cat|Katze
backpack|Rucksack
bottle|Flasche
suitcase|Koffer
potted plant|Pflanze
car|Auto
bus|Bus
truck|Lastwagen
motorcycle|Motorrad
umbrella|Regenschirm
traffic light|Ampel
fire hydrant|Hydrant
stop sign|Stoppschild
door|Tür
LISTE
echo "Fertig: $(ls "$OUT"/*.wav | wc -l) Ansagen in $OUT"
