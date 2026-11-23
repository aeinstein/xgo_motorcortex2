#!/bin/bash
# Neue Fahrten vom Rider holen: nach kimmy (fahrt/mit_imu + Link) und nach $LOKAL kopieren, Dateizahl und meta-md5
# prüfen, am Rider löschen, für ComfyUI einreihen (hinter den laufenden Lauf). Ausgabe: je Fahrt eine Zeile.
#   bash fahrt_holen.sh [lokales_verzeichnis]
# Nur Git Bash am PC. Löscht am Rider nur, was auf kimmy UND lokal geprüft angekommen ist.
set -u
KEY=/c/Users/aeinstein.ITNOX/Documents/aeinstein-ed.pem
SSH="ssh -o IdentitiesOnly=yes -i $KEY -o ConnectTimeout=5"
KIMMY=aeinstein@kimmy.intern.itnox.de
LOKAL=${1:-${TEMP:-/tmp}/xgo_fahrten}
mkdir -p "$LOKAL"

RIDER=
for h in 192.168.53.110 10.10.0.110; do
    if $SSH root@$h true 2>/dev/null; then RIDER=root@$h; break; fi
done
[ -n "$RIDER" ] || { echo "FEHLER: Rider unter keiner Adresse erreichbar"; exit 1; }
echo "Rider: $RIDER"

# Läuft gerade eine Aufnahme (GET /status: aufzeichnung.laeuft)? Dann deren Ordner nicht anfassen.
# (nicht per pgrep: rawrec ist ein Faden in xgonav, und pgrep -f fände seine eigene Befehlszeile)
REC=$($SSH $RIDER 'curl -s -m 3 "http://127.0.0.1:8088/status?full=1" | python3 -c "import sys,json,os
try: a = json.load(sys.stdin).get(\"aufzeichnung\") or {}
except Exception: a = {}
print(os.path.basename(a[\"ordner\"]) if a.get(\"laeuft\") and a.get(\"ordner\") else \"\")"')
FAHRTEN=$($SSH $RIDER 'cd /home/aeinstein/fahrt 2>/dev/null && ls -1d 2026*/ 2>/dev/null | tr -d /')
if [ -n "$REC" ]; then
    echo "Aufnahme läuft: $REC bleibt"
    FAHRTEN=$(echo "$FAHRTEN" | grep -vx "$REC")
fi
[ -n "$FAHRTEN" ] || { echo "keine neuen Fahrten"; }

OK=()
for d in $FAHRTEN; do
    r=$($SSH $RIDER "cd /home/aeinstein/fahrt && echo \$(ls $d | wc -l) \$(md5sum $d/meta.jsonl | cut -c1-12) \$(du -sh $d | cut -f1)")
    $SSH $RIDER "tar -C /home/aeinstein/fahrt -cf - $d" | tee >(tar -xf - -C "$LOKAL") | \
        $SSH $KIMMY "tar -C ~/floornet/data/rider/fahrt/mit_imu -xf - && cd ~/floornet/data/rider/fahrt && ln -sfn mit_imu/$d $d"
    sleep 1
    k=$($SSH $KIMMY "cd ~/floornet/data/rider/fahrt/mit_imu && echo \$(ls $d | wc -l) \$(md5sum $d/meta.jsonl | cut -c1-12)")
    l="$(ls "$LOKAL/$d" | wc -l) $(md5sum "$LOKAL/$d/meta.jsonl" | cut -c1-12)"
    l=$(echo $l)
    if [ "$k" = "$(echo $r | cut -d' ' -f1-2)" ] && [ "$l" = "$k" ]; then
        $SSH $RIDER "rm -rf /home/aeinstein/fahrt/$d"
        echo "OK $d: $r -> kimmy + lokal gleich, am Rider gelöscht"
        OK+=("$d")
    else
        echo "FEHLER $d: Rider '$r' kimmy '$k' lokal '$l' -> bleibt am Rider"
    fi
done

# Sonstiges im Fahrtenverzeichnis (karte*.npz u. ä.): nach kimmy fahrt/ohne_imu, md5 prüfen, dann löschen
SONST=$($SSH $RIDER 'cd /home/aeinstein/fahrt 2>/dev/null && ls -1p | grep -v / | grep -v "^2026"')
for f in $SONST; do
    a=$($SSH $RIDER "md5sum /home/aeinstein/fahrt/$f | cut -c1-32")
    $SSH $RIDER "cat /home/aeinstein/fahrt/$f" | $SSH $KIMMY "cat > ~/floornet/data/rider/fahrt/ohne_imu/$f"
    b=$($SSH $KIMMY "md5sum ~/floornet/data/rider/fahrt/ohne_imu/$f | cut -c1-32")
    if [ "$a" = "$b" ]; then $SSH $RIDER "rm -f /home/aeinstein/fahrt/$f"; echo "OK $f -> kimmy ohne_imu, gelöscht"
    else echo "FEHLER $f: md5 weicht ab, bleibt"; fi
done

[ ${#OK[@]} -gt 0 ] || exit 0
# Messfahrten (Drift, Kalibrierung) nicht für ComfyUI einreihen: NICHT_EINREIHEN=1 bash fahrt_holen.sh …
[ -z "${NICHT_EINREIHEN:-}" ] || { echo "nicht eingereiht (NICHT_EINREIHEN): ${OK[*]}"; exit 0; }

# Einreihen: Bildliste (jedes 2. Bild, ohne Standbilder), neuer laufNN.sh hinter dem laufenden Lauf
LISTE=$(IFS=,; echo "${OK[*]}")
$SSH $KIMMY bash -s -- "$LISTE" <<'EOF'
set -u
F=$1
cd ~/floornet/stage1_p11
N=$(ls lauf*.sh | sed -n 's/^lauf\([0-9]*\)\.sh$/\1/p' | sort -n | tail -1); N=$((N + 1))
IDS=ids_lauf$N.txt
cd ~/floornet/repo/tools/floornet
~/floornet/.venv/bin/python ids_fahrt.py ${F//,/ } > ~/floornet/comfy/$IDS 2> ~/floornet/comfy/${IDS%.txt}.err
cat ~/floornet/comfy/${IDS%.txt}.err
cd ~/floornet/stage1_p11
SICHTEN=""
for f in ${F//,/ }; do SICHTEN="$SICHTEN~/floornet/sweep/sichten.sh $f; "; done
cat > lauf$N.sh <<SH
#!/bin/bash
# Fahrten $F, Standard-Prompt T1 (fahrt-Skill). Wartet auf den laufenden Lauf (PID als Argument, 0 = keiner),
# rechnet die Fahrten und stellt sie zum Sichten in den Netz-Betrachter (p11_sichten). Stage2/3 alle 30 min und am Ende.
P=~/floornet/.venv/bin/python
stufen() {
    cd ~/floornet/repo/tools/floornet
    \$P stage2.py ~/floornet/stage1_p11 ~/floornet/stage2_p11 --gruen 30 --jobs 2
    \$P stage3.py ~/floornet/stage2_p11 ~/floornet/stage3_p11
}
while [ "\$1" != "0" ] && [ -d /proc/\$1 ]; do sleep 20; done
echo "Start Anhang \$(date +%H:%M)"
cd ~/floornet/comfy
\$P comfy_batch.py --prompt prompt_standard.txt --ids $IDS --edits ~/floornet/stage1_p11 --nur_edits --eingang auto \\
    --out ~/floornet/stage1_p11 > ~/floornet/stage1_p11/batch$N.log 2>&1 &
B=\$!
while [ -d /proc/\$B ]; do
    for i in \$(seq 180); do [ -d /proc/\$B ] && sleep 10; done
    stufen
done
stufen
$SICHTEN
echo "Ende Anhang \$(date +%H:%M)"
SH
chmod +x lauf$N.sh
# auf den zuletzt gestarteten, noch laufenden laufNN.sh warten (nicht auf diese Shell selbst)
W=$(ps -eo pid,args | awk '$2=="/bin/bash" && $3 ~ /^\.\/lauf[0-9]+\.sh$/ {print $1}' | tail -1)
nohup ./lauf$N.sh ${W:-0} > lauf$N.out 2>&1 < /dev/null & disown
echo "eingereiht: lauf$N.sh wartet auf ${W:-keinen} ($IDS)"
EOF
