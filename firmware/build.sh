#!/bin/bash
# ESP32-Firmware für den RiderPi: Referenz-Firmware holen, Patches anwenden, bauen.
# Flasht NICHT. Ergebnis: firmware/out/app0.bin (+ .sha256) für Partition app0 (0x10000).
#
#   firmware/build.sh [env]          # env: rider2-firmware (Default, klassischer PID-Regler)
#                                    #      rider2-trained | rider2-rl-only
#   REF_COMMIT=<sha> firmware/build.sh   # andere Referenz-Version (Patches müssen passen)
#
# Referenz: github.com/LuwuDynamics/xgorider2_rl (R-1.1.6-Reimplementierung, PlatformIO).
# Patches (firmware/patches/, in Reihenfolge):
#   0001 Register 0x0c (Rad-Encoder) und 0x0d (Lage + Räder + Zeit in einem Snapshot)
#   0002 Aktion 0x3e = 254: binäres Disarm/Not-Stopp wie !stop
#   0003 LED-Helligkeit halbiert (129 -> 64 von 256)
# Braucht: git, python3 (venv), Internet beim ersten Lauf (PlatformIO lädt Toolchain ~1 GB).
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
ENV=${1:-rider2-firmware}
REF_URL=${REF_URL:-https://github.com/LuwuDynamics/xgorider2_rl.git}
REF_COMMIT=${REF_COMMIT:-f534986}          # getestet mit diesem Stand
WORK=$HERE/build
SRC=$WORK/xgorider2_rl
OUT=$HERE/out

case "$ENV" in
    rider2-firmware|rider2-trained|rider2-rl-only) ;;
    *) echo "Unbekannte Umgebung: $ENV" >&2; exit 2 ;;
esac

mkdir -p "$WORK" "$OUT"

# Referenz holen bzw. auf den gewünschten Stand zurücksetzen (lokale Änderungen weg)
if [ ! -d "$SRC/.git" ]; then
    echo "== Referenz klonen: $REF_URL"
    git clone -q "$REF_URL" "$SRC"
fi
git -C "$SRC" fetch -q origin
git -C "$SRC" checkout -q -f "$REF_COMMIT"
git -C "$SRC" clean -q -fdx -e .pio -e .venv
echo "== Referenz: $(git -C "$SRC" log -1 --format='%h %ad %s' --date=short)"

# Patches anwenden
for p in "$HERE"/patches/*.patch; do
    echo "== Patch $(basename "$p")"
    git -C "$SRC" apply --whitespace=nowarn "$p"
done

# PlatformIO in eigenem venv (nicht im Repo-venv)
VENV=$WORK/venv
if [ ! -x "$VENV/bin/platformio" ]; then
    echo "== PlatformIO installieren"
    python3 -m venv "$VENV"
    "$VENV/bin/pip" install -q "platformio==6.1.*"
fi

echo "== Bauen: $ENV"
(cd "$SRC/firmware" && "$VENV/bin/platformio" run -e "$ENV")

BIN=$SRC/firmware/.pio/build/$ENV/firmware.bin
cp "$BIN" "$OUT/app0.bin"
(cd "$OUT" && sha256sum app0.bin > app0.bin.sha256)
{
    echo "ref $(git -C "$SRC" rev-parse HEAD)"
    echo "env $ENV"
    for p in "$HERE"/patches/*.patch; do echo "patch $(basename "$p") $(sha256sum "$p" | cut -d' ' -f1)"; done
    echo "built $(date -Iseconds)"
} > "$OUT/app0.build.txt"

echo
echo "Fertig: $OUT/app0.bin ($(stat -c %s "$OUT/app0.bin") Byte)"
cat "$OUT/app0.bin.sha256"
echo
echo "Flashen (nur app0, vorher Voll-Backup; Port prüfen, z. B. COM3/COM4 bzw. /dev/ttyUSB0):"
echo "  python -m esptool --port <PORT> --baud 115200 write-flash 0x10000 app0.bin"
echo "Nie 0x310000-0x32ffff (Beinnullpunkte, BLE-Name, Trimm) oder spiffs überschreiben."
echo "Nach dem Flashen ist der Rider disarmed: Aktion 255 (Aufstehen) zum Balancieren."
