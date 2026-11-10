#!/bin/bash
# ESP32-Firmware für den RiderPi: Referenz-Firmware holen, Patches anwenden, bauen.
# Flasht NICHT. Ergebnis: firmware/out/app0.bin (+ .sha256) für Partition app0 (0x10000).
#
#   firmware/build.sh [env]          # env: rider2-firmware (Default, klassischer PID-Regler)
#                                    #      rider2-trained | rider2-rl-only
#   REF_COMMIT=<sha> firmware/build.sh   # andere Referenz-Version (Patches müssen passen)
#   PIO=~/.platformio/penv/Scripts/platformio.exe firmware/build.sh   # vorhandenes PlatformIO
#
# Läuft unter Linux/macOS und unter Windows in Git-Bash (Python 3.8+ als python3 oder python).
#
# Referenz: github.com/LuwuDynamics/xgorider2_rl (R-1.1.6-Reimplementierung, PlatformIO).
# Patches (firmware/patches/, in Reihenfolge):
#   0001 Register 0x0c (Rad-Encoder) und 0x0d (Lage + Räder + Zeit in einem Snapshot)
#   0002 Aktion 0x3e = 254: binäres Disarm/Not-Stopp wie !stop
#   0003 LED-Helligkeit halbiert (129 -> 64 von 256)
#   0004 Aktion 0x3e = 253 / !standup: Aufstehen aus der Liegelage auf Befehl
#   0005 Kippraten ohne Gier-Anteil (Gierrate x tan(Roll/Nick)): sonst rollt der Rider beim Drehen weg
#   0006 Beschleunigungswinkel bei > 25 °/s Gierrate einfrieren (Fliehkraft verfälscht die Lage)
#   0007 Diagnose-Register 0x0e (32 Byte Reglergrößen, am Pi über raw_reg lesbar)
#   0008 Positionshalten auch beim Drehen auf der Stelle (Radmitte), Aktion 241 an / 240 aus
#   0009 Gyro-z-Nullpunkt laufend nachführen (Räder/Stillstand), Gier-Totband 2 -> 0,3 °/s, Register 0x0f
#   0010 Positionshalten beim Drehen per Default an
#   0011 Gier-Totband entfernt (Integration erst nach Startkalibrierung)
#   0012 Roll-Trimm aus dem Nivellieren behalten (über Höhenänderungen)
#   0013 Diagnose-Register 0x10 (Lagefusion Nick/Roll: Beschleunigungswinkel, Abzug 0005, Einfrieren 0006)
#   0014 IMU-/Rad-Datenstrom an den Pi (Register 0x11 = Rate in Hz, Pakete Typ 0x14)
#   0015 Integral des Geschwindigkeitsfehlers beim Fahren (Aktion 239 an / 238 aus), sonst bleibt er an Steigungen stehen
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

# PlatformIO: PIO=<pfad> nimmt eine vorhandene Installation, sonst eigenes venv (nicht das
# Repo-venv). venv-Layout: Linux/macOS bin/, Windows (Git-Bash) Scripts/*.exe.
venv_exe() {   # venv_exe <name>: Pfad des Programms im venv, leer wenn nicht vorhanden
    local c
    for c in "$VENV/bin/$1" "$VENV/Scripts/$1.exe"; do
        [ -x "$c" ] && { echo "$c"; return 0; }
    done
    return 0   # nichts gefunden: leere Ausgabe (set -e darf hier nicht abbrechen)
}
if [ -n "${PIO:-}" ]; then
    [ -x "$PIO" ] || { echo "PIO nicht ausführbar: $PIO" >&2; exit 2; }
else
    VENV=$WORK/venv
    PIO=$(venv_exe platformio)
    if [ -z "$PIO" ]; then
        # python3 bzw. python (Windows); der Store-Platzhalter python3.exe läuft nicht -> testen
        PY=${PYTHON:-}
        if [ -z "$PY" ]; then
            for c in python3 python; do
                if command -v "$c" >/dev/null && "$c" -c 'import sys; assert sys.version_info >= (3, 8)' 2>/dev/null; then
                    PY=$c; break
                fi
            done
        fi
        [ -n "$PY" ] || { echo "Kein Python >= 3.8 gefunden (PYTHON=<pfad> setzen)" >&2; exit 2; }
        echo "== PlatformIO installieren ($PY)"
        "$PY" -m venv "$VENV"
        "$(venv_exe python)" -m pip install -q "platformio==6.1.*"
        PIO=$(venv_exe platformio)
        [ -n "$PIO" ] || { echo "platformio fehlt nach Installation in $VENV" >&2; exit 1; }
    fi
fi

echo "== Bauen: $ENV ($("$PIO" --version))"
(cd "$SRC/firmware" && "$PIO" run -e "$ENV")

BIN=$SRC/firmware/.pio/build/$ENV/firmware.bin
cp "$BIN" "$OUT/app0.bin"
(cd "$OUT" && sha256sum app0.bin > app0.bin.sha256)
{
    echo "ref $(git -C "$SRC" rev-parse HEAD)"
    echo "env $ENV"
    for p in "$HERE"/patches/*.patch; do echo "patch $(basename "$p") $(sha256sum "$p" | cut -d' ' -f1)"; done
    echo "pio $("$PIO" --version)"
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
