#!/bin/bash
# Emotionen fürs Display (st7789-Daemon, FIFO /tmp/lcd0, RGB565 320x240).
#
# Bisher hat ffmpeg bei jedem Abspielen alle PNGs dekodiert und nach RGB565
# umgerechnet. Jetzt wird das einmalig erledigt ("build"), beim Abspielen
# werden nur noch fertige Rohbilder (je 153600 Byte) ins FIFO kopiert.
#
#   emotion.sh build QUELLE ZIEL     # QUELLE/<Name>/<Name><n>.png -> ZIEL/<Name>/0001.raw ...
#   emotion.sh play  VERZ [FPS]      # spielt VERZ/*.raw einmal ab (Standard 25 fps)
#
# 25 fps entspricht der Dauer des alten Aufrufs (image2 liest mit 25 fps ein;
# dessen "-r 10" hat nur Bilder verworfen, z. B. 24 von 55 bei Angry).
#
# Ohne .raw-Dateien in VERZ (nicht konvertiert) spielt "play" die PNGs wie
# früher direkt mit ffmpeg ab.
#
# Der zuletzt gestartete Player gewinnt: er trägt seine PID in $FIFO.owner ein,
# ältere Player sehen das vor dem nächsten Bild und beenden sich. Das geht auch
# zwischen Benutzern (www-data, aeinstein, root), die sich nicht per Signal
# beenden dürfen. Die Datei legt rc.local als root an (Gruppe video, 664) –
# wegen fs.protected_regular darf sie nicht einem der Benutzer gehören.
# Leer = keine Emotion aktiv (xgonav pausiert so lange seine LCD-Ausgabe).
set -euo pipefail

FIFO=${LCD_FIFO:-/tmp/lcd0}
OWNER=$FIFO.owner
export LC_ALL=C     # Dezimalpunkt für awk/read -t (deutsche Locale am Rider)

build() {
    local src=$1 dst=$2 d name tmp n
    mkdir -p "$dst"
    for d in "$src"/*/; do
        name=$(basename "$d")
        [[ $name =~ ^[A-Za-z]+$ ]] || continue
        if [ "$dst/$name" -nt "$d" ]; then echo "$name: aktuell"; continue; fi
        # Numerisch sortiert (sort -V), nicht lexikalisch wie der alte Glob.
        mapfile -t pngs < <(find "$d" -maxdepth 1 -name '*.png' -printf '%f\n' | sort -V)
        [ ${#pngs[@]} -gt 0 ] || continue
        tmp=$(mktemp -d "$dst/.$name.XXXX")
        n=0
        for f in "${pngs[@]}"; do
            n=$((n + 1))
            ffmpeg -loglevel error -nostdin -y -i "$d/$f" -s 320x240 \
                -f rawvideo -pix_fmt rgb565 "$tmp/$(printf '%04d' "$n").raw"
        done
        chmod 755 "$tmp"
        rm -rf "$dst/$name"
        mv "$tmp" "$dst/$name"
        echo "$name: $n Bilder"
    done
}

play() {
    local dir=$1 fps=${2:-25} pause f
    shopt -s nullglob
    local frames=("$dir"/*.raw)
    if [ ${#frames[@]} -eq 0 ]; then
        # Fallback: nicht vorkonvertiert -> wie früher mit ffmpeg.
        mapfile -t pngs < <(find "$dir" -maxdepth 1 -name '*.png' | sort -V)
        exec ffmpeg -loglevel error -nostdin -re -y -f image2pipe -framerate "$fps" \
            -i <(cat "${pngs[@]}") -s 320x240 -f rawvideo -pix_fmt rgb565 "$FIFO"
    fi
    pause=$(awk "BEGIN { printf \"%.3f\", 1 / $fps }")
    # Ohne Daemon (z. B. früh beim Booten) nicht ins Leere schreiben: ">" würde
    # sonst eine normale Datei anlegen, die der Daemon dann statt des FIFOs liest.
    local i
    for i in $(seq 50); do [ -p "$FIFO" ] && break; sleep 0.1; done
    [ -p "$FIFO" ] || { echo "$FIFO ist kein FIFO (Display-Daemon läuft nicht?)" >&2; exit 1; }
    echo $$ > "$OWNER" 2>/dev/null || true
    # FIFO nur einmal öffnen; der Daemon liest dann Bild für Bild am Stück.
    exec 3>"$FIFO"
    exec 4<> <(:)               # read -t auf leerer Pipe = sleep ohne fork
    local cur
    for f in "${frames[@]}"; do
        if [ -r "$OWNER" ]; then
            read -r cur < "$OWNER" || cur=
            [ "$cur" = "$$" ] || exit 0          # neuerer Player hat übernommen
        fi
        cat "$f" >&3
        read -r -t "$pause" -u 4 || true
    done
    read -r cur < "$OWNER" 2>/dev/null || cur=
    if [ "$cur" = "$$" ]; then : > "$OWNER"; fi
}

case "${1:-}" in
    build) [ $# -eq 3 ] || { echo "Aufruf: $0 build QUELLE ZIEL" >&2; exit 2; }; build "$2" "$3" ;;
    play)  [ $# -ge 2 ] || { echo "Aufruf: $0 play VERZ [FPS]" >&2; exit 2; }; play "$2" "${3:-25}" ;;
    *)     echo "Aufruf: $0 build QUELLE ZIEL | play VERZ [FPS]" >&2; exit 2 ;;
esac
