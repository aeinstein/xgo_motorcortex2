#!/bin/bash
# Emotionen fürs Display: der Augengenerator im Display-Daemon (st7789_server) zeichnet sie selbst
# (docs/display-emotions.md). Hierher kommt nur der Name; an $FIFO.augen geht eine Zeile "ausdruck NAME"
# (davor ggf. "farbe FARBE 0.2"). Die Namen, die der Daemon kennt, stehen in $FIFO.augen.namen.
#
#   emotion.sh play NAME [FARBE]     # z. B. play sleep, play like rot (ohne FARBE: die aktuelle bleibt)
#
# Wartet, bis der Ausdruck vorbei ist (höchstens 10 s), damit etwa die Schlafaugen vor dem Ausschalten fertig sind.
# Der zuletzt gestartete Spieler gewinnt: der Daemon trägt für die Dauer eines Ausdrucks seine PID in $FIFO.owner
# ein (legt rc.local als root an, Gruppe video, 664); leer = keine Emotion aktiv (xgonav pausiert so lange seine
# LCD-Ausgabe). Ohne Daemon mit Generator: Rückgabe 1, keine Bilder (vorgerenderte gibt es seit 2026-11-28 nicht mehr).
set -euo pipefail

FIFO=${LCD_FIFO:-/tmp/lcd0}
OWNER=$FIFO.owner
AUGEN=$FIFO.augen
export LC_ALL=C     # Dezimalpunkt für read -t (deutsche Locale am Rider)

play() {
    local name=$1 farbe=${2:-} zeilen i cur pid=
    [[ $name =~ ^[A-Za-z]+$ ]] || { echo "Name ungültig: $name" >&2; return 2; }
    [[ -z $farbe || $farbe =~ ^[a-z]{1,16}$ ]] || { echo "Farbe ungültig: $farbe" >&2; return 2; }
    for i in $(seq 50); do [ -p "$AUGEN" ] && break; sleep 0.1; done     # früh beim Booten: Daemon abwarten
    [ -p "$AUGEN" ] || { echo "$AUGEN fehlt (Display-Daemon mit Augengenerator läuft nicht?)" >&2; return 1; }
    grep -qx -- "$name" "$AUGEN.namen" 2>/dev/null || { echo "Ausdruck unbekannt: $name" >&2; return 1; }
    zeilen="ausdruck $name"
    [ -n "$farbe" ] && zeilen="farbe $farbe 0.2
$zeilen"
    # Öffnen zum Schreiben wartet auf einen Leser: läuft der Daemon nicht, nach 0,5 s aufgeben
    timeout 0.5 sh -c 'printf "%s\n" "$1" > "$2"' _ "$zeilen" "$AUGEN" 2>/dev/null \
        || { echo "Display-Daemon liest $AUGEN nicht" >&2; return 1; }
    exec 4<> <(:)               # read -t auf leerer Pipe = sleep ohne fork
    for i in $(seq 15); do      # bis 0,3 s: der Daemon trägt sich ein
        read -r cur < "$OWNER" 2>/dev/null || cur=
        [ -n "$cur" ] && { pid=$cur; break; }
        read -r -t 0.02 -u 4 || true
    done
    [ -n "$pid" ] || return 0
    for i in $(seq 250); do     # bis 10 s: Ausdruck vorbei (leer) oder ein neuerer Spieler
        read -r cur < "$OWNER" 2>/dev/null || cur=
        [ "$cur" = "$pid" ] || return 0
        read -r -t 0.04 -u 4 || true
    done
    return 0
}

case "${1:-}" in
    play) [ $# -ge 2 ] || { echo "Aufruf: $0 play NAME [FARBE]" >&2; exit 2; }; play "$2" "${3:-}" ;;
    *)    echo "Aufruf: $0 play NAME [FARBE]" >&2; exit 2 ;;
esac
