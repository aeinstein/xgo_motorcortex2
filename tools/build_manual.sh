#!/bin/sh
# Anleitung (HTML) aus docs/ bauen -> web/anleitung/ (nur am PC; am Rider liefert nginx sie statisch aus, keine Rechenzeit)
# Braucht mkdocs-material im venv: .venv/bin/pip install "mkdocs-material>=9.5,<10"
set -e
cd "$(dirname "$0")/.."
if [ -x .venv/bin/mkdocs ]; then M=.venv/bin/mkdocs; else M=.venv/Scripts/mkdocs; fi
NO_MKDOCS_2_WARNING=1 "$M" build --strict --clean "$@"
find web/anleitung -name '*.map' -delete   # Quellkarten der Theme-Skripte braucht am Rider niemand
du -sh web/anleitung
