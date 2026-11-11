#!/usr/bin/env python3
"""Webserver für betrachter.html: liefert die Dateien eines Verzeichnisses (wie python -m http.server) und nimmt das
Aussortieren untauglicher Bilder an.

  cd ~/floornet && python repo/tools/floornet/betrachter_server.py [--port 8765] [--bind 0.0.0.0] [--dir .]

POST /aus mit JSON {"ansicht": "hd", "id": "explore10/005514", "aus": true}: hängt die ID an die Handliste
<stage1 der Ansicht>/aus_hand.txt an ("aus": false schreibt "-id", holt das Bild also zurück). Gelöscht wird nichts;
stage2.py, stage3.py und stage3_npz.py lesen die Liste. Ohne Anmeldung, nur fürs Heimnetz.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
import threading
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

STAGE1 = {"hd": "stage1_hd", "g0h": "stage1_g0h", "p8": "stage1", "t1": "stage1_t1", "t2": "stage1_t2", "t3": "stage1_t3"}      # wie ANSICHTEN in betrachter.html
HAND = "aus_hand.txt"                                                   # wie stage2.HAND
ID = re.compile(r"[A-Za-z0-9_-]+/[0-9]+")
_lock = threading.Lock()


def eintragen(root: str, ansicht: str, bild: str, aus: bool) -> str:
    """Zeile an die Handliste hängen. Rückgabe: Fehlertext oder "" bei Erfolg."""
    if ansicht not in STAGE1:
        return "unbekannte Ansicht"
    if not isinstance(bild, str) or not ID.fullmatch(bild):
        return "ungültige ID"
    ziel = os.path.join(root, STAGE1[ansicht])
    if not os.path.exists(os.path.join(ziel, bild + ".png")):
        return "Bild gibt es in dieser Ansicht nicht"
    with _lock, open(os.path.join(ziel, HAND), "a", encoding="utf-8", newline="\n") as f:
        f.write(("" if aus else "-") + bild + "\n")
    return ""


class Handler(SimpleHTTPRequestHandler):
    def do_POST(self) -> None:
        n = int(self.headers.get("Content-Length") or 0)
        # application/json verlangt bei fremden Seiten eine Vorabfrage, die hier niemand beantwortet
        if not 0 < n <= 1000:
            self.close_connection = True
            self.send_error(400, "nur POST /aus mit kurzem JSON")
            return
        roh = self.rfile.read(n)                # erst lesen, sonst reißt die Verbindung beim Ablehnen ab
        if self.path != "/aus" or self.headers.get("Content-Type", "").split(";")[0].strip() != "application/json":
            self.send_error(400, "nur POST /aus mit JSON")
            return
        try:
            d = json.loads(roh)
            fehler = eintragen(self.directory, d.get("ansicht"), d.get("id"), bool(d.get("aus", True)))
        except (ValueError, AttributeError):
            fehler = "kein JSON-Objekt"
        body = json.dumps({"ok": not fehler, "fehler": fehler}, ensure_ascii=False).encode("utf-8")
        self.send_response(400 if fehler else 200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def end_headers(self) -> None:
        if getattr(self, "path", "").split("?")[0].endswith((HAND, "index.js")):        # Listen ändern sich, nie aus dem Zwischenspeicher
            self.send_header("Cache-Control", "no-store")
        super().end_headers()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--dir", default=".")
    a = ap.parse_args()
    srv = ThreadingHTTPServer((a.bind, a.port), partial(Handler, directory=os.path.abspath(a.dir)))
    print("Betrachter auf %s:%d, Verzeichnis %s" % (a.bind, a.port, os.path.abspath(a.dir)), file=sys.stderr, flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
