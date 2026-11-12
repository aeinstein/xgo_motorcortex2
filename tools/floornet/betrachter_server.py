#!/usr/bin/env python3
"""Webserver für betrachter.html: liefert die Dateien eines Verzeichnisses (wie python -m http.server) und nimmt das
Aussortieren untauglicher Bilder an.

  cd ~/floornet && python repo/tools/floornet/betrachter_server.py [--port 8765] [--bind 0.0.0.0] [--dir .]

POST /aus mit JSON {"ansicht": "hd", "id": "explore10/005514", "aus": true}: hängt die ID an die Handliste
<stage1 der Ansicht>/aus_hand.txt an ("aus": false schreibt "-id", holt das Bild also zurück). Gelöscht wird nichts;
stage2.py, stage3.py und stage3_npz.py lesen die Liste. POST /gesichtet mit {"ansicht", "id"} hängt die ID an
<stage1>/gesichtet.txt an; die liest nur der Betrachter (schon gezeigte Bilder nach dem Neuladen ausblenden).
POST /neu mit {"ansicht", "id", "prompt"} lässt das Bild von ComfyUI mit neuem Zufalls-Seed und der gewählten Prompt-Variante (leer = Standard) neu malen (neu_malen.py als eigener
Prozess mit --python, immer nur einer zugleich); die alte Bearbeitung und ihre Masken werden überschrieben. Antwort:
{"ok", "seed", "rows"} (neue Labelzeile je 8-px-Spalte).
Ohne Anmeldung, nur fürs Heimnetz.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import threading
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

STAGE1 = {"p11": "stage1_p11", "hd": "stage1_hd", "g0h": "stage1_g0h", "p8": "stage1", "t1": "stage1_t1", "t2": "stage1_t2", "t3": "stage1_t3"}      # wie ANSICHTEN in betrachter.html
HAND = "aus_hand.txt"                                                   # wie stage2.HAND
GESICHTET = "gesichtet.txt"                                             # nur für den Betrachter
ZIELE = {"/aus": HAND, "/gesichtet": GESICHTET}
ID = re.compile(r"[A-Za-z0-9_-]+/[0-9]+")
MAX_IDS, MAX_BYTES = 500, 20000                                         # Sammelmeldung von /gesichtet
_lock = threading.Lock()
_neu_lock = threading.Lock()                                            # ComfyUI: ein Bild nach dem anderen
NEU_CMD = [sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), "neu_malen.py")]
NEU_TIMEOUT_S = 180


def pruefen(root: str, ansicht: str, bild: str) -> str:
    """Fehlertext, wenn Ansicht oder ID nicht taugen, sonst ""."""
    if ansicht not in STAGE1:
        return "unbekannte Ansicht"
    if not isinstance(bild, str) or not ID.fullmatch(bild):
        return "ungültige ID"
    if not os.path.exists(os.path.join(root, STAGE1[ansicht], bild + ".png")):
        return "Bild gibt es in dieser Ansicht nicht"
    return ""


def eintragen(root: str, ansicht: str, bild: str, aus: bool, datei: str = HAND) -> str:
    """Zeile an die Liste datei hängen. Rückgabe: Fehlertext oder "" bei Erfolg."""
    fehler = pruefen(root, ansicht, bild)
    if not fehler:
        with _lock, open(os.path.join(root, STAGE1[ansicht], datei), "a", encoding="utf-8", newline="\n") as f:
            f.write(("" if aus else "-") + bild + "\n")
    return fehler


def eintragen_viele(root: str, ansicht: str, bilder: list, datei: str = GESICHTET) -> tuple[str, int]:
    """Mehrere IDs auf einmal anhängen; untaugliche fallen weg. Rückgabe: Fehlertext und Zahl der geschriebenen."""
    if ansicht not in STAGE1:
        return "unbekannte Ansicht", 0
    if len(bilder) > MAX_IDS:
        return "zu viele IDs", 0
    gut = [b for b in bilder if not pruefen(root, ansicht, b)]
    if gut:
        with _lock, open(os.path.join(root, STAGE1[ansicht], datei), "a", encoding="utf-8", newline="\n") as f:
            f.write("".join(b + "\n" for b in gut))
    return "", len(gut)


def neu_malen(root: str, ansicht: str, bild: str, prompt: str = "") -> tuple[str, dict]:
    """Bild neu malen lassen. Rückgabe: Fehlertext ("" bei Erfolg) und die Antwort von neu_malen.py (seed, rows)."""
    fehler = pruefen(root, ansicht, bild)
    if fehler:
        return fehler, {}
    if not isinstance(prompt, str) or (prompt and not (prompt.isascii() and prompt.isalnum() and len(prompt) <= 12)):
        return "ungültige Prompt-Variante", {}
    with _neu_lock:
        try:
            p = subprocess.run(NEU_CMD + ["--dir", root, "--prompt", prompt, ansicht, bild], capture_output=True, text=True, timeout=NEU_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            return "ComfyUI hat nicht rechtzeitig geantwortet", {}
    if p.returncode != 0:
        return "neu malen fehlgeschlagen: " + (p.stderr.strip().splitlines() or ["?"])[-1][:200], {}
    try:
        return "", dict(json.loads(p.stdout.strip().splitlines()[-1]))
    except (ValueError, IndexError):
        return "neu malen: unlesbare Antwort", {}


class Handler(SimpleHTTPRequestHandler):
    protocol_version = "HTTP/1.1"               # Verbindung bleibt offen: je Bild 4–5 Dateien ohne neuen Aufbau

    def do_POST(self) -> None:
        n = int(self.headers.get("Content-Length") or 0)
        # application/json verlangt bei fremden Seiten eine Vorabfrage, die hier niemand beantwortet
        if not 0 < n <= MAX_BYTES:
            self.close_connection = True
            self.send_error(400, "nur POST /aus oder /gesichtet mit kurzem JSON")
            return
        roh = self.rfile.read(n)                # erst lesen, sonst reißt die Verbindung beim Ablehnen ab
        if self.path not in (*ZIELE, "/neu") or self.headers.get("Content-Type", "").split(";")[0].strip() != "application/json":
            self.send_error(400, "nur POST /aus oder /gesichtet mit JSON")
            return
        antwort: dict = {}
        try:
            d = json.loads(roh)
            if self.path == "/neu":
                fehler, antwort = neu_malen(self.directory, d.get("ansicht"), d.get("id"), d.get("prompt") or "")
            elif self.path == "/gesichtet" and isinstance(d.get("ids"), list):
                fehler, antwort["n"] = eintragen_viele(self.directory, d.get("ansicht"), d["ids"])
                antwort["ausgelassen"] = len(d["ids"]) - antwort["n"]
            else:
                aus = self.path != "/aus" or bool(d.get("aus", True))   # gesichtet kennt kein Zurückholen
                fehler = eintragen(self.directory, d.get("ansicht"), d.get("id"), aus, ZIELE[self.path])
        except (ValueError, AttributeError):
            fehler = "kein JSON-Objekt"
        body = json.dumps({"ok": not fehler, "fehler": fehler, **antwort}, ensure_ascii=False).encode("utf-8")
        self.send_response(400 if fehler else 200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def send_response(self, code: int, message: str | None = None) -> None:
        self._code = code
        super().send_response(code, message)

    def end_headers(self) -> None:
        pfad = getattr(self, "path", "").split("?")[0]
        if pfad.endswith((HAND, GESICHTET, "index.js")):                # Listen ändern sich, nie aus dem Zwischenspeicher
            self.send_header("Cache-Control", "no-store")
        elif getattr(self, "_code", 0) == 200 and self.command == "GET" and (
                (pfad.startswith("/data/") and pfad.endswith(".jpg"))
                or (pfad.endswith(".png") and pfad.split("/")[1] in STAGE1.values())):
            self.send_header("Cache-Control", "max-age=86400")          # Originale und Stage1 ändern sich nie; Masken schon
        super().end_headers()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--dir", default=".")
    ap.add_argument("--python", default=sys.executable, help="Python mit OpenCV für neu_malen.py (POST /neu)")
    a = ap.parse_args()
    NEU_CMD[0] = a.python
    srv = ThreadingHTTPServer((a.bind, a.port), partial(Handler, directory=os.path.abspath(a.dir)))
    print("Betrachter auf %s:%d, Verzeichnis %s" % (a.bind, a.port, os.path.abspath(a.dir)), file=sys.stderr, flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
