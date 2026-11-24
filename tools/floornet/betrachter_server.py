#!/usr/bin/env python3
"""Webserver für betrachter.html: liefert die Dateien eines Verzeichnisses (wie python -m http.server) und nimmt das
Aussortieren untauglicher Bilder an.

  cd ~/floornet && python repo/tools/floornet/betrachter_server.py [--port 8765] [--bind 0.0.0.0] [--dir .]

POST /aus mit JSON {"ansicht": "hd", "id": "explore10/005514", "aus": true}: hängt die ID an die Handliste
<stage1 der Ansicht>/aus_hand.txt an ("aus": false schreibt "-id", holt das Bild also zurück). Gelöscht wird nichts;
stage2.py, stage3.py und stage3_npz.py lesen die Liste. POST /gesichtet mit {"ansicht", "id"} hängt die ID an
<stage1>/gesichtet.txt an; die liest nur der Betrachter (schon gezeigte Bilder nach dem Neuladen ausblenden).
POST /neu mit {"ansicht", "id", "prompt"[, "seed"]} (seed fehlt: zufällig) lässt das Bild von ComfyUI mit neuem Zufalls-Seed und der gewählten Prompt-Variante (leer = Standard) neu malen (neu_malen.py als eigener
Prozess mit --python, immer nur einer zugleich); die alte Bearbeitung und ihre Masken werden überschrieben. Antwort:
{"ok", "seed", "rows"} (neue Labelzeile je 8-px-Spalte).
POST /hand mit {"ansicht", "id", "rows": [80 Zeilen 0..480]} schreibt eine von Hand gezogene Labellinie als Stage2- und
Stage3-Maske (neu_malen.py --hand); das Grünbild in Stage1 bleibt. Antwort: {"ok", "rows"}.
POST /name mit {"fahrt": "20261115-161345", "name": "Leiter umgefahren"} setzt den Anzeigenamen einer Fahrt in
netz/namen.json (leerer Name löscht ihn). Umbenannt wird nur die Anzeige: Verzeichnisse, Bild-IDs, Labels und Listen
hängen alle an der Kennung (Datum-Uhrzeit) und bleiben, wie sie sind.
GET /prompte liefert {"prompte": {"11": "...", "t1": ..., "t2": ...}}, die ComfyUI-Prompte unter comfy/prompte/<name>.txt;
POST /prompt mit {"name": "11", "text": "..."} ersetzt einen davon (der alte Text bleibt in comfy/prompte/alt/). Beim Start
legt der Server fehlende Dateien aus comfy_prompt*.txt neben diesem Skript an. Sie gelten sofort für N (neu_malen.py) und
für Stapelläufe, deren --prompt darauf zeigt (comfy/prompt11.txt ist ein Link auf comfy/prompte/11.txt); ein laufender
Stapel hat seinen Prompt schon beim Start gelesen.
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
import time
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

STAGE1 = {"p11": "stage1_p11", "hd": "stage1_hd", "g0h": "stage1_g0h", "p8": "stage1", "t1": "stage1_t1", "t2": "stage1_t2", "t3": "stage1_t3"}      # wie ANSICHTEN in betrachter.html
HAND = "aus_hand.txt"                                                   # wie stage2.HAND
GESICHTET = "gesichtet.txt"                                             # nur für den Betrachter
ZIELE = {"/aus": HAND, "/gesichtet": GESICHTET}
ID = re.compile(r"[A-Za-z0-9_-]+/[0-9]+")
MAX_IDS, MAX_BYTES = 500, 20000                                         # Sammelmeldung von /gesichtet
NAMEN = os.path.join("netz", "namen.json")                             # Anzeigenamen der Fahrten (POST /name)
FAHRT = re.compile(r"[A-Za-z0-9_-]{1,64}")
MAX_NAME = 80
PROMPTE = os.path.join("comfy", "prompte")                             # bearbeitbare ComfyUI-Prompte (POST /prompt)
PROMPT_START = {"11": "comfy_prompt11.txt", "t1": "comfy_prompt_t1.txt", "t2": "comfy_prompt_t2.txt"}
PROMPT_NAME = re.compile(r"[A-Za-z0-9]{1,12}")
MAX_PROMPT = 4000
_lock = threading.Lock()
_neu_lock = threading.Lock()                                            # ComfyUI: ein Bild nach dem anderen
NEU_CMD = [sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), "neu_malen.py")]
NEU_TIMEOUT_S = 180
NCOLS, ZEILEN_H = 80, 480                                               # wie comfy_label.NCOLS, H (POST /hand)


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


def neu_malen(root: str, ansicht: str, bild: str, prompt: str = "", seed=None) -> tuple[str, dict]:
    """Bild neu malen lassen. Rückgabe: Fehlertext ("" bei Erfolg) und die Antwort von neu_malen.py (seed, rows)."""
    fehler = pruefen(root, ansicht, bild)
    if fehler:
        return fehler, {}
    if not isinstance(prompt, str) or (prompt and not (prompt.isascii() and prompt.isalnum() and len(prompt) <= 12)):
        return "ungültige Prompt-Variante", {}
    if seed is not None and (isinstance(seed, bool) or not isinstance(seed, int) or not 1 <= seed < 2 ** 31):
        return "ungültiger Seed", {}
    with _neu_lock:
        try:
            p = subprocess.run(NEU_CMD + ["--dir", root, "--prompt", prompt] + (["--seed", str(seed)] if seed else []) + [ansicht, bild], capture_output=True, text=True, timeout=NEU_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            return "ComfyUI hat nicht rechtzeitig geantwortet", {}
    if p.returncode != 0:
        return "neu malen fehlgeschlagen: " + (p.stderr.strip().splitlines() or ["?"])[-1][:200], {}
    try:
        return "", dict(json.loads(p.stdout.strip().splitlines()[-1]))
    except (ValueError, IndexError):
        return "neu malen: unlesbare Antwort", {}


def hand_linie(root: str, ansicht: str, bild: str, zeilen) -> tuple[str, dict]:
    """Von Hand gezogene Labellinie als Stage2/3-Maske schreiben lassen (neu_malen.py --hand, eigener Prozess wegen
    OpenCV). Rückgabe: Fehlertext ("" bei Erfolg) und die Antwort (rows, wie das Training die Maske liest)."""
    fehler = pruefen(root, ansicht, bild)
    if fehler:
        return fehler, {}
    if not isinstance(zeilen, list) or len(zeilen) != NCOLS or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or not 0 <= v <= ZEILEN_H for v in zeilen):
        return "Linie braucht %d Zahlen von 0 bis %d" % (NCOLS, ZEILEN_H), {}
    try:
        p = subprocess.run(NEU_CMD + ["--dir", root, "--hand", ansicht, bild], input=json.dumps(zeilen),
                           capture_output=True, text=True, timeout=60)
    except subprocess.TimeoutExpired:
        return "Linie speichern dauerte zu lange", {}
    if p.returncode != 0:
        return "Linie speichern fehlgeschlagen: " + (p.stderr.strip().splitlines() or ["?"])[-1][:200], {}
    try:
        return "", dict(json.loads(p.stdout.strip().splitlines()[-1]))
    except (ValueError, IndexError):
        return "Linie speichern: unlesbare Antwort", {}


def name_setzen(root: str, fahrt, name) -> tuple[str, dict]:
    """Anzeigenamen einer Fahrt setzen (leer: löschen). Rückgabe: Fehlertext ("" bei Erfolg) und alle Namen."""
    if not isinstance(fahrt, str) or not FAHRT.fullmatch(fahrt):
        return "ungültige Fahrt", {}
    if not os.path.isdir(os.path.join(root, "data", "rider", "fahrt", fahrt)):
        return "Fahrt gibt es nicht", {}
    if not isinstance(name, str):
        return "ungültiger Name", {}
    name = " ".join(name.split())                          # keine Zeilenumbrüche oder Tabs
    if len(name) > MAX_NAME or any(ord(c) < 32 for c in name):
        return "Name zu lang (höchstens %d Zeichen)" % MAX_NAME, {}
    pfad = os.path.join(root, NAMEN)
    with _lock:
        try:
            with open(pfad, encoding="utf-8") as f:
                namen = dict(json.load(f))
        except FileNotFoundError:
            namen = {}
        except ValueError:
            return "netz/namen.json ist unlesbar", {}
        if name:
            namen[fahrt] = name
        else:
            namen.pop(fahrt, None)
        tmp = pfad + ".tmp"
        with open(tmp, "w", encoding="utf-8", newline="\n") as f:
            json.dump(dict(sorted(namen.items())), f, ensure_ascii=False, indent=1)
        os.replace(tmp, pfad)                                  # nie halb geschrieben
    return "", namen


def prompte_anlegen(root: str) -> None:
    """Fehlende Prompt-Dateien aus den Vorlagen neben diesem Skript anlegen; vorhandene bleiben, wie sie sind."""
    os.makedirs(os.path.join(root, PROMPTE), exist_ok=True)
    hier = os.path.dirname(os.path.abspath(__file__))
    for name, vorlage in PROMPT_START.items():
        ziel = os.path.join(root, PROMPTE, name + ".txt")
        if not os.path.exists(ziel) and os.path.exists(os.path.join(hier, vorlage)):
            with open(os.path.join(hier, vorlage), encoding="utf-8") as f, open(ziel, "w", encoding="utf-8", newline="\n") as g:
                g.write(f.read().strip() + "\n")


def prompte_lesen(root: str) -> dict:
    d = os.path.join(root, PROMPTE)
    out = {}
    for f in sorted(os.listdir(d)) if os.path.isdir(d) else []:
        if f.endswith(".txt") and PROMPT_NAME.fullmatch(f[:-4]):
            with open(os.path.join(d, f), encoding="utf-8") as g:
                out[f[:-4]] = g.read().strip()
    return out


def prompt_setzen(root: str, name, text) -> tuple[str, dict]:
    """Einen vorhandenen Prompt ersetzen; der alte kommt nach comfy/prompte/alt/<name>-<Zeit>.txt.
    Rückgabe: Fehlertext ("" bei Erfolg) und alle Prompte."""
    if not isinstance(name, str) or not PROMPT_NAME.fullmatch(name):
        return "ungültiger Prompt-Name", {}
    pfad = os.path.join(root, PROMPTE, name + ".txt")
    if not os.path.exists(pfad):
        return "Prompt gibt es nicht", {}
    if not isinstance(text, str):
        return "ungültiger Text", {}
    text = text.replace("\r\n", "\n").strip()
    if not text or len(text) > MAX_PROMPT:
        return "Prompt leer oder zu lang (höchstens %d Zeichen)" % MAX_PROMPT, {}
    if any(ord(c) < 32 and c != "\n" for c in text):
        return "Steuerzeichen im Prompt", {}
    with _lock:
        with open(pfad, encoding="utf-8") as f:
            alt = f.read()
        if alt.strip() != text:
            os.makedirs(os.path.join(root, PROMPTE, "alt"), exist_ok=True)
            with open(os.path.join(root, PROMPTE, "alt", "%s-%s.txt" % (name, time.strftime("%Y%m%d-%H%M%S"))), "w",
                      encoding="utf-8", newline="\n") as f:
                f.write(alt)
            tmp = pfad + ".tmp"
            with open(tmp, "w", encoding="utf-8", newline="\n") as f:
                f.write(text + "\n")
            os.replace(tmp, pfad)                              # nie halb geschrieben: ein Stapellauf liest ihn beim Start
    return "", prompte_lesen(root)


class Handler(SimpleHTTPRequestHandler):
    protocol_version = "HTTP/1.1"               # Verbindung bleibt offen: je Bild 4–5 Dateien ohne neuen Aufbau

    def do_GET(self) -> None:
        if self.path.split("?")[0] == "/prompte":
            self._json(200, {"ok": True, "prompte": prompte_lesen(self.directory)})
            return
        super().do_GET()

    def _json(self, code: int, d: dict) -> None:
        body = json.dumps(d, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self) -> None:
        n = int(self.headers.get("Content-Length") or 0)
        # application/json verlangt bei fremden Seiten eine Vorabfrage, die hier niemand beantwortet
        if not 0 < n <= MAX_BYTES:
            self.close_connection = True
            self.send_error(400, "nur POST /aus oder /gesichtet mit kurzem JSON")
            return
        roh = self.rfile.read(n)                # erst lesen, sonst reißt die Verbindung beim Ablehnen ab
        if self.path not in (*ZIELE, "/neu", "/hand", "/name", "/prompt") or self.headers.get("Content-Type", "").split(";")[0].strip() != "application/json":
            self.send_error(400, "nur POST /aus oder /gesichtet mit JSON")
            return
        antwort: dict = {}
        try:
            d = json.loads(roh)
            if self.path == "/prompt":
                fehler, antwort["prompte"] = prompt_setzen(self.directory, d.get("name"), d.get("text"))
            elif self.path == "/name":
                fehler, antwort["namen"] = name_setzen(self.directory, d.get("fahrt"), d.get("name"))
            elif self.path == "/hand":
                fehler, antwort = hand_linie(self.directory, d.get("ansicht"), d.get("id"), d.get("rows"))
            elif self.path == "/neu":
                fehler, antwort = neu_malen(self.directory, d.get("ansicht"), d.get("id"), d.get("prompt") or "", d.get("seed"))
            elif self.path == "/gesichtet" and isinstance(d.get("ids"), list):
                fehler, antwort["n"] = eintragen_viele(self.directory, d.get("ansicht"), d["ids"])
                antwort["ausgelassen"] = len(d["ids"]) - antwort["n"]
            else:
                aus = self.path != "/aus" or bool(d.get("aus", True))   # gesichtet kennt kein Zurückholen
                fehler = eintragen(self.directory, d.get("ansicht"), d.get("id"), aus, ZIELE[self.path])
        except (ValueError, AttributeError):
            fehler = "kein JSON-Objekt"
        self._json(400 if fehler else 200, {"ok": not fehler, "fehler": fehler, **antwort})

    def send_response(self, code: int, message: str | None = None) -> None:
        self._code = code
        super().send_response(code, message)

    def end_headers(self) -> None:
        pfad = getattr(self, "path", "").split("?")[0]
        if pfad.endswith((HAND, GESICHTET, "index.js", "namen.json")):                # Listen ändern sich, nie aus dem Zwischenspeicher
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
    prompte_anlegen(os.path.abspath(a.dir))
    srv = ThreadingHTTPServer((a.bind, a.port), partial(Handler, directory=os.path.abspath(a.dir)))
    print("Betrachter auf %s:%d, Verzeichnis %s" % (a.bind, a.port, os.path.abspath(a.dir)), file=sys.stderr, flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
