"""Simulator-Zentrale (kimmy): Läufe im Browser starten, live in 3D ansehen, später abspielen.

    python tools/simserver.py [--host 0.0.0.0] [--port 8790] [--runs ~/xgo_sim/laeufe] [--max 4]

Jeder Lauf ist ein eigener Prozess `tests/sim_run.py … --view <lauf>/lauf.jsonl` (niedrige Priorität); die
3D-Ansicht (tools/sim3d.html) liest die Datei mit, solange er läuft, danach als Replay. Je Lauf ein Ordner mit
meta.json, lauf.jsonl und ausgabe.txt (letzte Zeile = Ergebnis von sim_run.run). Einspielen: tools/sim_kimmy.sh.
"""
from __future__ import annotations

import ast
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HIER = os.path.dirname(os.path.abspath(__file__))
MODES = ("explore", "home", "patrol", "follow", "approach", "pet")
SCENES = ("raum", "zwei", "spalt", "fuge", "unten", "unten_tag")
FLAGS = ("--wheels", "--dyn", "--rider", "--imu", "--fs-thread")
# --set geht in sim_run.py durch eval(): nur Zahlen, True/False und Zahlenlisten durchlassen
SET_RE = re.compile(r"^[a-z_]+\.[a-z_0-9]+=(-?\d+(\.\d+)?|True|False|[\[(][-\d., ]*[\])])$")
ID_RE = re.compile(r"^[0-9]{8}-[0-9]{6}-[a-z]+$")


def befehl(p: dict, pfad: str) -> list[str]:
    """Argumente für sim_run.py aus dem Formular; ValueError bei allem Unerwarteten."""
    mode = p.get("mode")
    if mode not in MODES:
        raise ValueError(f"Modus {mode!r} unbekannt")
    secs = float(p.get("secs", 60))
    if not 5 <= secs <= 1800:
        raise ValueError("Sekunden 5..1800")
    scene = p.get("scene", "raum")
    if scene not in SCENES:
        raise ValueError(f"Szene {scene!r} unbekannt")
    cmd = [sys.executable, "tests/sim_run.py", mode, f"{secs:g}", "--scene", scene]
    for f in p.get("flags", []):
        if f not in FLAGS:
            raise ValueError(f"Schalter {f!r} unbekannt")
        cmd.append(f)
    ph = int(p.get("phantoms", 0) or 0)
    if not 0 <= ph <= 50:
        raise ValueError("Phantome 0..50")
    if ph:
        cmd += ["--phantoms", str(ph)]
    for s in p.get("set", []):
        s = s.strip()
        if not s:
            continue
        if not SET_RE.match(s):
            raise ValueError(f"--set {s!r}: nur abschnitt.name=Zahl/True/False/[Zahlen]")
        k, v = s.split("=", 1)
        if k.startswith("behavior.height_") and not 60 <= float(v) <= 108:
            raise ValueError(f"{k}: Höhe 60..108 mm (Sollwert-Skala, height_limit_u8 205)")
        cmd += ["--set", s]
    return cmd + ["--view", pfad]


class Zentrale:
    def __init__(self, runs: str, max_parallel: int = 4):
        self.runs = runs
        self.max = max_parallel
        self.proc: dict[str, subprocess.Popen] = {}
        self.lock = threading.Lock()
        os.makedirs(runs, exist_ok=True)

    def ordner(self, lid: str) -> str:
        if not ID_RE.match(lid):
            raise KeyError(lid)
        p = os.path.join(self.runs, lid)
        if not os.path.isdir(p):
            raise KeyError(lid)
        return p

    def laeuft(self, lid: str) -> bool:
        with self.lock:
            pr = self.proc.get(lid)
            return pr is not None and pr.poll() is None

    def start(self, p: dict) -> str:
        with self.lock:
            n = sum(pr.poll() is None for pr in self.proc.values())
            if n >= self.max:
                raise ValueError(f"schon {n} Läufe aktiv (höchstens {self.max})")
            lid = time.strftime("%Y%m%d-%H%M%S") + "-" + str(p.get("mode"))
            d = os.path.join(self.runs, lid)
            cmd = befehl(p, os.path.join(d, "lauf.jsonl"))
            os.makedirs(d)
            with open(os.path.join(d, "meta.json"), "w", encoding="utf-8") as f:
                json.dump({"id": lid, "start": time.time(), "param": p, "cmd": cmd[1:]}, f, ensure_ascii=False)
            out = open(os.path.join(d, "ausgabe.txt"), "w", encoding="utf-8")
            self.proc[lid] = subprocess.Popen(cmd, cwd=ROOT, stdout=out, stderr=subprocess.STDOUT,
                                              stdin=subprocess.DEVNULL, preexec_fn=lambda: os.nice(10),
                                              env={**os.environ, "PYTHONUNBUFFERED": "1"})
            out.close()
            return lid

    def stop(self, lid: str) -> None:
        with self.lock:
            pr = self.proc.get(lid)
        if pr is not None and pr.poll() is None:
            pr.send_signal(signal.SIGINT)
            try:
                pr.wait(5)
            except subprocess.TimeoutExpired:
                pr.kill()

    def loeschen(self, lid: str) -> None:
        d = self.ordner(lid)
        if self.laeuft(lid):
            raise ValueError("läuft noch")
        shutil.rmtree(d)
        with self.lock:
            self.proc.pop(lid, None)

    def info(self, lid: str) -> dict:
        d = self.ordner(lid)
        with open(os.path.join(d, "meta.json"), encoding="utf-8") as f:
            m = json.load(f)
        m["laeuft"] = self.laeuft(lid)
        m["ergebnis"] = m["fehler"] = None
        try:
            with open(os.path.join(d, "ausgabe.txt"), encoding="utf-8", errors="replace") as f:
                zeilen = f.read().splitlines()
        except OSError:
            zeilen = []
        if not m["laeuft"] and zeilen:
            try:
                m["ergebnis"] = ast.literal_eval(zeilen[-1])
            except (ValueError, SyntaxError):
                m["fehler"] = "\n".join(zeilen[-15:])
        try:
            m["mb"] = round(os.path.getsize(os.path.join(d, "lauf.jsonl")) / 1e6, 1)
        except OSError:
            m["mb"] = 0
        return m

    def liste(self) -> list[dict]:
        out = []
        for lid in sorted(os.listdir(self.runs), reverse=True):
            try:
                out.append(self.info(lid))
            except (KeyError, OSError, ValueError):
                pass
        return out


def serve(z: Zentrale, host: str, port: int) -> ThreadingHTTPServer:
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code: int, body: bytes, ctype: str = "application/json; charset=utf-8"):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200):
            self._send(code, json.dumps(obj, ensure_ascii=False).encode())

        def _datei(self, name: str):
            with open(os.path.join(HIER, name), "rb") as f:
                self._send(200, f.read(), "text/html; charset=utf-8")

        def do_GET(self):
            pfad = self.path.split("?", 1)[0]
            teile = [t for t in pfad.split("/") if t]
            try:
                if not teile:
                    return self._datei("simserver.html")
                if teile == ["api", "laeufe"]:
                    return self._json(z.liste())
                if teile[0] == "lauf" and len(teile) >= 2:
                    d = z.ordner(teile[1])
                    if len(teile) == 2:
                        if not pfad.endswith("/"):        # relative Pfade (events) brauchen den Schrägstrich
                            self.send_response(301)
                            self.send_header("Location", pfad + "/")
                            self.end_headers()
                            return
                        return self._datei("sim3d.html")
                    if teile[2] == "events":
                        return self._events(teile[1], os.path.join(d, "lauf.jsonl"))
                    if teile[2] == "rider-model.bin":    # CAD-Modell wie im Web (web/rider-model.bin)
                        with open(os.path.join(ROOT, "web", "rider-model.bin"), "rb") as f:
                            return self._send(200, f.read(), "application/octet-stream")
                    if teile[2] == "lauf.jsonl":
                        with open(os.path.join(d, "lauf.jsonl"), "rb") as f:
                            return self._send(200, f.read(), "application/x-ndjson")
            except KeyError:
                pass
            self._send(404, b"nicht gefunden", "text/plain; charset=utf-8")

        def do_POST(self):
            teile = [t for t in self.path.split("?", 1)[0].split("/") if t]
            try:
                n = int(self.headers.get("Content-Length") or 0)
                body = json.loads(self.rfile.read(n) or b"{}")
                if teile == ["api", "start"]:
                    return self._json({"id": z.start(body)})
                if teile == ["api", "rider", "neu"]:
                    # virtueller Rider (tools/sim_rider.py): Szene, Karte und Pose zurück auf Anfang
                    r = subprocess.run(["systemctl", "--user", "restart", "xgo-rider.service"],
                                       capture_output=True, text=True, timeout=30)
                    if r.returncode:
                        return self._json({"fehler": (r.stderr or r.stdout).strip() or "Neustart fehlgeschlagen"}, 500)
                    return self._json({"ok": True})
                if len(teile) == 3 and teile[:2] == ["api", "stop"]:
                    z.ordner(teile[2])
                    z.stop(teile[2])
                    return self._json({"ok": True})
                if len(teile) == 3 and teile[:2] == ["api", "loeschen"]:
                    z.loeschen(teile[2])
                    return self._json({"ok": True})
            except KeyError:
                return self._json({"fehler": "Lauf unbekannt"}, 404)
            except (ValueError, TypeError) as e:
                return self._json({"fehler": str(e)}, 400)
            self._json({"fehler": "unbekannt"}, 404)

        def _events(self, lid: str, datei: str):
            """Datei mitlesen: erst alles, dann neue Zeilen, solange der Lauf läuft; danach 'ende'."""
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            rest, pos, ping = b"", 0, time.monotonic()
            try:
                while True:
                    lief = z.laeuft(lid)            # vor dem Lesen: was danach kommt, ist sicher geschrieben
                    try:
                        with open(datei, "rb") as f:
                            f.seek(pos)
                            neu = f.read()
                    except FileNotFoundError:
                        neu = b""
                    pos += len(neu)
                    rest += neu
                    *zeilen, rest = rest.split(b"\n")
                    for ln in zeilen:
                        if ln.strip():
                            self.wfile.write(b"data: " + ln + b"\n\n")
                    if zeilen:
                        self.wfile.flush()
                    if not lief and not neu:
                        self.wfile.write(b"event: ende\ndata: {}\n\n")
                        self.wfile.flush()
                        while True:                  # offen lassen, sonst spielt EventSource alles noch einmal
                            time.sleep(15)
                            self.wfile.write(b": ping\n\n")
                            self.wfile.flush()
                    if time.monotonic() - ping > 15:
                        ping = time.monotonic()
                        self.wfile.write(b": ping\n\n")
                        self.wfile.flush()
                    if not neu:
                        time.sleep(0.15)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                return

    srv = ThreadingHTTPServer((host, port), H)
    srv.daemon_threads = True
    return srv


if __name__ == "__main__":
    a = sys.argv
    host = a[a.index("--host") + 1] if "--host" in a else "0.0.0.0"
    port = int(a[a.index("--port") + 1]) if "--port" in a else 8790
    runs = os.path.expanduser(a[a.index("--runs") + 1]) if "--runs" in a else os.path.join(ROOT, "laeufe")
    mx = int(a[a.index("--max") + 1]) if "--max" in a else 4
    z = Zentrale(runs, mx)
    srv = serve(z, host, port)
    print(f"Simulator-Zentrale auf {host}:{port}, Läufe in {runs}", flush=True)

    def ende(*_):
        for lid in list(z.proc):
            z.stop(lid)
        raise SystemExit(0)
    signal.signal(signal.SIGTERM, ende)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        ende()
