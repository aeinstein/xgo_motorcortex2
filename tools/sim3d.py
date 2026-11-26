"""3D-Ansicht des Simulators im Browser (nur PC, liest nur mit).

Live:   python tests/sim_run.py explore 120 --wheels --dyn --rider --live [--live-port 8765]
Datei:  python tests/sim_run.py follow 40 --view lauf.jsonl
Replay: python tools/sim3d.py lauf.jsonl [--port 8765]     -> http://localhost:8765/

Feed liest je Takt Wahrheit (SimWorld) und Schätzung (Raumkarte, Weg) und schreibt sie als JSONL (Zeile 1 Kopf,
dann Takte) und/oder schickt sie per Server-Sent Events an tools/sim3d.html. Live und Replay sind derselbe Weg:
Der Server spielt jedem Client alle bisherigen Zeilen vor und dann die neuen.
"""
from __future__ import annotations

import base64
import json
import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

HTML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sim3d.html")
MODELL = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "web", "rider-model.bin")


def map_to_world(start, pts):
    """Punkte (x, y) im Karten-KS -> Welt; start = Weltpose (x, y, th) der Karten-Null (wie sim_run.py)."""
    x0, y0, th0 = start
    c, s = math.cos(th0), math.sin(th0)
    return [(round(x0 + x * c - y * s, 3), round(y0 + x * s + y * c, 3)) for x, y in pts]


def map_image(lo: np.ndarray, l_show: float, res: float, pad: int = 4) -> dict | None:
    """Bekannter Ausschnitt der Raumkarte als Zellwerte (0 unbekannt, 1 frei, 2 belegt; uint8 zeilenweise, base64).
    r0/c0: erste Zeile/Spalte des Ausschnitts in der Karte; Zelle (r, c) liegt bei Karten-KS
    x = (c + 0.5 - n/2) * res, y = (r + 0.5 - n/2) * res."""
    known = np.abs(lo) > l_show
    if not known.any():
        return None
    rr, cc = np.nonzero(known)
    n = lo.shape[0]
    r0, r1 = max(0, rr.min() - pad), min(n, rr.max() + pad + 1)
    c0, c1 = max(0, cc.min() - pad), min(n, cc.max() + pad + 1)
    sub = lo[r0:r1, c0:c1]
    z = np.zeros(sub.shape, np.uint8)
    z[sub < -l_show] = 1
    z[sub > l_show] = 2
    return {"zellen": base64.b64encode(z.tobytes()).decode(), "r0": int(r0), "c0": int(c0),
            "rows": int(r1 - r0), "cols": int(c1 - c0), "n": int(n), "res": float(res)}


class Hub:
    """Zeilen im Speicher; Clients bekommen alles ab Zeile 0 und warten dann auf neue. max_lines (virtueller
    Rider, läuft unbegrenzt): nur die letzten so vielen Zeilen, der Kopf (erste Zeile) bleibt immer.
    Zeilennummern sind absolut: base = Nummer von lines[0]."""

    def __init__(self, max_lines: int | None = None):
        self.lines: list[str] = []
        self.done = False
        self.cond = threading.Condition()
        self.max_lines = max_lines
        self.base = 0
        self.kopf: str | None = None

    @property
    def n(self) -> int:
        return self.base + len(self.lines)

    def add(self, line: str) -> None:
        with self.cond:
            if self.kopf is None:
                self.kopf = line
            self.lines.append(line)
            if self.max_lines and len(self.lines) > self.max_lines:
                d = len(self.lines) - self.max_lines
                del self.lines[:d]
                self.base += d
            self.cond.notify_all()

    def ab(self, i: int) -> tuple[list[str], int]:
        """Zeilen ab Nummer i (unter cond aufrufen) und die nächste Nummer. Wer neu kommt (i = 0), bekommt nach
        dem Abschneiden zuerst den Kopf; wer zurückliegt, überspringt das Abgeschnittene."""
        neu = self.lines[max(0, i - self.base):]
        if i == 0 and self.base > 0 and self.kopf is not None:
            neu = [self.kopf] + neu
        return neu, self.n

    def finish(self) -> None:
        with self.cond:
            self.done = True
            self.cond.notify_all()


def serve(hub: Hub, port: int = 8765, host: str = "127.0.0.1") -> ThreadingHTTPServer:
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            pfad = self.path.split("?", 1)[0]
            if pfad in ("/", "/index.html", "/rider-model.bin"):
                modell = pfad == "/rider-model.bin"
                try:
                    with open(MODELL if modell else HTML, "rb") as f:
                        body = f.read()
                except OSError:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", "application/octet-stream" if modell else "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if pfad != "/events":
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            i = 0
            try:
                while True:
                    with hub.cond:
                        hub.cond.wait_for(lambda: i < hub.n or hub.done, timeout=15)
                        fertig = hub.done and i >= hub.n
                        neu, i_neu = hub.ab(i)
                    if fertig:
                        self.wfile.write(b"event: ende\ndata: {}\n\n")
                        self.wfile.flush()
                        # offen lassen, sonst verbindet EventSource neu und spielt alles noch einmal
                        while True:
                            time.sleep(15)
                            self.wfile.write(b": ping\n\n")
                            self.wfile.flush()
                    if not neu:
                        self.wfile.write(b": ping\n\n")
                    for ln in neu:
                        self.wfile.write(b"data: " + ln.encode() + b"\n\n")
                    i = i_neu
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                return

    srv = ThreadingHTTPServer((host, port), H)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


class Feed:
    """Liest SimWorld und App im eigenen Thread mit (hz), nur lesend; Karte 1 Hz, Kamerabild cam_hz."""

    def __init__(self, world, app, mode: str, path: str | None = None, hub: Hub | None = None,
                 hz: float = 10.0, cam_hz: float = 4.0, cam_w: int = 320):
        self.world, self.app, self.mode = world, app, mode
        self.hub = hub
        self.f = open(path, "w", encoding="utf-8") if path else None
        self.hz, self.cam_hz, self.cam_w = hz, cam_hz, cam_w
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._run, daemon=True)
        self._last_cam = self._last_map = -1e9
        self._last_idx = -1

    def _emit(self, obj: dict) -> None:
        line = json.dumps(obj, separators=(",", ":"), ensure_ascii=False)
        if self.f:
            self.f.write(line + "\n")
            self.f.flush()                      # tools/simserver.py liest die Datei live mit
        if self.hub:
            self.hub.add(line)

    def head(self) -> dict:
        w = self.world
        sc = w.scene
        p = w.person
        L = w.log
        start = (L[0]["x"], L[0]["y"], L[0]["th"]) if L else (w.x, w.y, w.th)
        k = {"typ": "kopf", "modus": self.mode, "size": sc.size, "size_y": sc.sy, "wall_h": sc.wall_h,
             "box_h": sc.box_h, "boxes": [list(b) for b in sc.boxes], "rillen": [list(r) for r in sc.rillen],
             "robot_r": sc.robot_r, "start": [round(v, 4) for v in start],
             "hfov": self.app.cfg.camera.hfov_deg,
             "person": None if p is None else {"cls": p.cls, "h": p.height_m, "b": p.width_m},
             "dunkel": [list(z) for z in w.dark_zones]}
        farbe = getattr(w, "tex_farben", None)
        if getattr(sc, "erweitert", False) and farbe:
            # erweiterte Szene (z. B. "unten"): Wände in Stücken, Einrichtung mit Höhe und Farbe, Teppiche
            k["waende"] = [[x1, y1, x2, y2, farbe[t]] for x1, y1, x2, y2, t in
                           (sc.waende or [(0, 0, sc.size, 0, "wand"), (sc.size, 0, sc.size, sc.sy, "wand"),
                                          (sc.size, sc.sy, 0, sc.sy, "wand"), (0, sc.sy, 0, 0, "wand")])]
            def eine(tex):                  # Fototexturen je Seite: die erste Seite mit eigenem Bild gibt die Farbe
                if isinstance(tex, str):
                    return tex
                return next((n for n in tex if n != "putz"), tex[0])
            k["waende"] += [[x1, y1, x2, y2, farbe[t]] for x1, y1, x2, y2, z0, z1, t in getattr(sc, "flaechen", [])]
            k["teile"] = [[t.x1, t.y1, t.x2, t.y2, t.z0, t.z1, farbe[eine(t.tex)], t.tex if isinstance(t.tex, str) else "foto"]
                          for t in sc.teile if getattr(t, "sichtbar", True)]
            k["teppiche"] = [[x1, y1, x2, y2, "#%02x%02x%02x" % (c[2], c[1], c[0])] for x1, y1, x2, y2, c in sc.teppiche]
            k["boden"] = "fliesen" if sc.boden == "bild" else sc.boden
        return k

    def tick(self, now: float) -> dict | None:
        w, app = self.world, self.app
        L = w.log
        if not L:
            return None
        r = L[-1]
        start = (L[0]["x"], L[0]["y"], L[0]["th"])
        d = {"typ": "takt", "t": round(r["t"], 3), "x": round(r["x"], 4), "y": round(r["y"], 4),
             "th": round(r["th"], 4), "v": round(r["v"], 3), "w": round(r["w"], 3),
             "nick": round(float(getattr(w, "pitch", 0.0)), 4), "cam_h": round(float(getattr(w, "cam_h", 0.13)), 4),
             "koll": w.collisions, "status": getattr(app._cmd, "status", ""),
             "verhalten": getattr(app.behavior, "name", "")}
        rob = getattr(app, "robot", None)
        mm = getattr(rob, "height_mm", None)
        if mm is not None:          # Höhen-Sollwert: Beine des CAD-Modells (u8 wie /proc/XGORider/height)
            rc = rob.cfg
            d["h_mm"] = round(float(mm), 1)
            d["h_u8"] = round(255 * (mm - rc.height_min_mm) / (rc.height_max_mm - rc.height_min_mm), 1)
        if r.get("px") is not None:
            d["p"] = [round(r["px"], 3), round(r["py"], 3)]
        gm = getattr(app, "gridmap", None)
        if gm is not None:
            d["schaetz"] = map_to_world(start, [(gm.x, gm.y)])[0] + (round(start[2] + gm.th, 4),)
            d["plan"] = map_to_world(start, list(gm.plan or []))
            if gm.route:
                d["route"] = map_to_world(start, list(gm.route))
            if now - self._last_map >= 1.0:
                self._last_map = now
                m = map_image(gm.lo.copy(), gm.cfg.l_show, gm.res)
                if m:
                    d["karte"] = m
        if self.cam_hz > 0 and now - self._last_cam >= 1.0 / self.cam_hz:
            s = app.snapshot
            if s.frame is not None and s.idx != self._last_idx:
                self._last_cam, self._last_idx = now, s.idx
                img = s.frame
                h = int(img.shape[0] * self.cam_w / img.shape[1])
                ok, jpg = cv2.imencode(".jpg", cv2.resize(img, (self.cam_w, h), interpolation=cv2.INTER_AREA),
                                       [cv2.IMWRITE_JPEG_QUALITY, 55])
                if ok:
                    d["bild"] = base64.b64encode(jpg.tobytes()).decode()
        return d

    def _run(self) -> None:
        while not self.app.running and not self._stop.is_set():
            time.sleep(0.02)
        while not self.world.log and not self._stop.is_set():
            time.sleep(0.02)
        self._emit(self.head())
        while not self._stop.is_set():
            t0 = time.monotonic()
            d = self.tick(t0)
            if d:
                self._emit(d)
            self._stop.wait(max(0.0, 1.0 / self.hz - (time.monotonic() - t0)))

    def start(self) -> "Feed":
        self._th.start()
        return self

    def close(self) -> None:
        self._stop.set()
        self._th.join(timeout=2)
        if self.world.log:
            d = self.tick(time.monotonic() + 10)        # letzte Karte immer mitnehmen
            if d:
                self._emit(d)
        if self.f:
            self.f.close()
        if self.hub:
            self.hub.finish()


def wait_forever(port: int) -> None:
    print(f"3D-Ansicht: http://localhost:{port}/  (Strg+C beendet)", flush=True)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 8765
    hub = Hub()
    with open(sys.argv[1], encoding="utf-8") as f:
        for ln in f:
            if ln.strip():
                hub.add(ln.rstrip("\n"))
    hub.finish()
    serve(hub, port)
    wait_forever(port)
