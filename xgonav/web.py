"""Lokale Schnittstelle: Overlay-Bilder (WebSocket, Einzelbild, MJPEG), Status-JSON,
Moduswechsel, Bildverarbeitung an/aus.

Die Bedienoberfläche ist PHP (web/), ausgeliefert von nginx (deploy/nginx-xgonav.conf);
nginx reicht /xgonav/ws, /xgonav/frame und /xgonav/stream direkt hierher durch. Keine Authentifizierung: web_host
auf 127.0.0.1 lassen und nur über nginx veröffentlichen.
"""
from __future__ import annotations

import base64
import hashlib
import json
import logging
import math
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2

from .affinity import pin_thread
from .app import App, render

log = logging.getLogger(__name__)
START = time.time()      # Startzeit des Prozesses: Oberfläche verbindet den Stream nach Neustart neu

# Schnittstelle für die PHP-Oberfläche (web/) bzw. nginx. Keine HTML-Seite mehr hier.
API = {"GET": ["/status", "/ws (WebSocket: Bilder, Bestätigung je Bild)", "/frame?after=<idx>", "/stream",
               "/map.png?px=480&view=6 (Raumkarte)"],
       "POST": ["/mode?name=..&target=..", "/stop", "/pipeline?on=0|1", "/drive?v=..&w=.. (nur Modus manual)",
                "/exposure?us=0|100..33333 (0 = Automatik)", "/gain?g=0|1..16 (0 = Automatik)",
                "/map/reset (Raumkarte leeren, Pose auf 0)"]}


WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
WS_ACK_TIMEOUT_S = 10.0     # ohne Bestätigung so lange: Verbindung gilt als tot


def ws_accept(key: str) -> str:
    return base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()


def ws_frame(opcode: int, payload: bytes) -> bytes:
    """Server-Frame (unmaskiert, FIN gesetzt)."""
    n = len(payload)
    if n < 126:
        head = struct.pack("!BB", 0x80 | opcode, n)
    elif n < 1 << 16:
        head = struct.pack("!BBH", 0x80 | opcode, 126, n)
    else:
        head = struct.pack("!BBQ", 0x80 | opcode, 127, n)
    return head + payload


def ws_read(rfile) -> tuple[int, bytes] | None:
    """Einen Client-Frame lesen (maskiert). None bei Verbindungsende."""
    h = rfile.read(2)
    if len(h) < 2:
        return None
    opcode, n = h[0] & 0x0F, h[1] & 0x7F
    if n == 126:
        n = struct.unpack("!H", rfile.read(2))[0]
    elif n == 127:
        n = struct.unpack("!Q", rfile.read(8))[0]
    if n > 1 << 16:                     # der Browser schickt nur kurze Bestätigungen
        return None
    mask = rfile.read(4) if h[1] & 0x80 else b"\0\0\0\0"
    data = bytearray(rfile.read(n))
    if len(data) < n:
        return None
    for i in range(n):
        data[i] ^= mask[i % 4]
    return opcode, bytes(data)


def serve(app: App, host: str, port: int, fps: int, render_cpus=None) -> ThreadingHTTPServer:
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body: bytes, ctype="application/json"):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)
            if u.path == "/":
                return self._send(200, json.dumps({"xgonav": "Schnittstelle, Oberfläche unter /xgonav/ (nginx)", "api": API}).encode())
            if u.path == "/status":
                s = app.snapshot
                body = {"mode": app.mode, "pipeline": "läuft" if app.pipeline_on else "gestoppt",
                        "status": app._cmd.status if not app.pipeline_on else s.cmd.status, "target": s.cmd.target_id,
                        "cmd": {"v": round(s.sent[0], 3), "w": round(s.sent[1], 3)},
                        "tracks": [{"id": t.id, "cls": t.cls, "conf": round(t.conf, 2),
                                    "box": [round(x) for x in t.box]} for t in s.tracks],
                        "free": [round(x, 2) for x in (s.free.sectors if s.free else [])],
                        "ttc": round(s.ego.ttc, 2) if s.ego and s.ego.ttc < 99 else None,
                        **s.stats}
                body["mode"] = app.mode       # stats stammt vom letzten Bild und kann veraltet sein
                body["start"] = START
                if parse_qs(u.query).get("breakdown"):
                    body["loop_breakdown"] = app.loop_breakdown()
                e = s.ego                     # Werte der früheren Overlay-Textzeilen
                body["ego"] = None if e is None else {
                    "yaw_dps": round(math.degrees(e.yaw_rate), 1), "pitch_dps": round(math.degrees(e.pitch_rate), 1),
                    "sharp": round(e.sharpness)}
                body["frame"] = s.idx         # Bildzähler (Diagnose: tatsächliche Bildrate)
                body["frame_t"] = round(s.t, 4)
                meta = s.cam_meta or {}
                body["exposure"] = {"soll_us": app.src.exposure_us, "max_us": app.src.exposure_max_us,
                                    "ist_us": meta.get("exp_us"), "gain": meta.get("gain"),
                                    "gain_soll": app.src.gain, "gain_max": app.src.GAIN_MAX,
                                    "frame_us": meta.get("frame_us"), "crop": meta.get("crop")}
                if not app.pipeline_on:
                    # ohne Bildverarbeitung gibt es keine neuen Snapshots: Roboterwerte live
                    st = app.robot.state
                    body.update({"cmd": {"v": round(app.robot.command[0], 3), "w": round(app.robot.command[1], 3)},
                                 "battery": st.battery, "balancing": st.balancing, "yaw_drv": st.yaw_deg,
                                 "tracks": [], "free": [], "ttc": None})
                return self._send(200, json.dumps(body).encode())
            if u.path == "/map.png":
                return self._map_png(parse_qs(u.query))
            if u.path == "/ws":
                return self._ws()
            if u.path == "/frame":
                return self._frame(parse_qs(u.query))
            if u.path == "/stream":
                return self._stream()
            self._send(404, b"{}")

        def do_POST(self):
            u = urlparse(self.path)
            q = parse_qs(u.query)
            if u.path == "/stop":
                app.set_mode("idle")
                app.robot.stop()
                return self._send(200, b'{"ok":true}')
            if u.path == "/pipeline":
                app.set_pipeline(q.get("on", ["1"])[0] not in ("0", "false", "aus"))
                return self._send(200, json.dumps({"ok": True, "pipeline": app.pipeline_on}).encode())
            if u.path == "/drive":
                try:
                    v, w = float(q.get("v", ["0"])[0]), float(q.get("w", ["0"])[0])
                except ValueError:
                    return self._send(400, b'{"error":"v/w ungueltig"}')
                try:
                    app.set_manual(v, w)
                except ValueError as e:   # ModeNotAllowed
                    return self._send(409, json.dumps({"error": str(e)}).encode())
                return self._send(200, b'{"ok":true}')
            if u.path == "/map/reset":
                if app.gridmap is None:
                    return self._send(404, b'{"error":"keine Raumkarte"}')
                app.gridmap.reset()
                return self._send(200, b'{"ok":true}')
            if u.path == "/map/start":
                if app.gridmap is None:
                    return self._send(404, b'{"error":"keine Raumkarte"}')
                app.gridmap.set_start()
                return self._send(200, b'{"ok":true}')
            if u.path == "/map/save":
                if app.gridmap is None or not app.cfg.grid.file:
                    return self._send(404, b'{"error":"keine Raumkarte oder [grid] file leer"}')
                ok = app.gridmap.save(app.cfg.grid.file)
                return self._send(200 if ok else 500, json.dumps({"ok": ok, "file": app.cfg.grid.file}).encode())
            if u.path == "/exposure":
                try:
                    us = app.src.set_exposure(int(q.get("us", [""])[0]))
                except ValueError as e:
                    return self._send(400, json.dumps({"error": str(e) or "us ungültig"}).encode())
                return self._send(200, json.dumps({"ok": True, "exposure_us": us}).encode())
            if u.path == "/gain":
                try:
                    g = app.src.set_gain(float(q.get("g", [""])[0]))
                except ValueError as e:
                    return self._send(400, json.dumps({"error": str(e) or "g ungültig"}).encode())
                return self._send(200, json.dumps({"ok": True, "gain": g}).encode())
            if u.path == "/mode":
                try:
                    app.set_mode(q.get("name", ["idle"])[0], (q.get("target") or [None])[0] or None)
                except KeyError:
                    return self._send(400, b'{"error":"unbekannter Modus"}')
                except ValueError as e:   # ModeNotAllowed
                    return self._send(409, json.dumps({"error": str(e)}).encode())
                return self._send(200, b'{"ok":true}')
            self._send(404, b"{}")

        def _map_png(self, q):
            gm = app.gridmap
            if gm is None:
                return self._send(404, b'{"error":"keine Raumkarte (Bodenmodell oder [grid] aus)"}')
            try:
                px = min(960, max(120, int(q.get("px", ["480"])[0])))
                view = min(gm.cfg.size_m, max(1.0, float(q.get("view", [str(gm.cfg.view_m)])[0])))
            except ValueError:
                return self._send(400, b'{"error":"px/view ungueltig"}')
            pin_thread(render_cpus, "Bild-Thread", quiet=True)
            ok, png = cv2.imencode(".png", gm.image(px, view))
            if not ok:
                return self._send(500, b'{"error":"PNG"}')
            return self._send(200, png.tobytes(), "image/png")

        def _ws(self):
            """Bilder als WebSocket-Stream. Nach jedem Bild wartet der Server auf eine
            Bestätigung des Browsers (beliebige Nachricht) und schickt dann das neueste Bild.
            So bleibt es bei einer Verbindung, ohne dass sich Bilder in TCP-Puffern stauen
            (MJPEG über schwaches WLAN: bis 20 s). Binärnachricht: 4 Byte Bildnummer + JPEG."""
            pin_thread(render_cpus, "Bild-Thread", quiet=True)
            key = self.headers.get("Sec-WebSocket-Key")
            if not key or "websocket" not in self.headers.get("Upgrade", "").lower():
                return self._send(400, b'{"error":"WebSocket erwartet"}')
            self.send_response(101, "Switching Protocols")
            self.send_header("Upgrade", "websocket")
            self.send_header("Connection", "Upgrade")
            self.send_header("Sec-WebSocket-Accept", ws_accept(key))
            self.end_headers()
            self.close_connection = True
            sock = self.connection
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(WS_ACK_TIMEOUT_S)
            last = -1
            period = 1.0 / max(1, fps)
            try:
                while app.running:
                    t0 = time.monotonic()
                    with app.snap_cond:
                        app.snap_cond.wait_for(lambda: app.snapshot.idx != last or not app.running, timeout=1.0)
                        snap = app.snapshot
                    if not app.running:
                        break
                    # bei gestoppter Bildverarbeitung: dasselbe Bild höchstens einmal pro Sekunde
                    img = render(snap, text=False)   # Statuszeilen zeigt die Oberfläche
                    if img is None:
                        continue
                    ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    if not ok:
                        continue
                    last = snap.idx
                    self.wfile.write(ws_frame(0x2, struct.pack("!I", snap.idx & 0xFFFFFFFF) + jpg.tobytes()))
                    # auf Bestätigung warten; Ping beantworten, Close beenden
                    while True:
                        m = ws_read(self.rfile)
                        if m is None or m[0] == 0x8:
                            return
                        if m[0] == 0x9:
                            self.wfile.write(ws_frame(0xA, m[1]))
                            continue
                        break
                    rest = period - (time.monotonic() - t0)
                    if rest > 0:
                        time.sleep(rest)
            except (OSError, struct.error):     # Verbindung weg, Timeout ohne Bestätigung
                pass

        def _frame(self, q):
            """Ein JPEG pro Anfrage. Der Browser holt das nächste erst, wenn dieses angekommen
            ist: kein Rückstau in TCP-Puffern (beim MJPEG-Stream über schwaches WLAN bis 20 s).
            after=<idx>: bis 1 s auf ein neueres Bild warten. Header X-Frame = Bildnummer."""
            pin_thread(render_cpus, "Bild-Thread", quiet=True)
            try:
                after = int(q.get("after", ["-1"])[0])
            except ValueError:
                after = -1
            with app.snap_cond:
                app.snap_cond.wait_for(lambda: app.snapshot.idx != after or not app.running, timeout=1.0)
                snap = app.snapshot
            img = render(snap, text=False)   # Statuszeilen zeigt die Oberfläche
            if img is None:
                return self._send(503, b'{"error":"kein Bild"}')
            ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if not ok:
                return self._send(500, b'{"error":"JPEG"}')
            body = jpg.tobytes()
            try:
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Frame", str(snap.idx))
                self.end_headers()
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def _stream(self):
            pin_thread(render_cpus, "Bild-Thread", quiet=True)
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=f")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            last = -1
            period = 1.0 / max(1, fps)
            try:
                while app.running:
                    with app.snap_cond:
                        app.snap_cond.wait_for(lambda: app.snapshot.idx != last or not app.running, timeout=1.0)
                        snap = app.snapshot
                    if snap.idx == last:
                        continue
                    last = snap.idx
                    img = render(snap, text=False)   # Statuszeilen zeigt die Oberfläche
                    if img is None:
                        continue
                    ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    if not ok:
                        continue
                    self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpg))
                    self.wfile.write(jpg.tobytes())
                    self.wfile.write(b"\r\n")
                    time.sleep(period)
            except (BrokenPipeError, ConnectionResetError):
                pass

    srv = ThreadingHTTPServer((host, port), H)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, name="web", daemon=True).start()
    log.info("Schnittstelle für die Weboberfläche: http://%s:%d/", host, port)
    return srv
