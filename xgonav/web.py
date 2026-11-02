"""Lokale Schnittstelle: MJPEG-Overlay, Status-JSON, Moduswechsel, Bildverarbeitung an/aus.

Die Bedienoberfläche ist PHP (web/), ausgeliefert von nginx (deploy/nginx-xgonav.conf);
nginx reicht /xgonav/stream direkt hierher durch. Keine Authentifizierung: web_host
auf 127.0.0.1 lassen und nur über nginx veröffentlichen.
"""
from __future__ import annotations

import json
import logging
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2

from .app import App, render

log = logging.getLogger(__name__)
START = time.time()      # Startzeit des Prozesses: Oberfläche verbindet den Stream nach Neustart neu

# Schnittstelle für die PHP-Oberfläche (web/) bzw. nginx. Keine HTML-Seite mehr hier.
API = {"GET": ["/status", "/stream"],
       "POST": ["/mode?name=..&target=..", "/stop", "/pipeline?on=0|1", "/drive?v=..&w=.. (nur Modus manual)",
                "/exposure?us=0|100..33333 (0 = Automatik)", "/gain?g=0|1..16 (0 = Automatik)"]}


def serve(app: App, host: str, port: int, fps: int) -> ThreadingHTTPServer:
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

        def _stream(self):
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
                    img = render(snap)
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
