"""Lokale Schnittstelle: Overlay-Bilder (WebSocket, Einzelbild, MJPEG), Status-JSON,
Moduswechsel, Bildverarbeitung an/aus.

Die Bedienoberfläche ist PHP (web/), ausgeliefert von nginx (deploy/nginx-xgonav.conf);
nginx reicht /ws und /frame direkt hierher durch. Keine Authentifizierung: web_host
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
from .app import App, ModeNotAllowed, render
from .route import parse_points, parse_zones

log = logging.getLogger(__name__)
START = time.time()      # Startzeit des Prozesses: Oberfläche verbindet den Stream nach Neustart neu

def horizont_linie(app, snap) -> tuple[float, float, float]:
    """Cyan Horizontlinie fürs Overlay: (Zeile bei cx, Roll in Grad, cx), Vollbild-Pixel. Zeile aus dem Nick
    des Bildes (pitch_dev wie in der Geometrie, ungeklemmt; das Bodenmodell klemmt seine auf 20..80 %), Roll
    aus dem IMU-Wert des Bildes (Snapshot.imu, 0 ohne IMU). Ein tan je gesendetem Bild im Web-Thread."""
    roll = snap.imu.roll_deg if snap.imu is not None else 0.0
    return float(app.geo.horizon_row(snap.pitch_dev)), float(roll), float(app.geo.cx)


def _hz_rows(snap, geo) -> tuple[float, float]:
    """row_stand und row_imu (Vollbild-Zeilen) für „Bild am IMU-Nick festhalten“, fehlend NaN.
    Normallage gegen den IMU-Nick des Bildes (Snapshot.imu); zwei tan je gesendetem Bild im Web-Thread, die Bildpipeline rechnet nichts."""
    if snap.imu is None or snap.imu.pitch is None:
        return float("nan"), float("nan")
    return float(geo.horizon_row(0.0)), float(geo.horizon_row(snap.imu.pitch))


NAN = float("nan")


def ws_kopf(snap, geo, hl) -> bytes:
    """Kopf je Bild (/ws?nick=2), big endian: Bildnummer u32, dann 10 float32: row_stand, row_imu, Roll (Grad,
    + = rechts tief), cx, IMU-Nick (Grad, wie pitch_dev), IMU-Kurs (Grad), Alter der IMU-Probe (ms), Nickrate aus
    dem Bild und aus der IMU (°/s), Zustand (1 = IMU gültig, 0 = ohne IMU: Nick nur aus dem Bild, NaN = keine IMU).
    Alles aus dem Snapshot, nichts neu gerechnet."""
    fr, nr = snap.imu, snap.nickrate
    w = (*_hz_rows(snap, geo), hl[1], hl[2],
         NAN if fr is None or fr.pitch is None else math.degrees(fr.pitch),
         NAN if fr is None or fr.yaw is None else math.degrees(fr.yaw),
         NAN if fr is None or fr.alter_ms is None else fr.alter_ms,
         NAN if nr is None else nr[0], NAN if nr is None else nr[1],
         NAN if fr is None else float(fr.ok))
    return struct.pack("!I10f", snap.idx & 0xFFFFFFFF, *w)


API = {"GET": ["/status", "/ws?nick=2 (WebSocket: Bilder, Bestätigung je Bild; nick=2: IMU-Wert des Bildes im Kopf, ws_kopf)",
               "/frame?after=<idx>&raw=1",
               "/map.png?px=480&view=6 (Raumkarte)",
               "/detector (wählbare Ziele für follow/approach: Klassen des Detektors)",
               "/floornet (wählbare Bodenmodelle in [freespace] net_dir, aktives und geladenes)"],
       "POST": ["/mode?name=..&target=..", "/stop", "/action?n=3|4|5 (Fahr-Aktion der Firmware, nur idle; Bahnprüfung bleibt)","/pipeline?on=0|1", "/ruhe?on=0|1 (Ruhemodus; Wecken startet nichts)", "/wlan?on=0|1 (WLAN-Einrichtung: QR-Code, sonst Hotspot; wie Tasten C+D halten)", "/record?on=0|1 (Rohaufzeichnung, nur balancierend)",
                "/drive?v=..&w=.. (nur Modus manual)",
                "/exposure?us=0|100..max (0 = Automatik)", "/gain?g=0|1..63.9 (0 = Automatik)",
                "/map/reset (Raumkarte leeren, Pose auf 0)", "/map/pose?aktion=start|hier|verwerfen (Standort bestätigen)", "/level (Beine neu ausrichten, 3 s)",
                "/floornet?name=<datei>.bin (Bodenmodell wechseln: laden, prüfen, Symlink [freespace] net umstellen; nur idle/manual)"]}


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

        def _send(self, code, body: bytes, ctype="application/json", extra: dict | None = None):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)
            if u.path == "/":
                return self._send(200, json.dumps({"xgonav": "Schnittstelle, Oberfläche unter / (nginx)", "api": API}).encode())
            if u.path == "/status":
                s = app.snapshot
                body = {"mode": app.mode, "pipeline": "läuft" if app.pipeline_on else "gestoppt", "ruhe": app.ruhe,
                        "status": app._cmd.status if not app.pipeline_on else s.cmd.status, "target": s.cmd.target_id,
                        "cmd": {"v": round(s.sent[0], 3), "w": round(s.sent[1], 3)},
                        "tracks": [{"id": t.id, "cls": t.cls, "conf": round(t.conf, 2),
                                    "box": [round(x) for x in t.box]} for t in s.tracks],
                        "free": [round(x, 2) for x in (s.free.sectors if s.free else [])],
                        "ttc": round(s.ego.ttc, 2) if s.ego and s.ego.ttc < 99 else None,
                        **s.stats}
                body["mode"] = app.mode       # stats stammt vom letzten Bild und kann veraltet sein
                w = app.wlan
                body["wlan"] = w.status() if w is not None else None
                body["ortung"] = app.ortung.status() if app.ortung is not None else None
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
                                 "height_mm": app.robot.height_mm,
                                 "tracks": [], "free": [], "ttc": None})
                # Kamerahöhe für den Regler, nur zur Anfragezeit gerechnet: robot.height_mm kennt anders als
                # /proc die Mittelstellung nach dem Aufstehen und die Begrenzung der Firmware
                h = app.robot.height_mm
                body["cam_h_gerade"] = [round(x, 4) for x in app.geo.cam_h_line()]
                body["height_u8"] = None if h is None else app.geo.body_to_u8(h)
                body["cam_h_mm"] = None if h is None else round(app.geo.cam_h_at(h), 1)
                body["aufzeichnung"] = None if app.rawrec is None else app.rawrec.status()
                return self._send(200, json.dumps(body).encode())
            if u.path == "/route":
                gm = app.gridmap
                if gm is None:
                    return self._send(404, b'{"error":"keine Raumkarte"}')
                with gm._lock:
                    body = {"points": gm.route, "loop": gm.route_loop, "zones": gm.zones, "map_id": gm.map_id, "gesperrt": gm.locked}
                return self._send(200, json.dumps(body).encode())
            if u.path == "/detector":
                d = app.det                   # Klassen nur auf Anfrage (Seitenaufruf, Neustart), nicht je Bild
                cls = d.classes() if hasattr(d, "classes") else []
                body = {"on": bool(cls), "geladen": getattr(d, "model", None) is not None,
                        "backend": getattr(d, "backend", ""), "classes": cls, "modes": sorted(app.DET_MODES),
                        "default": app.cfg.behavior.target_class}
                return self._send(200, json.dumps(body).encode())
            if u.path == "/floornet":
                return self._send(200, json.dumps(app.floornet_info()).encode())
            if u.path == "/map.png":
                return self._map_png(parse_qs(u.query))
            if u.path == "/ws":
                return self._ws(parse_qs(u.query))
            if u.path == "/frame":
                return self._frame(parse_qs(u.query))
            self._send(404, b"{}")

        def do_POST(self):
            u = urlparse(self.path)
            q = parse_qs(u.query)
            if u.path == "/stop":
                # nur=pet: api.php beim Absetzen. Sonst stünde er nach einem Absetzen während der
                # Ohnmacht von selbst wieder auf; die anderen Modi behalten ihr bisheriges Verhalten.
                if q.get("nur", [""])[0] == "pet" and app.mode != "pet":
                    return self._send(200, b'{"ok":true}')
                app.set_mode("idle")
                app.robot.stop()
                return self._send(200, b'{"ok":true}')
            if u.path == "/action":
                # Fahr-Aktionen der Firmware (3, 4, 5): Keepalive aus, Bahnprüfung bleibt (App.fw_action)
                try:
                    app.fw_action(int(q.get("n", [""])[0]))
                except ModeNotAllowed as e:
                    return self._send(409, json.dumps({"error": str(e)}).encode())
                except (KeyError, ValueError):
                    return self._send(400, b'{"error":"Aktion ungueltig (3, 4, 5)"}')
                return self._send(200, b'{"ok":true}')
            if u.path == "/floornet":
                try:
                    name = app.floornet_wechseln(q.get("name", [""])[0])
                except ModeNotAllowed as e:
                    return self._send(409, json.dumps({"error": str(e)}).encode())
                except ValueError as e:
                    return self._send(400, json.dumps({"error": str(e)}).encode())
                except OSError as e:
                    return self._send(500, json.dumps({"error": f"Symlink nicht umstellbar: {e}"}).encode())
                return self._send(200, json.dumps({"ok": True, "geladen": name}).encode())
            if u.path == "/level":
                if not app.robot.request_level():
                    return self._send(409, b'{"error":"Rider balanciert nicht"}')
                return self._send(200, b'{"ok":true}')
            if u.path == "/pipeline":
                try:
                    app.set_pipeline(q.get("on", ["1"])[0] not in ("0", "false", "aus"))
                except ModeNotAllowed as e:
                    return self._send(409, json.dumps({"error": str(e)}).encode())
                return self._send(200, json.dumps({"ok": True, "pipeline": app.pipeline_on}).encode())
            if u.path == "/ruhe":
                app.set_ruhe(q.get("on", ["1"])[0] not in ("0", "false", "aus"))
                return self._send(200, json.dumps({"ok": True, "ruhe": app.ruhe}).encode())
            if u.path == "/wlan":
                app.wlan_request(q.get("on", ["1"])[0] not in ("0", "false", "aus"))
                return self._send(200, b'{"ok":true}')
            if u.path == "/record":
                if app.rawrec is None:
                    return self._send(404, b'{"error":"keine Aufzeichnung"}')
                app.rawrec.schalten(q.get("on", ["1"])[0] != "0")
                return self._send(200, json.dumps({"ok": True, "aufzeichnung": app.rawrec.status()}).encode())
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
            if u.path == "/route":
                if app.gridmap is None:
                    return self._send(404, b'{"error":"keine Raumkarte"}')
                if app.mode == "patrol":
                    return self._send(409, json.dumps({"error": "nicht während der Streife"}).encode())
                try:
                    pts = parse_points(q.get("pts", [""])[0])
                    zones = parse_zones(q["zones"][0]) if "zones" in q else ([] if "nozones" in q else None)
                except ValueError as e:
                    return self._send(400, json.dumps({"error": "Route ungültig: " + str(e)}).encode())
                errors = app.set_route(pts, q.get("loop", ["0"])[0] == "1", zones)
                if errors:
                    return self._send(409, json.dumps({"error": "; ".join(errors)}).encode())
                return self._send(200, b'{"ok":true}')
            if u.path == "/map/lock":
                if app.gridmap is None:
                    return self._send(404, b'{"error":"keine Raumkarte"}')
                if app.mode == "patrol":
                    return self._send(409, json.dumps({"error": "nicht während der Streife"}).encode())
                app.gridmap.lock(q.get("on", ["1"])[0] != "0")
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
            if u.path == "/map/pose":
                try:
                    app.set_pose_state(q.get("aktion", [""])[0])
                except KeyError:
                    return self._send(400, json.dumps({"error": "aktion: start, hier oder verwerfen"}).encode())
                except ModeNotAllowed as e:
                    return self._send(409, json.dumps({"error": str(e)}).encode())
                return self._send(200, b'{"ok":true}')
            if u.path == "/map/save":
                if app.gridmap is None or not app.cfg.grid.file:
                    return self._send(404, b'{"error":"keine Raumkarte oder [grid] file leer"}')
                ok = app.gridmap.save(app.cfg.grid.file)
                return self._send(200 if ok else 500, json.dumps({"ok": ok, "file": app.cfg.grid.file}).encode())
            if u.path == "/exposure":
                try:
                    us = app.src.set_exposure(int(q.get("us", [""])[0]))
                    app.reset_floor()      # andere Belichtung: Bodengrenze neu beginnen
                except ValueError as e:
                    return self._send(400, json.dumps({"error": str(e) or "us ungültig"}).encode())
                return self._send(200, json.dumps({"ok": True, "exposure_us": us}).encode())
            if u.path == "/gain":
                try:
                    g = app.src.set_gain(float(q.get("g", [""])[0]))
                    app.reset_floor()
                except ValueError as e:
                    return self._send(400, json.dumps({"error": str(e) or "g ungültig"}).encode())
                return self._send(200, json.dumps({"ok": True, "gain": g}).encode())
            if u.path == "/mode":
                try:
                    app.set_mode(q.get("name", ["idle"])[0], (q.get("target") or [None])[0] or None,
                                 confirmed=q.get("bestaetigt", ["0"])[0] == "1")
                except KeyError:
                    return self._send(400, b'{"error":"unbekannter Modus"}')
                except ValueError as e:   # ModeNotAllowed
                    return self._send(409, json.dumps({"error": str(e)}).encode())
                return self._send(200, b'{"ok":true}')
            self._send(404, b"{}")

        def _merk(self):
            """Punkte der Merkkarte für die Raumkarte (alle, bestätigte); None ohne laufende Bildverarbeitung.
            Nur auf Anfrage (Browser, 1x je s) und im Bild-Thread; die Felder der Merkkarte werden nie
            verändert, nur ersetzt."""
            lm = getattr(app, "lmap", None)
            if lm is None or not app.pipeline_on:
                return None
            pts = lm.pts
            if not len(pts):
                return None
            try:
                best = lm.confirmed_later(lm.max_range)()
            except Exception:                              # Felder mitten im Wechsel: dieses Bild ohne
                best = None
            return pts, best

        def _map_png(self, q):
            gm = app.gridmap
            if gm is None:
                return self._send(404, b'{"error":"keine Raumkarte (Bodenmodell oder [grid] aus)"}')
            try:
                px = min(960, max(120, int(q.get("px", ["480"])[0])))
                view = min(gm.cfg.size_m, max(1.0, float(q.get("view", [str(gm.cfg.view_m)])[0])))
                center = None
                if "cx" in q or "cy" in q:                 # ohne beide: Mitte = aktuelle Pose
                    center = (float(q["cx"][0]), float(q["cy"][0]))
                    if not all(math.isfinite(c) for c in center):
                        raise ValueError
                    center = tuple(min(gm.cfg.size_m, max(-gm.cfg.size_m, c)) for c in center)
            except (ValueError, KeyError):
                return self._send(400, b'{"error":"px/view/cx/cy ungueltig"}')
            pin_thread(render_cpus, "Bild-Thread", quiet=True)
            img, mx, my, kante = gm.render(px, view, center, self._merk())
            ok, png = cv2.imencode(".png", img)
            if not ok:
                return self._send(500, b'{"error":"PNG"}')
            # Für die Oberfläche: was das Bild wirklich zeigt (Verschieben/Zoomen ohne Warten)
            return self._send(200, png.tobytes(), "image/png",
                              {"X-Map-View": f"{mx:.3f},{my:.3f},{kante:.3f},{gm.cfg.size_m:.1f}"})

        def _ws(self, q):
            """Bilder als WebSocket-Stream. Nach jedem Bild wartet der Server auf eine
            Bestätigung des Browsers (beliebige Nachricht) und schickt dann das neueste Bild.
            So bleibt es bei einer Verbindung, ohne dass sich Bilder in TCP-Puffern stauen
            (MJPEG über schwaches WLAN: bis 20 s). Binärnachricht: 4 Byte Bildnummer + JPEG.
            nick=2: Kopf aus ws_kopf (IMU-Wert des Bildes: Zeilen, Roll, cx, Nick, Kurs, Alter, Nickraten, Zustand);
            der Browser verschiebt/dreht damit und zeigt die Werte je Bild an."""
            pin_thread(render_cpus, "Bild-Thread", quiet=True)
            nick = q.get("nick", ["0"])[0] == "2"
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
                    if (app.ruhe or not app.pipeline_on) and snap.idx == last:
                        continue                    # Ruhe/Bildverarbeitung aus: dasselbe Bild nicht neu kodieren
                    hl = horizont_linie(app, snap)
                    img = render(snap, text=False, horizont=hl)   # Statuszeilen zeigt die Oberfläche
                    if img is None:
                        continue
                    ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    if not ok:
                        continue
                    last = snap.idx
                    kopf = ws_kopf(snap, app.geo, hl) if nick else struct.pack("!I", snap.idx & 0xFFFFFFFF)
                    self.wfile.write(ws_frame(0x2, kopf + jpg.tobytes()))
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
            raw = q.get("raw", ["0"])[0] == "1"     # Rohbild ohne Overlay (Kalibrierung)
            img = snap.frame if raw else render(snap, text=False, horizont=horizont_linie(app, snap))   # Statuszeilen zeigt die Oberfläche
            if img is None:
                return self._send(503, b'{"error":"kein Bild"}')
            ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 95 if raw else 70])
            if not ok:
                return self._send(500, b'{"error":"JPEG"}')
            body = jpg.tobytes()
            try:
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Frame", str(snap.idx))
                self.send_header("X-Frame-T", "%.6f" % snap.t)
                self.end_headers()
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

    srv = ThreadingHTTPServer((host, port), H)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, name="web", daemon=True).start()
    log.info("Schnittstelle für die Weboberfläche: http://%s:%d/", host, port)
    return srv
