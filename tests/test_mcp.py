"""web/mcp.php: MCP-Server (Streamable HTTP) gegen ein nachgestelltes xgonav. Ohne php übersprungen."""
import base64
import json
import os
import shutil
import socket
import subprocess
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import pytest

ROOT = Path(__file__).resolve().parent.parent
PHP = shutil.which("php")
pytestmark = pytest.mark.skipif(PHP is None, reason="kein php im PATH")

JPEG = b"\xff\xd8\xff\xe0JFIF-Attrappe\xff\xd9"
PNG = b"\x89PNG\r\n\x1a\nAttrappe"


def _freier_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class Xgonav:
    """Nachgestellte lokale Schnittstelle: merkt sich Aufrufe, dreht den Kurs mit jedem /drive."""

    def __init__(self):
        self.mode, self.pipeline, self.frame, self.yaw, self.m = "idle", True, 100, 0.0, 0.0
        self.calls: list[str] = []
        self.meldung = ""
        self.neu = True           # kennt /drive?sicher=1
        x = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _send(self, code, body, ctype="application/json", extra=None):
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                for k, v in (extra or {}).items():
                    self.send_header(k, v)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                u = urlparse(self.path)
                x.calls.append("GET " + self.path)
                if u.path == "/":
                    return self._send(200, json.dumps({"api": {"POST": ["/drive?v=..&w=..&sicher=0|1" if x.neu else "/drive?v=..&w=.."]}}).encode())
                if u.path == "/status":
                    x.frame += 1
                    return self._send(200, json.dumps({
                        "mode": x.mode, "pipeline": "läuft" if x.pipeline else "gestoppt", "ruhe": False,
                        "status": "Handsteuerung" + x.meldung, "frame": x.frame, "yaw_deg": round(x.yaw, 1),
                        "pitch_deg": 0.5, "free": [1.0, 0.8, 0.2], "fs_src": "net", "odom": {"m": round(x.m, 2)},
                        "grid": {"pose": [x.m, 0.0, x.yaw], "start": [0, 0, 0], "pose_ok": True, "zum_start_m": x.m},
                        "tracks": [], "cam_h_mm": 123.8, "height_mm": 90.0, "battery": 80}).encode())
                if u.path == "/frame":
                    return self._send(200, JPEG, "image/jpeg", {"X-Frame": str(x.frame)})
                if u.path == "/map.png":
                    return self._send(200, PNG, "image/png", {"X-Map-View": "1.000,2.000,6.000,24.0"})
                if u.path == "/route":
                    return self._send(200, b'{"points": [[1, 2]], "loop": false, "zones": []}')
                if u.path == "/detector":
                    return self._send(200, b'{"on": true, "classes": ["person"]}')
                self._send(404, b"{}")

            def do_POST(self):
                u = urlparse(self.path)
                q = parse_qs(u.query)
                x.calls.append("POST " + self.path)
                if u.path == "/mode":
                    x.mode = q["name"][0]
                elif u.path == "/stop":
                    x.mode = "idle"
                elif u.path == "/pipeline":
                    x.pipeline = q["on"][0] == "1"
                elif u.path == "/drive":
                    if x.mode != "manual":
                        return self._send(409, json.dumps({"error": "Handsteuerung nur im Modus manual"}).encode())
                    x.yaw += 60.0 * float(q["w"][0])          # je Sollwert: bei w 0,25 also 15°
                    x.m += 0.1 * float(q["v"][0])
                self._send(200, b'{"ok":true}')

        self.srv = ThreadingHTTPServer(("127.0.0.1", 0), H)
        self.port = self.srv.server_address[1]
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()


class Mcp:
    def __init__(self, port: int):
        self.url, self.n = f"http://127.0.0.1:{port}/mcp.php", 0

    def roh(self, body: bytes, method="POST", kopf=None):
        r = urllib.request.Request(self.url, data=body if method == "POST" else None, method=method,
                                   headers={"Content-Type": "application/json", "Accept": "application/json, text/event-stream",
                                            **(kopf or {})})
        try:
            with urllib.request.urlopen(r, timeout=60) as f:
                return f.status, f.read()
        except urllib.error.HTTPError as e:
            return e.code, e.read()

    def rpc(self, method: str, params=None, kopf=None) -> dict:
        self.n += 1
        code, body = self.roh(json.dumps({"jsonrpc": "2.0", "id": self.n, "method": method, "params": params or {}}).encode(),
                              kopf=kopf)
        assert code == 200, (code, body)
        d = json.loads(body)
        assert d["id"] == self.n
        return d

    def call(self, name: str, **arg) -> dict:
        return self.rpc("tools/call", {"name": name, "arguments": arg})["result"]


@pytest.fixture
def mcp(tmp_path):
    procs = []

    def start(stufe: str | None = "fahren", token: str = "", state: int = 1):
        proc = tmp_path / "proc"
        (proc / "leds").mkdir(parents=True, exist_ok=True)
        for name, wert in (("state", state), ("battery", 77), ("height", 128), ("roll", 0), ("action", 0)):
            (proc / name).write_text(f"{wert}\n")
        conf = tmp_path / "mcp.json"
        if stufe is not None:
            conf.write_text(json.dumps({"stufe": stufe, "token": token}))
        x = Xgonav()
        port = _freier_port()
        env = {**os.environ, "XGO_MCP_XGONAV": f"http://127.0.0.1:{x.port}", "XGO_PROC": str(proc),
               "XGO_MCP_CONF": str(conf), "XGO_MCP_SCHNELL": "20", "PHP_CLI_SERVER_WORKERS": "3"}
        p = subprocess.Popen([PHP, "-S", f"127.0.0.1:{port}", "-t", str(ROOT / "web")], env=env,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(p)
        for _ in range(100):
            try:
                socket.create_connection(("127.0.0.1", port), timeout=0.1).close()
                break
            except OSError:
                time.sleep(0.05)
        return Mcp(port), x, proc

    yield start
    for p in procs:
        p.kill()
        p.wait()


def test_protokoll(mcp):
    m, _, _ = mcp()
    r = m.rpc("initialize", {"protocolVersion": "2025-03-26", "capabilities": {}, "clientInfo": {"name": "t", "version": "0"}})
    assert r["result"]["protocolVersion"] == "2025-03-26" and "tools" in r["result"]["capabilities"]
    assert "fahren" in r["result"]["instructions"]
    assert m.rpc("initialize", {"protocolVersion": "1999-01-01"})["result"]["protocolVersion"] == "2025-06-18"
    assert m.roh(b'{"jsonrpc":"2.0","method":"notifications/initialized"}') == (202, b"")
    assert m.rpc("ping")["result"] == {}
    assert m.rpc("gibt/es/nicht")["error"]["code"] == -32601
    assert m.roh(b"", method="GET")[0] == 405
    assert m.roh(b"{kaputt")[0] == 400
    assert m.roh(b"{}", kopf={"Content-Type": "text/plain"})[0] == 415      # Formular-POST einer fremden Webseite
    tools = {t["name"]: t for t in m.rpc("tools/list")["result"]["tools"]}
    assert {"status", "bild", "karte", "rundblick", "drehen", "fahren", "modus", "stopp", "aufstehen", "absetzen"} <= set(tools)
    for t in tools.values():
        assert t["inputSchema"]["type"] == "object" and isinstance(t["inputSchema"]["properties"], dict)
        assert t["description"]
    assert tools["bild"]["annotations"]["readOnlyHint"] and not tools["fahren"]["annotations"]["readOnlyHint"]


def test_schauen(mcp):
    m, x, _ = mcp()
    s = json.loads(m.call("status")["content"][0]["text"])
    assert s["modus"] == "idle" and s["balanciert"] and s["akku_prozent"] == 77 and s["freigabe"] == "fahren"
    assert s["karte"]["pose_x_y_grad"] == [0.0, 0.0, 0.0]
    r = m.call("bild")
    assert not r["isError"] and [c["type"] for c in r["content"]] == ["text", "image"]
    assert base64.b64decode(r["content"][1]["data"]) == JPEG and r["content"][1]["mimeType"] == "image/jpeg"
    assert any(c.startswith("GET /frame?raw=1&after=") for c in x.calls)
    m.call("bild", overlay=True)
    assert any(c.startswith("GET /frame?raw=0") for c in x.calls)
    r = m.call("karte", ausschnitt_m=8)
    assert base64.b64decode(r["content"][1]["data"]) == PNG and r["content"][1]["mimeType"] == "image/png"
    info = json.loads(r["content"][0]["text"])
    assert info["ausschnitt"] == {"mitte_x": 1.0, "mitte_y": 2.0, "kante_m": 6.0} and info["route"]["punkte"] == [[1, 2]]
    x.pipeline = False
    r = m.call("bild")
    assert r["isError"] and "bildverarbeitung" in r["content"][0]["text"]      # kein altes Bild als aktuell ausgeben


def test_freigabe(mcp):
    m, x, proc = mcp(stufe=None)                                               # keine Datei: nur schauen
    assert not m.call("status")["isError"]
    for name, arg in (("drehen", {"grad": 30}), ("aufstehen", {}), ("fahren", {"v": 0.1, "sekunden": 1}),
                      ("modus", {"name": "explore"})):
        r = m.call(name, **arg)
        assert r["isError"] and "Nicht freigegeben" in r["content"][0]["text"]
    assert not any(c.startswith("POST") for c in x.calls)
    assert (proc / "action").read_text().strip() == "0"
    assert not m.call("stopp")["isError"] and "POST /stop" in x.calls           # Stopp geht immer
    r = m.call("ruhemodus", an=True)                                            # schlafen legt ihn ab: braucht drehen
    assert r["isError"] and "Nicht freigegeben" in r["content"][0]["text"] and "POST /ruhe?on=1" not in x.calls
    m.call("ruhemodus", an=False)                                               # wecken geht immer (sudo fehlt im Test)
    assert "POST /ruhe?on=0" in x.calls


def test_freigabe_drehen_ohne_fahren(mcp):
    m, x, _ = mcp(stufe="drehen")
    assert m.call("fahren", v=0.1, sekunden=1)["isError"]
    r = m.call("drehen", grad=45, bild=False)
    assert not r["isError"], r
    assert abs(x.yaw - 45) <= 15 and x.mode == "idle"                           # danach Stillstand + idle
    antrieb = [c for c in x.calls if c.startswith("POST /drive")]
    assert antrieb and all("v=0&" in c for c in antrieb) and "w=0" in antrieb[-1].split("&")[-1]


def test_rundblick(mcp):
    m, x, _ = mcp()
    r = m.call("rundblick", bilder=4)
    assert not r["isError"], r
    assert [c["type"] for c in r["content"]].count("image") == 4
    assert abs(x.yaw - 360) <= 15 and x.mode == "idle"


def test_fahren_nur_geprueft(mcp):
    m, x, _ = mcp()
    x.meldung = " | Freiraum!"
    r = m.call("fahren", v=0.1, sekunden=2)
    assert not r["isError"], r
    info = json.loads(r["content"][0]["text"].split("\n")[0])
    assert info["gebremst"] and "hinweis" in info and r["content"][-1]["type"] == "image"
    fahrt = [c for c in x.calls if c.startswith("POST /drive") and "v=0.100" in c]
    assert fahrt and all("sicher=1" in c for c in fahrt)                        # nie ohne Bodengrenze und Bahnprüfung
    assert x.mode == "idle"
    assert m.call("fahren", v=0.5, sekunden=1)["isError"]                       # über der Grenze
    x.neu, n = False, len(x.calls)
    r = m.call("fahren", v=0.1, sekunden=1)                                     # altes xgonav: nie ungeprüft fahren
    assert r["isError"] and not any(c.startswith("POST") for c in x.calls[n:])
    x.neu = True
    x.mode = "explore"
    r = m.call("fahren", v=0.1, sekunden=1)
    assert r["isError"] and "explore" in r["content"][0]["text"] and x.mode == "explore"   # greift nicht ein


def test_liegt_und_akku(mcp):
    m, x, proc = mcp(state=0)
    r = m.call("drehen", grad=30)
    assert r["isError"] and "liegt" in r["content"][0]["text"]
    (proc / "state").write_text("1\n")
    (proc / "battery").write_text("12\n")
    assert "Akku" in m.call("drehen", grad=30)["content"][0]["text"]
    assert not any(c.startswith("POST /drive") for c in x.calls)


def test_token(mcp):
    m, _, _ = mcp(token="geheim")
    body = b'{"jsonrpc":"2.0","id":1,"method":"ping"}'
    assert m.roh(body)[0] == 401
    assert m.roh(body, kopf={"Authorization": "Bearer falsch"})[0] == 401
    assert m.roh(body, kopf={"Authorization": "Bearer geheim"})[0] == 200
