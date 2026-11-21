"""tools/sprech.php: Gegensprechen per WebSocket (nur Modus manual), mit Ersatz für arecord/aplay und /status.
Ohne php übersprungen."""
import base64
import json
import os
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
PHP = shutil.which("php")
pytestmark = pytest.mark.skipif(PHP is None or os.name == "nt", reason="kein php im PATH (oder Windows)")

# arecord-Ersatz: alle 40 ms 640 Byte mit laufender Nummer; aplay-Ersatz: stdin in eine Datei
FAKE_REC = "import sys,time\ni=0\nwhile True:\n    sys.stdout.buffer.write(bytes([i % 256]) * 640); sys.stdout.flush(); i += 1; time.sleep(0.04)\n"
FAKE_PLAY = ("import sys\nf=open(sys.argv[1],'ab',buffering=0)\nwhile True:\n    d=sys.stdin.buffer.read1(4096)\n"
             "    if not d: break\n    f.write(d)\n")


LAUF: dict = {}                  # vom Fixture: Display-FIFO-Inhalt, tmp-Ordner, PID des Dienstes
BILD = 320 * 240 * 2


def _freier_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class Client:
    def __init__(self, port: int, pfad: str = "/"):
        self.s = socket.create_connection(("127.0.0.1", port), timeout=3)
        key = base64.b64encode(os.urandom(16)).decode()
        self.s.sendall(f"GET {pfad} HTTP/1.1\r\nHost: x\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                       f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n".encode())
        self.buf = b""
        while b"\r\n\r\n" not in self.buf:
            self.buf += self.s.recv(4096)
        kopf, self.buf = self.buf.split(b"\r\n\r\n", 1)
        assert b" 101 " in kopf

    def _lies(self, n: int) -> bytes:
        while len(self.buf) < n:
            d = self.s.recv(65536)
            if not d:
                raise EOFError
            self.buf += d
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def recv(self) -> tuple[int, bytes]:
        b0, b1 = self._lies(2)
        n = b1 & 0x7F
        if n == 126:
            n = struct.unpack("!H", self._lies(2))[0]
        elif n == 127:
            n = struct.unpack("!Q", self._lies(8))[0]
        return b0 & 0x0F, self._lies(n)

    def text(self) -> dict:
        while True:
            op, d = self.recv()
            if op == 1:
                return json.loads(d)
            assert op == 2, op

    def send(self, d: bytes, op: int) -> None:
        m = os.urandom(4)
        n = len(d)
        if n < 126:
            laenge = bytes([0x80 | n])
        elif n < 65536:
            laenge = bytes([0x80 | 126]) + struct.pack("!H", n)
        else:
            laenge = bytes([0x80 | 127]) + struct.pack("!Q", n)
        maske = (m * (n // 4 + 1))[:n]
        roh = (int.from_bytes(d, "big") ^ int.from_bytes(maske, "big")).to_bytes(n, "big") if n else b""
        self.s.sendall(bytes([0x80 | op]) + laenge + m + roh)


@pytest.fixture
def sprech(tmp_path):
    zustand = {"mode": "manual"}

    class H(BaseHTTPRequestHandler):
        def do_GET(self):
            body = json.dumps(zustand).encode()
            self.send_response(200)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *a):
            pass

    http = ThreadingHTTPServer(("127.0.0.1", 0), H)
    threading.Thread(target=http.serve_forever, daemon=True).start()
    (tmp_path / "rec.py").write_text(FAKE_REC)
    fifo = tmp_path / "lcd0"
    os.mkfifo(fifo)
    (tmp_path / "lcd0.owner").write_text("")
    lcd = bytearray()

    def lesen():                                             # wie der Display-Daemon: liest, solange jemand schreibt
        while True:
            with open(fifo, "rb", buffering=0) as f:
                while d := f.read(65536):
                    lcd.extend(d)

    threading.Thread(target=lesen, daemon=True).start()
    (tmp_path / "play.py").write_text(FAKE_PLAY)
    port = _freier_port()
    env = {**os.environ, "XGO_SPRECH_PORT": str(port),
           "XGO_STATUS_URL": f"http://127.0.0.1:{http.server_address[1]}/status",
           "XGO_ARECORD": f"{sys.executable} {tmp_path / 'rec.py'}",
           "XGO_APLAY": f"{sys.executable} {tmp_path / 'play.py'} {tmp_path / 'laut.raw'}",
           "XGO_LCD_FIFO": str(fifo), "XGO_LCD_ENDE": f"touch {tmp_path / 'ende'}"}
    p = subprocess.Popen([PHP, str(ROOT / "tools" / "sprech.php")], env=env, stderr=subprocess.PIPE)
    for _ in range(50):
        try:
            socket.create_connection(("127.0.0.1", port), timeout=0.1).close()
            break
        except OSError:
            time.sleep(0.05)
    LAUF.update(lcd=lcd, tmp=tmp_path, pid=p.pid)
    yield port, zustand, tmp_path / "laut.raw"
    p.kill()
    p.wait()
    http.shutdown()


def test_nur_im_handmodus(sprech):
    port, zustand, _ = sprech
    zustand["mode"] = "idle"
    c = Client(port)
    m = c.text()
    assert m == {"sprech": "aus", "grund": "nur im Handmodus"}
    op, _ = c.recv()
    assert op == 8                                           # Close


def test_beide_richtungen_und_ende_bei_moduswechsel(sprech):
    port, zustand, laut = sprech
    c = Client(port)
    assert c.text()["sprech"] == "an"
    rahmen = []
    while len(rahmen) < 5:
        op, d = c.recv()
        assert op == 2
        rahmen.append(d)
    assert all(len(d) == 640 for d in rahmen)
    assert len({d[0] for d in rahmen}) == 5                  # laufende Nummern: nichts doppelt
    ton = bytes(range(256)) * 5
    c.send(ton, 2)
    for _ in range(40):
        if laut.exists() and laut.stat().st_size >= len(ton):
            break
        time.sleep(0.05)
    assert laut.read_bytes() == ton
    zweiter = Client(port)                                   # ein Gespräch zugleich
    assert zweiter.text()["grund"].startswith("besetzt")
    zustand["mode"] = "explore"
    t0 = time.monotonic()
    while True:                                              # binnen STATUS_S beendet
        op, d = c.recv()
        if op == 1:
            assert json.loads(d) == {"sprech": "aus", "grund": "nur im Handmodus"}
            break
        assert time.monotonic() - t0 < 3
    assert c.recv()[0] == 8
    zustand["mode"] = "manual"                               # danach wieder frei
    neu = Client(port)
    assert neu.text()["sprech"] == "an"


def test_laut_aus_verwirft(sprech):
    port, _, laut = sprech
    c = Client(port)
    assert c.text()["sprech"] == "an"
    c.send(b"laut 0", 1)
    time.sleep(0.1)
    c.send(b"\x01\x02" * 100, 2)
    time.sleep(0.3)
    assert not laut.exists() or laut.stat().st_size == 0


def _warte(bed, s: float = 3.0) -> bool:
    t0 = time.monotonic()
    while time.monotonic() - t0 < s:
        if bed():
            return True
        time.sleep(0.02)
    return False


def test_bild_nur_mit_gespraech(sprech):
    port, _, _ = sprech
    b = Client(port, "/?bild")
    assert b.text() == {"bild": "aus", "grund": "kein Gespräch"}


def test_bild_aufs_display_und_besitz(sprech):
    port, zustand, _ = sprech
    lcd, tmp = LAUF["lcd"], LAUF["tmp"]
    owner = tmp / "lcd0.owner"
    c = Client(port)
    assert c.text()["sprech"] == "an"
    b = Client(port, "/?bild")
    assert b.text() == {"bild": "an", "grund": ""}
    assert Client(port, "/?bild").text()["grund"].startswith("besetzt")
    for k in range(3):                                       # drei Bilder, jedes mit eigenem Füllwert
        b.send(bytes([k + 1]) * BILD, 2)
        assert _warte(lambda: len(lcd) >= (k + 1) * BILD), len(lcd)
    assert len(lcd) == 3 * BILD                              # ganze Bilder, nichts doppelt
    assert all(lcd[i * BILD:(i + 1) * BILD].count(i + 1) == BILD for i in range(3))
    assert owner.read_text().strip() == str(LAUF["pid"])
    b.send(b"x" * 1000, 2)                                   # falsche Länge: verworfen
    owner.write_text("999999")                               # Emotion startet: nichts mehr schreiben
    time.sleep(0.1)
    b.send(bytes([9]) * BILD, 2)
    time.sleep(0.5)
    assert len(lcd) == 3 * BILD
    owner.write_text("")                                     # Emotion vorbei: wieder übernehmen
    b.send(bytes([7]) * BILD, 2)
    time.sleep(0.05)
    b.send(bytes([8]) * BILD, 2)                             # während der Wartezeit: nur das Neueste
    assert _warte(lambda: len(lcd) >= 4 * BILD)
    time.sleep(0.3)
    assert len(lcd) == 4 * BILD and lcd[3 * BILD] == 8
    zustand["mode"] = "idle"                                 # Gespräch endet, Bild mit
    assert _warte(lambda: (tmp / "ende").exists())
    assert owner.read_text() == ""
    assert b.text() == {"bild": "aus", "grund": "Gespräch beendet"}


def _rahmen(d: bytes, op: int, fin: bool) -> bytes:
    """Ein maskierter Client-Rahmen; fin=False = weitere Fragmente folgen (wie Chromium bei großen Nachrichten)."""
    m, n = os.urandom(4), len(d)
    if n < 126:
        laenge = bytes([0x80 | n])
    elif n < 65536:
        laenge = bytes([0x80 | 126]) + struct.pack("!H", n)
    else:
        laenge = bytes([0x80 | 127]) + struct.pack("!Q", n)
    maske = (m * (n // 4 + 1))[:n]
    roh = (int.from_bytes(d, "big") ^ int.from_bytes(maske, "big")).to_bytes(n, "big") if n else b""
    return bytes([(0x80 if fin else 0) | op]) + laenge + m + roh


def test_bild_fragmentiert(sprech):
    port, _, _ = sprech
    lcd = LAUF["lcd"]
    c = Client(port)
    assert c.text()["sprech"] == "an"
    b = Client(port, "/?bild")
    assert b.text()["bild"] == "an"
    bild = bytes([5]) * BILD
    teile = [bild[:65536], bild[65536:131072], bild[131072:]]
    b.s.sendall(_rahmen(teile[0], 2, False) + _rahmen(b"x", 9, True)       # Ping mitten in der Nachricht
                + _rahmen(teile[1], 0, False) + _rahmen(teile[2], 0, True))
    assert _warte(lambda: len(lcd) >= BILD)
    time.sleep(0.2)
    assert len(lcd) == BILD and lcd.count(5) == BILD
    op, d = b.recv()
    assert (op, d) == (0xA, b"x")                            # Pong
