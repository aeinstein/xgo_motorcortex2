"""tools/live.php: Laufzeitdaten per WebSocket (Takte, IMU-Filter, Steuernachricht). Ohne php übersprungen."""
import base64
import json
import os
import shutil
import socket
import struct
import subprocess
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
PHP = shutil.which("php")
pytestmark = pytest.mark.skipif(PHP is None, reason="kein php im PATH")


def _freier_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _proc(d: Path, state: int, n_imu: int = 5, start: int = 1) -> None:
    d.mkdir(exist_ok=True)
    (d / "imu_log").write_text("".join(f"{s} {s * 20_000_000} 100 -2500 9000\n" for s in range(start, start + n_imu)))
    (d / "wheels_log").write_text("".join(f"{s} {s * 20_000_000} {s * 10} {-s * 10} 0\n" for s in range(1, 4)))
    for name, wert in (("state", state), ("battery", 77), ("height", 120), ("roll", 0)):
        (d / name).write_text(f"{wert}\n")


class Client:
    def __init__(self, port: int):
        self.s = socket.create_connection(("127.0.0.1", port), timeout=3)
        key = base64.b64encode(os.urandom(16)).decode()
        self.s.sendall(f"GET / HTTP/1.1\r\nHost: x\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                       f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n".encode())
        self.buf = b""
        while b"\r\n\r\n" not in self.buf:
            self.buf += self.s.recv(4096)
        kopf, self.buf = self.buf.split(b"\r\n\r\n", 1)
        assert b" 101 " in kopf

    def _lies(self, n: int) -> bytes:
        while len(self.buf) < n:
            d = self.s.recv(65536)
            assert d, "Verbindung zu"
            self.buf += d
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def recv(self) -> dict:
        b0, b1 = self._lies(2)
        n = b1 & 0x7F
        if n == 126:
            n = struct.unpack("!H", self._lies(2))[0]
        elif n == 127:
            n = struct.unpack("!Q", self._lies(8))[0]
        assert b0 & 0x0F == 1
        return json.loads(self._lies(n))

    def send(self, text: str) -> None:
        d, m = text.encode(), os.urandom(4)
        self.s.sendall(bytes([0x81, 0x80 | len(d)]) + m + bytes(c ^ m[i % 4] for i, c in enumerate(d)))


@pytest.fixture
def live(tmp_path):
    procs = []

    def start(state: int):
        _proc(tmp_path / "proc", state)
        port = _freier_port()
        env = {**os.environ, "XGO_PROC": str(tmp_path / "proc"), "XGO_LIVE_PORT": str(port),
               "XGO_STATUS_URL": f"http://127.0.0.1:{_freier_port()}/status"}   # niemand lauscht
        p = subprocess.Popen([PHP, str(ROOT / "tools" / "live.php")], env=env, stderr=subprocess.PIPE)
        procs.append(p)
        for _ in range(50):
            try:
                socket.create_connection(("127.0.0.1", port), timeout=0.1).close()
                break
            except OSError:
                time.sleep(0.05)
        return port, tmp_path / "proc"

    yield start
    for p in procs:
        p.kill()
        p.wait()


def _sammeln(c: Client, dauer: float) -> list[tuple[float, dict]]:
    out, t0 = [], time.monotonic()
    c.s.settimeout(0.2)
    while time.monotonic() - t0 < dauer:
        try:
            out.append((time.monotonic() - t0, c.recv()))
        except (socket.timeout, TimeoutError):
            pass
    return out


def test_balanciert_takte_und_nur_neue_proben(live):
    port, proc = live(1)
    c = Client(port)
    erstes = c.recv()
    assert erstes["sys"]["battery"] == 77 and erstes["sys"]["state"] == 1
    assert erstes["status"] is None and erstes["status_fehler"]
    assert [p[0] for p in erstes["imu"]["imu"]] == [1, 2, 3, 4, 5]
    assert erstes["imu"]["imu"][0][2:] == [0.1, -2.5, 9.0]
    assert len(erstes["imu"]["wheels"]) == 3
    _proc(proc, 1, n_imu=8)                                  # drei neue Proben
    msgs = _sammeln(c, 2.1)
    imu = [m["imu"] for _, m in msgs if "imu" in m]
    sys_ = [m for _, m in msgs if "sys" in m]
    assert 5 <= len(imu) <= 8                                # 0,3 s
    assert 1 <= len(sys_) <= 3                               # 1 s
    neu = [p[0] for i in imu for p in i["imu"]]
    assert neu == [6, 7, 8]                                  # keine Doppelten
    assert all(not i["wheels"] for i in imu)


def test_liegt_1hz_und_imu_aus(live):
    port, _ = live(254)
    c = Client(port)
    c.recv()
    msgs = _sammeln(c, 2.1)
    assert 1 <= len([m for _, m in msgs if "imu" in m]) <= 3
    c.send("imu 0")
    time.sleep(0.2)
    msgs = _sammeln(c, 2.1)
    assert msgs and all("imu" not in m and "sys" in m for _, m in msgs)


def test_since_beim_neuverbinden(live):
    port, _ = live(1)
    c = Client(port)
    c.recv()                                                 # alles bis seq 5
    c.send("imu 1 2 1")                                      # wie der Browser nach dem Neuverbinden
    imu = [m["imu"] for _, m in _sammeln(c, 0.8) if "imu" in m]
    assert [p[0] for p in imu[0]["imu"]] == [3, 4, 5]
    assert [p[0] for p in imu[0]["wheels"]] == [2, 3]
