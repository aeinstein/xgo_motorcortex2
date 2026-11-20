"""xgo-ble (deploy/xgo-ble/xgo-ble.c): Gegenstelle des BLE-Tunnels am Pi.

Baut das Programm mit gcc und spricht es über ein SOCK_SEQPACKET-Paar an (ein Datensatz je read wie
/proc/XGORider/ble: Byte 0 = Kennung, dann höchstens 23 Byte). xgo-wlan ist ein Shell-Skript, das seine
Eingabe ausgibt, xgonav ein kleiner HTTP-Server. Nur unter Linux mit gcc (Rider, GPU-Rechner); am
Windows-PC übersprungen.
"""
import http.server
import os
import shutil
import socket
import subprocess
import sys
import threading

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "deploy", "xgo-ble", "xgo-ble.c")

pytestmark = pytest.mark.skipif(
    not sys.platform.startswith("linux") or shutil.which("gcc") is None or not hasattr(socket, "SOCK_SEQPACKET"),
    reason="braucht Linux und gcc")

WLAN = """#!/bin/sh
case "$1" in
    status) echo "con=xgo-Test"; echo "ssid=Test"; echo "ip=192.168.53.110" ;;
    scan) echo "70:WPA2:Test"; echo "40::Offen" ;;
    rescan) echo "71:WPA2:Test" ;;
    connect)
        IFS= read -r ssid; IFS= read -r pw; IFS= read -r typ; IFS= read -r hidden
        echo "ssid=$ssid"; echo "pw=$pw"; echo "typ=$typ"; echo "hidden=$hidden"
        [ "$ssid" = "Falsch" ] && { echo "Verbinden fehlgeschlagen: kein Netz"; exit 1; }
        echo "ip=10.10.0.110" ;;
    *) exit 2 ;;
esac
"""


class Nav(http.server.BaseHTTPRequestHandler):
    seen = []

    def _antwort(self):
        Nav.seen.append((self.command, self.path))
        if self.path.startswith("/status"):
            code, body = 200, b'{"mode": "idle"}'
        elif self.path.startswith("/gross"):
            code, body = 200, b"x" * 40000
        elif self.path.startswith("/mode"):
            code, body = 409, b'{"error": "blind"}'
        else:
            code, body = 404, b"nichts"
        self.send_response(code)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    do_GET = do_POST = _antwort

    def log_message(self, *a):
        pass


class Tunnel:
    """Die Seite des Treibers: Datensätze hinein, Bytes heraus."""

    def __init__(self, exe, wlan, port):
        self.sock, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.proc = subprocess.Popen([exe, "--fd", str(child.fileno()), "--wlan", wlan, "--port", str(port)],
                                     pass_fds=[child.fileno()])
        child.close()
        self.sock.settimeout(10)
        self.buf = b""

    def meldung(self, kennung):
        self.sock.send(bytes([kennung]))

    def sende(self, text, stueck=23):
        data = text.encode()
        for i in range(0, len(data), stueck):
            self.sock.send(b"\0" + data[i:i + stueck])

    def _lies(self, n=None):
        while (n is None and b"\n" not in self.buf) or (n is not None and len(self.buf) < n):
            self.buf += self.sock.recv(4096)
        if n is None:
            zeile, self.buf = self.buf.split(b"\n", 1)
            return zeile
        teil, self.buf = self.buf[:n], self.buf[n:]
        return teil

    def antwort(self):
        ident, code, n = self._lies().split()
        return ident.decode(), int(code), self._lies(int(n))

    def frage(self, text):
        self.sende(text + "\n")
        return self.antwort()

    def ende(self):
        self.sock.close()
        assert self.proc.wait(timeout=5) == 0


@pytest.fixture(scope="module")
def umgebung(tmp_path_factory):
    d = tmp_path_factory.mktemp("xgoble")
    exe = str(d / "xgo-ble")
    subprocess.run(["gcc", "-O2", "-Wall", "-Wextra", "-Werror", "-o", exe, SRC], check=True)
    wlan = str(d / "xgo-wlan")
    with open(wlan, "w", newline="\n") as f:
        f.write(WLAN)
    os.chmod(wlan, 0o755)
    srv = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Nav)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield exe, wlan, srv.server_address[1]
    srv.shutdown()


@pytest.fixture
def tunnel(umgebung):
    t = Tunnel(*umgebung)
    yield t
    t.ende()


def test_wlan_status_und_scan(tunnel):
    assert tunnel.frage("1 wlan status") == ("1", 0, b"con=xgo-Test\nssid=Test\nip=192.168.53.110\n")
    assert tunnel.frage("2 wlan scan") == ("2", 0, b"70:WPA2:Test\n40::Offen\n")
    assert tunnel.frage("3 wlan rescan") == ("3", 0, b"71:WPA2:Test\n")


def test_wlan_connect_prozentkodiert(tunnel):
    ident, code, body = tunnel.frage("7 wlan connect WPA 1 Mein%20Netz%25 geh%20heim%21%c3%a4")
    assert (ident, code) == ("7", 0)
    assert body.decode() == "ssid=Mein Netz%\npw=geh heim!ä\ntyp=WPA\nhidden=1\nip=10.10.0.110\n"
    # offenes Netz: kein Passwort
    assert tunnel.frage("8 wlan connect NOPASS 0 Cafe")[2] == b"ssid=Cafe\npw=\ntyp=NOPASS\nhidden=0\nip=10.10.0.110\n"


def test_wlan_connect_fehler_und_unsinn(tunnel):
    ident, code, body = tunnel.frage("9 wlan connect WPA 0 Falsch 12345678")
    assert (ident, code) == ("9", 1) and body.endswith(b"Verbinden fehlgeschlagen: kein Netz\n")
    # Zeilenende im Namen würde xgo-wlan eine weitere Eingabezeile unterschieben
    assert tunnel.frage("10 wlan connect WPA 0 a%0aNOPASS 12345678")[1] == 400
    assert tunnel.frage("11 wlan connect WEP 0 a 12345678")[1] == 400
    assert tunnel.frage("12 wlan connect WPA 0 a%zz 12345678")[1] == 400
    assert tunnel.frage("13 wlan hotspot")[1] == 400
    assert tunnel.frage("14 quatsch")[1] == 400
    assert tunnel.frage("15 wlan status")[1] == 0          # danach geht es normal weiter


def test_nav_durchgereicht(tunnel):
    Nav.seen.clear()
    assert tunnel.frage("20 nav GET /status?breakdown=1") == ("20", 200, b'{"mode": "idle"}')
    assert tunnel.frage("21 nav POST /mode?name=follow&target=person") == ("21", 409, b'{"error": "blind"}')
    assert Nav.seen == [("GET", "/status?breakdown=1"), ("POST", "/mode?name=follow&target=person")]
    assert tunnel.frage("22 nav GET /gibtsnicht")[1] == 404


def test_nav_grenzen(tunnel):
    Nav.seen.clear()
    for pfad in ("/ws", "/ws?nick=2", "/frame?after=3", "/map.png"):
        assert tunnel.frage("30 nav GET " + pfad)[1] == 403
    assert tunnel.frage("31 nav DELETE /status")[1] == 400
    assert tunnel.frage("32 nav GET status")[1] == 400
    assert Nav.seen == []
    ident, code, body = tunnel.frage("33 nav GET /gross")       # lange Antwort wird abgeschnitten
    assert (ident, code, len(body)) == ("33", 200, 16384)


def test_nav_ohne_xgonav(umgebung):
    exe, wlan, _ = umgebung
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    frei = s.getsockname()[1]
    s.close()
    t = Tunnel(exe, wlan, frei)
    assert t.frage("40 nav GET /status")[:2] == ("40", 599)
    t.ende()


def test_trennen_verwirft_angefangene_zeile(tunnel):
    tunnel.sende("50 wlan conn")       # Client trennt mitten in der Zeile
    tunnel.meldung(2)
    tunnel.meldung(1)
    assert tunnel.frage("51 wlan status")[:2] == ("51", 0)
    # zwei Anfragen in einem Zug, Stücke beliebig geschnitten
    tunnel.sende("52 wlan scan\n53 wlan status\n", stueck=5)
    assert tunnel.antwort()[0] == "52" and tunnel.antwort()[0] == "53"


def test_zu_lange_zeile(tunnel):
    tunnel.sende("60 nav GET /" + "a" * 2000 + "\n")
    assert tunnel.antwort()[:2] == ("0", 400)
    assert tunnel.frage("61 wlan status")[1] == 0
