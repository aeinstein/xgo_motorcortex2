"""WLAN-Einrichtung über die Tasten (xgonav/wlan.py): QR-Inhalt, Tasten, Ablauf mit Fake-Helfer."""
import os
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from xgonav.config import Config
from xgonav.wlan import (HoldDetector, PressDetector, QrLeser, WlanSetup, _qr_image, parse_wifi_qr,
                         wifi_qr_text)


# ---------------------------------------------------------------- QR-Inhalt
def test_parse_wifi_qr_formen():
    assert parse_wifi_qr("WIFI:T:WPA;S:Heimnetz;P:geheim123;;") == ("Heimnetz", "geheim123", "WPA", False)
    # andere Reihenfolge, H:true, ohne abschließendes ;;
    assert parse_wifi_qr("WIFI:S:Netz;H:true;T:WPA;P:12345678") == ("Netz", "12345678", "WPA", True)
    # Escapes in SSID und Passwort (Hersteller-Parser scheiterte daran)
    assert parse_wifi_qr(r"WIFI:T:WPA;S:a\;b\:c;P:pw\;\\x\,yz12;;") == ("a;b:c", "pw;\\x,yz12", "WPA", False)
    # offenes Netz
    assert parse_wifi_qr("WIFI:T:nopass;S:Cafe;;") == ("Cafe", "", "NOPASS", False)
    # Kleinschreibung des Präfixes, Typ fehlt -> WPA
    assert parse_wifi_qr("wifi:S:x;P:abcdefgh;;") == ("x", "abcdefgh", "WPA", False)


@pytest.mark.parametrize("text", ["", "https://example.org", "WIFI:T:WPA;P:12345678;;", "WIFI:T:WPA;S:x;P:kurz;;",
                                  "WIFI:T:WPA;S:" + "x" * 33 + ";P:12345678;;"])
def test_parse_wifi_qr_ungueltig(text):
    assert parse_wifi_qr(text) is None


def test_wifi_qr_text_rundweg():
    for ssid, pw in [("RiderKerk", "abcd2345"), ('a;b:c,"d\\', "p;w:x,y\\z")]:
        assert parse_wifi_qr(wifi_qr_text(ssid, pw))[:2] == (ssid, pw)


# ---------------------------------------------------------------- Tasten
def test_hold_detector():
    h = HoldDetector(12, 3.0)
    assert not h.update(4, 0.0)            # nur C
    assert not h.update(12, 0.0)
    assert not h.update(12, 2.9)
    assert h.update(12 | 1, 3.0)           # weitere Taste dazu stört nicht
    assert not h.update(12, 10.0)          # weiter gehalten: nur einmal
    assert not h.update(0, 10.1)
    assert not h.update(12, 11.0)
    assert not h.update(8, 12.0)           # losgelassen -> neu zählen
    assert not h.update(12, 12.5)
    assert h.update(12, 15.5)


def test_hold_detector_ruhetakt():
    """Im Ruhemodus liest xgonav nur 1x/s: auch dann löst gut 3 s Halten aus."""
    h = HoldDetector(12, 3.0)
    hits = [h.update(12, float(t)) for t in range(0, 6)]
    assert hits == [False, False, False, True, False, False]


def test_press_detector():
    p = PressDetector(2)
    assert not p.update(2)                 # beim Start gedrückt: keine Flanke
    assert not p.update(0)
    assert p.update(2)
    assert not p.update(2)
    assert not p.update(0)
    assert p.update(3)


# ---------------------------------------------------------------- Ablauf
class FakeHelfer:
    def __init__(self, antworten):
        self.antworten = antworten          # cmd -> (rc, out)
        self.calls = []
        self._res = None
        self.busy = False

    def start(self, cmd, stdin="", timeout=60.0):
        self.calls.append((cmd, stdin))
        self._res = self.antworten.get(cmd, (0, ""))

    def result(self):
        r, self._res = self._res, None
        return r


def _qr_frame(text, size=200):
    """Kamerabild 640x480 mit QR-Code (wie ein Handy, etwas verrauscht)."""
    img = np.full((480, 640, 3), 120, np.uint8)
    q = _qr_image(text, size)
    assert q is not None
    pad = cv2.copyMakeBorder(q, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value=(255, 255, 255))
    y, x = 140, 200
    img[y:y + pad.shape[0], x:x + pad.shape[1]] = pad
    rng = np.random.default_rng(0)
    return np.clip(img.astype(np.int16) + rng.integers(-15, 16, img.shape), 0, 255).astype(np.uint8)


def test_qr_verbindet():
    cfg = Config.neutral().wlan
    h = FakeHelfer({"connect": (0, "con=xgo-Heim\nssid=Heim\nip=192.168.1.50")})
    w = WlanSetup(cfg, h, 0.0)
    frame = _qr_frame(wifi_qr_text("Heim", "geheim123"))
    w.step(frame, 1.0)
    assert w.state == "verbinde"
    assert h.calls == [("status", ""), ("connect", "Heim\ngeheim123\nWPA\n0\n")]
    w.step(None, 1.2)
    assert w.state == "ok" and w.ip == "192.168.1.50"
    w.step(None, 1.2 + WlanSetup.OK_S)
    assert w.done
    v, img = w.lcd_frame()
    assert img.shape == (240, 320, 3)


def test_qr_fehler_dann_zeit_um():
    """Ohne Code endet die Einrichtung; einen Hotspot öffnet sie nicht mehr (macht xgo-wlan auto)."""
    cfg = Config.neutral().wlan
    h = FakeHelfer({"connect": (1, "Verbinden fehlgeschlagen: Secrets were required")})
    w = WlanSetup(cfg, h, 0.0)
    w.step(_qr_frame(wifi_qr_text("Heim", "falsch123")), 0.5)
    w.step(None, 0.6)
    assert w.state == "fehler"
    w.step(None, 0.6 + WlanSetup.FEHLER_S)
    assert w.state == "qr"
    t0 = w.t_state
    leer = np.full((480, 640, 3), 120, np.uint8)
    w.step(leer, t0 + cfg.qr_timeout_s + 0.01)
    assert w.state == "ende" and w.done
    assert [c for c, _ in h.calls] == ["status", "connect"]


def test_hinweis_wenn_hotspot_laeuft():
    cfg = Config.neutral().wlan
    h = FakeHelfer({"status": (0, "con=xgo-hotspot\nssid=RiderKerk\nip=10.42.0.1\nclients=1")})
    w = WlanSetup(cfg, h, 0.0)
    assert h.calls[0][0] == "status"
    w.step(None, 0.1)
    assert w.state == "qr" and w.hotspot == "Hotspot RiderKerk: 10.42.0.1/wlan.php"
    assert w.status()["hotspot"] == w.hotspot
    v, img = w.lcd_frame()
    assert img.shape == (240, 320, 3)
    w.cancel(1.0)
    assert w.done


def test_kein_hinweis_im_wlan():
    cfg = Config.neutral().wlan
    h = FakeHelfer({"status": (0, "con=xgo-Heim\nssid=Heim\nip=192.168.1.7")})
    w = WlanSetup(cfg, h, 0.0)
    w.step(None, 0.1)
    assert w.hotspot == "" and w.status()["hotspot"] is None


# ---------------------------------------------------------------- QR-Leser
def test_qr_leser_einfach_ohne_modelle():
    leser = QrLeser("")                     # ohne Modellverzeichnis: cv2.QRCodeDetector, Bildmitte 2x
    assert leser.art == "opencv"
    assert parse_wifi_qr(leser.lesen(cv2.cvtColor(_qr_frame(wifi_qr_text("Heim", "geheim123")),
                                                  cv2.COLOR_BGR2GRAY)))[:2] == ("Heim", "geheim123")
    assert QrLeser("/gibt/es/nicht").art == "opencv"


MODELLE = os.path.join(os.path.dirname(__file__), "..", "models", "wechat_qr")


@pytest.mark.skipif(not hasattr(cv2, "wechat_qrcode_WeChatQRCode") or not os.path.isdir(MODELLE),
                    reason="opencv-contrib (WeChat-QR) oder models/wechat_qr fehlt (am Rider vorhanden)")
def test_qr_leser_wechat():
    leser = QrLeser(MODELLE)
    assert leser.art == "wechat"
    assert parse_wifi_qr(leser.lesen(cv2.cvtColor(_qr_frame(wifi_qr_text("Heim", "geheim123")),
                                                  cv2.COLOR_BGR2GRAY)))[:2] == ("Heim", "geheim123")
