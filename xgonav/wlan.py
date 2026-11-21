"""WLAN-Einrichtung über die Tasten am Rider, ohne bestehende Netzverbindung.

Ablauf (wie beim Hersteller, aber ohne pyzbar und ohne wpa_supplicant.conf zu überschreiben):
  C+D halten -> Kamera sucht einen WLAN-QR-Code (Handy: WLAN teilen) -> NetworkManager-Profil anlegen.
  Ohne Code nach [wlan] qr_timeout_s Ende. Den Hotspot öffnet seit 2026-11-21 nicht mehr diese Einrichtung,
  sondern xgo-wlan-auto.service von selbst (offen, RiderKerk, wenn wlan0 60 s nicht verbunden ist); läuft er,
  steht am Display die Adresse von web/wlan.php.
Den root-Teil macht deploy/xgo-wlan (sudo-Regel deploy/sudoers-xgonav-wlan); Passwörter gehen über stdin,
nie über die Kommandozeile oder ins Log.

Kosten am Pi: ohne Einrichtung nur der Tastenvergleich je Schleifendurchlauf. Während der Einrichtung ruht
die Bildverarbeitung; QR-Suche höchstens [wlan] qr_hz, Display-Bild nur bei Änderung.
"""
from __future__ import annotations

import logging
import math
import os
import subprocess
import threading
import time

import cv2
import numpy as np

log = logging.getLogger(__name__)

HOTSPOT_IP = "10.42.0.1"                      # NetworkManager-Hotspot (ipv4.method shared)
WLAN_URL = f"{HOTSPOT_IP}/wlan.php"
HOTSPOT_CON = "xgo-hotspot"                   # Profil des Hotspots (deploy/xgo-wlan, SSID RiderKerk)


# ---------------------------------------------------------------- QR-Inhalt
def parse_wifi_qr(text: str) -> tuple[str, str, str, bool] | None:
    """WLAN-QR (`WIFI:T:WPA;S:<ssid>;P:<pw>;H:false;;`) -> (ssid, passwort, typ, versteckt), sonst None.
    Felder in beliebiger Reihenfolge, Escapes `\\;` `\\,` `\\:` `\\\\` `\\"` wie im Format vorgesehen."""
    if not text or not text[:5].upper() == "WIFI:":
        return None
    felder: dict[str, str] = {}
    key, buf, esc, in_key = "", [], False, True
    for ch in text[5:]:
        if esc:
            buf.append(ch)
            esc = False
        elif ch == "\\":
            esc = True
        elif in_key and ch == ":":
            key, buf, in_key = "".join(buf).strip().upper(), [], False
        elif ch == ";":
            if not in_key and key and key not in felder:
                felder[key] = "".join(buf)
            key, buf, in_key = "", [], True
        else:
            buf.append(ch)
    if not in_key and key and key not in felder:   # letztes Feld ohne abschließendes ;
        felder[key] = "".join(buf)
    ssid = felder.get("S", "")
    typ = (felder.get("T", "") or "WPA").upper()
    pw = felder.get("P", "")
    if typ in ("NOPASS", "NONE", ""):
        typ, pw = "NOPASS", ""
    if not ssid or len(ssid.encode()) > 32:
        return None
    if typ != "NOPASS" and not (8 <= len(pw) <= 63 or (len(pw) == 64 and all(c in "0123456789abcdefABCDEF" for c in pw))):
        if typ != "WEP":
            return None
    return ssid, pw, typ, felder.get("H", "").lower() == "true"


def wifi_qr_text(ssid: str, pw: str) -> str:
    """Gegenstück zu parse_wifi_qr (Tests, Prüfbilder)."""
    def esc(s: str) -> str:
        return "".join("\\" + c if c in '\\;,:"' else c for c in s)
    return f"WIFI:T:WPA;S:{esc(ssid)};P:{esc(pw)};;"


# ---------------------------------------------------------------- Tasten
class HoldDetector:
    """Alle Tasten aus `mask` mindestens hold_s gleichzeitig gedrückt -> einmal True; erst nach dem Loslassen wieder."""

    def __init__(self, mask: int, hold_s: float):
        self.mask, self.hold_s = mask, hold_s
        self._t0: float | None = None
        self._fired = False

    def update(self, buttons: int, now: float) -> bool:
        if not self.mask or (buttons & self.mask) != self.mask:
            self._t0, self._fired = None, False
            return False
        if self._fired:
            return False
        if self._t0 is None:
            self._t0 = now
        if now - self._t0 >= self.hold_s:
            self._fired = True
            return True
        return False


class PressDetector:
    """Flanke: Taste(n) aus `mask` neu gedrückt -> True."""

    def __init__(self, mask: int):
        self.mask = mask
        self._prev = True              # beim Start gedrückt gehaltene Taste löst nicht aus

    def update(self, buttons: int) -> bool:
        cur = bool(self.mask) and (buttons & self.mask) == self.mask
        hit = cur and not self._prev
        self._prev = cur
        return hit


# ---------------------------------------------------------------- QR lesen
_WECHAT = ("detect.prototxt", "detect.caffemodel", "sr.prototxt", "sr.caffemodel")


class QrLeser:
    """WeChat-QR (opencv-contrib, CNN-Detektor + Hochskalierung) liest auch gestaltete Codes: Samsung „WLAN
    teilen“ hat runde Suchmuster, Punkte und ein Logo; cv2.QRCodeDetector fand ihn am Rider nur aus ~4 cm
    (Aufnahme 2026-11-19: WeChat 64/135 Bilder, QRCodeDetector 0, zbar 4). Ohne Modelle oder contrib:
    QRCodeDetector auf dem Vollbild, sonst Bildmitte doppelt so groß. Läuft nur in der Einrichtung (≤ qr_hz)."""

    def __init__(self, model_dir: str = ""):
        self._wc = None
        pfade = [os.path.join(model_dir, n) for n in _WECHAT] if model_dir else []
        if pfade and all(os.path.exists(p) for p in pfade) and hasattr(cv2, "wechat_qrcode_WeChatQRCode"):
            try:
                self._wc = cv2.wechat_qrcode_WeChatQRCode(*pfade)
            except cv2.error as e:
                log.warning("WLAN-Einrichtung: WeChat-QR nicht ladbar (%s), einfacher QR-Detektor", e)
        elif model_dir:
            log.warning("WLAN-Einrichtung: WeChat-QR fehlt (%s bzw. opencv-contrib), einfacher QR-Detektor: "
                        "gestaltete Codes (Samsung) nur aus der Nähe", model_dir)
        self._cv = cv2.QRCodeDetector()
        self.art = "wechat" if self._wc is not None else "opencv"

    def lesen(self, gray: np.ndarray) -> str | None:
        if self._wc is not None:
            try:
                texte, _ = self._wc.detectAndDecode(gray)
            except cv2.error:
                return None
            return next((t for t in texte if t), None)
        text = self._cv_lesen(gray)
        if text is None:
            # kleiner Code (Handy weiter weg, Weitwinkel): Bildmitte doppelt so groß
            h, w = gray.shape[:2]
            mitte = gray[h // 4: h - h // 4, w // 4: w - w // 4]
            text = self._cv_lesen(cv2.resize(mitte, None, fx=2, fy=2, interpolation=cv2.INTER_LINEAR))
        return text

    def _cv_lesen(self, gray: np.ndarray) -> str | None:
        try:
            text, _, _ = self._cv.detectAndDecode(gray)
        except cv2.error:
            return None
        return text or None


_leser: dict[str, QrLeser] = {}


def qr_leser(model_dir: str) -> QrLeser:
    """Einmal je Verzeichnis laden (Modelle ~1 MB), dann bei jeder Einrichtung wiederverwenden."""
    if model_dir not in _leser:
        _leser[model_dir] = QrLeser(model_dir)
    return _leser[model_dir]


# ---------------------------------------------------------------- root-Teil
class Helfer:
    """Ruft deploy/xgo-wlan per sudo in einem eigenen Thread auf; höchstens ein Auftrag zugleich."""

    def __init__(self, path: str):
        self.path = path
        self._res: tuple[int, str] | None = None
        self._busy = False
        self._lock = threading.Lock()

    @property
    def busy(self) -> bool:
        return self._busy

    def start(self, cmd: str, stdin: str = "", timeout: float = 60.0) -> None:
        with self._lock:
            self._busy, self._res = True, None

        def lauf():
            try:
                p = subprocess.run(["sudo", "-n", self.path, cmd], input=stdin, capture_output=True,
                                   text=True, timeout=timeout)
                res = (p.returncode, (p.stdout + p.stderr).strip())
            except (OSError, subprocess.SubprocessError) as e:
                res = (127, str(e))
            with self._lock:
                self._res, self._busy = res, False

        threading.Thread(target=lauf, name="wlan-helfer", daemon=True).start()

    def result(self) -> tuple[int, str] | None:
        """Ergebnis des letzten Auftrags, einmal; None solange er läuft oder keiner da ist."""
        with self._lock:
            r, self._res = self._res, None
            return r


def _kv(out: str) -> dict[str, str]:
    d = {}
    for z in out.splitlines():
        k, _, v = z.partition("=")
        if _ and k.strip():
            d[k.strip()] = v.strip()
    return d


def _ascii(s: str) -> str:
    return "".join(c if 32 <= ord(c) < 127 else "?" for c in s)


# ---------------------------------------------------------------- Ablauf
class WlanSetup:
    """Zustände: qr -> verbinde -> ok | fehler(-> qr);  qr (Zeit um) -> ende;  -> ende.
    step() läuft in der Hauptschleife (statt App._step), lcd_frame() im LCD-Thread."""

    OK_S = 20.0           # Erfolg so lange anzeigen
    FEHLER_S = 8.0

    def __init__(self, cfg, helfer, now: float):
        self.cfg, self.helfer = cfg, helfer
        self.state, self.t_state = "qr", now
        self.text = "WLAN-QR vor die Kamera halten"
        self.detail = ""
        self.ssid = ""
        self.ip = ""
        self.hotspot = ""                   # Hinweis, wenn der automatische Hotspot gerade läuft
        self._t_qr = 0.0
        self._qr = qr_leser(cfg.qr_model_dir)
        self._job = ""                      # laufender Helfer-Auftrag
        self.rest_s: int | None = None      # Countdown der QR-Suche in ganzen Sekunden
        self._bgr: np.ndarray | None = None
        self.version = 0                    # erhöht sich bei jeder Änderung der Anzeige
        self._lcd_cache: tuple[tuple[int, int], np.ndarray] | None = None
        self._anim = 0                      # Zähler der LCD-Bilder in der QR-Suche
        self._eyes = QrEyes()
        log.warning("WLAN-Einrichtung: QR-Code suchen (%.0f s)", cfg.qr_timeout_s)
        self._run("status", timeout=10.0)   # einmal: läuft der Hotspot? Dann die Adresse von wlan.php zeigen

    # -------------------------------------------------------------- Zustand
    @property
    def done(self) -> bool:
        return self.state == "ende" and not self.helfer.busy

    def _set(self, state: str, now: float, text: str, detail: str = "") -> None:
        self.state, self.t_state, self.text, self.detail = state, now, text, detail
        self.version += 1
        log.warning("WLAN-Einrichtung: %s %s", text, detail)

    def status(self) -> dict:
        return {"state": self.state, "text": self.text, "detail": self.detail, "ssid": self.ssid, "ip": self.ip,
                "hotspot": self.hotspot or None, "rest_s": self.rest_s}

    def _run(self, cmd: str, stdin: str = "", timeout: float = 60.0) -> None:
        self._job = cmd
        self.helfer.start(cmd, stdin, timeout)

    def cancel(self, now: float) -> None:
        if self.state == "ende":
            return
        self._set("ende", now, "Abgebrochen")

    # -------------------------------------------------------------- Schritt
    def step(self, bgr: np.ndarray | None, now: float) -> None:
        if bgr is not None and bgr is not self._bgr:
            self._bgr = bgr
            if self.state == "qr":
                self.version += 1
        res = self.helfer.result()
        if res is not None:
            job, self._job = self._job, ""
            self._job_done(job, res, now)
        st, dt = self.state, now - self.t_state
        if st == "qr":
            if (bgr is not None and not self.helfer.busy
                    and now - self._t_qr >= 1.0 / max(0.1, self.cfg.qr_hz)):
                self._t_qr = now
                self._scan(bgr, now)
            if self.state == "qr" and dt >= self.cfg.qr_timeout_s:
                self._set("ende", now, "Kein WLAN-Code", self.hotspot)
        elif st == "ok" and dt >= self.OK_S:
            self._set("ende", now, "Fertig", self.detail)
        elif st == "fehler" and dt >= self.FEHLER_S:
            self._set("qr", now, "WLAN-QR vor die Kamera halten")
        rest = max(0, math.ceil(self.cfg.qr_timeout_s - (now - self.t_state))) if self.state == "qr" else None
        if rest != self.rest_s:
            self.rest_s = rest
            self.version += 1

    def _job_done(self, job: str, res: tuple[int, str], now: float) -> None:
        rc, out = res
        kv = _kv(out)
        last = out.splitlines()[-1] if out else f"rc {rc}"
        if job == "connect" and self.state == "verbinde":
            if rc == 0:
                self.ip = kv.get("ip", "")
                self.hotspot = ""
                self._set("ok", now, f"Verbunden: {self.ssid}", self.ip)
            else:
                self._set("fehler", now, f"Fehler: {self.ssid}", last)
        elif job == "status" and rc == 0:
            self.hotspot = f"Hotspot {kv.get('ssid') or 'RiderKerk'}: {WLAN_URL}" \
                if kv.get("con") == HOTSPOT_CON else ""
            self.version += 1
        elif rc != 0:
            log.warning("WLAN-Einrichtung: %s: %s", job, last)

    def _scan(self, bgr: np.ndarray, now: float) -> None:
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
        text = self._qr.lesen(gray)
        if text is None:
            return
        wifi = parse_wifi_qr(text)
        if wifi is None:
            if self.detail != "Kein WLAN-Code":
                self.detail = "Kein WLAN-Code"
                self.version += 1
            return
        self.ssid, pw, typ, hidden = wifi
        self._set("verbinde", now, f"Verbinde mit {self.ssid} ...")
        self._run("connect", f"{self.ssid}\n{pw}\n{typ}\n{'1' if hidden else '0'}\n", timeout=60.0)

    # -------------------------------------------------------------- Display (LCD-Thread)
    def lcd_frame(self) -> tuple[tuple[int, int], np.ndarray]:
        """(Schlüssel, BGR 320x240); neu gezeichnet nur bei Änderung, in der QR-Suche jedes Mal (Augen)."""
        animiert = self.state == "qr"
        if animiert:
            self._anim += 1
        key = (self.version, self._anim if animiert else 0)
        if self._lcd_cache is not None and self._lcd_cache[0] == key:
            return self._lcd_cache
        img = self._draw()
        self._lcd_cache = (key, img)
        return self._lcd_cache

    def _draw(self) -> np.ndarray:
        font = cv2.FONT_HERSHEY_SIMPLEX
        if self.state == "qr":
            img = self._eyes.draw(time.monotonic())
            if self._bgr is not None:                                       # kleines Kamerabild zum Zielen
                klein = cv2.resize(self._bgr, (96, 72), interpolation=cv2.INTER_AREA)
                cv2.rectangle(klein, (24, 18), (72, 54), (0, 255, 255), 1)  # Mitte: dort sucht er vergrößert
                img[4:76, 4:100] = klein
                cv2.rectangle(img, (3, 3), (100, 76), (90, 90, 90), 1)
        else:
            img = np.zeros((240, 320, 3), np.uint8)
        if self.state == "qr":
            _countdown(img, (280, 40), 32, self.rest_s, self.cfg.qr_timeout_s)
        farbe = {"ok": (0, 200, 0), "fehler": (0, 0, 255)}.get(self.state, (255, 255, 255))
        cv2.rectangle(img, (0, 180), (320, 240), (0, 0, 0), -1)
        hinweis = self.state == "qr" and bool(self.hotspot)    # automatischer Hotspot läuft: Adresse zeigen
        y = (197, 233) if hinweis else (202, 228)
        cv2.putText(img, _ascii(self.text)[:34], (6, y[0]), font, 0.5, farbe, 1, cv2.LINE_AA)
        if hinweis:
            cv2.putText(img, _ascii(self.hotspot)[:44], (6, 215), font, 0.42, QrEyes.FARBE, 1, cv2.LINE_AA)
        unten = _ascii(self.detail)[:40]
        if self.state == "qr":
            unten = (unten + "  " if unten else "") + "B = Abbruch"
        cv2.putText(img, unten, (6, y[1]), font, 0.42, (200, 200, 200), 1, cv2.LINE_AA)
        return img


class QrEyes:
    """Augen wie tools/eyes.py (Türkis, abgerundet), die Pupillen sind QR-Suchmuster (7:5:3 Module, dunkel/hell/
    dunkel). Sie schauen sich um: Blick springt (Sakkade 0,12 s) zu zufälligen Zielen, hält 0,4-1,4 s, dazwischen
    Lidschlag. Direkt mit cv2 in 320x240 gezeichnet, ~1 ms je Bild, nur während der QR-Suche."""

    FARBE = (210, 225, 0)          # BGR von tools/eyes.py FARBE (0, 225, 210)
    CX, CY, W, H = (98.0, 222.0), 128.0, 74.0, 84.0
    MODUL = 5.0                    # Pupille 7 Module = 35 px
    SAKKADE = 0.12

    def __init__(self, seed: int = 7):
        self._rng = np.random.default_rng(seed)
        self._von = self._nach = (0.0, 0.0)
        self._t_ziel = None            # Zeitpunkt der letzten Sakkade
        self._t_next = 0.0
        self._t_blink = 0.0

    def _blick(self, t: float) -> tuple[float, float]:
        if self._t_ziel is None:
            self._t_ziel, self._t_next, self._t_blink = t, t + 0.8, t + 2.5
        if t >= self._t_next:
            self._von = self._blick_bei(t)
            alt = self._nach
            while True:                # neues Ziel, deutlich woanders
                z = (float(self._rng.uniform(-1, 1)), float(self._rng.uniform(-1, 1)) * 0.8)
                if abs(z[0] - alt[0]) + abs(z[1] - alt[1]) > 0.7:
                    break
            if self._rng.random() < 0.25:
                z = (0.0, 0.0)         # zwischendurch geradeaus (in die Kamera)
            self._nach, self._t_ziel = z, t
            self._t_next = t + self.SAKKADE + float(self._rng.uniform(0.4, 1.4))
        return self._blick_bei(t)

    def _blick_bei(self, t: float) -> tuple[float, float]:
        x = 1.0 if self._t_ziel is None else min(1.0, (t - self._t_ziel) / self.SAKKADE)
        x = x * x * (3 - 2 * x)
        return (self._von[0] + (self._nach[0] - self._von[0]) * x,
                self._von[1] + (self._nach[1] - self._von[1]) * x)

    def _lid(self, t: float) -> float:
        """0 offen .. 1 zu (Lidschlag 0,2 s)."""
        d = t - self._t_blink
        if d > 0.2:
            self._t_blink = t + float(self._rng.uniform(2.5, 5.0))
            return 0.0
        return 0.0 if d < 0 else 1.0 - abs(d / 0.1 - 1.0)

    def draw(self, t: float) -> np.ndarray:
        gx, gy = self._blick(t)
        lid = self._lid(t)
        S = 4                                       # Subpixel-Bits
        f = 1 << S
        alpha = np.zeros((240, 320), np.uint8)
        for cx0 in self.CX:
            cx, cy = cx0 + 18 * gx, self.CY + 8 * gy
            h = max(7.0, self.H * (1 - lid))
            auge = np.zeros_like(alpha)
            pts = rrect(self.W, h, 0.36 * min(self.W, h) / 2) + (cx, cy)
            cv2.fillPoly(auge, [np.round(pts * f).astype(np.int32)], 255, cv2.LINE_AA, S)
            m = auge.copy()
            px = cx + gx * (self.W / 2 - 3.5 * self.MODUL - 4)
            py = cy + gy * (self.H / 2 - 3.5 * self.MODUL - 4) * (h / self.H)
            for n, val in ((7, 0), (5, 255), (3, 0)):   # Suchmuster: dunkel, hell, dunkel; Lid staucht mit
                r = n * self.MODUL / 2
                ry = r * h / self.H
                a = (int(round((px - r) * f)), int(round((py - ry) * f)))
                b = (int(round((px + r) * f)), int(round((py + ry) * f)))
                cv2.rectangle(m, a, b, val, -1, cv2.LINE_AA, S)
            np.maximum(alpha, np.minimum(m, auge), out=alpha)
        return _faerben(alpha)


_LUT = [np.round(np.arange(256) * c / 255).astype(np.uint8) for c in (210, 225, 0)]   # QrEyes.FARBE (BGR)


def _faerben(alpha: np.ndarray) -> np.ndarray:
    """Alpha 0..255 -> BGR in Augenfarbe, per Tabelle (am CM4 statt Fließkomma)."""
    return cv2.merge([cv2.LUT(alpha, t) for t in _LUT])


def rrect(w: float, h: float, r: float, n: int = 10) -> np.ndarray:
    """Abgerundetes Rechteck um (0, 0) wie tools/eyes.py."""
    r = min(r, w / 2, h / 2)
    pts = []
    for cx, cy, a0 in ((w / 2 - r, -h / 2 + r, -90), (w / 2 - r, h / 2 - r, 0),
                       (-w / 2 + r, h / 2 - r, 90), (-w / 2 + r, -h / 2 + r, 180)):
        for a in np.linspace(math.radians(a0), math.radians(a0 + 90), n):
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return np.array(pts)


def _countdown(img: np.ndarray, c: tuple[int, int], r: int, rest: int | None, total: float,
               hell: bool = False) -> None:
    """WLAN-Zeichen mit Ring, der mit der Zeit abläuft, und den Restsekunden darunter."""
    if rest is None:
        return
    vg, hg = ((0, 0, 0), (225, 225, 225)) if hell else ((255, 255, 255), (60, 60, 60))
    akzent = (0, 160, 255) if rest > 10 else (0, 0, 255)          # orange, die letzten 10 s rot
    if not hell:
        cv2.circle(img, c, r + 4, (0, 0, 0), -1, cv2.LINE_AA)
    cv2.circle(img, c, r, hg, 3, cv2.LINE_AA)
    anteil = max(0.0, min(1.0, rest / max(1.0, total)))
    if anteil > 0:
        cv2.ellipse(img, c, (r, r), -90, 0, 360 * anteil, akzent, 3, cv2.LINE_AA)
    # WLAN-Zeichen: drei Bögen und ein Punkt, oben im Ring
    x, y = c[0], c[1] + 1
    for rr in (6, 11, 16):
        cv2.ellipse(img, (x, y), (rr, rr), 0, 225, 315, vg, 2, cv2.LINE_AA)
    cv2.circle(img, (x, y), 2, vg, -1, cv2.LINE_AA)
    t = str(rest)
    (tw, th), _ = cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    cv2.putText(img, t, (x - tw // 2, y + 6 + th), cv2.FONT_HERSHEY_SIMPLEX, 0.5, vg, 1, cv2.LINE_AA)


def _qr_image(text: str, size: int) -> np.ndarray | None:
    try:
        enc = cv2.QRCodeEncoder.create()
        q = enc.encode(text)
    except (cv2.error, AttributeError):
        return None
    q = cv2.resize(q, (size, size), interpolation=cv2.INTER_NEAREST)
    return cv2.cvtColor(q, cv2.COLOR_GRAY2BGR)
