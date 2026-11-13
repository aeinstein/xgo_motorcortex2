"""WLAN-Einrichtung über die Tasten am Rider, ohne bestehende Netzverbindung.

Ablauf (wie beim Hersteller, aber ohne pyzbar und ohne wpa_supplicant.conf zu überschreiben):
  C+D halten -> Kamera sucht einen WLAN-QR-Code (Handy: WLAN teilen) -> NetworkManager-Profil anlegen.
  Ohne Code nach [wlan] qr_timeout_s -> Hotspot mit Zufallspasswort; am Display SSID, Passwort, QR zum
  Beitreten und die Adresse von web/wlan.php, wo man das WLAN einträgt.
Den root-Teil macht deploy/xgo-wlan (sudo-Regel deploy/sudoers-xgonav-wlan); Passwörter gehen über stdin,
nie über die Kommandozeile oder ins Log.

Kosten am Pi: ohne Einrichtung nur der Tastenvergleich je Schleifendurchlauf. Während der Einrichtung ruht
die Bildverarbeitung; QR-Suche höchstens [wlan] qr_hz, Display-Bild nur bei Änderung.
"""
from __future__ import annotations

import logging
import math
import secrets
import subprocess
import threading
import time

import cv2
import numpy as np

log = logging.getLogger(__name__)

HOTSPOT_IP = "10.42.0.1"                      # NetworkManager-Hotspot (ipv4.method shared)
WLAN_URL = f"{HOTSPOT_IP}/xgonav/wlan.php"
_PW_ZEICHEN = "abcdefghjkmnpqrstuvwxyz23456789"   # ohne l/1/i/o/0: vom Display abtippen


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
    """Gegenstück zu parse_wifi_qr (Hotspot zum Beitreten)."""
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
    """Zustände: qr -> verbinde -> ok | fehler(-> qr);  qr -> hotspot_start -> hotspot -> ok;  -> ende.
    step() läuft in der Hauptschleife (statt App._step), lcd_frame() im LCD-Thread."""

    OK_S = 20.0           # Erfolg so lange anzeigen
    FEHLER_S = 8.0
    STATUS_S = 5.0        # im Hotspot so oft nachsehen, ob über wlan.php verbunden wurde

    def __init__(self, cfg, helfer, now: float):
        self.cfg, self.helfer = cfg, helfer
        self.state, self.t_state = "qr", now
        self.text = "WLAN-QR vor die Kamera halten"
        self.detail = ""
        self.ssid = ""
        self.ip = ""
        self.hotspot_pw = ""
        self._t_qr = 0.0
        self._t_status = 0.0
        self._qr = cv2.QRCodeDetector()
        self._job = ""                      # laufender Helfer-Auftrag
        self._off_pending = False           # Hotspot schließen, sobald der Helfer frei ist
        self.rest_s: int | None = None      # Countdown (QR-Suche, Hotspot) in ganzen Sekunden
        self._bgr: np.ndarray | None = None
        self.version = 0                    # erhöht sich bei jeder Änderung der Anzeige
        self._lcd_cache: tuple[int, np.ndarray] | None = None
        log.warning("WLAN-Einrichtung: QR-Code suchen (%.0f s, danach Hotspot)", cfg.qr_timeout_s)

    # -------------------------------------------------------------- Zustand
    @property
    def done(self) -> bool:
        return self.state == "ende" and not self.helfer.busy and not self._off_pending

    def _set(self, state: str, now: float, text: str, detail: str = "") -> None:
        self.state, self.t_state, self.text, self.detail = state, now, text, detail
        self.version += 1
        log.warning("WLAN-Einrichtung: %s %s", text, detail)

    def status(self) -> dict:
        return {"state": self.state, "text": self.text, "detail": self.detail, "ssid": self.ssid, "ip": self.ip,
                "hotspot": self.cfg.hotspot_ssid if self.state == "hotspot" else None, "rest_s": self.rest_s}

    def _run(self, cmd: str, stdin: str = "", timeout: float = 60.0) -> None:
        self._job = cmd
        self.helfer.start(cmd, stdin, timeout)

    def cancel(self, now: float) -> None:
        if self.state == "ende":
            return
        self._off_pending = self.state in ("hotspot", "hotspot_start")
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
        if self._off_pending and not self.helfer.busy:
            self._off_pending = False
            self._run("hotspot-off")
        st, dt = self.state, now - self.t_state
        if st == "qr":
            if bgr is not None and now - self._t_qr >= 1.0 / max(0.1, self.cfg.qr_hz):
                self._t_qr = now
                self._scan(bgr, now)
            if self.state == "qr" and dt >= self.cfg.qr_timeout_s:
                self.hotspot_pw = "".join(secrets.choice(_PW_ZEICHEN) for _ in range(8))
                self._set("hotspot_start", now, "Hotspot startet ...")
                self._run("hotspot", f"{self.cfg.hotspot_ssid}\n{self.hotspot_pw}\n")
        elif st == "hotspot":
            if dt >= self.cfg.hotspot_timeout_s:
                self._off_pending = True
                self._set("ende", now, "Hotspot beendet (Zeit um)")
            elif not self.helfer.busy and now - self._t_status >= self.STATUS_S:
                self._t_status = now
                self._run("status", timeout=10.0)
        elif st == "ok" and dt >= self.OK_S:
            self._set("ende", now, "Fertig", self.detail)
        elif st == "fehler" and dt >= self.FEHLER_S:
            self._set("qr", now, "WLAN-QR vor die Kamera halten")
        frist = {"qr": self.cfg.qr_timeout_s, "hotspot": self.cfg.hotspot_timeout_s}.get(self.state)
        rest = None if frist is None else max(0, math.ceil(frist - (now - self.t_state)))
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
                self._set("ok", now, f"Verbunden: {self.ssid}", self.ip)
            else:
                self._set("fehler", now, f"Fehler: {self.ssid}", last)
        elif job == "hotspot" and self.state == "hotspot_start":
            if rc == 0:
                self._t_status = now
                self._set("hotspot", now, f"Hotspot {self.cfg.hotspot_ssid}", WLAN_URL)
            else:
                self._set("ende", now, "Hotspot fehlgeschlagen", last)
        elif job == "status" and self.state == "hotspot" and rc == 0:
            con = kv.get("con", "")
            if con and con != "xgo-hotspot" and kv.get("ip"):
                self.ssid, self.ip = kv.get("ssid", con), kv["ip"]
                self._set("ok", now, f"Verbunden: {self.ssid}", self.ip)
        elif rc != 0:
            log.warning("WLAN-Einrichtung: %s: %s", job, last)

    def _scan(self, bgr: np.ndarray, now: float) -> None:
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
        text = self._decode(gray)
        if text is None:
            # kleiner Code (Handy weiter weg, Weitwinkel): Bildmitte doppelt so groß
            h, w = gray.shape[:2]
            mitte = gray[h // 4: h - h // 4, w // 4: w - w // 4]
            text = self._decode(cv2.resize(mitte, None, fx=2, fy=2, interpolation=cv2.INTER_LINEAR))
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

    def _decode(self, gray: np.ndarray) -> str | None:
        try:
            text, pts, _ = self._qr.detectAndDecode(gray)
        except cv2.error:
            return None
        return text or None

    # -------------------------------------------------------------- Display (LCD-Thread)
    def lcd_frame(self) -> tuple[int, np.ndarray]:
        """(version, BGR 320x240); neu gezeichnet nur, wenn sich etwas geändert hat."""
        v = self.version
        if self._lcd_cache is not None and self._lcd_cache[0] == v:
            return self._lcd_cache
        img = self._draw()
        self._lcd_cache = (v, img)
        return self._lcd_cache

    def _draw(self) -> np.ndarray:
        font = cv2.FONT_HERSHEY_SIMPLEX
        if self.state == "hotspot":
            img = np.full((240, 320, 3), 255, np.uint8)
            qr = _qr_image(wifi_qr_text(self.cfg.hotspot_ssid, self.hotspot_pw), 140)
            if qr is not None:
                img[6:146, 4:144] = qr
            zeilen = [("WLAN:", 0.45, 30), (_ascii(self.cfg.hotspot_ssid), 0.55, 52), ("Passwort:", 0.45, 82),
                      (self.hotspot_pw, 0.7, 108)]
            for t, sc, y in zeilen:
                cv2.putText(img, t, (152, y), font, sc, (0, 0, 0), 2 if sc > 0.5 else 1, cv2.LINE_AA)
            _countdown(img, (282, 160), 28, self.rest_s, self.cfg.hotspot_timeout_s, hell=True)
            cv2.putText(img, "Dann im Browser:", (8, 172), font, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
            cv2.putText(img, WLAN_URL, (8, 202), font, 0.5, (160, 0, 0), 1, cv2.LINE_AA)
            cv2.putText(img, "B = Abbruch", (8, 230), font, 0.4, (90, 90, 90), 1, cv2.LINE_AA)
            return img
        if self.state == "qr" and self._bgr is not None:
            img = cv2.resize(self._bgr, (320, 240), interpolation=cv2.INTER_AREA)
            cv2.rectangle(img, (80, 60), (240, 180), (0, 255, 255), 1)      # Mitte: dort sucht er vergrößert
        else:
            img = np.zeros((240, 320, 3), np.uint8)
        if self.state == "qr":
            _countdown(img, (280, 40), 32, self.rest_s, self.cfg.qr_timeout_s)
        farbe = {"ok": (0, 200, 0), "fehler": (0, 0, 255)}.get(self.state, (255, 255, 255))
        cv2.rectangle(img, (0, 180), (320, 240), (0, 0, 0), -1)
        cv2.putText(img, _ascii(self.text)[:34], (6, 202), font, 0.5, farbe, 1, cv2.LINE_AA)
        unten = _ascii(self.detail)[:40]
        if self.state == "qr":
            unten = (unten + "  " if unten else "") + "danach Hotspot, B = Abbruch"
        cv2.putText(img, unten, (6, 228), font, 0.42, (200, 200, 200), 1, cv2.LINE_AA)
        return img


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
