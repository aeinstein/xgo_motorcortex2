"""Augen-Animationen für die Emotionen auf dem 2"-Display (ST7789, 320x240, RGB565).

Nur stilisierte Augen, einfarbig auf Schwarz (mit Antialiasing). Erzeugt für jede
Emotion eine Bildfolge direkt im Format von tools/emotion.sh build:

  OUTDIR/<Name>/0001.raw ...   je 153600 Byte, RGB565 LE (wie ffmpeg -pix_fmt rgb565)

  python tools/eyes.py OUTDIR                      # alle Emotionen, Farbe Türkis
  python tools/eyes.py OUTDIR --color 255,160,0    # andere Farbe (R,G,B)
  python tools/eyes.py OUTDIR --only sad --only shy
  python tools/eyes.py OUTDIR --png                # zusätzlich <Name><n>.png
  python tools/eyes.py OUTDIR --preview VORSCHAU   # Kontaktabzug (PNG) und MP4 je Emotion
  python tools/eyes.py --list                      # Emotionen mit Bildanzahl
  python tools/eyes.py --gaze DIR                  # Augen-Atlas des Modus pet (Blick x Lid)

Abgespielt wird wie bisher mit tools/emotion.sh play OUTDIR/<Name> (25 fps). Jede
Animation beginnt mit den neutralen Augen; außer sleep (endet geschlossen) endet
sie auch dort, lässt sich also nahtlos wiederholen.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import cv2
import numpy as np

W, H = 320, 240
FPS = 25                    # Standard von tools/emotion.sh play
SS = 4                      # Überabtastung: gezeichnet wird in 4x, dann INTER_AREA
SHIFT = 2                   # Subpixel-Bits für cv2-Zeichenfunktionen
FARBE = (0, 225, 210)       # Türkis (R, G, B)

# Neutrales Auge (Bildschirm-Koordinaten). L = links im Bild, R = rechts.
CX_L, CX_R, CY = 98.0, 222.0, 118.0
NEUTRAL = dict(dx=0.0, dy=0.0, w=78.0, h=96.0, r=0.36, rot=0.0, sc=1.0,
               top=0.0, tslope=0.0, bot=0.0, bcurve=0.0,
               pupil=0.0, ppx=0.0, ppy=0.0, hl=0.0,
               kind="eye", mix=0.0, spin=0.0, tear=-1.0, blink=0.0)
MIN_H = 7.0                 # geschlossenes Lid = schmaler Strich


# ---------------------------------------------------------------- Hilfen

def smooth(x: float) -> float:
    x = min(1.0, max(0.0, x))
    return x * x * (3 - 2 * x)


def lerp(a: float, b: float, x: float) -> float:
    return a + (b - a) * x


def track(t: float, keys: list[tuple[float, float]]) -> float:
    """Stückweise mit weichem Übergang zwischen Schlüsselwerten (Zeit in s)."""
    if t <= keys[0][0]:
        return keys[0][1]
    for (t0, v0), (t1, v1) in zip(keys, keys[1:]):
        if t <= t1:
            return lerp(v0, v1, smooth((t - t0) / (t1 - t0)))
    return keys[-1][1]


def envelope(t: float, dauer: float, rein: float, raus: float) -> float:
    """0 -> 1 in 'rein' s, halten, 1 -> 0 in den letzten 'raus' s."""
    return smooth(t / rein) * (1 - smooth((t - (dauer - raus)) / raus))


def blink(t: float, t0: float, dauer: float = 0.2) -> float:
    """Lidschlag um t0: 0 (offen) .. 1 (zu) .. 0."""
    x = (t - t0) / dauer
    return math.sin(math.pi * x) ** 2 if 0 <= x <= 1 else 0.0


def pose(base: dict, ziel: dict, x: float) -> dict:
    """Zahlwerte von base nach ziel überblenden (andere Schlüssel aus ziel)."""
    p = dict(base)
    for k, v in ziel.items():
        p[k] = lerp(base[k], v, x) if isinstance(v, (int, float)) else v
    return p


def neutral(**kw) -> dict:
    p = dict(NEUTRAL)
    p.update(kw)
    return p


# ---------------------------------------------------------------- Zeichnen

def _pts(xy: np.ndarray) -> np.ndarray:
    return np.round(xy * SS * (1 << SHIFT)).astype(np.int32).reshape(-1, 1, 2)


def _fill(m: np.ndarray, xy: np.ndarray, val: int) -> None:
    cv2.fillPoly(m, [_pts(xy)], val, cv2.LINE_AA, SHIFT)


def _line(m: np.ndarray, xy: np.ndarray, dicke: float, val: int = 255) -> None:
    cv2.polylines(m, [_pts(xy)], False, val, max(1, int(round(dicke * SS))), cv2.LINE_AA, SHIFT)


def _circle(m: np.ndarray, x: float, y: float, r: float, val: int) -> None:
    f = SS * (1 << SHIFT)
    cv2.circle(m, (int(round(x * f)), int(round(y * f))), max(1, int(round(r * f))), val, -1,
               cv2.LINE_AA, SHIFT)


def rrect(w: float, h: float, r: float, n: int = 14) -> np.ndarray:
    """Abgerundetes Rechteck um (0, 0); r = Eckradius."""
    r = min(r, w / 2, h / 2)
    ecken = [(w / 2 - r, -h / 2 + r, -90), (w / 2 - r, h / 2 - r, 0),
             (-w / 2 + r, h / 2 - r, 90), (-w / 2 + r, -h / 2 + r, 180)]
    pts = []
    for cx, cy, a0 in ecken:
        for a in np.linspace(math.radians(a0), math.radians(a0 + 90), n):
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return np.array(pts)


def heart(w: float, h: float) -> np.ndarray:
    t = np.linspace(0, 2 * math.pi, 120)
    x = 16 * np.sin(t) ** 3
    y = -(13 * np.cos(t) - 5 * np.cos(2 * t) - 2 * np.cos(3 * t) - np.cos(4 * t))
    x = x / 32 * w
    y = (y + 2.5) / 30 * h
    return np.stack([x, y], 1)


def arc(rx: float, ry: float, n: int = 40) -> np.ndarray:
    """Halbellipse um (0, 0): ry > 0 nach oben gewölbt (^, fröhlich), < 0 nach unten (‿)."""
    a = np.linspace(0, math.pi, n)
    return np.stack([rx * np.cos(a), -ry * np.sin(a) + ry / 2], 1)


def spiral(r: float, spin: float, windungen: float = 2.6, n: int = 160) -> np.ndarray:
    a = np.linspace(0, windungen * 2 * math.pi, n)
    rr = r * a / a[-1]
    return np.stack([rr * np.cos(a + spin), rr * np.sin(a + spin)], 1)


def draw_kind(kind: str, p: dict, s: int) -> np.ndarray:
    """Maske eines Auges (Vollbild, SS-fach) in Bildkoordinaten."""
    m = np.zeros((H * SS, W * SS), np.uint8)
    cx = (CX_L if s < 0 else CX_R) + p["dx"]
    cy = CY + p["dy"]
    sc = p["sc"]
    w, h = p["w"] * sc, p["h"] * sc
    hb = max(MIN_H, h * (1 - p["blink"]))           # Lidschlag staucht senkrecht
    rot = math.radians(p["rot"] * -s)               # gespiegelt: beide Augen kippen nach außen/innen

    def g(xy: np.ndarray) -> np.ndarray:            # lokal (um Augenmitte, x nach rechts) -> Bild
        c, sn = math.cos(rot), math.sin(rot)
        x, y = xy[:, 0], xy[:, 1]
        return np.stack([cx + c * x - sn * y, cy + sn * x + c * y], 1)

    if kind == "eye":
        _fill(m, g(rrect(w, hb, p["r"] * min(w, hb) / 2)), 255)
        if p["pupil"] * min(w, h) > 3:
            pr = p["pupil"] * min(w, h) / 2
            px, py = p["ppx"] * w / 2, p["ppy"] * hb / 2
            (gx, gy), = g(np.array([[px, py]]))
            _circle(m, gx, gy, pr, 0)
            if p["hl"] > 0:                         # Lichtreflexe in Augenfarbe
                for ox, oy, rf in ((-0.38, -0.38, 0.34), (0.42, 0.40, 0.16)):
                    (hx, hy), = g(np.array([[px + ox * pr, py + oy * pr]]))
                    _circle(m, hx, hy, pr * rf * p["hl"], 255)
        # Oberlid: Gerade, tslope > 0 = außen tiefer (traurig), < 0 = innen tiefer (böse)
        if p["top"] > 0 or p["tslope"] != 0:
            k = math.tan(math.radians(p["tslope"]))
            y0 = -hb / 2 + p["top"] * hb
            xs = np.array([-w, w])
            ys = y0 + k * xs * s                    # xs * s = Abstand nach außen
            poly = np.array([[xs[0], ys[0]], [xs[1], ys[1]], [xs[1], -h * 2], [xs[0], -h * 2]])
            _fill(m, g(poly), 0)
        # Unterlid: von unten, bcurve > 0 wölbt nach oben (lächelnd)
        if p["bot"] > 0 or p["bcurve"] > 0:
            u = np.linspace(-1.2, 1.2, 50)
            y = hb / 2 - p["bot"] * hb - p["bcurve"] * hb * np.clip(1 - u ** 2, -1, 1)
            poly = np.vstack([np.stack([u * w / 2, y], 1), [[w, h * 2], [-w, h * 2]]])
            _fill(m, g(poly), 0)
    elif kind == "heart":
        _fill(m, g(heart(w * 1.15, h * 0.95)), 255)
    elif kind == "happy":
        _line(m, g(arc(w * 0.40, h * 0.30)), 14 * sc)
    elif kind == "closed":
        _line(m, g(arc(w * 0.42, -h * 0.13) + [0, h * 0.12]), 8 * sc)
    elif kind == "spiral":
        _line(m, g(spiral(min(w, h) * 0.52, p["spin"] * -s)), 7 * sc)
    elif kind == "chev":                            # > <, Spitze zeigt nach innen
        a = -s
        pts = np.array([[-a * w * 0.38, -h * 0.30], [a * w * 0.38, 0], [-a * w * 0.38, h * 0.30]])
        _line(m, g(pts), 12 * sc)
    else:
        raise ValueError(kind)
    if p["tear"] >= 0:                              # Träne: wächst am inneren Unterrand, fällt dann
        ph = p["tear"]
        gr = smooth(ph / 0.5)
        fall = max(0.0, ph - 0.5) / 0.5
        tx = cx + (-s) * w * 0.22
        ty = cy + hb / 2 + 4 + 9 * gr + 150 * fall ** 2
        rr = 3 + 5 * gr
        _circle(m, tx, ty, rr, 255)
        tri = np.array([[tx - rr * 0.93, ty - rr * 0.35], [tx + rr * 0.93, ty - rr * 0.35],
                        [tx, ty - rr * 2.3]])
        _fill(m, tri, 255)
    return m


def draw_eye(p: dict, s: int) -> np.ndarray:
    """Übergang 'eye' -> p['kind'] über p['mix'] (0..1): das Auge schließt sich,
    erst kurz vor dem Schluss wird zur neuen Form überblendet, die dabei wächst."""
    mix = p["mix"]
    if p["kind"] == "eye" or mix <= 0:
        return draw_kind("eye", p, s)
    if mix >= 1:
        return draw_kind(p["kind"], p, s)
    x = smooth((mix - 0.35) / 0.35)
    a = draw_kind("eye", dict(p, blink=max(p["blink"], mix)), s).astype(np.float32)
    b = draw_kind(p["kind"], dict(p, sc=p["sc"] * (0.7 + 0.3 * mix)), s).astype(np.float32)
    return (a * (1 - x) + b * x).astype(np.uint8)


def render(frame: tuple[dict, dict, dict]) -> np.ndarray:
    """(global, links, rechts) -> Alpha 0..1 in 320x240."""
    gl, pl, pr = frame
    m = np.maximum(draw_eye(pl, -1), draw_eye(pr, 1))
    if gl.get("tilt", 0) or gl.get("gx", 0) or gl.get("gy", 0):
        M = cv2.getRotationMatrix2D((W * SS / 2, H * SS / 2), gl.get("tilt", 0), 1.0)
        M[0, 2] += gl.get("gx", 0) * SS
        M[1, 2] += gl.get("gy", 0) * SS
        m = cv2.warpAffine(m, M, (W * SS, H * SS), flags=cv2.INTER_LINEAR)
    return cv2.resize(m, (W, H), interpolation=cv2.INTER_AREA).astype(np.float32) / 255


# ---------------------------------------------------------------- Emotionen
# Jede Funktion: t (s) -> (global, links, rechts). Dauer steht in EMOTIONS.

def angry(t: float, T: float):
    e = envelope(t, T, 0.35, 0.4)
    zit = e * math.sin(2 * math.pi * 11 * t) * 1.6 * (t > 0.4)
    puls = 0.05 * (blink(t, 0.9, 0.25) + blink(t, 1.45, 0.25))
    z = dict(dy=6, w=84, h=86, r=0.34, top=0.30, tslope=-22, sc=1 + puls)
    l = pose(neutral(), dict(z, dx=4), e)
    r = pose(neutral(), dict(z, dx=-4), e)
    return dict(gx=zit), l, r


def cute(t: float, T: float):
    e = envelope(t, T, 0.4, 0.4)
    z = dict(w=96, h=96, r=1.0, pupil=0.72, ppy=0.08, hl=1.0, sc=1.0 + 0.03 * math.sin(2 * math.pi * 1.5 * t))
    l, r = pose(neutral(), z, e), pose(neutral(), z, e)
    # in der Mitte kurz fröhlich zukneifen (^ ^)
    hap = smooth((t - 1.45) / 0.15) * (1 - smooth((t - 2.05) / 0.15))
    for p in (l, r):
        p.update(kind="happy", mix=hap)
        p["blink"] = hap * 0.6
    tilt = e * 9 * math.sin(2 * math.pi * (t - 0.4) / 1.6) * (t > 0.4)
    return dict(tilt=tilt, gy=-3 * e), l, r


def doubt(t: float, T: float):
    e = envelope(t, T, 0.4, 0.45)
    blick = track(t, [(0.3, 0), (0.7, 16), (1.4, 16), (1.7, -10), (2.2, -10), (2.5, 0)])
    l = pose(neutral(), dict(dy=-10, h=104, tslope=10, top=0.06), e)     # gehobene „Braue“
    r = pose(neutral(), dict(dy=4, top=0.40, tslope=-12, bot=0.18, w=82), e)  # zusammengekniffen
    for p in (l, r):
        p["dx"] += blick * e
        p["dy"] += -5 * e * (abs(blick) > 1)
    return dict(tilt=-4 * e), l, r


def embarrassed(t: float, T: float):
    e = envelope(t, T, 0.25, 0.35)
    blick = track(t, [(0.0, 0), (0.25, -22), (0.5, -22), (0.62, 20), (0.82, 20), (0.95, -20), (1.1, -20), (1.25, 0)])
    chev = smooth((t - 1.2) / 0.15) * (1 - smooth((t - 2.35) / 0.15))
    zit = math.sin(2 * math.pi * 9 * t) * 2.0 * chev
    l, r = neutral(), neutral()
    for p in (l, r):
        p.update(dx=blick * e, dy=6 * e, sc=1 - 0.08 * e, kind="chev", mix=chev, blink=chev * 0.7)
    return dict(gx=zit), l, r


def grievance(t: float, T: float):
    e = envelope(t, T, 0.4, 0.45)
    wackel = math.sin(2 * math.pi * 7 * t) * e
    z = dict(dy=-6, w=90, h=98, r=0.75, top=0.14, tslope=16, bot=0.10, pupil=0.70, ppy=-0.22, hl=1.0)
    l, r = pose(neutral(), z, e), pose(neutral(), z, e)
    for p in (l, r):
        p["bot"] += 0.035 * wackel
        p["dx"] += 0.8 * wackel
    if 0.7 <= t <= 2.6:
        r["tear"] = (t - 0.7) / 1.9
    return dict(), l, r


def hate(t: float, T: float):
    e = envelope(t, T, 0.3, 0.45)
    # Augenrollen: hoch, im Bogen von links nach rechts, dann genervt halb zu
    a = track(t, [(0.2, math.pi), (1.2, 0.0)])
    roll = smooth((t - 0.15) / 0.25) * (1 - smooth((t - 1.15) / 0.3))
    dx, dy = 22 * math.cos(a) * roll, -26 * math.sin(a) * roll - 4 * roll
    flach = smooth((t - 1.1) / 0.35)
    l, r = neutral(), neutral()
    for p in (l, r):
        p.update(dx=dx + 14 * flach * smooth((t - 1.7) / 0.35) * e, dy=dy + 6 * flach * e,
                 top=e * (0.12 * roll + 0.40 * flach), tslope=-10 * flach * e, bot=e * 0.20 * flach,
                 w=lerp(78, 88, e * flach))
    return dict(), l, r


def laugh(t: float, T: float):
    e = envelope(t, T, 0.25, 0.3)
    hupf = -9 * abs(math.sin(2 * math.pi * 3.2 * t)) * e
    kipp = 5 * math.sin(2 * math.pi * 1.6 * t) * e
    l, r = neutral(), neutral()
    for p in (l, r):
        p.update(kind="happy", mix=e, dy=hupf, blink=0.7 * e)
    return dict(tilt=kipp), l, r


def like(t: float, T: float):
    e = envelope(t, T, 0.3, 0.35)
    tt = (t - 0.35) % 0.85                          # Herzschlag: bum-bum, Pause
    schlag = 0.13 * (blink(tt, 0.0, 0.18) + 0.7 * blink(tt, 0.2, 0.18)) * (t > 0.35)
    l, r = neutral(), neutral()
    for p in (l, r):
        p.update(kind="heart", mix=e, sc=1 + (schlag + 0.12 * math.sin(math.pi * min(1, t / 0.3)) * (t < 0.3)) * e,
                 dy=-4 * e + 3 * math.sin(2 * math.pi * 0.8 * t) * e)
    return dict(), l, r


def sad(t: float, T: float):
    e = envelope(t, T, 0.6, 0.5)
    z = dict(dy=14, top=0.28, tslope=22, h=88)
    l, r = pose(neutral(dx=-3), z, e), pose(neutral(dx=3), z, e)
    langsam = blink(t, 1.6, 0.6)
    blick = track(t, [(0.6, 0), (1.2, -6), (2.4, -6), (2.9, 0)])
    for p in (l, r):
        p["blink"] = langsam
        p["dx"] += blick * e
    return dict(), l, r


def shy(t: float, T: float):
    e = envelope(t, T, 0.45, 0.45)
    zurueck = smooth((t - 1.2) / 0.2) * (1 - smooth((t - 1.6) / 0.15))   # kurzer Blick zum Gegenüber
    weg = 1 - zurueck
    z = dict(sc=0.84, top=0.36, tslope=6)
    l, r = pose(neutral(), z, e), pose(neutral(), z, e)
    for p in (l, r):
        p["dx"] += -24 * e * weg
        p["dy"] += 16 * e * weg
        p["top"] -= 0.25 * e * zurueck
        p["blink"] = blink(t, 2.0, 0.3)
    return dict(tilt=6 * e * weg), l, r


def sleep(t: float, T: float):
    # schwere Lider, zweimal hochgeschreckt, dann zu; danach ruhiges Atmen
    top = track(t, [(0.2, 0), (1.0, 0.55), (1.15, 0.12), (2.0, 0.72), (2.15, 0.35), (3.0, 0.88)])
    zu = smooth((t - 2.85) / 0.25)
    atem = 3 * math.sin(2 * math.pi * 0.6 * (t - 3.1)) * smooth((t - 3.0) / 0.3)
    l, r = neutral(), neutral()
    for p in (l, r):
        p.update(top=top, tslope=10 * min(1.0, t / 1.0), dy=8 * smooth(t / 3.0) + atem,
                 kind="closed", mix=zu)
    return dict(), l, r


def surprised(t: float, T: float):
    e = envelope(t, T, 0.12, 0.45)
    knall = track(t, [(0.0, 0), (0.08, -0.12), (0.22, 0.30), (0.4, 0.16), (0.55, 0.20)])   # Überschwinger
    z = dict(w=90, h=90, r=1.0, dy=-8)
    l = pose(neutral(), dict(z, dx=-8), smooth(t / 0.22) * e)
    r = pose(neutral(), dict(z, dx=8), smooth(t / 0.22) * e)
    for p in (l, r):
        p["sc"] = 1 + knall * e
        p["blink"] = blink(t, 1.45, 0.16) + blink(t, 1.68, 0.16)
    return dict(), l, r


def vertigo(t: float, T: float):
    e = envelope(t, T, 0.3, 0.35)
    l, r = neutral(), neutral()
    for i, p in enumerate((l, r)):
        ph = 2 * math.pi * 1.4 * t + i * math.pi
        p.update(kind="spiral", mix=e, spin=6 * t, dx=7 * math.cos(ph) * e, dy=7 * math.sin(ph) * e,
                 sc=1.05)
    return dict(tilt=10 * math.sin(2 * math.pi * 0.9 * t) * e), l, r


# Name -> (Funktion, Dauer in s)
EMOTIONS = {
    "Angry": (angry, 2.4),
    "cute": (cute, 3.0),
    "doubt": (doubt, 3.0),
    "embarrassed": (embarrassed, 2.8),
    "grievance": (grievance, 3.0),
    "hate": (hate, 2.8),
    "laugh": (laugh, 2.2),
    "like": (like, 2.8),
    "sad": (sad, 3.0),
    "shy": (shy, 2.8),
    "sleep": (sleep, 4.0),
    "surprised": (surprised, 2.4),
    "vertigo": (vertigo, 2.4),
}


def frames(name: str):
    fn, dauer = EMOTIONS[name]
    n = int(round(dauer * FPS))
    for i in range(n):                              # erstes und letztes Bild = Ruhelage
        yield render(fn(i * dauer / (n - 1), dauer))


def gaze_alpha(g: int, lid: int) -> np.ndarray:
    """Augen des Modus pet (xgonav/pet.py): Blick g 0..6 (3 = Mitte, größer = rechts im Bild), Lid 0 offen bis
    4 zu (langsames Aufwachen). Runde Augen wie cute, damit der Übergang zu den Emotionen nicht springt."""
    x = (g - 3) / 3.0
    z = dict(w=96.0, h=96.0, r=1.0, pupil=0.72, ppx=0.26 * x, ppy=0.08, hl=1.0, dx=14.0 * x)
    if lid == 4:
        z.update(pupil=0.0, hl=0.0, blink=1.0)
    elif lid > 0:
        z["top"] = 0.24 * lid
    return render((dict(), neutral(**z), neutral(**z)))


def write_gaze(ziel: Path, farbe: tuple[int, int, int]) -> int:
    ziel.mkdir(parents=True, exist_ok=True)
    n = 0
    for g in range(7):
        for lid in range(5):
            (ziel / f"g{g}_l{lid}.raw").write_bytes(to_rgb565(to_bgr(gaze_alpha(g, lid), farbe)))
            n += 1
    return n


# ---------------------------------------------------------------- Ausgabe

def to_bgr(alpha: np.ndarray, farbe: tuple[int, int, int]) -> np.ndarray:
    bgr = np.array(farbe[::-1], np.float32)
    return np.round(alpha[..., None] * bgr).astype(np.uint8)


def to_rgb565(bgr: np.ndarray) -> bytes:
    """Wie ffmpeg -pix_fmt rgb565 (LE), siehe xgonav/lcd.py."""
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2BGR565).tobytes()


def kontaktabzug(bilder: list[np.ndarray], name: str, n: int = 8) -> np.ndarray:
    idx = np.linspace(0, len(bilder) - 1, n).round().astype(int)
    kacheln = []
    for i in idx:
        k = cv2.copyMakeBorder(bilder[i], 2, 2, 2, 2, cv2.BORDER_CONSTANT, value=(70, 70, 70))
        cv2.putText(k, f"{i + 1}", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1, cv2.LINE_AA)
        kacheln.append(k)
    zeilen = [np.hstack(kacheln[j:j + 4]) for j in range(0, n, 4)]
    blatt = np.vstack(zeilen)
    kopf = np.full((28, blatt.shape[1], 3), 30, np.uint8)
    cv2.putText(kopf, f"{name}  ({len(bilder)} Bilder, {len(bilder) / FPS:.1f} s)", (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 1, cv2.LINE_AA)
    return np.vstack([kopf, blatt])


def main() -> int:
    ap = argparse.ArgumentParser(description="Augen-Animationen (RGB565 320x240) für tools/emotion.sh")
    ap.add_argument("outdir", nargs="?", type=Path, help="Zielverzeichnis, je Emotion ein Unterverzeichnis")
    ap.add_argument("--color", default=",".join(map(str, FARBE)), help="Farbe R,G,B (Standard Türkis)")
    ap.add_argument("--only", action="append", help="nur diese Emotion (mehrfach möglich)")
    ap.add_argument("--png", action="store_true", help="zusätzlich <Name><n>.png schreiben")
    ap.add_argument("--no-raw", action="store_true", help="keine .raw-Dateien schreiben")
    ap.add_argument("--preview", type=Path, help="Kontaktabzug <Name>.png und <Name>.mp4 hierhin")
    ap.add_argument("--list", action="store_true", help="Emotionen auflisten")
    ap.add_argument("--gaze", type=Path, help="Augen-Atlas für den Modus pet hierhin (g<Blick>_l<Lid>.raw)")
    a = ap.parse_args()

    if a.list:
        for name, (_, d) in EMOTIONS.items():
            print(f"{name:12s} {int(round(d * FPS)):3d} Bilder  {d:.1f} s")
        return 0
    if a.outdir is None and a.preview is None and a.gaze is None:
        ap.error("OUTDIR, --preview oder --gaze angeben")
    try:
        farbe = tuple(int(c) for c in a.color.split(","))
        assert len(farbe) == 3 and all(0 <= c <= 255 for c in farbe)
    except (ValueError, AssertionError):
        ap.error("--color erwartet R,G,B mit Werten 0..255")
    if a.gaze is not None:
        print(f"Augen-Atlas: {write_gaze(a.gaze, farbe)} Bilder in {a.gaze}")
        if a.outdir is None and a.preview is None:
            return 0
    namen = a.only or list(EMOTIONS)
    for n in namen:
        if n not in EMOTIONS:
            ap.error(f"unbekannte Emotion {n!r}; bekannt: {', '.join(EMOTIONS)}")

    for name in namen:
        bilder = [to_bgr(al, farbe) for al in frames(name)]
        if a.outdir is not None:
            ziel = a.outdir / name
            ziel.mkdir(parents=True, exist_ok=True)
            for alt in list(ziel.glob("*.raw")) + list(ziel.glob(f"{name}*.png")):
                alt.unlink()                        # alte, evtl. längere Folge entfernen
            for i, b in enumerate(bilder, 1):
                if not a.no_raw:
                    (ziel / f"{i:04d}.raw").write_bytes(to_rgb565(b))
                if a.png:
                    cv2.imwrite(str(ziel / f"{name}{i}.png"), b)
        if a.preview is not None:
            a.preview.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(a.preview / f"{name}.png"), kontaktabzug(bilder, name))
            vw = cv2.VideoWriter(str(a.preview / f"{name}.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))
            for b in bilder:
                vw.write(b)
            vw.release()
        print(f"{name}: {len(bilder)} Bilder")
    return 0


if __name__ == "__main__":
    sys.exit(main())
