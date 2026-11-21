"""Sichtfeld von FloorNet und xgonav als halbtransparente Masken (RGBA, 640 x 480) zum Überlagern beliebiger
Eingangsbilder (entzerrt, wie sie ins Netz gehen). Nur am PC, am Rider läuft davon nichts.

  grün: was ins Netz geht (Ausschnitt; die Helligkeitsnorm aus net.prep ist aus, train.py --norm 0, der C++-Kern kennt sie nicht)
  rot:  was das Netz ausgeben kann (80 Spalten, 64 Zeilenfächer; Skala links), dazu die 5 Sektoren von xgonav

Zweite Lage (nominal, Nick 0): Horizont und Bodenentfernungen 0,5 / 1,0 / 1,5 m (1,5 = grid.max_range_m).

  python tools/floornet/sichtfeld.py                                   # docs/img/floornet_maske*.png neu
  python tools/floornet/sichtfeld.py --bild a.png --out vorschau.png   # beide Lagen über ein Bild legen

Die Konstanten kommen aus net.py (per ast gelesen, ohne torch zu laden), die Geometrie aus den Rider-Defaults
(xgonav/config.py); bei Änderungen dort neu erzeugen."""
from __future__ import annotations

import argparse
import ast
import math
import os
import sys

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from xgonav.config import Config            # noqa: E402
from xgonav.geometry import CamGeometry      # noqa: E402

W, H = 640, 480
NAMEN = ("CROP_Y", "CROP_H", "DOWN", "IN_W", "IN_H", "NCOLS", "NBINS", "ROW_TOP", "ROW_BOTTOM", "BIN_RATIO")


def net_konstanten() -> dict:
    """Konstanten aus net.py, ohne torch zu importieren."""
    baum = ast.parse(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "net.py"), encoding="utf-8").read())
    k = {}
    for n in baum.body:
        if isinstance(n, ast.Assign):
            ziele = n.targets[0].elts if isinstance(n.targets[0], ast.Tuple) else n.targets
            werte = n.value.elts if isinstance(n.value, ast.Tuple) else [n.value]
            for z, v in zip(ziele, werte):
                if isinstance(z, ast.Name) and z.id in NAMEN:
                    k[z.id] = ast.literal_eval(v)
    fehlt = set(NAMEN) - set(k)
    if fehlt:
        raise SystemExit("net.py: fehlt %s" % sorted(fehlt))
    return k


def bin_edges(k: dict) -> np.ndarray:
    """wie net.bin_edges"""
    w = k["BIN_RATIO"] ** np.arange(k["NBINS"])
    return k["ROW_TOP"] + np.concatenate([[0.0], np.cumsum(w)]) * (k["ROW_BOTTOM"] - k["ROW_TOP"]) / w.sum()


def fuellen(rgba, y0, y1, farbe, alpha):
    """Zeilen y0..y1 halbtransparent einfärben, über das schon Gezeichnete gelegt (Alpha-over): wo zwei Flächen
    sich überlappen, entsteht die Mischfarbe, die man beim Übereinanderlegen sähe."""
    t = rgba[int(y0):int(y1)].astype(np.float32) / 255.0
    a_alt = t[:, :, 3:4]
    a = alpha + a_alt * (1 - alpha)
    farbe = np.array(farbe, np.float32) / 255.0
    t[:, :, :3] = np.where(a > 0, (farbe * alpha + t[:, :, :3] * a_alt * (1 - alpha)) / np.maximum(a, 1e-6), 0)
    t[:, :, 3:4] = a
    rgba[int(y0):int(y1)] = np.round(t * 255).astype(np.uint8)


def linie(rgba, p0, p1, farbe, alpha=1.0, dicke=1, strich=0):
    """Linie mit eigener Deckkraft in die RGBA-Lage (überschreibt Farbe und Alpha)."""
    m = np.zeros((H, W), np.uint8)
    if strich:
        (x0, y0), (x1, y1) = p0, p1
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) // strich))
        for i in range(0, n, 2):
            a, b = i / n, min(1.0, (i + 1) / n)
            cv2.line(m, (round(x0 + (x1 - x0) * a), round(y0 + (y1 - y0) * a)),
                     (round(x0 + (x1 - x0) * b), round(y0 + (y1 - y0) * b)), 255, dicke)
    else:
        cv2.line(m, p0, p1, 255, dicke)
    rgba[m > 0, :3] = farbe
    rgba[m > 0, 3] = int(round(alpha * 255))


def text(rgba, s, x, y, farbe=(255, 255, 255), gross=0.4):
    """Text mit dunklem Rand, voll deckend."""
    schrift = np.zeros((H, W), np.uint8)
    cv2.putText(schrift, s, (x, y), cv2.FONT_HERSHEY_SIMPLEX, gross, 255, 1, cv2.LINE_AA)
    glow = cv2.GaussianBlur(cv2.dilate(schrift, np.ones((3, 3), np.uint8)), (3, 3), 0)   # 1 px Hof, mittig
    hof = (glow > 0) & (schrift < 128)
    rgba[hof, :3] = 0
    rgba[hof, 3] = np.maximum(rgba[hof, 3], (glow[hof] * 0.6).astype(np.uint8))
    voll = schrift > 0
    a = schrift[voll].astype(np.float32)[:, None] / 255.0
    rgba[voll, :3] = (np.array(farbe, np.float32) * a).astype(np.uint8)      # Kante zum Hof hin dunkel
    rgba[voll, 3] = np.maximum(rgba[voll, 3], np.maximum(schrift[voll], 153))


def maske(k: dict, sektoren: int) -> np.ndarray:
    """Feste Lage (unabhängig von Nick und Höhe). RGBA, Farben als RGB."""
    rgba = np.zeros((H, W, 4), np.uint8)
    y0, y1 = k["CROP_Y"], k["CROP_Y"] + k["CROP_H"]
    fuellen(rgba, y0, y1, (0, 220, 0), 0.25)                      # Eingang (ganzer Ausschnitt)
    fuellen(rgba, k["ROW_TOP"], H, (230, 30, 30), 0.25)           # Ausgabe darüber: Mischfarbe = beides
    linie(rgba, (0, y0), (W - 1, y0), (0, 255, 0), 0.9, 2)
    linie(rgba, (0, int(k["ROW_TOP"])), (W - 1, int(k["ROW_TOP"])), (255, 40, 40), 0.9, 2)
    sp = W / k["NCOLS"]
    for i in range(1, k["NCOLS"]):                                 # Spaltengrenzen des Netzes
        x = round(i * sp)
        linie(rgba, (x, int(k["ROW_TOP"])), (x, H - 1), (255, 60, 60), 0.55)
    for e in bin_edges(k):                                         # Zeilenfächer als Skala links
        y = min(H - 1, int(round(e)))
        linie(rgba, (0, y), (12, y), (255, 120, 120), 0.9)
    for i in range(1, sektoren):                                   # Sektoren von xgonav (freespace.sectors)
        x = round(i * W / sektoren)
        linie(rgba, (x, y0), (x, H - 1), (255, 255, 255), 0.8, 1, strich=8)
    text(rgba, "Netzeingang ab Zeile %d (%dx%d, 2:1)" % (y0, k["IN_W"], k["IN_H"]), 18, y0 + 16, (120, 255, 120))
    text(rgba, "Ausgabe Zeile %d-%d: %d Spalten x %d Faecher" % (k["ROW_TOP"], k["ROW_BOTTOM"], k["NCOLS"],
                                                                  k["NBINS"]), 18, int(k["ROW_TOP"]) + 16,
         (255, 150, 150))
    text(rgba, "%d Sektoren (xgonav)" % sektoren, W - 150, H - 8)
    return rgba


def maske_entfernung(geo: CamGeometry, meter=(0.5, 1.0, 1.5)) -> np.ndarray:
    """Zusatzlage bei Nick 0: Horizont und Bodenentfernungen ab Kamerafußpunkt."""
    rgba = np.zeros((H, W, 4), np.uint8)
    tilt = math.radians(geo.tilt_deg)
    hz = int(round(geo.horizon_row()))
    linie(rgba, (0, hz), (W - 1, hz), (255, 220, 0), 0.9, 1, strich=10)
    text(rgba, "Horizont (Nick 0)", W - 150, hz - 5, (255, 220, 0))
    for d in meter:
        y = int(round(H / 2 + geo.f * math.tan(math.atan2(geo.cam_h, d) + tilt)))
        if 0 <= y < H:
            linie(rgba, (20, y), (W - 61, y), (0, 200, 255), 0.9, 1)
            text(rgba, "%.1f m" % d, W - 56, y + (-3 if d == max(meter) else 12), (0, 200, 255))
    text(rgba, "Kamerahoehe %.0f mm, f %.0f px, bei Nick 0 Blick %.2f Grad %s" % (
        geo.cam_h * 1000, geo.f, abs(geo.tilt_deg), "nach oben" if geo.tilt_deg >= 0 else "nach unten"),
        18, 14, (0, 200, 255))
    return rgba


def ueberlagern(bild: np.ndarray, *lagen: np.ndarray) -> np.ndarray:
    """BGR-Bild + RGBA-Lagen -> BGR."""
    out = bild.astype(np.float32)
    for l in lagen:
        a = l[:, :, 3:4].astype(np.float32) / 255.0
        out = out * (1 - a) + l[:, :, 2::-1].astype(np.float32) * a
    return np.clip(out + 0.5, 0, 255).astype(np.uint8)


def speichern_rgba(pfad: str, rgba: np.ndarray) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(pfad)), exist_ok=True)
    cv2.imwrite(pfad, cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGRA))
    print(pfad)


def lage_korrigieren(bild: np.ndarray, geo: CamGeometry, nick_dev: float, roll_deg: float) -> np.ndarray:
    """Bild so drehen, als wäre es bei Nick 0 (pitch_dev, rad, + = schaut höher) und Roll 0 (Grad, + = rechts tief)
    aufgenommen: Roll um die Bildmitte zurückdrehen, Nick als Drehung der Kamera um die waagerechte Achse
    (Homographie K R K^-1). Danach liegt der echte Horizont dort, wo maske_entfernung ihn zeichnet."""
    f, cx, cy = geo.f, W / 2, H / 2
    K = np.array([[f, 0, cx], [0, f, cy], [0, 0, 1.0]])
    c, s = math.cos(nick_dev), math.sin(nick_dev)     # Zeile f·tan(tilt + nick) -> f·tan(tilt) (Test)
    Rx = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    a = math.radians(roll_deg)                     # Roll + = rechts tief: Szene erscheint gegen den Uhrzeigersinn
    Rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1.0]])
    Hm = K @ Rx @ Rz @ np.linalg.inv(K)
    return cv2.warpPerspective(bild, Hm, (W, H), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)


def horizont_zeile(geo: CamGeometry, nick_dev: float) -> float:
    return H / 2 + geo.f * math.tan(math.radians(geo.tilt_deg) + nick_dev)


def lage_aus_meta(meta: str, datei: str) -> tuple[float, float]:
    """pitch_dev (rad) und IMU-Roll (Grad, erste Probe zum Bild) einer Aufnahme aus meta.jsonl."""
    import json
    name = os.path.basename(datei)
    with open(meta, encoding="utf-8") as fh:
        for z in fh:
            if '"%s"' % name in z:
                d = json.loads(z)
                imu = d.get("imu") or []
                return float(d.get("pitch_dev") or 0.0), float(imu[0][1]) if imu else 0.0
    raise SystemExit("%s: %s nicht gefunden" % (meta, name))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--ziel", default=os.path.join(ROOT, "docs", "img"), help="Ordner für die Masken")
    ap.add_argument("--kamerahoehe", type=float, default=0.0, help="mm; 0 = Rider-Default (Fahrhöhe)")
    ap.add_argument("--bild", help="Eingangsbild 640x480 (entzerrt): Vorschau statt Masken schreiben")
    ap.add_argument("--out", default="vorschau.png", help="Vorschau-Datei zu --bild")
    ap.add_argument("--meta", help="meta.jsonl der Fahrt: Nick und Roll des Bildes herausrechnen (wie --nick/--roll)")
    ap.add_argument("--nick", type=float, default=0.0, help="pitch_dev des Bildes in Grad (herausrechnen)")
    ap.add_argument("--roll", type=float, default=0.0, help="IMU-Roll des Bildes in Grad, + = rechts tief")
    a = ap.parse_args()

    cfg = Config()
    geo = CamGeometry.from_cfg(cfg)
    if a.kamerahoehe > 0:
        geo.cam_h_u8 = ()
        geo.mount_offset_mm = a.kamerahoehe - geo.body_height_mm
    k = net_konstanten()
    m1, m2 = maske(k, cfg.freespace.sectors), maske_entfernung(geo, (0.5, 1.0, cfg.grid.max_range_m))
    if a.bild:
        bild = cv2.imread(a.bild)
        if bild is None or bild.shape[:2] != (H, W):
            raise SystemExit("%s: kein Bild 640x480" % a.bild)
        nick, roll = (lage_aus_meta(a.meta, a.bild) if a.meta else (math.radians(a.nick), a.roll))
        if nick or roll:
            print("Nick %.2f° (Horizont vorher Zeile %.1f) und Roll %.2f° herausgerechnet" % (
                math.degrees(nick), horizont_zeile(geo, nick), roll))
            bild = lage_korrigieren(bild, geo, nick, roll)
        cv2.imwrite(a.out, ueberlagern(bild, m1, m2))
        print(a.out)
        return
    speichern_rgba(os.path.join(a.ziel, "floornet_maske.png"), m1)
    speichern_rgba(os.path.join(a.ziel, "floornet_maske_entfernung.png"), m2)


if __name__ == "__main__":
    main()
