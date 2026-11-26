"""Closed-Loop-Simulator zum Testen ohne Hardware.

Raycasting-Renderer (Raum + Kisten + Personen-Billboard), Kamera auf
balancierendem Roboter: Nicken (Pendeln + Beschleunigungsneigung), Kurszittern
und vertikale Bewegungsunschärfe. Die Fahrbefehle des SimRobot bewegen die
Kamera. Ein FakeDetector liefert Ground-Truth-Boxen mit realistischer Latenz, SimFloorNet die
Bodengrenze aus der Ground Truth (statt FloorNet, das nur echte Bilder kennt).
"""
from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass, field

import cv2
import numpy as np

from .camera import FrameSource
from .config import CameraCfg
from .detector import Detection, DetectionBatch
from .mathe import w_to_yaw_rate, wrap
from .robot import FW_DEADBAND, SimRobot

SIM_NET_COLS = 80          # Spalten der Bodengrenze wie FloorNet (floornet.NCOLS)


def _blob_tex(seed: int, base, w=256, h=128) -> np.ndarray:
    rng = np.random.default_rng(seed)
    t = np.full((h, w, 3), base, np.uint8)
    for _ in range(140):
        c = tuple(int(x) for x in np.clip(np.array(base) + rng.integers(-90, 90, 3), 0, 255))
        cv2.circle(t, (int(rng.integers(0, w)), int(rng.integers(0, h))), int(rng.integers(2, 12)), c, -1)
    for x in range(0, w, 64):  # vertikale Kanten (Türrahmen/Möbel)
        cv2.line(t, (x, 0), (x, h), (40, 40, 40), 2)
    return cv2.GaussianBlur(t, (3, 3), 0)


@dataclass
class Person:
    path: list[tuple[float, float]]
    speed: float = 0.35
    x: float = 0.0
    y: float = 0.0
    seg: int = 0
    pause_until: float = 0.0
    cls: str = "person"       # Klasse für FakeDetector; für Katze oder Hund height_m/width_m anpassen
    height_m: float = 1.72
    width_m: float = 0.50
    loop: bool = True         # False: bleibt am letzten Punkt
    vanish: bool = False      # mit loop False: am letzten Punkt verschwinden (aus dem Raum, nicht mehr zu sehen)

    def __post_init__(self):
        self.x, self.y = self.path[0]

    def step(self, t: float, dt: float) -> None:
        if not self.loop and self.seg >= len(self.path) - 1:
            if self.vanish:
                self.x, self.y = -50.0, -50.0
            return
        if t < self.pause_until:
            return
        tx, ty = self.path[(self.seg + 1) % len(self.path)]
        dx, dy = tx - self.x, ty - self.y
        d = math.hypot(dx, dy)
        s = self.speed * dt
        if d <= s:
            self.x, self.y = tx, ty
            self.seg = (self.seg + 1) % len(self.path)
            self.pause_until = t + 1.5
        else:
            self.x += dx / d * s
            self.y += dy / d * s


# Nicken des balancierenden Riders, gemessen an den IMU-Proben (92 Hz) von sieben Rohfahrten am RiderPi
# (2026-11-10/11, zusammen ~1060 s, Fenster 4 s). Standardabweichung des Nickens in Fahrt, Median / 90 %:
# 0,5–1,5 Hz 0,94 / 2,10°, 1,5–4 Hz 0,33 / 0,94°, 4–8 Hz 0,12 / 0,26°, 8–15 Hz 0,05 / 0,16°; im Stand etwa
# ein Viertel bis ein Sechstel davon (0,16 / 0,09 / 0,04 / 0,02°). Stärkste Linie bei 0,5–0,75 Hz, nicht bei
# 2,5 Hz. Die Werte hier liegen zwischen Median und 90 % (Sinus-Amplitude = 1,41 x Standardabweichung).
# Bis 2026-11-11 standen hier 3,0° bei 2,5 Hz, 1,5° bei 6,1 Hz, 0,5° bei 9,3 Hz und 0,3° bei 13,1 Hz, auch im
# Stand: bei 2,5 Hz etwa sechsmal, bei 6 Hz etwa neunmal so viel wie gemessen.
NICK = ((0.8, 1.0), (2.5, 0.6), (6.1, 0.2), (9.3, 0.1), (13.1, 0.05))     # (Hz, Grad) in Fahrt
STAND_FACTOR = 0.25                                                        # Anteil davon im Stand
# Neigung beim Beschleunigen: gemessen 8,9° je m/s² (r = 0,98, Radwege gegen IMU), Beschleunigung p95 0,40 m/s².
# Begrenzt, weil der Simulator ohne --dyn die Geschwindigkeit sprunghaft ändert (sonst 20° und mehr).
ACC_DEG = 9.0
ACC_CLIP = 0.5


@dataclass
class Teil:
    """Einrichtung der erweiterten Szenen (z. B. "unten"): Quader x1..x2, y1..y2 (m) von z0 bis z1 über dem Boden.
    tex: Name einer Textur oder vier Namen für die Seiten Süd, Ost, Nord, West (Fototexturen je Seite).
    fest = Hindernis (Kollision, Kartenwahrheit); schwebende Teile (Tischplatte, Sitz, Blätter; z0 über der Kamera)
    verdecken den Boden nicht. sichtbar = False: nur Hindernis, wird nicht gezeichnet."""
    x1: float
    y1: float
    x2: float
    y2: float
    z0: float
    z1: float
    tex: object
    fest: bool = True
    sichtbar: bool = True


@dataclass
class SimScene:
    size: float = 6.0
    boxes: list = field(default_factory=lambda: [(2.0, 3.6, 2.7, 4.3), (4.2, 0.9, 4.8, 2.6)])
    wall_h: float = 1.0
    box_h: float = 0.6
    robot_r: float = 0.12              # Kollisionsradius (rund); der echte Rider ist 11,5 cm breit, 13,3 cm lang
    # Fugen im Boden (x1, y1, x2, y2), unsichtbar: Die Radachse kommt nur mit Fahrwert >= rille_v darüber,
    # sonst stehen Rider und Räder (RiderPi 2026-11-23: Fuge vor der Fensterstufe, bei 0,23 fest)
    rillen: list = field(default_factory=list)
    rille_v: float = 0.35
    # Erweiterte Szenen (sonst wie bisher, bitgleich): Raum size x size_y, Außenwände in Stücken mit Textur
    # (x1, y1, x2, y2, tex; gegen den Uhrzeigersinn), Einrichtung (Teil), Teppiche (x1, y1, x2, y2, BGR) auf dem
    # Boden, Bodenart, Deckenfarbe, Startpose und Weg der Person
    size_y: float | None = None
    waende: list | None = None
    teile: list = field(default_factory=list)
    teppiche: list = field(default_factory=list)
    boden: str = "holz"
    decke: tuple = (200, 190, 180)
    start: tuple | None = None
    person_weg: list | None = None
    # Welt aus einer Datei (xgonav/welten/, siehe tools/simwelt/): zusätzliche Flächen (x1, y1, x2, y2, z0, z1, tex; von
    # rechts der Strecke sichtbar, auch schräg), eigene Texturen (Name -> BGR, tex_m Texel je Meter), Bodenbild
    # (boden = "bild", Zeile = y, Spalte = x, boden_res m je Pixel)
    flaechen: list = field(default_factory=list)
    texturen: dict | None = None
    tex_m: float = 100.0
    boden_bild: object = None
    boden_res: float = 0.02
    fugen: tuple | None = None          # (Länge in x, Breite in y) der Fliesen auf dem Bodenbild, Halbverband

    @property
    def sy(self) -> float:
        return self.size if self.size_y is None else self.size_y

    @property
    def erweitert(self) -> bool:
        return bool(self.teile or self.waende or self.flaechen or self.size_y is not None or self.boden != "holz")

    def hindernisse(self) -> list:
        """Rechtecke, die den Rider aufhalten: Kisten und feste Teile."""
        return list(self.boxes) + [(t.x1, t.y1, t.x2, t.y2) for t in self.teile if t.fest]

    def render_segmente(self) -> list:
        """Erweiterte Szenen: (x1, y1, x2, y2, z0, z1, tex, sicht); sicht +1 = von links sichtbar (Außenwand von
        innen), -1 = von rechts (Seiten eines Quaders von außen)."""
        sx, sy = self.size, self.sy
        w = self.waende or [(0, 0, sx, 0, "wand"), (sx, 0, sx, sy, "wand"), (sx, sy, 0, sy, "wand"),
                            (0, sy, 0, 0, "wand")]
        out = [(x1, y1, x2, y2, 0.0, self.wall_h, tex, 1) for x1, y1, x2, y2, tex in w]
        for t in self.teile:
            if not t.sichtbar:
                continue
            for i, (a, b, c, d) in enumerate(((t.x1, t.y1, t.x2, t.y1), (t.x2, t.y1, t.x2, t.y2),
                                              (t.x2, t.y2, t.x1, t.y2), (t.x1, t.y2, t.x1, t.y1))):
                out.append((a, b, c, d, t.z0, t.z1, t.tex if isinstance(t.tex, str) else t.tex[i], -1))
        out += [(x1, y1, x2, y2, z0, z1, tex, -1) for x1, y1, x2, y2, z0, z1, tex in self.flaechen]
        for x1, y1, x2, y2 in self.boxes:
            for a, b, c, d in ((x1, y1, x2, y1), (x2, y1, x2, y2), (x2, y2, x1, y2), (x1, y2, x1, y1)):
                out.append((a, b, c, d, 0.0, self.box_h, "kiste", -1))
        return out

    def kreuzt_rille(self, x0: float, y0: float, x1: float, y1: float) -> bool:
        """Schneidet der Weg der Radachse (x0, y0) -> (x1, y1) eine Fuge?"""
        def seite(ax, ay, bx, by, px, py):
            return (bx - ax) * (py - ay) - (by - ay) * (px - ax)
        for ax, ay, bx, by in self.rillen:
            if (seite(ax, ay, bx, by, x0, y0) * seite(ax, ay, bx, by, x1, y1) <= 0
                    and seite(x0, y0, x1, y1, ax, ay) * seite(x0, y0, x1, y1, bx, by) <= 0
                    and (x0, y0) != (x1, y1)):
                return True
        return False

    def segments(self):
        """Wände und Hindernisse als Strecken (x1, y1, x2, y2, Textur 0/1 Wand, 2 Kiste): Wahrheit für die Karte
        und Strahlen des einfachen Renderers. Erweiterte Szenen: Außenwände und feste Teile am Boden."""
        s = self.size
        if self.erweitert:
            seg = [(x1, y1, x2, y2, 0) for x1, y1, x2, y2, *_ in self.render_segmente()[:len(self.waende or [0] * 4)]]
            for x1, y1, x2, y2 in [(t.x1, t.y1, t.x2, t.y2) for t in self.teile if t.fest and t.z0 <= 1e-6] + list(self.boxes):
                seg += [(x1, y1, x2, y1, 2), (x2, y1, x2, y2, 2), (x2, y2, x1, y2, 2), (x1, y2, x1, y1, 2)]
            return np.array(seg, np.float64)
        seg = [(0, 0, s, 0, 0), (s, 0, s, s, 0), (s, s, 0, s, 1), (0, s, 0, 0, 1)]
        for i, (x1, y1, x2, y2) in enumerate(self.boxes):
            k = 2
            seg += [(x1, y1, x2, y1, k), (x2, y1, x2, y2, k), (x2, y2, x1, y2, k), (x1, y2, x1, y1, k)]
        return np.array(seg, np.float64)

    def collides(self, x: float, y: float, r: float | None = None) -> bool:
        r = self.robot_r if r is None else r
        if x < r or y < r or x > self.size - r or y > self.sy - r:
            return True
        # runder Roboter: Abstand zum nächsten Punkt der Kiste (an den Ecken nicht quadratisch,
        # sonst zählt ein Vorbeifahren mit 3 cm Luft an einer Ecke als Kollision)
        return any(math.hypot(x - min(max(x, x1), x2), y - min(max(y, y1), y2)) < r
                   for (x1, y1, x2, y2) in (self.hindernisse() if self.teile else self.boxes))


def _stuhl(x: float, y: float, lehne: str) -> list:
    """Gelber Stuhl wie unten am Esstisch: Sitz 45 x 45 cm auf 42–50 cm, vier dünne schwarze Beine (fest), Lehne an
    der Seite lehne ("s" = kleines y, "n" = großes y, "w" = kleines x, "o" = großes x) bis 88 cm."""
    s, b = 0.45, 0.025
    t = [Teil(x, y, x + s, y + s, 0.42, 0.50, "gelb", fest=False)]
    for bx, by in ((x + 0.02, y + 0.02), (x + s - 0.02 - b, y + 0.02), (x + 0.02, y + s - 0.02 - b),
                   (x + s - 0.02 - b, y + s - 0.02 - b)):
        t.append(Teil(bx, by, bx + b, by + b, 0.0, 0.42, "schwarz"))
    if lehne in ("s", "n"):
        ly = y if lehne == "s" else y + s - 0.05
        t.append(Teil(x + 0.03, ly, x + s - 0.03, ly + 0.05, 0.50, 0.88, "gelb", fest=False))
    else:
        lx = x if lehne == "w" else x + s - 0.05
        t.append(Teil(lx, y + 0.03, lx + 0.05, y + s - 0.03, 0.50, 0.88, "gelb", fest=False))
    return t


def _pflanze(x: float, y: float, r_topf: float = 0.2, r_blatt: float = 0.45, h: float = 1.5) -> list:
    """Topf (fest) und Blätter darüber (durchscheinend, schwebend)."""
    return [Teil(x - r_topf, y - r_topf, x + r_topf, y + r_topf, 0.0, 0.40, "topf"),
            Teil(x - r_blatt, y - r_blatt, x + r_blatt, y + r_blatt, 0.40, h, "pflanze", fest=False)]


_WELTEN: dict = {}


def _welt_datei(name: str, licht: str) -> tuple:
    """Weltdatei xgonav/welten/<name>.npz (tools/simwelt/textur.py): Modell (JSON), je Licht Fototexturen der Flächen
    (PNG, 2 cm je Texel) und das Bodenbild. Einmal je Licht dekodiert."""
    key = (name, licht)
    if key not in _WELTEN:
        import json
        pf = os.path.join(os.path.dirname(os.path.abspath(__file__)), "welten", name + ".npz")
        with np.load(pf) as z:
            modell = json.loads(bytes(z["modell"]).decode("utf-8"))
            tex = {k.split("/", 1)[1]: cv2.imdecode(z[k], cv2.IMREAD_COLOR) for k in z.files if k.startswith(licht + "/")}
        _WELTEN[key] = (modell, tex)
    return _WELTEN[key]


def _unten(licht: str = "abend") -> SimScene:
    """Erdgeschoss aus den Fahrten ab 2026-11-23 (UntenSonne, UntenAbend): Grundriss und Möbel aus der gemeinsamen
    Karte von 20 Fahrten (Tiefenschätzung je Bild, Kurs aus den Wandrichtungen, tools/simwelt/), Aussehen der Flächen
    und des Bodens aus den Kamerabildern, je Licht ("tag", "abend") eigene Texturen. Küche als Zeile mit zwei Reihen,
    Essnische mit Tisch und gelben Stühlen (unterfahrbar), Hauptraum mit Schrankwand, schräger Wand, TV-Schrank,
    Teppich, Sofa, Treppe, Podest an der Fensterfront. Maße aus der Karte abgelesen (±10 cm), nicht vermessen."""
    m, fotos = _welt_datei("unten", licht)
    tex = {k: v for k, v in fotos.items() if k != "boden"}
    hoch = [np.median(v.reshape(-1, 3), axis=0) for k, v in tex.items()
            if ":" in k and m["teile"][int(k.split(":")[0])]["z1"] >= m["wall_h"] - 0.01]
    putz = np.median(hoch, axis=0) if hoch else np.array([200.0, 205.0, 205.0])
    tex["putz"] = np.broadcast_to(putz.astype(np.uint8), (4, 4, 3)).copy()
    # gemalte Texturen (Stühle, Tisch, Pflanze, Treppe) in der Helligkeit des Lichts, in der Auflösung der Fotos
    hell = float(np.clip(putz.mean() / 200.0, 0.35, 1.0))
    gemalt = _texturen()
    for n in ("gelb", "schwarz", "tisch", "topf", "pflanze", "holz"):
        t = gemalt[n]
        t = cv2.resize(t, (max(1, t.shape[1] // 2), max(1, t.shape[0] // 2)), interpolation=cv2.INTER_NEAREST)
        t[..., :3] = np.clip(t[..., :3] * hell, 0, 255).astype(np.uint8)
        tex[n] = t
    teile: list = []
    for i, t in enumerate(m["teile"]):
        tx = t["tex"]
        if tx == "foto":
            da = ["%d:%s" % (i, s) for s in "SONW" if "%d:%s" % (i, s) in tex]
            ersatz = "putz"
            if da and t["z1"] < m["wall_h"] - 0.01:          # Möbel: fehlende Seiten in der mittleren Farbe der übrigen
                ersatz = "m%d" % i
                mittel = np.median(np.concatenate([tex[k].reshape(-1, 3) for k in da]), axis=0)
                tex[ersatz] = np.broadcast_to(mittel.astype(np.uint8), (4, 4, 3)).copy()
            tx = tuple(k if k in tex else ersatz for k in ("%d:%s" % (i, s) for s in "SONW"))
        teile.append(Teil(t["x1"], t["y1"], t["x2"], t["y2"], t["z0"], t["z1"], tx, fest=t["fest"],
                          sichtbar=t.get("sichtbar", True)))
    for x, y, lehne in m.get("stuehle", []):
        teile += _stuhl(x - 0.225, y - 0.225, lehne)
    flaechen = []
    for i, f in enumerate(m.get("schraeg", [])):
        flaechen.append((f["x1"], f["y1"], f["x2"], f["y2"], f["z0"], f["z1"], "s%d" % i if "s%d" % i in tex else "putz"))
        # Hindernis dazu: kleine unsichtbare Quader hinter der Fläche (links der Strecke)
        L = math.hypot(f["x2"] - f["x1"], f["y2"] - f["y1"])
        ex, ey = (f["x2"] - f["x1"]) / L, (f["y2"] - f["y1"]) / L
        n = max(1, int(math.ceil(L / 0.1)))
        for k in range(n):
            cx, cy = f["x1"] + ex * (k + 0.5) * L / n - ey * 0.06, f["y1"] + ey * (k + 0.5) * L / n + ex * 0.06
            teile.append(Teil(cx - 0.06, cy - 0.06, cx + 0.06, cy + 0.06, 0.0, f["z1"], "putz", sichtbar=False))
    return SimScene(size=m["size"], size_y=m["size_y"], boxes=[], wall_h=m["wall_h"], teile=teile, flaechen=flaechen,
                    waende=[(0, 0, m["size"], 0, "putz"), (m["size"], 0, m["size"], m["size_y"], "putz"),
                            (m["size"], m["size_y"], 0, m["size_y"], "putz"), (0, m["size_y"], 0, 0, "putz")],
                    texturen=tex, tex_m=50.0, boden="bild", boden_bild=cv2.GaussianBlur(fotos["boden"], (0, 0), 2.5),
                    boden_res=0.02, fugen=(1.2, 0.6),
                    teppiche=[(t["x1"], t["y1"], t["x2"], t["y2"], tuple(int(v) for v in fotos["boden"][
                        int(t["y1"] / 0.02):int(t["y2"] / 0.02), int(t["x1"] / 0.02):int(t["x2"] / 0.02)].reshape(-1, 3).mean(0)))
                              for t in m.get("teppiche", [])],
                    decke=tuple(int(v) for v in np.clip(putz * 1.05, 0, 255)),
                    start=tuple(m["start"]), person_weg=[(5.0, 4.6), (8.3, 3.9), (6.0, 3.8), (4.6, 4.9)])


SCENES = {
    "raum": SimScene,
    # zwei Zimmer, dazwischen eine Wand mit 0,8 m breiter Tür (Wände als flache Kisten)
    "zwei": lambda: SimScene(boxes=[(3.0, 0.0, 3.1, 2.6), (3.0, 3.4, 3.1, 6.0), (1.2, 4.4, 1.9, 5.1), (4.3, 0.9, 4.9, 1.8)]),
    # wie "zwei", aber die Tür nur 25 cm breit; Kollisionsradius wie der echte Rider (halbe Länge 7 cm)
    "spalt": lambda: SimScene(boxes=[(3.0, 0.0, 3.1, 2.875), (3.0, 3.125, 3.1, 6.0), (1.2, 4.4, 1.9, 5.1),
                                     (4.3, 0.9, 4.9, 1.8)], robot_r=0.07),
    # wie "raum", dazu eine Fuge quer durch den Raum (x = 3) und rings um die rechte Kiste eine Fuge, an der die
    # Radachse 0,12 m Luft zur Kiste hat (wie vor der Fensterstufe)
    # rille_v 0,5: explore.v 0,4 des Riders bleibt hängen, push_v 0,45 + Korrektur des Temporeglers kommt drüber
    "fuge": lambda: SimScene(rillen=[(3.0, 0.0, 3.0, 6.0)] + _ring((4.2, 0.9, 4.8, 2.6), 0.24), rille_v=0.5),
    # Erdgeschoss aus den Fahrten ab 2026-11-23 (Weltdatei xgonav/welten/unten.npz): abends bei Lampenlicht (wie die
    # meisten Fahrten) und bei Tageslicht mit Sonnenflecken
    "unten": _unten,
    "unten_tag": lambda: _unten("tag"),
}


def _texturen() -> dict:
    """Texturen der erweiterten Szenen, BGRA, 1 px = 1 cm; Zeile 0 = oben, unten verankert (die unterste Zeile liegt
    am Fuß des Teils). Alpha 0 = durchsichtig (Blätter)."""
    rng = np.random.default_rng(21)

    def flach(bgr, w, h, rausch=5):
        t = np.empty((h, w, 4), np.uint8)
        t[..., :3] = np.clip(np.array(bgr, np.int16) + rng.integers(-rausch, rausch + 1, (h, w, 1)), 0, 255)
        t[..., 3] = 255
        return t

    def wand(w=512, h=250):
        t = flach((222, 225, 226), w, h, 3)
        t[h - 7:, :, :3] = (72, 72, 74)                              # Sockelleiste 7 cm
        for x in (60, 330):                                           # Steckdosen, Schalter
            cv2.rectangle(t, (x, h - 38), (x + 8, h - 30), (200, 200, 200, 255), -1)
        cv2.rectangle(t, (180, h - 112), (186, h - 104), (195, 195, 195, 255), -1)
        cv2.rectangle(t, (400, h - 175), (470, h - 110), (60, 80, 110, 255), -1)      # Bild an der Wand
        cv2.rectangle(t, (400, h - 175), (470, h - 110), (40, 40, 40, 255), 2)
        return t

    def tuer(w=90, h=250):
        t = flach((222, 225, 226), w, h, 3)
        t[h - 212:, 4:w - 4, :3] = (78, 74, 72)                       # Türblatt dunkelgrau, 2,1 m
        t[h - 214:h - 212, :, :3] = (60, 60, 60)
        t[h - 212:, :4, :3] = t[h - 212:, w - 4:, :3] = (55, 55, 58)  # Zarge
        cv2.rectangle(t, (w - 18, h - 108), (w - 8, h - 102), (190, 190, 195, 255), -1)   # Drücker
        return t

    def fenster(w=120, h=250, unten_cm=90):
        t = wand(w, h)
        cv2.rectangle(t, (6, h - 220), (w - 6, h - unten_cm), (248, 244, 238, 255), -1)
        cv2.rectangle(t, (6, h - 220), (w - 6, h - unten_cm), (90, 90, 92, 255), 3)
        cv2.line(t, (w // 2, h - 220), (w // 2, h - unten_cm), (90, 90, 92, 255), 2)
        cv2.rectangle(t, (2, h - unten_cm), (w - 2, h - unten_cm + 4), (205, 205, 205, 255), -1)   # Bank
        return t

    def vorhang(w=300, h=250):
        t = flach((45, 40, 42), w, h, 4)
        for x in range(0, w, 15):
            t[:, x:x + 4, :3] = np.clip(t[:, x:x + 4, :3].astype(np.int16) - 15, 0, 255)
        return t

    def kueche(w=60, h=220):
        t = flach((52, 52, 56), w, h, 3)
        t[h - 10:, :, :3] = (25, 25, 25)                              # Sockel
        t[h - 94:h - 90, :, :3] = (175, 180, 182)                     # Arbeitsplatte
        t[:, :2, :3] = (35, 35, 38)                                   # Fugen zwischen den Fronten
        t[h - 84:h - 82, 8:w - 8, :3] = (150, 150, 155)               # Griffleiste
        return t

    def sofa(w=100, h=80):
        t = flach((118, 116, 112), w, h, 6)
        t[h - 4:, :, :3] = (35, 35, 35)                               # Schatten unter dem Sofa
        t[h - 42:h - 40, :, :3] = (95, 93, 90)                        # Sitzkante
        t[:, ::50, :3] = (95, 93, 90)
        return t

    def holz(w=120, h=150):
        t = flach((62, 102, 150), w, h, 8)
        for y in range(0, h, 9):
            t[y, :, :3] = (50, 85, 130)
        t[h - 2:, :, :3] = (40, 60, 90)
        return t

    def pflanze(w=90, h=160):
        t = np.zeros((h, w, 4), np.uint8)
        for _ in range(110):
            c = (int(rng.integers(30, 70)), int(rng.integers(110, 170)), int(rng.integers(40, 90)), 255)
            x, y = int(rng.integers(0, w)), int(rng.integers(0, h))
            cv2.ellipse(t, (x, y), (int(rng.integers(3, 9)), int(rng.integers(8, 20))), float(rng.integers(0, 180)),
                        0, 360, c, -1)
        return t

    def tv(w=140, h=63):
        t = flach((18, 18, 20), w, h, 2)
        t[:3, :, :3] = t[-3:, :, :3] = (40, 40, 40)
        return t

    return {"wand": wand(), "tuer": tuer(), "fenster": fenster(), "fenster_tief": fenster(260, 250, 5),
            "vorhang": vorhang(), "kueche": kueche(), "sofa": sofa(), "holz": holz(), "pflanze": pflanze(),
            "tv": tv(), "lowboard": flach((235, 235, 235), 100, 45, 3), "topf": flach((62, 62, 66), 40, 40, 4),
            "gelb": flach((30, 168, 222), 45, 45, 8), "schwarz": flach((28, 28, 30), 10, 90, 3),
            "metall": flach((175, 175, 178), 10, 80, 4), "tisch": flach((236, 236, 234), 160, 4, 2),
            "kiste": flach((150, 160, 170), 64, 64, 20)}


def textur_farbe(t: np.ndarray) -> str:
    """Mittlere Farbe einer Textur (BGRA) als #rrggbb, für die 3D-Ansicht."""
    m = t[..., 3] > 0
    b, g, r = (int(t[..., i][m].mean()) for i in range(3))
    return "#%02x%02x%02x" % (r, g, b)


def _ring(box, d):
    """Vier Fugen im Abstand d um eine Kiste."""
    x1, y1, x2, y2 = box
    a, b, c, e = x1 - d, y1 - d, x2 + d, y2 + d
    return [(a, b, c, b), (c, b, c, e), (c, e, a, e), (a, e, a, b)]


def scene_floor(scene: SimScene, x: np.ndarray, y: np.ndarray, margin: float = 0.0) -> np.ndarray:
    """Liegt der Punkt auf echtem Boden (im Raum, nicht in einer Kiste)? margin: Abstand zu den
    Rändern, den er mindestens haben muss."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    ok = (x >= margin) & (y >= margin) & (x <= scene.size - margin) & (y <= scene.sy - margin)
    for x1, y1, x2, y2 in (scene.hindernisse() if scene.teile else scene.boxes):
        ok &= ~((x >= x1 - margin) & (x <= x2 + margin) & (y >= y1 - margin) & (y <= y2 + margin))
    return ok


def map_quality(lo: np.ndarray, res: float, l_show: float, scene: SimScene, start: tuple, path=None) -> dict:
    """Kartenqualität gegen die Wahrheit. lo: Log-Odds, start: (x, y, th) der Karten-Null in der
    Welt, path: gefahrene Punkte (Welt, für die Abdeckung nahe am Weg)."""
    n = lo.shape[0]
    x0, y0, th0 = start
    c0, s0 = math.cos(th0), math.sin(th0)

    def to_world(r, c):
        mx, my = (c + 0.5 - n / 2) * res, (r + 0.5 - n / 2) * res
        return x0 + mx * c0 - my * s0, y0 + mx * s0 + my * c0

    r, c = np.nonzero(lo > l_show)
    ox, oy = to_world(r, c)
    seg = scene.segments()
    ax, ay, bx, by = seg[:, 0], seg[:, 1], seg[:, 2], seg[:, 3]
    dx, dy = bx - ax, by - ay
    if len(ox):
        t = np.clip(((ox[:, None] - ax) * dx + (oy[:, None] - ay) * dy) / (dx * dx + dy * dy), 0, 1)
        d = np.hypot(ax + t * dx - ox[:, None], ay + t * dy - oy[:, None]).min(axis=1)
        prec = float((d < 0.15).mean())
    else:
        prec = float("nan")
    r, c = np.nonzero(lo < -l_show)
    fx, fy = to_world(r, c)
    # Eine Zelle Toleranz: eine Zelle auf der Wand gilt nicht als "frei hinter der Wand"
    wrong = ~scene_floor(scene, fx, fy, -res) if len(fx) else np.zeros(0, bool)
    # Wahrer Boden auf einem Raster: wie viel davon kennt die Karte als frei?
    gx, gy = np.meshgrid(np.arange(res / 2, scene.size, res), np.arange(res / 2, scene.sy, res))
    fl = scene_floor(scene, gx, gy)
    gx, gy = gx[fl], gy[fl]
    wx, wy = gx - x0, gy - y0
    mx, my = wx * c0 + wy * s0, -wx * s0 + wy * c0
    cc = np.floor(mx / res + n / 2).astype(np.int64)
    rr = np.floor(my / res + n / 2).astype(np.int64)
    inside = (rr >= 0) & (rr < n) & (cc >= 0) & (cc < n)
    known = np.zeros(len(gx), bool)
    known[inside] = lo[rr[inside], cc[inside]] < -l_show
    # Wandpunkte alle 5 cm: Anteil mit belegter Zelle in <= 10 cm, gesamt und <= 2 m vom Weg
    pts = []
    for ax_, ay_, bx_, by_, _ in seg:
        k = max(2, int(math.hypot(bx_ - ax_, by_ - ay_) / 0.05))
        pts += [(ax_ + (bx_ - ax_) * i / k, ay_ + (by_ - ay_) * i / k) for i in range(k + 1)]
    pts = np.array(pts)
    if len(ox):
        step = max(1, len(pts) * len(ox) // 4_000_000)
        dd = np.hypot(pts[::step, None, 0] - ox[None, :], pts[::step, None, 1] - oy[None, :]).min(axis=1)
        cover = float((dd <= 0.10).mean())
    else:
        dd, step, cover = None, 1, float("nan")
    recall = float("nan")
    if path is not None and dd is not None and len(path):
        p = np.asarray(path, float)[::5]
        near = np.hypot(pts[::step, None, 0] - p[None, :, 0], pts[::step, None, 1] - p[None, :, 1]).min(axis=1) <= 2.0
        if near.any():
            recall = float((dd[near] <= 0.10).mean())
    return {"praezision_15cm": round(prec, 3), "abdeckung_10cm": round(recall, 3), "waende_10cm": round(cover, 3),
            "frei_aussen_m2": round(float(wrong.sum()) * res * res, 2),
            "frei_innen_m2": round(float((~wrong).sum()) * res * res, 2),
            "boden_bekannt": round(float(known.mean()), 3),
            "belegt_m2": round(len(ox) * res * res, 2)}


class SimWorld(FrameSource):
    def __init__(self, cfg: CameraCfg, robot: SimRobot, person: Person | None = None,
                 start=(1.0, 1.0, 0.0), shake: float = 1.0, render_scale: int = 2, fps: int = 20,
                 vx_mps: float = 1.0, mount_offset_mm: float = 45.0, dynamics: bool = False,
                 scene: SimScene | None = None):
        super().__init__(cfg)
        self.robot = robot
        self.dynamics = dynamics
        # dunkle Zonen (x0, y0, x1, y1, Restlicht 0..1): Dunkelangst prüfen (tests/sim_run.py --dunkel)
        self.dark_zones: list[tuple[float, float, float, float, float]] = []
        self._vf = self._wf = 0.0
        self.scene = scene or SimScene()
        self.segs = self.scene.segments()
        self.person = person
        self.x, self.y, self.th = start
        self.shake = shake
        self.rs = render_scale
        self.fps = fps
        self.vx_mps = vx_mps
        self.mount = mount_offset_mm
        self.tex = [_blob_tex(1, (150, 160, 170)), _blob_tex(2, (120, 140, 120)), _blob_tex(3, (60, 90, 170))]
        if self.scene.erweitert:
            self._ext_init()
        self.collisions = 0
        self.dist_travelled = 0.0
        self.log: list[dict] = []
        self.truth: dict = {}
        self.truth_by_t: dict = {}
        self.coll_frames: list[int] = []
        self._t0 = None
        self._prev_v = 0.0
        self._acc_lp = 0.0
        self._mot = STAND_FACTOR
        self.pitch = 0.0
        self._pitch_prev = 0.0
        self._rng = np.random.default_rng(7)
        # Radwege (m) aus dem Fahrbefehl: drehen auch weiter, wenn der Rider an einer Wand
        # hängt (Schlupf) – wie echte Encoder. Spurweite wie am RiderPi.
        self.track_m = 0.090
        self.wheel_l = self.wheel_r = 0.0
        # Bodengrenze je Bild aus der Ground Truth (SimFloorNet): (Bild, Zeilen) der letzten Bilder
        self.floor_rows: list[tuple[np.ndarray, np.ndarray]] = []

    # ------------------------------------------------------------------
    def _grab(self) -> bool:
        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now
            self._last = now
        target = self._last + 1.0 / self.fps
        if target > now:
            time.sleep(target - now)
        now = time.monotonic()
        dt = now - self._last
        self._last = now
        t = now - self._t0
        self._integrate(t, dt)
        img, box, rows = self._render(t, dt)
        self.floor_rows = (self.floor_rows + [(img, rows)])[-8:]
        self.truth = {"t": now, "box": box, "pitch": self.pitch}
        self.truth_by_t[now] = self.truth
        if len(self.truth_by_t) > 200:
            self.truth_by_t.pop(next(iter(self.truth_by_t)))
        self._publish(img, now)
        return True

    def _integrate(self, t: float, dt: float) -> None:
        vn, wn = self.robot.command
        v = vn * self.vx_mps
        w = w_to_yaw_rate(wn, self.robot.cfg)
        if self.dynamics:
            v, w = self._dyn(v, w, wn, dt)
        self.th += w * dt
        nx = self.x + v * math.cos(self.th) * dt
        ny = self.y + v * math.sin(self.th) * dt
        if self.scene.rillen and abs(vn) < self.scene.rille_v and self.scene.kreuzt_rille(self.x, self.y, nx, ny):
            # Rad hängt in der Fuge: Rider und Encoder stehen (nur Drehen geht)
            v, self._vf, nx, ny = 0.0, 0.0, self.x, self.y
            self.rille_steps = getattr(self, "rille_steps", 0) + 1
        self.wheel_l += (v - w * self.track_m / 2) * dt
        self.wheel_r += (v + w * self.track_m / 2) * dt
        if self.scene.collides(nx, ny):
            if v != 0 and not getattr(self, "_in_coll", False):
                self.collisions += 1
                self.coll_frames.append(self._idx)
            self._in_coll = True
        else:
            self._in_coll = False
            self.dist_travelled += math.hypot(nx - self.x, ny - self.y)
            self.x, self.y = nx, ny
        if self.person:
            self.person.step(t, dt)
        acc = (v - self._prev_v) / max(dt, 1e-3)
        self._prev_v = v
        self._acc_lp = 0.7 * self._acc_lp + 0.3 * float(np.clip(acc, -ACC_CLIP, ACC_CLIP))
        # Im Stand pendelt der Rider nur etwa ein Viertel so stark wie in Fahrt; Übergang über ~0,5 s
        ziel = 1.0 if abs(v) > 0.03 or abs(w) > 0.2 else STAND_FACTOR
        self._mot += (ziel - self._mot) * min(1.0, dt / 0.5)
        self.pitch = self.pitch_truth(t)
        self.yaw_jit = self.shake * self._mot * math.radians(0.8) * math.sin(2 * math.pi * 1.7 * t)
        cam_h = ((self.robot.height_mm or 85.0) + self.mount) / 1000.0
        self.cam_h = cam_h
        self.log.append({"t": t, "x": self.x, "y": self.y, "th": self.th, "v": v, "w": w,
                         "px": self.person.x if self.person else None,
                         "py": self.person.y if self.person else None})

    # Trägheit wie am RiderPi (dynamics=True; Messfahrt 2026-11-05, docs/limitations.md):
    # Drehbefehle unter 6/127,5 ignoriert die Firmware, die Drehung läuft ~0,3 s nach
    # (10–17° Überschwingen nach dem Stopp), und beim Anfahren rollt der balancierende Rider
    # erst kurz zurück (nichtminimalphasig; nur wenn er schneller wird, beim Bremsen auf 0 lief er 2026-11-23
    # nur so weit wie die Verzögerung allein ergibt).
    # Fahrt 2026-11-23 (Räder gegen Fahrwert, 0,1-0,7 m/s): Anfahren und Langsamerwerden folgen mit ~1,2 s
    # (angepasst 1,5-2 s, davon ~0,15 s Fenster der Radgeschwindigkeit), nur Fahrwert 0 (unter dem Totband)
    # bremst in ~0,25 s.
    DYN_TAU_V, DYN_TAU_STOP, DYN_TAU_W, DYN_NMP_S, DYN_W_DEAD = 1.2, 0.25, 0.3, 0.4, 6 / 127.5

    # Fahrrahmen (Firmware-Patch 08-velocity, robot.vel): Der Sollwert ist schon gerampt (robot.command = Generator der
    # Firmware), die Firmware regelt Tempo und Drehrate darauf; ohne Totband. Annahme bis zur Messfahrt: Nachlauf
    # 0,15 s (Tempo) / 0,1 s (Drehen), wie im Modellkreis von test_velocity_ctrl.cpp. Ohne Zurückrollen: Das Modell
    # dafür (DYN_NMP_S x Änderung) gehört zu Sprüngen im Fahrwert; bei einer Rampe zöge es die ganze
    # Beschleunigungsphase lang 0,4 s x a ab, die Vorlage baut sich mit der Vorsteuerung aber allmählich auf.
    DYN_TAU_VEL_V, DYN_TAU_VEL_W = 0.15, 0.1

    def _dyn(self, v: float, w: float, wn: float, dt: float) -> tuple[float, float]:
        if getattr(self.robot, "vel", False):
            self._vf += (v - self._vf) * (1.0 - math.exp(-dt / self.DYN_TAU_VEL_V))
            self._wf += (w - self._wf) * (1.0 - math.exp(-dt / self.DYN_TAU_VEL_W))
            return self._vf, self._wf
        if abs(wn) < self.DYN_W_DEAD:
            w = 0.0
        prev = self._vf
        tau = self.DYN_TAU_STOP if abs(v) < FW_DEADBAND * self.vx_mps else self.DYN_TAU_V   # unter dem Totband = 0
        self._vf += (v - self._vf) * (1.0 - math.exp(-dt / tau))
        v_eff = self._vf
        if abs(self._vf) > abs(prev):
            v_eff -= self.DYN_NMP_S * (self._vf - prev) / max(dt, 1e-3)
        self._wf += (w - self._wf) * (1.0 - math.exp(-dt / self.DYN_TAU_W))
        return v_eff, self._wf

    def pitch_truth(self, t: float) -> float:
        """Kameranicken (rad, + = hoch) zur Zeit t (s seit Start), zeitkontinuierlich:
        Balancier-Pendeln + Vibration + Neigung beim Beschleunigen. Amplituden siehe NICK."""
        return self.shake * (self._mot * sum(math.radians(a) * math.sin(2 * math.pi * f * t + i)
                                             for i, (f, a) in enumerate(NICK))
                             - math.radians(ACC_DEG) * self._acc_lp)

    # --- erweiterte Szenen: Schichten je Spalte, Teile mit eigener Höhe, Texturen in cm, Fliesen ---------------
    EXT_SCHICHTEN = 10        # so viele nächste Flächen je Bildspalte werden gezeichnet (von hinten nach vorn)

    def _ext_init(self) -> None:
        tex = _texturen()
        eigen = self.scene.texturen or {}
        for n, t in eigen.items():                          # Fototexturen (BGR) deckend
            tex[n] = np.dstack([t, np.full(t.shape[:2], 255, np.uint8)]) if t.shape[2] == 3 else t
        segs = self.scene.render_segmente()
        namen = sorted({s[6] for s in segs})
        self._ext_namen = namen
        self._ext_seg = np.array([s[:6] + (namen.index(s[6]), s[7]) for s in segs], np.float64)
        hmax = max(tex[n].shape[0] for n in namen)
        wmax = max(tex[n].shape[1] for n in namen)
        self._ext_atlas = np.zeros((len(namen), hmax, wmax, 4), np.uint8)
        self._ext_th = np.array([tex[n].shape[0] for n in namen])
        self._ext_tw = np.array([tex[n].shape[1] for n in namen])
        self._ext_ts = np.array([self.scene.tex_m if n in eigen else 100.0 for n in namen])     # Texel je Meter
        for i, n in enumerate(namen):
            h, w = tex[n].shape[:2]
            self._ext_atlas[i, hmax - h:, :w] = tex[n]          # unten bündig: Zeile hmax-1 = Fuß
        self.tex_farben = {n: textur_farbe(tex[n]) for n in namen}

    def _ext_bild(self, W: int, H: int, f: float, th: float, cx: float, cy: float, lat, fwd, left, dirs, tb):
        """Bild der erweiterten Szene ohne Person: (img, Boden-Maske, Tiefe der nächsten Fläche am Boden je Spalte)."""
        sc, R = self.scene, self._ext_seg
        a, b = R[:, 0:2], R[:, 2:4]
        e = b - a
        p = np.array([cx, cy])
        denom = dirs[:, None, 0] * e[None, :, 1] - dirs[:, None, 1] * e[None, :, 0]
        ap = a[None] - p[None, None]
        with np.errstate(divide="ignore", invalid="ignore"):
            tt = (ap[..., 0] * e[None, :, 1] - ap[..., 1] * e[None, :, 0]) / denom
            ss = (ap[..., 0] * dirs[:, None, 1] - ap[..., 1] * dirs[:, None, 0]) / denom
        # nur die Seite, die der Kamera zugewandt ist (Wand von innen, Quader von außen)
        quer = e[:, 0] * (cy - a[:, 1]) - e[:, 1] * (cx - a[:, 0])
        sicht = quer * R[:, 7] > 0
        valid = (tt > 1e-3) & (ss >= 0) & (ss <= 1) & np.isfinite(tt) & sicht[None, :]
        tt = np.where(valid, tt, np.inf)
        am_boden = R[:, 4] <= 1e-6
        gdepth = np.where(am_boden[None, :], tt, np.inf).min(1)
        cam_h = self.cam_h
        img = np.empty((H, W, 3), np.uint8)
        img[:] = sc.decke
        with np.errstate(divide="ignore"):
            zf = np.where(tb > 1e-4, cam_h / tb, np.inf)
        floor = zf[:, None] < gdepth[None, :]
        if floor.any():
            Z = np.broadcast_to(np.where(np.isfinite(zf), zf, 0.0)[:, None], (H, W))
            wx = cx + Z * (fwd[0] + lat[None, :] * left[0])
            wy = cy + Z * (fwd[1] + lat[None, :] * left[1])
            img[floor] = self._ext_boden(wx[floor], wy[floor], Z[floor], f)
        ar = np.arange(W)
        L = min(self.EXT_SCHICHTEN, tt.shape[1])
        order = np.argpartition(tt, L - 1, axis=1)[:, :L] if L < tt.shape[1] else np.tile(np.arange(L), (W, 1))
        order = np.take_along_axis(order, np.argsort(np.take_along_axis(tt, order, 1), 1), 1)
        seglen = np.linalg.norm(e, axis=1)
        hmax = self._ext_atlas.shape[1]
        for j in range(L - 1, -1, -1):                     # von hinten nach vorn
            k = order[:, j]
            d = tt[ar, k]
            ok = np.isfinite(d)
            if not ok.any():
                continue
            z = cam_h - d[None, :] * tb[:, None]
            z0, z1 = R[k, 4], R[k, 5]
            m = (z >= z0[None, :]) & (z <= z1[None, :]) & ok[None, :]
            yy, xx = np.nonzero(m)
            if not len(yy):
                continue
            kk = k[xx]
            tid = R[kk, 6].astype(np.int64)
            ts = self._ext_ts[tid]
            u = (ss[xx, kk] * seglen[kk] * ts).astype(np.int64) % self._ext_tw[tid]
            v = np.clip(hmax - 1 - ((z[yy, xx] - R[kk, 4]) * ts).astype(np.int64), hmax - self._ext_th[tid], hmax - 1)
            px = self._ext_atlas[tid, v, u]
            sel = px[:, 3] > 0
            img[yy[sel], xx[sel]] = px[sel, :3]
        return img, floor, gdepth

    def _ext_boden(self, wx, wy, Z, f):
        """Bodenfarbe (BGR) je Punkt: Fliesen 60 x 120 cm im Halbverband mit Fugen, Teppiche."""
        sc = self.scene
        if sc.boden == "bild":
            B = sc.boden_bild
            ix = np.clip((wx / sc.boden_res).astype(np.int64), 0, B.shape[1] - 1)
            iy = np.clip((wy / sc.boden_res).astype(np.int64), 0, B.shape[0] - 1)
            fein = ((((wx * 200).astype(np.int64) * 83492791) ^ ((wy * 200).astype(np.int64) * 2654435761)) & 3
                    ).astype(np.int16) - 1
            col = B[iy, ix].astype(np.int16) + fein[:, None]
            if sc.fugen:
                lx, ly = sc.fugen
                jy = np.floor(wy / ly)
                xs = wx + (jy % 2) * lx / 2
                fx, fy = xs - np.floor(xs / lx) * lx, wy - jy * ly
                hw = np.maximum(0.002, 0.6 * Z / f)        # Fuge mindestens ~1 px breit, sonst flimmert sie
                fuge = (fx < hw) | (fx > lx - hw) | (fy < hw) | (fy > ly - hw)
                for x1, y1, x2, y2, _ in sc.teppiche:      # auf dem Teppich keine Fugen
                    fuge &= ~((wx >= x1) & (wx <= x2) & (wy >= y1) & (wy <= y2))
                col[fuge] = col[fuge] * 7 // 10
            return np.clip(col, 0, 255).astype(np.uint8)
        if sc.boden == "fliesen":
            jy = np.floor(wy / 0.6)
            xs = wx + (jy % 2) * 0.6
            jx = np.floor(xs / 1.2)
            fx, fy = xs - jx * 1.2, wy - jy * 0.6
            hw = np.maximum(0.002, 0.6 * Z / f)            # Fuge mindestens ~1 px breit, sonst flimmert sie
            fuge = (fx < hw) | (fx > 1.2 - hw) | (fy < hw) | (fy > 0.6 - hw)
            ix, iy = jx.astype(np.int64), jy.astype(np.int64)
            var = (((ix * 73856093) ^ (iy * 19349663)) & 7).astype(np.int16) - 4
            fein = ((((wx * 40).astype(np.int64) * 83492791) ^ ((wy * 40).astype(np.int64) * 2654435761)) & 7
                    ).astype(np.int16) - 3
            base = np.array([150, 152, 152], np.int16)
            col = base[None, :] + (var + fein)[:, None]
            col[fuge] = (98, 98, 100)
        else:
            ix, iy = np.floor(wx * 12).astype(np.int64), np.floor(wy * 12).astype(np.int64)
            n = (((ix * 73856093) ^ (iy * 19349663)) & 15).astype(np.int16) - 8
            col = np.array([95, 120, 150], np.int16)[None, :] + n[:, None]
        for x1, y1, x2, y2, bgr in sc.teppiche:
            m = (wx >= x1) & (wx <= x2) & (wy >= y1) & (wy <= y2)
            if m.any():
                n = ((((wx[m] * 60).astype(np.int64) * 73856093) ^ ((wy[m] * 60).astype(np.int64) * 19349663)) & 15
                     ).astype(np.int16) - 8
                col[m] = np.array(bgr, np.int16)[None, :] + n[:, None]
        return np.clip(col, 0, 255).astype(np.uint8)

    def _render(self, t: float, dt: float):
        W, H = self.cfg.width // self.rs, self.cfg.height // self.rs
        f = (W / 2) / math.tan(math.radians(self.cfg.hfov_deg) / 2)
        th = self.th + self.yaw_jit
        cols = np.arange(W) + 0.5
        c0 = W / 2                                    # Spalte der Fahrtrichtung
        self._c0 = c0
        lat = (c0 - cols) / f                         # tan(Winkel) links +
        fwd = np.array([math.cos(th), math.sin(th)])
        left = np.array([-math.sin(th), math.cos(th)])
        dirs = fwd[None] + lat[:, None] * left[None]  # Strahlen (nicht normiert, Tiefe=1)
        # Strahl-Segment-Schnitt
        # Kamera sitzt cam_fwd_mm vor der Radachse (self.x/y = Achse, dort dreht und kollidiert der Rider)
        off = getattr(self.cfg, "cam_fwd_mm", 0.0) / 1000.0
        cx, cy = self.x + off * math.cos(self.th), self.y + off * math.sin(self.th)
        self._cam_xy = (cx, cy)
        if self.scene.erweitert:
            tb_e = np.tan(np.arctan2(np.arange(H) + 0.5 - H / 2, f) - self.pitch)
            img, floor, depth = self._ext_bild(W, H, f, th, cx, cy, lat, fwd, left, dirs, tb_e)
            return self._render_schluss(img, floor, depth, f, th, W, H, dt)
        p = np.array([cx, cy])
        a, b = self.segs[:, 0:2], self.segs[:, 2:4]
        e = b - a
        denom = dirs[:, None, 0] * e[None, :, 1] - dirs[:, None, 1] * e[None, :, 0]
        ap = a[None] - p[None, None]
        with np.errstate(divide="ignore", invalid="ignore"):
            tt = (ap[..., 0] * e[None, :, 1] - ap[..., 1] * e[None, :, 0]) / denom
            ss = (ap[..., 0] * dirs[:, None, 1] - ap[..., 1] * dirs[:, None, 0]) / denom
        valid = (tt > 1e-3) & (ss >= 0) & (ss <= 1) & np.isfinite(tt)
        tt = np.where(valid, tt, np.inf)
        k = tt.argmin(1)
        depth = tt[np.arange(W), k]                   # = Tiefe entlang optischer Achse
        seglen = np.linalg.norm(e, axis=1)
        u = ss[np.arange(W), k] * seglen[k]
        tex_id = self.segs[k, 4].astype(int)
        wall_h = np.where(tex_id == 2, self.scene.box_h, self.scene.wall_h)

        rows = np.arange(H) + 0.5
        beta = np.arctan2(rows - H / 2, f) - self.pitch   # Winkel unter Horizontal
        tb = np.tan(beta)
        img = np.empty((H, W, 3), np.uint8)
        img[:] = (200, 190, 180)                          # Decke/Himmel
        cam_h = self.cam_h
        # Wandhöhe je Pixel
        z = cam_h - depth[None, :] * tb[:, None]
        wall = (z >= 0) & (z <= wall_h[None, :])
        for tid in range(3):
            m = wall & (tex_id[None, :] == tid)
            if m.any():
                tx = self.tex[tid]
                ui = (u[None, :] * 100).astype(int) % tx.shape[1]
                vi = np.clip(((1 - z / wall_h[None, :]) * (tx.shape[0] - 1)).astype(int), 0, tx.shape[0] - 1)
                ui = np.broadcast_to(ui, z.shape)
                img[m] = tx[vi[m], ui[m]]
        # Boden
        with np.errstate(divide="ignore"):
            zf = np.where(tb > 1e-4, cam_h / tb, np.inf)  # Tiefe des Bodenpunkts je Zeile
        floor = zf[:, None] < depth[None, :]
        if floor.any():
            Z = np.broadcast_to(np.where(np.isfinite(zf), zf, 0.0)[:, None], (H, W))
            wx = cx + Z * (fwd[0] + lat[None, :] * left[0])
            wy = cy + Z * (fwd[1] + lat[None, :] * left[1])
            ix, iy = np.floor(wx * 12).astype(np.int64), np.floor(wy * 12).astype(np.int64)
            n = (((ix * 73856093) ^ (iy * 19349663)) & 15).astype(np.int16) - 8
            base = np.array([95, 120, 150], np.int16)  # Holzboden-ähnlich (BGR)
            fl = np.clip(base[None, None, :] + n[..., None], 0, 255).astype(np.uint8)
            img[floor] = fl[floor]
        return self._render_schluss(img, floor, depth, f, th, W, H, dt)

    def _render_schluss(self, img, floor, depth, f, th, W, H, dt):
        """Person, Bodengrenze (Ground Truth für SimFloorNet), Vollbild, Bewegungsunschärfe, Dunkelzonen."""
        box = None
        obst = ~floor
        if self.person:
            box = self._draw_person(img, f, th, depth, W, H, obst)
        # Bodengrenze je Spalte wie FloorNet: unterste Zeile, ab der von unten gesehen kein Boden mehr ist
        # (Vollbild); je Netzspalte die nächste (größte Zeile) ihrer Bildspalten
        run = np.where(obst.any(axis=0), np.argmax(obst[::-1], axis=0), H)
        grenze = (H - run).astype(np.float32) * self.rs
        rows = grenze.reshape(SIM_NET_COLS, -1).max(axis=1)
        img = cv2.resize(img, (self.cfg.width, self.cfg.height), interpolation=cv2.INTER_LINEAR)
        # Bewegungsunschärfe ~ Nickrate * Belichtungszeit (hier übertrieben, 8 ms)
        prate = (self.pitch - self._pitch_prev) / max(dt, 1e-3)
        self._pitch_prev = self.pitch
        blur_px = abs(prate) * 0.008 * (self.cfg.width / 2) / math.tan(math.radians(self.cfg.hfov_deg) / 2)
        if blur_px > 1.5:
            kz = int(min(15, blur_px)) | 1
            kern = np.zeros((kz, kz), np.float32)
            kern[:, kz // 2] = 1.0 / kz
            img = cv2.filter2D(img, -1, kern)
        if self.dark_zones:
            k = self.light_factor()
            if k < 0.999:
                img = cv2.convertScaleAbs(img, alpha=k)
        if box is not None:
            box = tuple(v * self.rs for v in box)
        return img, box, rows

    def _licht_bei(self, x: float, y: float) -> float:
        k = 1.0
        for x0, y0, x1, y1, rest in self.dark_zones:
            d = min(x - x0, x1 - x, y - y0, y1 - y)       # > 0 innen
            tief = min(max(d / 0.3, 0.0), 1.0)              # 30 cm Übergang
            k = min(k, 1.0 - tief * (1.0 - rest))
        return k

    def light_factor(self) -> float:
        """Licht im Bild: wo die Kamera steht und wohin sie schaut (1,5 m voraus); aus einer dunklen Ecke
        heraus in den hellen Raum ist es heller als in die Ecke hinein."""
        cx, cy = getattr(self, "_cam_xy", (self.x, self.y))
        ax, ay = cx + 1.5 * math.cos(self.th), cy + 1.5 * math.sin(self.th)
        return 0.35 * self._licht_bei(cx, cy) + 0.65 * self._licht_bei(ax, ay)

    def _draw_person(self, img, f, th, depth, W, H, obst=None):
        pr = self.person
        cx, cy = self._cam_xy
        rel = np.array([pr.x - cx, pr.y - cy])
        Z = rel @ np.array([math.cos(th), math.sin(th)])
        X = rel @ np.array([-math.sin(th), math.cos(th)])
        if Z < 0.15:
            return None
        c = self._c0 - f * X / Z
        half = f * pr.width_m / 2 / Z  # Person 0.50 m breit (Annahme im Code: 0.45 -> 10 % Bias)
        c1, c2 = int(math.floor(c - half)), int(math.ceil(c + half))

        def row(zw):
            return H / 2 + f * math.tan(math.atan2(self.cam_h - zw, Z) + self.pitch)

        r_top, r_bot = row(pr.height_m), row(0.0)
        x1, x2 = max(0, c1), min(W, c2)
        y1, y2 = max(0, int(r_top)), min(H, int(math.ceil(r_bot)))
        if x2 <= x1 or y2 <= y1:
            return None
        vis = depth[x1:x2] > Z
        if not vis.any():
            return None
        hh = r_bot - r_top
        for yy in range(y1, y2):
            fr = (yy - r_top) / hh
            col = (150, 180, 220) if fr < 0.13 else (180, 60, 30) if fr < 0.55 else (50, 40, 40)
            seg = img[yy, x1:x2]
            seg[vis] = col
            if obst is not None:
                obst[yy, x1:x2] |= vis
        vx = np.nonzero(vis)[0]
        bx1, bx2 = x1 + vx.min(), x1 + vx.max() + 1
        if (bx2 - bx1) < 0.3 * (c2 - c1):
            return None  # größtenteils verdeckt
        return (float(bx1), float(max(0, r_top)), float(bx2), float(min(H, r_bot)))


class SimImu:
    """IMU des Simulators: 50 Hz, Rauschen, absichtlich vertauschtes Vorzeichen,
    Grundneigung (ref) und Zeitstempelversatz – die App muss das selbst herausfinden.
    yaw_scale: Kurs zählt jede Drehung so viel mal (Skalenfehler des Kreisels, für den Abgleich am Startbild)."""

    def __new__(cls, *a, **k):
        from .imu import ImuSource

        class _SimImu(ImuSource):
            def __init__(self, world, cfg, sign=-1.0, ref_deg=2.5, t_err_ms=-8.0, noise_deg=0.05, hz=50.0,
                         yaw_scale=1.0):
                super().__init__(cfg)
                self.yaw_scale = yaw_scale
                self._th_alt = self._kurs = None
                self.world, self.sgn, self.ref = world, sign, math.radians(ref_deg)
                self.t_err, self.noise, self.hz = t_err_ms / 1000, math.radians(noise_deg), hz
                self._seq = 0
                self._rng = np.random.default_rng(11)

            def _open(self) -> bool:
                return True

            def _run(self) -> None:
                while not self._stop.wait(1.0 / self.hz):
                    w = self.world
                    if w._t0 is None:
                        continue
                    now = time.monotonic()
                    # liegend ruht die Lage (die Simulation kennt das Absetzen nur über den Zustand)
                    p = w.pitch_truth(now - w._t0) if w.robot.state.balancing else math.radians(60.0)
                    raw = self.sgn * p + self.ref + self.noise * self._rng.standard_normal()
                    self._seq += 1
                    if self._th_alt is None:
                        self._kurs = w.th
                    else:
                        self._kurs += self.yaw_scale * wrap(w.th - self._th_alt)
                    self._th_alt = w.th
                    self.add(self._seq, now + self.t_err, 0.0, raw, wrap(self._kurs))
                    self.rate_hz = self.hz

        return _SimImu(*a, **k)


class SimWheels:
    """Rad-Encoder des Simulators: 50 Hz, 1024 Ticks/U aus den Radwegen von SimWorld,
    mit optionalem Maßstabsfehler (scale, z. B. 1.03 = Rad 3 % größer als angenommen)."""

    def __new__(cls, *a, **k):
        from .wheels import WheelOdom

        class _SimWheels(WheelOdom):
            def __init__(self, world, cfg, hz=50.0, scale=1.0):
                super().__init__(cfg)
                self.world, self.hz, self.scale = world, hz, scale
                self._seq = 0

            def _open(self) -> bool:
                return True

            def _run(self) -> None:
                while not self._stop.wait(1.0 / self.hz):
                    w = self.world
                    if w._t0 is None:
                        continue
                    k = self.scale / self.m_per_tick
                    self._seq += 1
                    self.add(self._seq, time.monotonic(), int(w.wheel_l * k), int(w.wheel_r * k),
                             int((time.monotonic() - w._t0) * 1000))
                    self.rate_hz = self.hz

        return _SimWheels(*a, **k)


class SimFloorNet:
    """Ersetzt FloorNet im Simulator: Bodengrenze je Netzspalte (Vollbildzeile) aus der Ground Truth des gerenderten
    Bildes, Wände, Kisten und Person. Fehlalarme gibt es nur mit Phantoms."""

    def __init__(self, world: "SimWorld"):
        self.world = world

    def run(self, bgr: np.ndarray) -> np.ndarray:
        for img, rows in reversed(self.world.floor_rows):
            if img is bgr:
                return rows.copy()
        raise ValueError("SimFloorNet: Bild unbekannt (zu alt?)")


class Phantoms:
    """Fehlalarme des Bodenmodells, die der Simulator sonst nicht kennt: n ortsfeste Flecken (wie
    Spiegelungen oder Lichtpfützen), die aus manchen Blickwinkeln und nur zeitweise eine Bodengrenze
    vortäuschen. Wickelt SimFloorNet.run ein (Zeilen im Vollbild)."""

    def __init__(self, world: "SimWorld", net: SimFloorNet, geo, n: int = 6, seed: int = 3, radius: float = 0.15,
                 on_prob: float = 0.5):
        self.world, self.net, self.geo = world, net, geo
        rng = np.random.default_rng(seed)
        pts = []
        while len(pts) < n:
            x, y = rng.uniform(0.4, world.scene.size - 0.4, 2)
            if world.scene.size_y is not None:
                y = 0.4 + (y - 0.4) * (world.scene.sy - 0.8) / (world.scene.size - 0.8)
            if scene_floor(world.scene, x, y, 0.3):
                pts.append((float(x), float(y)))
        self.pts = pts
        self.radius, self.on_prob = radius, on_prob
        # Fleck wirkt nur aus einem Blickwinkelbereich (Spiegelung)
        self.dirs = rng.uniform(-math.pi, math.pi, n)
        self._rng = rng
        self.hits = 0
        self._run = net.run
        net.run = self.run

    def run(self, bgr: np.ndarray) -> np.ndarray:
        rows = self._run(bgr)
        w, geo = self.world, self.geo
        n = len(rows)
        bear = geo.bearing((np.arange(n) + 0.5) * geo.W / n)
        pitch = w.pitch                                   # Nicken des Bildes, wie gerendert
        cx, cy = getattr(w, "_cam_xy", (w.x, w.y))      # Kamera, nicht Radachse
        for (px, py), d0 in zip(self.pts, self.dirs):
            dx, dy = px - cx, py - cy
            dist = math.hypot(dx, dy)
            if dist < 0.25 or dist > 2.0 or self._rng.random() > self.on_prob:
                continue
            seen_from = math.atan2(-dy, -dx)
            if abs(wrap(seen_from - d0)) > math.radians(70):
                continue
            ang = wrap(math.atan2(dy, dx) - w.th)
            half = math.asin(min(1.0, self.radius / dist))
            cols = np.nonzero(np.abs(bear - ang) < half)[0]
            if not len(cols):
                continue
            fwd = dist * math.cos(ang)
            if fwd <= 0.05:
                continue
            y = geo.H / 2 + geo.f * math.tan(math.atan2(geo.cam_h, fwd) + math.radians(geo.tilt_deg) + pitch)
            if 0.0 <= y < geo.H:
                rows[cols] = np.fmax(rows[cols], y)       # näher = weiter unten im Bild
                self.hits += 1
        return rows


class FakeDetector:
    """Ersetzt DetectorWorker: Ground Truth aus SimWorld, mit Inferenzlatenz und Rauschen."""

    def __init__(self, world: SimWorld, latency_s: float = 0.18, period_s: float = 0.2):
        self.world, self.lat, self.period = world, latency_s, period_s
        self.infer_ms = latency_s * 1000
        self.busy = False
        self._pending: list[tuple[float, DetectionBatch]] = []
        self._next = 0.0
        self._lock = threading.Lock()
        self._rng = np.random.default_rng(3)

    backend = "sim"

    def start(self) -> bool:
        return True

    def close(self) -> None: ...

    def classes(self) -> list[str]:
        return [self.world.person.cls if self.world.person is not None else "person"]

    def offer(self, frame, sharpness, median) -> None:
        now = time.monotonic()
        if now < self._next:
            return
        self._next = now + self.period
        tr = self.world.truth_by_t.get(frame.t, {})
        dets = []
        if tr.get("box") is not None:
            x1, y1, x2, y2 = tr["box"]
            n = self._rng.normal(0, 3, 4)
            cls = self.world.person.cls if self.world.person is not None else "person"
            dets.append(Detection((x1 + n[0], y1 + n[1], x2 + n[2], y2 + n[3]), cls, 0.85))
        with self._lock:
            self._pending.append((now + self.lat, DetectionBatch(frame.idx, frame.t, dets, self.lat * 1000)))

    def poll(self):
        now = time.monotonic()
        with self._lock:
            if self._pending and self._pending[0][0] <= now:
                return self._pending.pop(0)[1]
        return None
