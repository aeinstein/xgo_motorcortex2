"""Gesichter wiedererkennen (Modus pet): YuNet findet das Gesicht samt 5 Punkten, MobileFaceNet (ncnn) rechnet
einen Merkmalsvektor, der Kosinus gegen die angelernten Fotos entscheidet.

Rechenzeit: läuft im Detektor-Thread direkt nach YOLO, nur auf dem Kopfausschnitt einer Personenbox, höchstens
[gesicht] max_hz und nur für Personen, deren Track noch keinen Namen trägt (detector.DetectorWorker). Anlernen
(Web-Upload) einmal je Foto. Modelle: tools/export_face.sh (am PC).
"""
from __future__ import annotations

import json
import logging
import os
import re
import threading
import time

import cv2
import numpy as np

from .config import GesichtCfg

log = logging.getLogger(__name__)

# ArcFace-Vorlage der 5 Punkte im 112×112-Bild (Augen, Nase, Mundwinkel; Bildlinks zuerst wie bei YuNet)
VORLAGE = np.array([[38.2946, 51.6963], [73.5318, 51.5014], [56.0252, 71.7366],
                    [41.5493, 92.3655], [70.7299, 92.2041]], dtype=np.float32)
SEITE = 112
NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,32}$")
MAX_FOTOS = 50                    # je Name
DET_MAX_PX = 320                  # Kopfausschnitt höchstens so groß an YuNet (größer bringt nichts, kostet nur)
GRUSS = "gruss.json"              # je Name: {"emotion", "sound", "augen"}; fehlt ein Feld: [pet] greet_* bzw. Türkis
EMO_RE = re.compile(r"^(-|[A-Za-z]{1,32})?$")                  # "-" = keine, leer = Standard
TON_RE = re.compile(r"^(-|[a-z]{1,32}|stimme:\d\d)?$")         # Ereignis pet_<ton>_*.wav oder Stimmprobe NN
AUGEN_RE = re.compile(r"^([a-z]{1,16})?$")                     # Farbe aus tools/eyes.py PALETTE, leer = Türkis
FELDER = ("emotion", "sound", "augen")


class GesichtFehler(ValueError):
    pass


def ausrichten(bgr: np.ndarray, punkte: np.ndarray) -> np.ndarray:
    """5 Punkte (5×2, Bildkoordinaten) -> 112×112-Ausschnitt in ArcFace-Lage (Ähnlichkeitstransformation)."""
    M, _ = cv2.estimateAffinePartial2D(np.asarray(punkte, np.float32).reshape(5, 2), VORLAGE, method=cv2.LMEDS)
    if M is None:
        raise GesichtFehler("Gesicht nicht ausrichtbar")
    return cv2.warpAffine(bgr, M, (SEITE, SEITE), borderValue=0)


def kopf_box(box, W: int, H: int) -> tuple[int, int, int, int] | None:
    """Kopfbereich einer Personenbox (x1, y1, x2, y2): oberer Teil, etwas Rand. None = zu klein/außerhalb."""
    x1, y1, x2, y2 = box
    w, h = x2 - x1, y2 - y1
    if w < 8 or h < 8:
        return None
    # stehend: Kopf im oberen Fünftel; sitzend/nah ist die Box gedrungener -> höchstens bis zur Hälfte
    kh = min(h, max(0.5 * h, w)) if h < 1.5 * w else 0.5 * h
    pad = 0.1 * w
    a = (int(max(0, x1 - pad)), int(max(0, y1 - pad)), int(min(W, x2 + pad)), int(min(H, y1 + kh)))
    return a if a[2] - a[0] >= 8 and a[3] - a[1] >= 8 else None


class Gesichter:
    """Erkennen und Anlernen. Thread-sicher (ein Lock): Detektor-Thread und Web-Thread teilen die Netze."""

    def __init__(self, cfg: GesichtCfg, finder=None, netz=None):
        self.cfg = cfg
        self._lock = threading.Lock()
        self._finder = finder               # Tests: callable(bgr) -> [(box xywh, punkte 5×2, score)]
        self._netz = netz                   # Tests: callable(bgr112) -> Vektor
        self._yunet = None
        self._ncnn = None
        self.ok = finder is not None and netz is not None
        self.merkmale: dict[str, np.ndarray] = {}    # Name -> (n, d), L2-normiert
        self.gruesse: dict[str, dict] = {}           # Name -> eigene Begrüßung (gruss.json)
        self.ms = 0.0                                # zuletzt je Prüfung (Finden + Merkmal), geglättet

    # ------------------------------------------------------------------ Laden
    def start(self) -> bool:
        if self.ok:
            self.laden()
            return True
        c = self.cfg
        if not c.enabled:
            return False
        try:
            if not os.path.isfile(c.det_model):
                raise FileNotFoundError(c.det_model)
            self._yunet = cv2.FaceDetectorYN.create(c.det_model, "", (DET_MAX_PX, DET_MAX_PX),
                                                    c.det_conf, 0.3, 5)
            import ncnn
            param, binf = os.path.join(c.model, "model.ncnn.param"), os.path.join(c.model, "model.ncnn.bin")
            net = ncnn.Net()
            net.opt.num_threads = max(1, int(c.threads))
            net.opt.use_vulkan_compute = False
            net.opt.lightmode = True
            if net.load_param(param) != 0 or net.load_model(binf) != 0:
                raise RuntimeError(f"NCNN-Modell {c.model} nicht ladbar")
            try:
                ein, aus = net.input_names()[0], net.output_names()[0]
            except Exception:
                ein, aus = "in0", "out0"
            self._ncnn = (ncnn, net, ein, aus)
        except Exception as e:
            log.warning("Gesichtserkennung aus: %s (Modelle per tools/export_face.sh)", e)
            return False
        self.ok = True
        self.laden()
        log.info("Gesichter geladen: %s + %s, %d Namen", c.det_model, c.model, len(self.merkmale))
        return True

    def laden(self) -> None:
        """Angelernte Merkmale von der Platte: <dir>/<Name>/NN.npy."""
        m, gr = {}, {}
        d = self.cfg.dir
        for name in sorted(os.listdir(d)) if os.path.isdir(d) else []:
            if not NAME_RE.match(name):
                continue
            vs = []
            for f in sorted(os.listdir(os.path.join(d, name))):
                if f.endswith(".npy"):
                    try:
                        vs.append(np.load(os.path.join(d, name, f)).astype(np.float32).ravel())
                    except Exception as e:
                        log.warning("Gesicht %s/%s nicht lesbar: %s", name, f, e)
            if vs:
                m[name] = np.stack(vs)
                try:
                    with open(os.path.join(d, name, GRUSS), encoding="utf-8") as fh:
                        g = json.load(fh)
                    gr[name] = {k: str(g[k]) for k in FELDER if g.get(k)}
                except FileNotFoundError:
                    pass
                except Exception as e:
                    log.warning("Begrüßung %s nicht lesbar: %s", name, e)
        with self._lock:
            self.merkmale, self.gruesse = m, gr

    def gruss(self, name: str) -> dict:
        """Eigene Begrüßung des Namens {emotion, sound, augen}; fehlt ein Feld: Standard ([pet] greet_*, Türkis),
        "-" = keine Emotion/kein Ton."""
        return dict(self.gruesse.get(name) or {})

    def gruss_setzen(self, name: str, emotion: str = "", sound: str = "", augen: str = "") -> bool:
        """Begrüßung eines angelernten Namens speichern; leer = Standard. False: Name unbekannt oder Wert ungültig."""
        if not NAME_RE.match(name or "") or name not in self.merkmale:
            return False
        if not EMO_RE.match(emotion or "") or not TON_RE.match(sound or "") or not AUGEN_RE.match(augen or ""):
            return False
        f = os.path.join(self.cfg.dir, name, GRUSS)
        g = {k: v for k, v in zip(FELDER, (emotion, sound, augen)) if v}
        if g:
            with open(f + ".tmp", "w", encoding="utf-8") as fh:
                json.dump(g, fh)
            os.replace(f + ".tmp", f)
        elif os.path.isfile(f):
            os.remove(f)
        self.laden()
        return True

    # ------------------------------------------------------------------ Rechnen
    def _finden(self, bgr: np.ndarray):
        """-> [(x, y, w, h), punkte 5×2, score] in Koordinaten von bgr."""
        if self._finder is not None:
            return self._finder(bgr)
        h, w = bgr.shape[:2]
        k = min(1.0, DET_MAX_PX / max(w, h))
        img = cv2.resize(bgr, (max(1, round(w * k)), max(1, round(h * k))), interpolation=cv2.INTER_AREA) if k < 1 else bgr
        self._yunet.setInputSize((img.shape[1], img.shape[0]))
        _, faces = self._yunet.detect(img)
        out = []
        for f in faces if faces is not None else []:
            out.append((tuple(float(v) / k for v in f[:4]), f[4:14].reshape(5, 2) / k, float(f[14])))
        return out

    def _merkmal(self, al: np.ndarray) -> np.ndarray:
        if self._netz is not None:
            v = np.asarray(self._netz(al), np.float32).ravel()
        else:
            ncnn, net, ein, aus = self._ncnn
            m = ncnn.Mat.from_pixels(np.ascontiguousarray(al), ncnn.Mat.PixelType.PIXEL_BGR2RGB, SEITE, SEITE)
            m.substract_mean_normalize([127.5] * 3, [1 / 127.5] * 3)
            ex = net.create_extractor()
            ex.input(ein, m)
            ret, out = ex.extract(aus)
            if ret != 0:
                raise RuntimeError(f"NCNN extract fehlgeschlagen ({ret})")
            v = np.asarray(out, np.float32).ravel()
        n = float(np.linalg.norm(v))
        return v / n if n > 0 else v

    def wer(self, v: np.ndarray) -> tuple[str | None, float]:
        """Bester Name (Maximum über seine Fotos) und Kosinus; Name None unter match_sim."""
        best, sim = None, -1.0
        for name, m in self.merkmale.items():
            s = float(np.max(m @ v))
            if s > sim:
                best, sim = name, s
        return (best if sim >= self.cfg.match_sim else None), sim

    def pruefen(self, bgr: np.ndarray, box) -> tuple[str | None, float] | None:
        """Personenbox im Vollbild -> (Name|None, Kosinus) oder None, wenn kein brauchbares Gesicht zu sehen ist."""
        if not self.ok or not self.merkmale:
            return None
        H, W = bgr.shape[:2]
        kb = kopf_box(box, W, H)
        if kb is None:
            return None
        x1, y1, x2, y2 = kb
        aus = bgr[y1:y2, x1:x2]
        t0 = time.perf_counter()
        with self._lock:
            faces = [f for f in self._finden(aus) if f[0][2] >= self.cfg.min_face_px]
            if not faces:
                return None
            f = max(faces, key=lambda f: f[0][2] * f[0][3])
            r = self.wer(self._merkmal(ausrichten(aus, f[1])))
        ms = (time.perf_counter() - t0) * 1000
        self.ms = 0.8 * self.ms + 0.2 * ms if self.ms else ms
        return r

    # ------------------------------------------------------------------ Anlernen (Web)
    def anlernen(self, bgr: np.ndarray, name: str) -> tuple[str, bytes]:
        """Größtes Gesicht des Fotos unter name ablegen. -> (NN, JPEG des ausgerichteten Ausschnitts)."""
        if not NAME_RE.match(name or ""):
            raise GesichtFehler("Name ungültig (Buchstaben, Ziffern, _ und -, höchstens 32)")
        if not self.ok:
            raise GesichtFehler("Gesichtserkennung aus (Modelle fehlen?)")
        with self._lock:
            faces = self._finden(bgr)
            if not faces:
                raise GesichtFehler("kein Gesicht gefunden")
            f = max(faces, key=lambda f: f[0][2] * f[0][3])
            al = ausrichten(bgr, f[1])
            v = self._merkmal(al)
        d = os.path.join(self.cfg.dir, name)
        os.makedirs(d, exist_ok=True)
        belegt = {f[:2] for f in os.listdir(d) if f.endswith(".npy")}
        if len(belegt) >= MAX_FOTOS:
            raise GesichtFehler(f"schon {MAX_FOTOS} Fotos für {name}")
        nn = next(f"{i:02d}" for i in range(100) if f"{i:02d}" not in belegt)
        ok, jpg = cv2.imencode(".jpg", al, [cv2.IMWRITE_JPEG_QUALITY, 90])
        np.save(os.path.join(d, nn + ".npy"), v)
        with open(os.path.join(d, nn + ".jpg"), "wb") as fh:
            fh.write(jpg.tobytes())
        self.laden()
        return nn, jpg.tobytes()

    def loeschen(self, name: str, nr: str | None = None) -> bool:
        """Ein Foto (nr = NN) oder alle eines Namens löschen."""
        if not NAME_RE.match(name or "") or (nr is not None and not re.match(r"^\d\d$", nr)):
            return False
        d = os.path.join(self.cfg.dir, name)
        if not os.path.isdir(d):
            return False
        weg = False
        for f in os.listdir(d):
            if nr is None or f[:2] == nr:
                os.remove(os.path.join(d, f))
                weg = True
        if nr is None or not any(f.endswith(".npy") for f in os.listdir(d)):
            for f in os.listdir(d):                  # ohne Fotos auch die Begrüßung weg
                os.remove(os.path.join(d, f))
            os.rmdir(d)
        self.laden()
        return weg

    def status(self) -> dict:
        return {"on": self.ok, "namen": {n: len(m) for n, m in self.merkmale.items()}, "gruesse": self.gruesse,
                "ms": round(self.ms, 1)}
