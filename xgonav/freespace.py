"""Freiraum vor dem Rider aus der Bodengrenze von FloorNet.

FloorNet (floornet.py, C++-Kern) liefert je Netzspalte die Vollbildzeile, an der der Boden endet. Hier wird daraus
FreeSpace: je Spalte der Arbeitsauflösung ([freespace] width) der Anteil der Strecke bis zum Horizont, der frei ist,
um den Roll der Kamera geradegedreht, dazu Sektoren für safety(). Das Netz rechnet im eigenen Thread
(FreeSpaceWorker), wahlweise über die letzten Ausgaben beruhigt (floornet.ZeilenGlaetter).

Es gibt keinen Ersatzweg: Fehlt das Netz oder fällt es aus, ist der Rider blind (FreeSpaceWorker.blind), dann
steht safety() still. Im Simulator liefert sim.SimFloorNet dieselben Zeilen aus der Ground Truth.
"""
from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass, field

import numpy as np

from .affinity import pin_thread
from .config import FreespaceCfg
from .geometry import CamGeometry

log = logging.getLogger(__name__)


@dataclass
class FreeSpace:
    sectors: list[float] = field(default_factory=list)  # 0 = blockiert direkt vor uns, 1 = frei bis Horizont
    columns: np.ndarray | None = None                   # pro Spalte, gleiche Normierung
    horizon_row: float = 0.0                            # in Arbeitsauflösung
    valid: bool = False
    # je Spalte, gleiche Normierung wie columns: darunter ist nichts messbar, dort gilt "unbekannt".
    # None = überall messbar (FloorNet). Die Karten können damit umgehen.
    known_from: np.ndarray | None = None
    # Nick (rad) und Kartenpose (x, y, th) der Aufnahme. Das Netz rechnet im eigenen Thread: das Ergebnis kommt
    # 1-3 Bilder später an, Entfernungen aus dem Nick dieses späteren Bildes streuen um Dezimeter (Nicken ±3° bei
    # 2,5 Hz). None = nicht bekannt (Tests).
    pitch_dev: float | None = None
    pose: tuple | None = None
    # Wegsumme der Merkkarte (LocalMap.odo) zur Aufnahme; None = Ergebnis gilt für die aktuelle Pose
    odo: tuple | None = None
    # Grenzzeile je Netzspalte (Vollbild), aus der columns stammt
    net_rows: np.ndarray | None = None
    # Mit net_smooth_n: Ausgabe des Netzes vor dem Glätten über die Zeit (net_rows ist dann die geglättete)
    net_rows_raw: np.ndarray | None = None
    src: str = "net"

    @property
    def center(self) -> float:
        if not self.sectors:
            return 1.0
        n = len(self.sectors)
        return min(self.sectors[n // 2 - (n % 2 == 0): n // 2 + 1])


def net_to_columns(rows: np.ndarray, horizon: float, H: int, W: int, scale: float) -> np.ndarray:
    """Grenzzeilen des Netzes (Vollbild) -> FreeSpace.columns (W Spalten, 0 = direkt vor uns, 1 = bis zum Horizont).
    Umkehrung von Zeile = H - Wert * (H - Horizont). Jede Spalte nimmt die Netzspalte, in der sie liegt: Glätten
    würde Grenzen erfinden."""
    idx = ((np.arange(W) + 0.5) * len(rows) / W).astype(int)
    return np.clip((H - np.asarray(rows, np.float32)[idx] / scale) / max(1.0, H - horizon), 0.0, 1.0).astype(np.float32)


def roll_columns(columns: np.ndarray, horizon: float, H: int, W: int, cx: float, roll_deg: float) -> np.ndarray:
    """Bodengrenze um den Roll der Kamera gedreht: Bei Roll liegt die Grenze je Spalte um (Spalte - Mitte) *
    tan(Roll) Zeilen falsch (cx = Bildmitte in Spalten der Arbeitsauflösung). Spalten ohne Treffer (frei bis zum
    Horizont) und unbekannte bleiben. Wie tools/map_replay.roll_spalten."""
    cols = np.asarray(columns, np.float32).copy()
    span = max(1.0, H - horizon)
    u = (np.arange(len(cols)) + 0.5) * (W / len(cols)) - cx
    with np.errstate(invalid="ignore"):
        hit = cols < 0.98
    cols[hit] = np.clip(cols[hit] - u[hit] * (math.tan(math.radians(roll_deg)) / span), 0.0, 0.979)
    return cols


def _p20(a: np.ndarray) -> float:
    """20-%-Quantil mit linearer Interpolation (wie np.percentile, aber ohne dessen Overhead).
    NaN-Spalten (unbekannt) zählen nicht; ganz unbekannt = 0.0 (nicht als frei melden)."""
    a = a[np.isfinite(a)]
    if not len(a):
        return 0.0
    s = np.sort(a)
    pos = 0.2 * (len(s) - 1)
    lo = int(pos)
    hi = min(lo + 1, len(s) - 1)
    return float(s[lo] + (s[hi] - s[lo]) * (pos - lo))


def column_runs(columns) -> list[tuple[int, int]]:
    """Zusammenhängende Bereiche bekannter Spalten (Start, Ende exklusiv) – zum Zeichnen der
    Bodengrenze, die an unbekannten (NaN-)Spalten unterbrochen wird."""
    ok = np.isfinite(np.asarray(columns, float))
    runs, i, n = [], 0, len(ok)
    while i < n:
        if ok[i]:
            j = i
            while j < n and ok[j]:
                j += 1
            runs.append((i, j))
            i = j
        else:
            i += 1
    return runs


class FreeSpaceEstimator:
    """Rechnet Grenzzeilen des Netzes in FreeSpace der Arbeitsauflösung um (W x H, Faktor scale zum Vollbild).
    Die Konstanten W, H, scale nutzen auch die Karten."""

    def __init__(self, cfg: FreespaceCfg, full_size: tuple[int, int], geo: CamGeometry | None = None):
        self.cfg = cfg
        self.geo = geo
        self.W = cfg.width
        self.H = int(round(full_size[1] * cfg.width / full_size[0]))
        self.scale = full_size[0] / cfg.width
        self._edges_x: dict[int, np.ndarray] = {}

    def horizon(self, pitch_dev: float = 0.0) -> float:
        """Horizontzeile in Arbeitsauflösung, auf 20..80 % der Bildhöhe begrenzt."""
        hz_full = self.geo.horizon_row(pitch_dev) if self.geo else self.H * self.scale / 2
        return min(max(float(hz_full / self.scale), self.H * 0.2), self.H * 0.8)

    def from_net(self, rows: np.ndarray, pitch_dev: float = 0.0, roll_deg: float = 0.0) -> FreeSpace:
        """FreeSpace aus den Grenzzeilen des Netzes. roll_deg: Roll der Kamera zur Bildzeit."""
        fs = FreeSpace([1.0] * self.cfg.sectors, None, self.horizon(pitch_dev), True, None, pitch_dev)
        self.adopt_net(fs, rows, roll_deg)
        return fs

    def adopt_net(self, fs: FreeSpace, rows: np.ndarray, roll_deg: float = 0.0) -> None:
        """columns und sectors aus den Zeilen des Netzes (auch für Auswertungen auf Aufnahmen). roll_deg: Roll der
        Kamera zur Bildzeit (imu.imu_frame().roll_deg), die Grenze wird damit geradegedreht."""
        fs.columns = net_to_columns(rows, fs.horizon_row, self.H, self.W, self.scale)
        if roll_deg and self.geo is not None:
            fs.columns = roll_columns(fs.columns, fs.horizon_row, self.H, self.W, self.geo.cx / self.scale, roll_deg)
        n = len(fs.sectors)
        ex = self._edges_x.get(n)
        if ex is None:                           # konstant je Sektorzahl
            ex = self._edges_x[n] = np.linspace(0, self.W, n + 1).astype(int)
        fs.sectors = [_p20(fs.columns[ex[i]:ex[i + 1]]) for i in range(n)]
        fs.known_from = None
        fs.net_rows = np.asarray(rows, np.float32)
        fs.src = "net"


class FreeSpaceWorker:
    """FloorNet in eigenem Thread, nach dem Muster des DetectorWorker: offer() legt den neuesten Auftrag ab (ein
    älterer, noch nicht begonnener wird ersetzt), poll() holt das neueste fertige Ergebnis. Ohne Thread
    (threaded = false, Simulator und Tests) rechnet compute() im Aufrufer.

    net: Objekt mit run(bgr) -> Grenzzeile je Netzspalte (Vollbild), FloorNet oder sim.SimFloorNet. None oder ein
    Fehler im Netz: blind, ab dann kein Ergebnis mehr (safety() steht still)."""

    def __init__(self, est: FreeSpaceEstimator, net, cpus=None, net_smooth_n: int = 0):
        self.est = est
        self.net = net
        self.blind = net is None
        self._net_lock = threading.Lock()  # set_net tauscht das Netz nie während run
        self.smooth = None                 # Kante über die Zeit beruhigen
        if net_smooth_n > 1:
            from .floornet import ZeilenGlaetter
            self.smooth = ZeilenGlaetter(net_smooth_n)
        self.net_ms = 0.0                  # gleitendes Mittel der Rechenzeit
        self.cpus = list(cpus or [])
        self._cond = threading.Condition()
        self._job = None
        self._result: FreeSpace | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.busy = False

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="freespace", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        with self._cond:
            self._cond.notify_all()
        if self._thread:
            self._thread.join(timeout=2)
        if self.net is not None and hasattr(self.net, "close") and not (self._thread and self._thread.is_alive()):
            self.net.close()

    def set_net(self, net):
        """Netz tauschen (Bodenmodell aus dem Web wechseln): wartet ein laufendes run ab, dann gilt das neue Netz
        ab dem nächsten Auftrag, das Glätten beginnt neu. Gibt das alte Netz zurück (der Aufrufer schließt es)."""
        with self._net_lock:
            alt, self.net = self.net, net
            self.blind = net is None
            if self.smooth is not None:
                self.smooth.reset()
        return alt

    def reset(self) -> None:
        """Glätten neu beginnen (nach Kamerastart, Aufstehen, geänderter Belichtung)."""
        if self.smooth is not None:
            self.smooth.reset()

    @staticmethod
    def job(bgr, pitch_dev: float, pose=None, t: float | None = None, roll_deg: float = 0.0, odo=None) -> tuple:
        """pose: Kartenpose zum Bild, wird dem Ergebnis mitgegeben. t: Aufnahmezeit (s, monoton), für das Glätten.
        roll_deg: Roll der Kamera zur Bildzeit. odo: Wegsumme der Merkkarte zum Bild (FreeSpace.odo)."""
        return (bgr, pitch_dev, pose, time.monotonic() if t is None else t, roll_deg, odo)

    def offer(self, *a, **k) -> None:
        """Auftrag für den Thread, Argumente wie job()."""
        with self._cond:
            self._job = self.job(*a, **k)
            self._cond.notify()

    def poll(self) -> FreeSpace | None:
        with self._cond:
            r, self._result = self._result, None
            return r

    def _run(self) -> None:
        if self.cpus:
            pin_thread(self.cpus, "Bodenmodell")
        while not self._stop.is_set():
            with self._cond:
                while self._job is None and not self._stop.is_set():
                    self._cond.wait(0.1)
                if self._stop.is_set():
                    return
                job, self._job = self._job, None
            self.busy = True
            fs = self.compute(job)
            self.busy = False
            if fs is not None:
                with self._cond:
                    self._result = fs

    def compute(self, job: tuple) -> FreeSpace | None:
        """Ein Auftrag (job()): Netz rechnen, glätten, in FreeSpace umrechnen. None, wenn blind."""
        if self.net is None:
            return None
        bgr, pitch_dev, pose, t, roll_deg, odo = job
        t0 = time.perf_counter()
        with self._net_lock:
            if self.net is None:
                return None
            try:
                rows = np.asarray(self.net.run(bgr), np.float32)
                if not np.isfinite(rows).all():
                    raise ValueError("FloorNet liefert ungültige Zeilen")
                roh = rows
                if self.smooth is not None:
                    rows = self.smooth.glaetten(roh, pose, pitch_dev, t, self.est.geo)
                fs = self.est.from_net(rows, pitch_dev, roll_deg)
                if self.smooth is not None:
                    fs.net_rows_raw = roh
                fs.pose, fs.odo = pose, odo
            except Exception:
                log.exception("FloorNet fehlgeschlagen: blind, der Rider fährt nicht mehr selbst")
                self.net = None
                self.blind = True
                return None
        ms = (time.perf_counter() - t0) * 1000
        self.net_ms = 0.8 * self.net_ms + 0.2 * ms if self.net_ms else ms
        return fs
