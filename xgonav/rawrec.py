"""Rohaufzeichnung (JPEG + meta.jsonl für die Auswertung am PC), zur Laufzeit schaltbar.

Aufgezeichnet wird nur, solange der Schalter an ist, der Rider balanciert und die Bildverarbeitung läuft.
Absetzen beendet die laufende Aufnahme (Dateien geschlossen); der Schalter bleibt an, das nächste Aufstehen
beginnt ein neues Unterverzeichnis. Nach einem Neustart von xgonav ist er wieder aus.

Kosten am Pi: aus = kein Thread (erst das erste Einschalten startet ihn). An, aber Rider liegt = fünf
Attributabfragen je Sekunde. Beim Aufzeichnen ein JPEG je Bild in diesem Thread auf [cpu] render_cpus, nie in
der Hauptschleife; kommt er nicht nach, fehlen Bilder (idx zeigt die Lücken), die Bildpipeline wartet nie.

Je Bild stehen in meta.jsonl auch die IMU- und Radproben seit dem letzten Bild ("imu": t, Roll, Nick, Kurs
in Grad; "raeder": t, Weg links, rechts in m; t = CLOCK_MONOTONIC wie die Bildzeit). Damit rechnet
tools/map_replay.py Kurs und Roll zur Bildzeit nach. Sie kommen aus den Puffern, die imu.py und wheels.py
ohnehin führen (~4 Proben je Bild bei 100 Hz), gelesen in diesem Thread.
"""
from __future__ import annotations

import json
import logging
import math
import os
import shutil
import threading
import time

import cv2

from .affinity import pin_thread
from .jpeg import Jpeg

log = logging.getLogger(__name__)

# Schreibrate bis zur ersten Aufnahme seit dem Start (geschätzt: 640x480 JPEG q95 ~90 kB + meta.jsonl ~2 kB,
# ~25 Bilder/s); danach gilt die gemessene Rate der letzten Aufnahme. Nur für die Restminuten in der Oberfläche.
RATE_SCHAETZUNG_MB_S = 2.2
RATE_MIN_S = 3.0          # erst nach so vielen Sekunden Aufnahme gilt die gemessene Rate
PLATZ_ALTER_S = 2.0       # freier Platz für status() höchstens so oft neu geholt


def raw_run_dir(base: str) -> str:
    """Neues Unterverzeichnis anlegen. idx beginnt nach jedem Start bei 0, im selben Verzeichnis würden
    sonst die Bilder der letzten Aufnahme überschrieben."""
    os.makedirs(base, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    name, n = stamp, 1
    while True:                # Gleiche Sekunde oder Uhr nach Neustart noch nicht gestellt
        path = os.path.join(base, name)
        try:
            os.mkdir(path)
            return path
        except FileExistsError:
            n += 1
            name = f"{stamp}-{n}"


PROBEN_VORLAUF_S = 0.2      # erstes Bild einer Aufnahme: Proben ab so lange vor der Bildzeit


def _meta_zeile(snap, name: str, imu=None, raeder=None, jpeg: str | None = None) -> str:
    """imu: Proben (t, roll, pitch, yaw in rad), raeder: (t, s_links, s_rechts in m); None = Quelle fehlt.
    jpeg: Kodierer des Bildes ("hw95" Hardware, "cv92" libjpeg; fehlt in Aufnahmen vor 2026-11-19 = cv92)."""
    st = snap.stats
    return json.dumps({
        "idx": snap.idx, "t": round(snap.t, 4), "file": name, "jpeg": jpeg,
        "pitch_dev": round(snap.pitch_dev, 6), "pitch_src": st.get("pitch_src"),
        "sent": [round(snap.sent[0], 4), round(snap.sent[1], 4)],
        "status": snap.cmd.status, "target": snap.cmd.target_id,
        "yaw_vis_deg": st.get("yaw_vis_deg"), "balancing": st.get("balancing"),
        # Kurs/Pose für die Auswertung von explore/home: Räder, IMU (Firmware), Raumkarte
        "mode": st.get("mode"), "yaw_deg": st.get("yaw_deg"), "yaw_src": st.get("yaw_src"),
        "yaw_imu": st.get("yaw_imu"), "odom": st.get("odom"), "height_mm": st.get("height_mm"),
        "speed_ctrl": st.get("speed_ctrl"),
        # Tempo-Profil des Wegfolgers (home.free_slow): freie Strecke bis Knick/Hindernis auf Weg/Bogen, None = frei
        "frei": snap.cmd.extra.get("frei") if snap.cmd is not None else None,
        "pose": (st.get("grid") or {}).get("pose"), "start": (st.get("grid") or {}).get("start"),
        "tracks": [{"id": t.id, "cls": t.cls, "conf": round(t.conf, 3),
                    "box": [round(v, 1) for v in t.box]} for t in snap.tracks],
        "tracks_all": [{"id": t.id, "cls": t.cls, "hits": t.hits, "seen": round(snap.t - t.t_seen, 2),
                        "box": [round(v, 1) for v in t.box]} for t in snap.tracks_all],
        "cam": snap.cam_meta,
        "imu": None if imu is None else [[round(t, 4), round(math.degrees(r), 3), round(math.degrees(p), 3),
                                          round(math.degrees(y), 3)] for t, r, p, y in imu],
        "raeder": None if raeder is None else [[round(t, 4), round(l, 5), round(r, 5)] for t, l, r in raeder],
    }) + "\n"


class RawRecorder:
    def __init__(self, app, basis: str, min_frei_mb: int = 500, cpus=None):
        self.app = app
        self.basis = os.path.expanduser(basis)
        self.min_frei = int(min_frei_mb) * 1024 * 1024
        self.cpus = cpus
        self.an = False
        self.fehler = ""             # warum sie sich selbst abgeschaltet hat, für die Oberfläche
        self.ordner = ""             # letzte oder laufende Aufnahme
        self.bilder = 0
        self._meta = None
        self._ende = False
        self._wake = threading.Event()
        self._th: threading.Thread | None = None
        self._t_platz = 0.0
        self._t_imu = self._t_rad = 0.0     # Zeit der letzten geschriebenen Probe
        self._t_start = 0.0                 # Start der laufenden Aufnahme (monotonic)
        self._bytes = 0                     # geschrieben in der laufenden Aufnahme
        self._rate: float | None = None     # MB/s der letzten Aufnahme (gemessen)
        self._frei_mb: float | None = None
        self._t_frei = -1e9
        oc = getattr(getattr(app, "cfg", None), "output", None)
        self._q_hw = int(getattr(oc, "record_jpeg_q_hw", 95))
        self._q_cv = int(getattr(oc, "record_jpeg_q_cv", 92))
        self._jpeg_native = bool(getattr(oc, "jpeg_native", True))
        self._jpeg_dev = getattr(oc, "jpeg_dev", "/dev/video31")
        self._jpeg: Jpeg | None = None      # eigener Kodierer, erst im Aufzeichnungs-Faden geöffnet
        self._karte_s = float(getattr(oc, "record_map_s", 0.0))
        self._karte_px = int(getattr(oc, "record_map_px", 480))
        self._karte_m = float(getattr(oc, "record_map_view_m", 6.0))
        self._t_karte = -1e9
        self.karten = 0

    def schalten(self, an: bool) -> None:
        if an:
            self.fehler = ""
        self.an = an
        if an and self._th is None:
            self._th = threading.Thread(target=self._laufen, name="aufzeichnung", daemon=True)
            self._th.start()
        self._wake.set()

    def _frei(self) -> float | None:
        """Nutzbarer Platz in MB bis zur Abbruchgrenze min_frei (ein statvfs höchstens alle PLATZ_ALTER_S)."""
        now = time.monotonic()
        if now - self._t_frei >= PLATZ_ALTER_S:
            self._t_frei = now
            pfad = self.basis
            while pfad and not os.path.isdir(pfad):          # Ordner noch nicht angelegt: Elternordner
                oben = os.path.dirname(pfad)
                pfad = "" if oben == pfad else oben
            try:
                frei = shutil.disk_usage(pfad).free if pfad else None
            except OSError:
                frei = None
            self._frei_mb = None if frei is None else max(0.0, (frei - self.min_frei) / (1024 * 1024))
        return self._frei_mb

    def status(self) -> dict:
        frei = self._frei()
        return {"an": self.an, "laeuft": self._meta is not None, "ordner": self.ordner, "bilder": self.bilder,
                "fehler": self.fehler, "frei_mb": None if frei is None else round(frei, 1),
                "rate_mb_s": round(self._rate if self._rate is not None else RATE_SCHAETZUNG_MB_S, 3),
                "rate_gemessen": self._rate is not None}

    def beenden(self) -> None:
        self._ende = True
        self._wake.set()
        if self._th is not None:
            self._th.join(timeout=5)

    def _warte(self, timeout: float | None) -> None:
        self._wake.wait(timeout)
        self._wake.clear()

    def _darf(self) -> bool:
        a = self.app
        return a.running and a.pipeline_on and a.robot.state.balancing

    def _laufen(self) -> None:
        pin_thread(self.cpus, "Aufzeichnung")
        a = self.app
        last, bereit = -1, False
        try:
            while not self._ende:
                if not self.an or not self._darf():
                    self._schliessen()
                    bereit = False
                    self._warte(None if not self.an else 0.2)   # aus: schlafen bis zum Einschalten
                    continue
                if not bereit:                                  # nur neue Bilder, kein Standbild von vorher
                    last, bereit = a.snapshot.idx, True
                with a.snap_cond:
                    a.snap_cond.wait_for(lambda: a.snapshot.idx != last or not a.running, timeout=0.5)
                    snap = a.snapshot
                if snap.idx == last or snap.frame is None:
                    continue
                last = snap.idx
                if not snap.stats.get("balancing", True):
                    continue
                try:
                    self._schreiben(snap)
                except OSError as e:
                    self.fehler = str(e)
                    self.an = False
                    log.error("Rohaufzeichnung abgebrochen: %s", e)
        finally:
            self._schliessen()

    def _platz_pruefen(self) -> None:
        frei = shutil.disk_usage(self.basis).free
        if frei < self.min_frei:
            raise OSError("nur noch %d MB frei" % (frei // (1024 * 1024)))

    def _schreiben(self, snap) -> None:
        now = time.monotonic()
        if self._meta is None:
            os.makedirs(self.basis, exist_ok=True)
            self._platz_pruefen()
            self.ordner = raw_run_dir(self.basis)
            self.bilder = self.karten = 0
            self._t_karte = -1e9
            self._meta = open(os.path.join(self.ordner, "meta.jsonl"), "x", buffering=1)
            self._t_platz = self._t_start = now
            self._bytes = 0
            self._t_imu = self._t_rad = snap.t - PROBEN_VORLAUF_S
            log.info("Rohaufzeichnung nach %s", self.ordner)
        elif now - self._t_platz >= 1.0:
            self._t_platz = now
            self._platz_pruefen()
        name = "%06d.jpg" % snap.idx
        if self._jpeg is None:
            self._jpeg = Jpeg(self._jpeg_native, self._jpeg_dev)
        jpg = self._jpeg.encode(snap.frame, self._q_hw, self._q_cv)
        if jpg is None:
            raise OSError("JPEG nicht kodiert: " + name)
        with open(os.path.join(self.ordner, name), "wb") as f:
            f.write(jpg)
        imu = raeder = None
        src = getattr(self.app, "imu", None)
        if src is not None:
            imu = src.since(self._t_imu)
            if imu:
                self._t_imu = imu[-1][0]
        src = getattr(self.app, "wheels", None)
        if src is not None:
            raeder = src.since(self._t_rad)
            if raeder:
                self._t_rad = raeder[-1][0]
        if self._karte_s > 0 and now - self._t_karte >= self._karte_s:
            self._t_karte = now
            self._karte(snap.idx)
        zeile = _meta_zeile(snap, name, imu, raeder, self._jpeg.zuletzt)
        self._meta.write(zeile)
        self.bilder += 1
        self._bytes += len(jpg) + len(zeile)
        dauer = now - self._t_start
        if dauer >= RATE_MIN_S and dauer > 0:
            self._rate = self._bytes / dauer / (1024 * 1024)

    def _karte(self, idx: int) -> None:
        """Raumkarte samt Merkkarte als PNG (karte/<idx>.png), Bildnummer wie das JPEG zur selben Zeit. Ein Fehler
        beim Zeichnen beendet die Aufnahme nicht."""
        gm = getattr(self.app, "gridmap", None)
        if gm is None:
            return
        try:
            img = gm.render(self._karte_px, self._karte_m, None, self.app.merk_punkte())[0]
            ok, png = cv2.imencode(".png", img)
        except Exception as e:                          # noqa: BLE001 - nur Anzeige
            log.warning("Kartenbild nicht gezeichnet: %s", e)
            return
        if not ok:
            return
        d = os.path.join(self.ordner, "karte")
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, "%06d.png" % idx), "wb") as f:
            f.write(png.tobytes())
        self.karten += 1
        self._bytes += len(png)

    def _schliessen(self) -> None:
        if self._meta is None:
            return
        m, self._meta = self._meta, None
        try:
            m.close()
        except OSError as e:
            log.error("meta.jsonl nicht geschlossen: %s", e)
        log.info("Rohaufzeichnung: %d Bilder in %s", self.bilder, self.ordner)
