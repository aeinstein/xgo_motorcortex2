"""Rohaufzeichnung (JPEG + meta.jsonl für die Auswertung am PC), zur Laufzeit schaltbar.

Aufgezeichnet wird nur, solange der Schalter an ist, der Rider balanciert und die Bildverarbeitung läuft.
Absetzen beendet die laufende Aufnahme (Dateien geschlossen); der Schalter bleibt an, das nächste Aufstehen
beginnt ein neues Unterverzeichnis. Nach einem Neustart von xgonav ist er wieder aus.

Kosten am Pi: aus = kein Thread (erst das erste Einschalten startet ihn). An, aber Rider liegt = fünf
Attributabfragen je Sekunde. Beim Aufzeichnen ein JPEG je Bild in diesem Thread auf [cpu] render_cpus, nie in
der Hauptschleife; kommt er nicht nach, fehlen Bilder (idx zeigt die Lücken), die Bildpipeline wartet nie.
"""
from __future__ import annotations

import json
import logging
import os
import shutil
import threading
import time

import cv2

from .affinity import pin_thread

log = logging.getLogger(__name__)


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


def _meta_zeile(snap, name: str) -> str:
    st = snap.stats
    return json.dumps({
        "idx": snap.idx, "t": round(snap.t, 4), "file": name,
        "pitch_dev": round(snap.pitch_dev, 6), "pitch_src": st.get("pitch_src"),
        "sent": [round(snap.sent[0], 4), round(snap.sent[1], 4)],
        "status": snap.cmd.status, "target": snap.cmd.target_id,
        "yaw_vis_deg": st.get("yaw_vis_deg"), "balancing": st.get("balancing"),
        # Kurs/Pose für die Auswertung von explore/home: Räder, IMU (Firmware), Raumkarte
        "mode": st.get("mode"), "yaw_deg": st.get("yaw_deg"), "yaw_src": st.get("yaw_src"),
        "yaw_drv": st.get("yaw_drv"), "odom": st.get("odom"), "height_mm": st.get("height_mm"),
        "pose": (st.get("grid") or {}).get("pose"), "start": (st.get("grid") or {}).get("start"),
        "tracks": [{"id": t.id, "cls": t.cls, "conf": round(t.conf, 3),
                    "box": [round(v, 1) for v in t.box]} for t in snap.tracks],
        "tracks_all": [{"id": t.id, "cls": t.cls, "hits": t.hits, "seen": round(snap.t - t.t_seen, 2),
                        "box": [round(v, 1) for v in t.box]} for t in snap.tracks_all],
        "fs": snap.fs_diag, "cam": snap.cam_meta,
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

    def schalten(self, an: bool) -> None:
        if an:
            self.fehler = ""
        self.an = an
        if an and self._th is None:
            self._th = threading.Thread(target=self._laufen, name="aufzeichnung", daemon=True)
            self._th.start()
        self._wake.set()

    def status(self) -> dict:
        return {"an": self.an, "laeuft": self._meta is not None, "ordner": self.ordner, "bilder": self.bilder,
                "fehler": self.fehler}

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
            self.bilder = 0
            self._meta = open(os.path.join(self.ordner, "meta.jsonl"), "x", buffering=1)
            self._t_platz = now
            log.info("Rohaufzeichnung nach %s", self.ordner)
        elif now - self._t_platz >= 1.0:
            self._t_platz = now
            self._platz_pruefen()
        name = "%06d.jpg" % snap.idx
        if not cv2.imwrite(os.path.join(self.ordner, name), snap.frame, [cv2.IMWRITE_JPEG_QUALITY, 92]):
            raise OSError("JPEG nicht geschrieben: " + name)
        self._meta.write(_meta_zeile(snap, name))
        self.bilder += 1

    def _schliessen(self) -> None:
        if self._meta is None:
            return
        m, self._meta = self._meta, None
        try:
            m.close()
        except OSError as e:
            log.error("meta.jsonl nicht geschlossen: %s", e)
        log.info("Rohaufzeichnung: %d Bilder in %s", self.bilder, self.ordner)
