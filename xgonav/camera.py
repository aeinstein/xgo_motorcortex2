"""Bildquellen. Liefern immer nur den *neuesten* Frame (alte werden verworfen)."""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np

from .config import CameraCfg

log = logging.getLogger(__name__)


@dataclass
class Frame:
    idx: int
    t: float            # Aufnahmezeit, Sekunden (monoton)
    bgr: np.ndarray     # volle Auflösung
    exposure_us: int | None = None
    meta: dict | None = None   # Kamera-Metadaten für Diagnose (Weißabgleich, Verstärkung)


class FrameSource:
    def __init__(self, cfg: CameraCfg):
        self.cfg = cfg
        self._cond = threading.Condition()
        self._latest: Frame | None = None
        self._idx = 0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.fps_measured = 0.0
        self.eof = False
        self._paused = threading.Event()
        self._device_suspended = False

    def start(self) -> None:
        self._open()
        self._thread = threading.Thread(target=self._run, name="camera", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        self._close()

    @property
    def paused(self) -> bool:
        return self._paused.is_set()

    def pause(self) -> None:
        """Aufnahme anhalten (Kamera wird im Aufnahme-Thread gestoppt, spart Strom/CPU)."""
        self._paused.set()

    def resume(self) -> None:
        self._paused.clear()

    def get(self, after_idx: int = -1, timeout: float = 1.0) -> Frame | None:
        """Neuesten Frame mit idx > after_idx, blockiert bis timeout."""
        end = time.monotonic() + timeout
        with self._cond:
            while self._latest is None or self._latest.idx <= after_idx:
                rem = end - time.monotonic()
                if rem <= 0 or self.eof:
                    return None
                self._cond.wait(rem)
            return self._latest

    def _publish(self, bgr: np.ndarray, t: float, exp: int | None = None, meta: dict | None = None) -> None:
        with self._cond:
            prev = self._latest
            self._latest = Frame(self._idx, t, bgr, exp, meta)
            self._idx += 1
            if prev is not None and t > prev.t:
                inst = 1.0 / (t - prev.t)
                self.fps_measured = 0.9 * self.fps_measured + 0.1 * inst if self.fps_measured else inst
            self._cond.notify_all()

    def _run(self) -> None:
        while not self._stop.is_set():
            if self._paused.is_set():
                if not self._device_suspended:
                    self._suspend()
                    self._device_suspended = True
                self._stop.wait(0.1)
                continue
            if self._device_suspended:
                self._resume_device()
                self._device_suspended = False
            try:
                if not self._grab():
                    break
            except Exception:
                log.exception("Kamerafehler")
                time.sleep(0.1)
        with self._cond:
            self.eof = True
            self._cond.notify_all()

    def _open(self) -> None: ...
    def _close(self) -> None: ...
    # Belichtung zur Laufzeit (Weboberfläche). 0 = Automatik (kurz), sonst feste Zeit in µs.
    EXPOSURE_MIN_US = 100

    @property
    def exposure_max_us(self) -> int:
        return int(1e6 / self.cfg.fps)       # länger als ein Bild geht bei fester Bildrate nicht

    @property
    def exposure_us(self) -> int:
        return int(self.cfg.exposure_us)

    def set_exposure(self, us: int) -> int:
        """Belichtungszeit setzen (0 = Automatik). Wirkt nur zur Laufzeit, nicht in config.toml."""
        us = int(us)
        if us != 0 and not (self.EXPOSURE_MIN_US <= us <= self.exposure_max_us):
            raise ValueError(f"Belichtung {us} µs außerhalb {self.EXPOSURE_MIN_US}..{self.exposure_max_us} (0 = Auto)")
        self.cfg.exposure_us = us
        self._apply_exposure(us)
        log.info("Belichtung -> %s", f"{us} µs" if us else "Automatik")
        return us

    GAIN_MIN, GAIN_MAX = 1.0, 16.0           # OV5647 laut libcamera; die Automatik geht nur bis 8

    @property
    def gain(self) -> float:
        return float(self.cfg.analogue_gain)

    def set_gain(self, gain: float) -> float:
        """Analoge Verstärkung setzen (0 = Automatik). Wirkt nur zur Laufzeit, nicht in config.toml."""
        gain = round(float(gain), 2)
        if gain != 0 and not (self.GAIN_MIN <= gain <= self.GAIN_MAX):
            raise ValueError(f"Verstärkung {gain} außerhalb {self.GAIN_MIN:g}..{self.GAIN_MAX:g} (0 = Auto)")
        self.cfg.analogue_gain = gain
        self._apply_gain(gain)
        log.info("Verstärkung -> %s", f"{gain:g}" if gain else "Automatik")
        return gain

    def _apply_exposure(self, us: int) -> None: ...   # Quellen ohne steuerbare Kamera
    def _apply_gain(self, gain: float) -> None: ...
    def _suspend(self) -> None: ...          # Gerät anhalten (im Aufnahme-Thread)
    def _resume_device(self) -> None: ...
    def _grab(self) -> bool: raise NotImplementedError


class PiCameraSource(FrameSource):
    """OV5647 über libcamera/picamera2. Kurze feste Belichtung, AGC regelt nur Gain
    (beides zur Laufzeit umstellbar, siehe set_exposure/set_gain)."""

    def _open(self) -> None:
        from picamera2 import Picamera2  # nur auf dem Pi verfügbar

        self.cam = Picamera2()
        frame_us = int(1e6 / self.cfg.fps)
        conf = self.cam.create_video_configuration(
            main={"size": (self.cfg.width, self.cfg.height), "format": "RGB888"},  # = BGR im Speicher
            buffer_count=4,
            controls={"FrameDurationLimits": (frame_us, frame_us)},
        )
        self.cam.configure(conf)
        self._apply_exposure(int(self.cfg.exposure_us))
        self._apply_gain(float(self.cfg.analogue_gain))
        self.cam.start()
        log.info("PiCamera gestartet %dx%d@%d, Belichtung %s µs, Verstärkung %s",
                 self.cfg.width, self.cfg.height, self.cfg.fps, self.cfg.exposure_us or "auto/short",
                 self.cfg.analogue_gain or "auto")

    def _apply_exposure(self, us: int) -> None:
        # Fixe Belichtungszeit, AnalogueGain bleibt automatisch. ExposureTime = 0 gibt die
        # Belichtung an die Automatik zurück (libcamera/rpi), dort im Modus "kurz".
        from libcamera import controls as lc
        self.cam.set_controls({"ExposureTime": max(0, int(us)),
                               "AeExposureMode": lc.AeExposureModeEnum.Short})

    def _apply_gain(self, gain: float) -> None:
        # AnalogueGain = 0 gibt die Verstärkung an die Automatik zurück (libcamera/rpi).
        self.cam.set_controls({"AnalogueGain": max(0.0, float(gain))})

    def _grab(self) -> bool:
        req = self.cam.capture_request()
        try:
            arr = req.make_array("main")
            md = req.get_metadata()
        finally:
            req.release()
        ts = md.get("SensorTimestamp")
        t = ts / 1e9 if ts else time.monotonic()
        cg = md.get("ColourGains")
        self._publish(arr, t, md.get("ExposureTime"),
                      {"awb": [round(float(v), 3) for v in cg] if cg else None,
                       "gain": round(float(md.get("AnalogueGain", 0.0)), 2),
                       "exp_us": md.get("ExposureTime")})
        return True

    def _suspend(self) -> None:
        self.cam.stop()
        log.info("PiCamera angehalten")

    def _resume_device(self) -> None:
        self.cam.start()
        log.info("PiCamera läuft wieder")

    def _close(self) -> None:
        try:
            self.cam.stop()
            self.cam.close()
        except Exception:
            pass


class OpenCVSource(FrameSource):
    """USB-Kamera (Index) oder Videodatei für Replay/Tests."""

    def _open(self) -> None:
        src = self.cfg.source
        self.is_file = not src.isdigit()
        self.cap = cv2.VideoCapture(src if self.is_file else int(src))
        if not self.cap.isOpened():
            raise RuntimeError(f"Kann Quelle nicht öffnen: {src}")
        if not self.is_file:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cfg.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cfg.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.cfg.fps)
        fps = self.cap.get(cv2.CAP_PROP_FPS) or self.cfg.fps
        self.period = 1.0 / (fps if fps > 1 else self.cfg.fps)
        self._t_next = time.monotonic()
        self._vt = 0.0  # Videozeit, für nicht-Echtzeit-Replay

    def _grab(self) -> bool:
        ok, img = self.cap.read()
        if not ok:
            if self.is_file and self.cfg.loop_file:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, img = self.cap.read()
            if not ok:
                return False
        if img.shape[1] != self.cfg.width or img.shape[0] != self.cfg.height:
            img = cv2.resize(img, (self.cfg.width, self.cfg.height), interpolation=cv2.INTER_AREA)
        if self.is_file and self.cfg.realtime_file:
            self._t_next += self.period
            delay = self._t_next - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                self._t_next = time.monotonic()
            t = time.monotonic()
        elif self.is_file:
            self._vt += self.period
            t = self._vt
            # Nicht-Echtzeit: warten bis der Konsument den Frame abgeholt hat
            with self._cond:
                while self._latest is not None and not getattr(self, "_consumed", True) and not self._stop.is_set():
                    self._cond.wait(0.05)
                self._consumed = False
        else:
            t = time.monotonic()
        self._publish(img, t)
        return True

    def mark_consumed(self) -> None:
        with self._cond:
            self._consumed = True
            self._cond.notify_all()

    def _close(self) -> None:
        self.cap.release()


def make_source(cfg: CameraCfg) -> FrameSource:
    if cfg.source == "picam":
        return PiCameraSource(cfg)
    return OpenCVSource(cfg)
