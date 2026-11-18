"""Bildquellen. Liefern immer nur den *neuesten* Frame (alte werden verworfen)."""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass

import ctypes
import math

import cv2
import numpy as np

from . import fastlib
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
        self._dt_avg = 0.0
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
                # Bildabstand mitteln, nicht 1/dt: bei Zeitstempel-Jitter ist der Mittelwert von 1/dt
                # zu hoch (am Rider 26,5 statt 25 fps bei FrameDuration 39969 µs).
                dt = t - prev.t
                self._dt_avg = 0.9 * self._dt_avg + 0.1 * dt if self._dt_avg else dt
                self.fps_measured = 1.0 / self._dt_avg
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
                                             # (bezogen auf die Sensor-Zeitbasis, also ohne Korrektur)

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

    GAIN_MIN, GAIN_MAX = 1.0, 63.9           # Treiber 16..1023 = 1..63,94x (am Rider gemessen); Automatik nur bis 8

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


def has_lens(cfg: CameraCfg) -> bool:
    return bool(cfg.lens_knots_px) and len(cfg.lens_knots_px) == len(cfg.lens_g_px) and len(cfg.lens_center) == 2


def lens_radial(cfg: CameraCfg, p: np.ndarray) -> np.ndarray:
    """Radialverzeichnung: gemessenes Pixel (…, 2) -> ideales Pixel, ideal = c + q * (1 + g(r) / r)."""
    kn, g = np.asarray(cfg.lens_knots_px, float), np.asarray(cfg.lens_g_px, float)
    c = np.asarray(cfg.lens_center, float)
    q = np.asarray(p, float) - c
    r = np.hypot(q[..., 0], q[..., 1])
    return c + q * (1.0 + np.interp(r, kn, g) / np.maximum(r, 1e-6))[..., None]


def lens_rot(cfg: CameraCfg) -> np.ndarray:
    """Homographie ideales Pixel -> Zielpixel (virtuelle Kamera): Einbau-Roll und -Gier herausgedreht, um die
    Bildmitte W/2, H/2 mit f aus hfov_deg. Ohne Roll/Gier die Einheitsmatrix."""
    f = (cfg.width / 2) / math.tan(math.radians(cfg.hfov_deg) / 2)
    K = np.array([[f, 0, cfg.width / 2], [0, f, cfg.height / 2], [0, 0, 1.0]])
    # Vorzeichen: lens_roll_deg wirkt wie früher [imu] roll_ref_deg in freespace.roll_columns (test_linse_roll_gier)
    a, b = math.radians(cfg.lens_roll_deg), math.radians(cfg.lens_yaw_deg)
    # Roll: Bild um die Mitte gedreht (Kamera um die Blickachse); Gier: um die senkrechte Achse
    Rr = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1.0]])
    Ry = np.array([[math.cos(b), 0, math.sin(b)], [0, 1, 0], [-math.sin(b), 0, math.cos(b)]])
    # ideal = K Ry Rr K^-1 ziel  ->  ziel = (K Ry Rr K^-1)^-1 ideal
    return np.linalg.inv(K @ Ry @ Rr @ np.linalg.inv(K))


def _homog(H: np.ndarray, p: np.ndarray) -> np.ndarray:
    q = np.asarray(p, float) @ H[:, :2].T + H[:, 2]
    return q[..., :2] / q[..., 2:3]


def lens_forward(cfg: CameraCfg, p: np.ndarray) -> np.ndarray:
    """Gemessenes Pixel (…, 2) -> Pixel im entzerrten Bild (dieselbe Abbildung wie lens_maps, vorwärts)."""
    return _homog(lens_rot(cfg), lens_radial(cfg, p))


def lens_source(cfg: CameraCfg, v: np.ndarray, iters: int = 10) -> np.ndarray:
    """Pixel im entzerrten Bild (…, 2) -> gemessenes Pixel: Drehung zurück, dann Radialverzeichnung per
    Festpunktiteration umgekehrt. Nicht begrenzt."""
    u = _homog(np.linalg.inv(lens_rot(cfg)), v)
    p = u.copy()
    for _ in range(iters):
        p = p + (u - lens_radial(cfg, p))
    return p


def lens_id(cfg: CameraCfg) -> str | None:
    """Kurze Kennung des Linsenblocks (alle Werte, die die Tabelle bestimmen); steht je Bild als cam.entzerrt in den
    Aufnahmen. None ohne Linsenmodell. Ältere Aufnahmen tragen true (nur Radialverzeichnung, ohne Roll/Gier)."""
    if not has_lens(cfg):
        return None
    import hashlib
    s = repr((list(map(float, cfg.lens_knots_px)), list(map(float, cfg.lens_g_px)), list(map(float, cfg.lens_center)),
              float(cfg.lens_roll_deg), float(cfg.lens_yaw_deg), int(cfg.width), int(cfg.height), float(cfg.hfov_deg)))
    return "L" + hashlib.md5(s.encode()).hexdigest()[:8]


def lens_maps(cfg: CameraCfg):
    """Tabellen für cv2.remap (Festkomma), die das Kamerabild entzerren, oder None ohne Linsenmodell.
    Je Zielpixel das Quellpixel im Kamerabild (lens_source): Einbau-Roll und -Gier zurückgedreht, dann die
    Radialverzeichnung umgekehrt. Quellen außerhalb des Bilds werden auf den Rand begrenzt (Rand aufgefüllt, keine
    schwarzen Pixel). Einmal beim Start gerechnet."""
    if not has_lens(cfg):
        return None
    gx, gy = np.meshgrid(np.arange(cfg.width, dtype=np.float64), np.arange(cfg.height, dtype=np.float64))
    p = lens_source(cfg, np.stack([gx, gy], -1))
    px = np.clip(p[..., 0], 0.0, cfg.width - 1.0).astype(np.float32)
    py = np.clip(p[..., 1], 0.0, cfg.height - 1.0).astype(np.float32)
    return cv2.convertMaps(px, py, cv2.CV_16SC2)


_LENS_ABI = 1


def lens_remap(cfg: CameraCfg):
    """Funktion Bild -> entzerrtes Bild für diese Kamera, oder None ohne Linsenmodell. Mit C++-Kern
    (native/linsefast.cpp), wenn er für Rauschbild und glattes Bild bitgleich zu cv2.remap rechnet; sonst cv2.remap."""
    maps = lens_maps(cfg)
    if maps is None:
        return None
    m1, m2 = np.ascontiguousarray(maps[0]), np.ascontiguousarray(maps[1])
    h, w = m2.shape

    def mit_cv(img):
        return cv2.remap(img, m1, m2, cv2.INTER_LINEAR)

    lib = fastlib.load() if cfg.lens_native else None
    if not cfg.lens_native or not fastlib.check_abi(lib, "lens_abi", _LENS_ABI):
        log.info("Linsenverzeichnung wird entzerrt (cv2.remap)")
        return mit_cv
    lib.lens_remap.restype = ctypes.c_int
    lib.lens_remap.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_void_p,
                               ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int]

    def mit_kern(img):
        if img.ndim != 3 or img.dtype != np.uint8 or not img.flags.c_contiguous:
            return mit_cv(img)
        out = np.empty((h, w, img.shape[2]), np.uint8)
        if lib.lens_remap(img.ctypes.data, img.shape[1], img.shape[0], img.strides[0], img.shape[2], m1.ctypes.data,
                          m2.ctypes.data, out.ctypes.data, w, h, out.strides[0]) != 0:
            return mit_cv(img)
        return out

    rng = np.random.default_rng(11)
    probe = rng.integers(0, 256, (cfg.height, cfg.width, 3), dtype=np.uint8)
    for img in (probe, cv2.GaussianBlur(probe, (0, 0), 3)):
        if not np.array_equal(mit_kern(img), mit_cv(img)):
            log.info("lens_remap weicht von cv2.remap ab – Linsenverzeichnung wird mit OpenCV entzerrt")
            return mit_cv
    log.info("Linsenverzeichnung wird entzerrt (C++-Kern)")
    return mit_kern


class PiCameraSource(FrameSource):
    """OV5647 über libcamera/picamera2. Kurze feste Belichtung, AGC regelt nur Gain
    (beides zur Laufzeit umstellbar, siehe set_exposure/set_gain)."""

    def _open(self) -> None:
        from picamera2 import Picamera2  # nur auf dem Pi verfügbar

        self.cam = Picamera2()
        self._lens = lens_remap(self.cfg)
        self._lens_id = lens_id(self.cfg) if self._lens is not None else None
        frame_us = int(1e6 / self.cfg.fps * self.cfg.frame_clock_corr)
        controls = {"FrameDurationLimits": (frame_us, frame_us)}
        if self.cfg.scaler_crop:
            controls["ScalerCrop"] = tuple(int(v) for v in self.cfg.scaler_crop)
        kw = {}
        if self.cfg.sensor_mode:
            w, h = (int(v) for v in self.cfg.sensor_mode.lower().split("x"))
            kw["raw"] = {"size": (w, h)}
        conf = self.cam.create_video_configuration(
            main={"size": (self.cfg.width, self.cfg.height), "format": "RGB888"},  # = BGR im Speicher
            buffer_count=4, controls=controls, **kw,
        )
        self.cam.configure(conf)
        self._apply_exposure(int(self.cfg.exposure_us))
        self._apply_gain(float(self.cfg.analogue_gain))
        self.cam.start()
        log.info("PiCamera gestartet %dx%d@%d (Sensor %s), Belichtung %s µs, Verstärkung %s",
                 self.cfg.width, self.cfg.height, self.cfg.fps,
                 "x".join(map(str, self.cam.camera_configuration()["raw"]["size"])),
                 self.cfg.exposure_us or "auto/short", self.cfg.analogue_gain or "auto")

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
        if self._lens is not None:
            arr = self._lens(arr)
        ts = md.get("SensorTimestamp")
        t = ts / 1e9 if ts else time.monotonic()
        cg = md.get("ColourGains")
        self._publish(arr, t, md.get("ExposureTime"),
                      {"awb": [round(float(v), 3) for v in cg] if cg else None,
                       "gain": round(float(md.get("AnalogueGain", 0.0)), 2),
                       "exp_us": md.get("ExposureTime"), "frame_us": md.get("FrameDuration"),
                       "crop": md.get("ScalerCrop"), "entzerrt": self._lens_id or False})
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
