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

    BILD_MIN, BILD_MAX = 0.5, 4.0            # Kontrast/Sättigung im ISP (libcamera erlaubt 0..32, 1 = Standard)
    HELL_MIN, HELL_MAX = -0.5, 0.5           # Helligkeit im ISP (libcamera -1..1, 0 = Standard)

    @property
    def bild(self) -> tuple[float, float, float]:
        """(Kontrast, Sättigung, Helligkeit), Sollwerte."""
        return (float(getattr(self.cfg, "contrast", 1.0)), float(getattr(self.cfg, "saturation", 1.0)),
                float(getattr(self.cfg, "brightness", 0.0)))

    def set_bild(self, kontrast: float, saettigung: float, hell: float | None = None) -> tuple[float, float, float]:
        """Kontrast, Sättigung und Helligkeit an der Quelle (ISP) setzen. Wirkt nur zur Laufzeit, nicht in config.toml.
        FloorNet ist auf 1,0/1,0/0 trainiert; die Werte stehen je Bild in den Metadaten (meta.jsonl cam).
        hell None = unverändert. Ungültig = nichts geändert."""
        k, s = round(float(kontrast), 2), round(float(saettigung), 2)
        b = round(float(self.bild[2] if hell is None else hell), 2)
        for name, v in (("Kontrast", k), ("Sättigung", s)):
            if not (self.BILD_MIN <= v <= self.BILD_MAX):
                raise ValueError(f"{name} {v} außerhalb {self.BILD_MIN:g}..{self.BILD_MAX:g}")
        if not (self.HELL_MIN <= b <= self.HELL_MAX):
            raise ValueError(f"Helligkeit {b} außerhalb {self.HELL_MIN:g}..{self.HELL_MAX:g}")
        self.cfg.contrast, self.cfg.saturation, self.cfg.brightness = k, s, b
        self._apply_bild(k, s, b)
        log.info("Kontrast %.2f, Sättigung %.2f, Helligkeit %+.2f", k, s, b)
        return k, s, b

    def _apply_exposure(self, us: int) -> None: ...   # Quellen ohne steuerbare Kamera
    def _apply_gain(self, gain: float) -> None: ...
    def _apply_bild(self, kontrast: float, saettigung: float, hell: float) -> None: ...
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
_CAM_ABI = 4


def _lens_kern(lib, m1: np.ndarray, m2: np.ndarray):
    """Entzerren mit lens_remap aus lib (libxgofast.so oder libxgocam.so); None-Rückgabe des Kerns -> Ausnahme."""
    lib.lens_remap.restype = ctypes.c_int
    lib.lens_remap.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_void_p,
                               ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int]
    h, w = m2.shape

    def kern(img):
        out = np.empty((h, w, img.shape[2]), np.uint8)
        if lib.lens_remap(img.ctypes.data, img.shape[1], img.shape[0], img.strides[0], img.shape[2], m1.ctypes.data,
                          m2.ctypes.data, out.ctypes.data, w, h, out.strides[0]) != 0:
            raise RuntimeError("lens_remap")
        return out
    return kern


def _lens_kern_ok(cfg: CameraCfg, kern, mit_cv) -> bool:
    """Selbstprüfung: Kern rechnet für Rauschbild und glattes Bild bitgleich zu cv2.remap."""
    rng = np.random.default_rng(11)
    probe = rng.integers(0, 256, (cfg.height, cfg.width, 3), dtype=np.uint8)
    try:
        return all(np.array_equal(kern(img), mit_cv(img)) for img in (probe, cv2.GaussianBlur(probe, (0, 0), 3)))
    except RuntimeError:
        return False


def lens_remap(cfg: CameraCfg):
    """Funktion Bild -> entzerrtes Bild für diese Kamera, oder None ohne Linsenmodell. Mit C++-Kern
    (native/linsefast.cpp), wenn er für Rauschbild und glattes Bild bitgleich zu cv2.remap rechnet; sonst cv2.remap."""
    maps = lens_maps(cfg)
    if maps is None:
        return None
    m1, m2 = np.ascontiguousarray(maps[0]), np.ascontiguousarray(maps[1])

    def mit_cv(img):
        return cv2.remap(img, m1, m2, cv2.INTER_LINEAR)

    lib = fastlib.load() if cfg.lens_native else None
    if not cfg.lens_native or not fastlib.check_abi(lib, "lens_abi", _LENS_ABI):
        log.info("Linsenverzeichnung wird entzerrt (cv2.remap)")
        return mit_cv
    kern = _lens_kern(lib, m1, m2)

    def mit_kern(img):
        if img.ndim != 3 or img.dtype != np.uint8 or not img.flags.c_contiguous:
            return mit_cv(img)
        try:
            return kern(img)
        except RuntimeError:
            return mit_cv(img)

    if not _lens_kern_ok(cfg, kern, mit_cv):
        log.info("lens_remap weicht von cv2.remap ab – Linsenverzeichnung wird mit OpenCV entzerrt")
        return mit_cv
    log.info("Linsenverzeichnung wird entzerrt (C++-Kern)")
    return mit_kern


class _CamMeta(ctypes.Structure):
    """Wie CamMeta in native/camfast.cpp."""
    _fields_ = [("ts_ns", ctypes.c_int64), ("frame_us", ctypes.c_int64), ("exp_us", ctypes.c_int32),
                ("gain", ctypes.c_float), ("awb", ctypes.c_float * 2), ("has_awb", ctypes.c_int32),
                ("crop", ctypes.c_int32 * 4), ("has_crop", ctypes.c_int32), ("dropped", ctypes.c_int64)]


def _cam_lib():
    """libxgocam.so mit passender ABI und Signaturen, sonst None."""
    lib = fastlib.load_cam()
    if not fastlib.check_abi(lib, "cam_abi", _CAM_ABI, fastlib.CAM_PATH):
        return None
    vp, i, i64 = ctypes.c_void_p, ctypes.c_int, ctypes.c_int64
    lib.cam_open.restype = vp
    lib.cam_open.argtypes = [i, i, i, i, i64, vp, i, ctypes.c_char_p, i]
    for name, args in (("cam_set_lens", [vp, vp, vp, i, i]), ("cam_start", [vp]), ("cam_stop", [vp]),
                       ("cam_set_exposure", [vp, i]), ("cam_set_gain", [vp, ctypes.c_float]),
                       ("cam_get", [vp, i, vp, i, ctypes.POINTER(_CamMeta)]), ("cam_info", [vp, vp]),
                       ("cam_zeiten", [vp, vp]), ("cam_set_bild", [vp, ctypes.c_float, ctypes.c_float, ctypes.c_float])):
        getattr(lib, name).restype = i
        getattr(lib, name).argtypes = args
    lib.cam_close.restype = None
    lib.cam_close.argtypes = [vp]
    return lib


class PiCameraSource(FrameSource):
    """OV5647 über libcamera. Kurze feste Belichtung, AGC regelt nur Gain (beides zur Laufzeit umstellbar, siehe
    set_exposure/set_gain). Mit libxgocam.so ([camera] capture_native) direkt aus C++: das Bild wird aus dem
    DMA-Puffer heraus entzerrt und kommt fertig in Python an; sonst über picamera2."""

    _nat = None      # Handle in libxgocam.so, None = picamera2
    ZEITEN_S = 60.0  # Abstand der Zeitmessung im Log (libcamera direkt)
    _nlib = None

    def _sensor_size(self) -> tuple[int, int]:
        if not self.cfg.sensor_mode:
            return 0, 0
        w, h = (int(v) for v in self.cfg.sensor_mode.lower().split("x"))
        return w, h

    def _open(self) -> None:
        frame_us = int(1e6 / self.cfg.fps * self.cfg.frame_clock_corr)
        if self.cfg.capture_native and self._open_native(frame_us):
            return
        self._open_picam(frame_us)

    # --- direkt über libcamera (native/camfast.cpp) ---

    def _open_native(self, frame_us: int) -> bool:
        lib = _cam_lib()
        if lib is None:
            log.info("libxgocam.so fehlt – Kamera über picamera2 (bauen: make -C xgonav/native cam)")
            return False
        maps = lens_maps(self.cfg)
        if maps is not None:
            if not self.cfg.lens_native:
                log.info("lens_native = false – Kamera über picamera2")
                return False
            m1, m2 = np.ascontiguousarray(maps[0]), np.ascontiguousarray(maps[1])
            if not fastlib.check_abi(lib, "lens_abi", _LENS_ABI, fastlib.CAM_PATH) or not _lens_kern_ok(
                    self.cfg, _lens_kern(lib, m1, m2), lambda img: cv2.remap(img, m1, m2, cv2.INTER_LINEAR)):
                log.info("lens_remap in libxgocam.so weicht von cv2.remap ab – Kamera über picamera2")
                return False
        rw, rh = self._sensor_size()
        crop = (ctypes.c_int32 * 4)(*(int(v) for v in self.cfg.scaler_crop)) if self.cfg.scaler_crop else None
        err = ctypes.create_string_buffer(256)
        h = lib.cam_open(self.cfg.width, self.cfg.height, rw, rh, frame_us, crop, 4, err, len(err))
        if not h:
            log.warning("libcamera direkt: %s – Kamera über picamera2", err.value.decode("utf-8", "replace"))
            return False
        if maps is not None and lib.cam_set_lens(h, m1.ctypes.data, m2.ctypes.data, self.cfg.width,
                                                 self.cfg.height) != 0:
            lib.cam_close(h)
            log.warning("libcamera direkt: Entzerrtabelle passt nicht – Kamera über picamera2")
            return False
        self._nlib, self._nat = lib, h
        self._lens_id = lens_id(self.cfg) if maps is not None else None
        self._meta = _CamMeta()
        self._zeiten = (ctypes.c_double * 4)()
        self._zeiten_t = time.monotonic()
        self._apply_exposure(int(self.cfg.exposure_us))
        self._apply_gain(float(self.cfg.analogue_gain))
        self._apply_bild(*self.bild)
        if lib.cam_start(h) != 0:
            lib.cam_close(h)
            self._nlib = self._nat = None
            log.warning("libcamera direkt: Start scheitert – Kamera über picamera2")
            return False
        info = (ctypes.c_int32 * 3)()
        lib.cam_info(h, info)
        log.info("Kamera gestartet (libcamera direkt, %s) %dx%d@%d (Sensor %dx%d, Zeile %d B), Belichtung %s µs, "
                 "Verstärkung %s", "entzerrt im Kern" if maps is not None else "ohne Linsenmodell",
                 self.cfg.width, self.cfg.height, self.cfg.fps, info[1], info[2], info[0],
                 self.cfg.exposure_us or "auto/short", self.cfg.analogue_gain or "auto")
        return True

    def _grab_native(self) -> bool:
        out = np.empty((self.cfg.height, self.cfg.width, 3), np.uint8)   # je Bild neu: Frame.bgr wird nie überschrieben
        md = self._meta
        rc = self._nlib.cam_get(self._nat, 1000, out.ctypes.data, out.strides[0], ctypes.byref(md))
        if rc == 0:
            return True          # kein Bild in 1 s; _run fragt _stop/_paused ab und versucht es wieder
        if rc < 0:
            raise RuntimeError("cam_get scheitert")
        if time.monotonic() - self._zeiten_t >= self.ZEITEN_S:      # einmal je Minute, nicht je Bild
            self._zeiten_t = time.monotonic()
            n = self._nlib.cam_zeiten(self._nat, self._zeiten)
            z = self._zeiten
            log.info("Kamera je Bild (%d Bilder): Cache %.2f ms, Kopie %.2f ms, Entzerren %.2f ms, Metadaten %.2f ms, "
                     "verworfen %d", n, z[0] / 1e3, z[1] / 1e3, z[2] / 1e3, z[3] / 1e3, md.dropped)
        t = md.ts_ns / 1e9 if md.ts_ns else time.monotonic()
        exp = int(md.exp_us) if md.exp_us >= 0 else None
        self._publish(out, t, exp,
                      {"awb": [round(float(v), 3) for v in md.awb] if md.has_awb else None,
                       "gain": round(float(md.gain), 2),
                       "exp_us": exp, "frame_us": int(md.frame_us) if md.frame_us >= 0 else None,
                       "crop": tuple(int(v) for v in md.crop) if md.has_crop else None,
                       "contrast": self.cfg.contrast, "saturation": self.cfg.saturation, "brightness": self.cfg.brightness,
                       "entzerrt": self._lens_id or False})
        return True

    # --- über picamera2 ---

    def _open_picam(self, frame_us: int) -> None:
        from picamera2 import Picamera2  # nur auf dem Pi verfügbar

        self.cam = Picamera2()
        self._lens = lens_remap(self.cfg)
        self._lens_id = lens_id(self.cfg) if self._lens is not None else None
        controls = {"FrameDurationLimits": (frame_us, frame_us)}
        if self.cfg.scaler_crop:
            controls["ScalerCrop"] = tuple(int(v) for v in self.cfg.scaler_crop)
        kw = {}
        if self.cfg.sensor_mode:
            kw["raw"] = {"size": self._sensor_size()}
        conf = self.cam.create_video_configuration(
            main={"size": (self.cfg.width, self.cfg.height), "format": "RGB888"},  # = BGR im Speicher
            buffer_count=4, controls=controls, **kw,
        )
        self.cam.configure(conf)
        self._apply_exposure(int(self.cfg.exposure_us))
        self._apply_gain(float(self.cfg.analogue_gain))
        self._apply_bild(*self.bild)
        self.cam.start()
        log.info("PiCamera gestartet %dx%d@%d (Sensor %s), Belichtung %s µs, Verstärkung %s",
                 self.cfg.width, self.cfg.height, self.cfg.fps,
                 "x".join(map(str, self.cam.camera_configuration()["raw"]["size"])),
                 self.cfg.exposure_us or "auto/short", self.cfg.analogue_gain or "auto")

    def _apply_exposure(self, us: int) -> None:
        # Fixe Belichtungszeit, AnalogueGain bleibt automatisch. ExposureTime = 0 gibt die
        # Belichtung an die Automatik zurück (libcamera/rpi), dort im Modus "kurz".
        if self._nat is not None:
            self._nlib.cam_set_exposure(self._nat, int(us))
            return
        from libcamera import controls as lc
        self.cam.set_controls({"ExposureTime": max(0, int(us)),
                               "AeExposureMode": lc.AeExposureModeEnum.Short})

    def _apply_gain(self, gain: float) -> None:
        # AnalogueGain = 0 gibt die Verstärkung an die Automatik zurück (libcamera/rpi).
        if self._nat is not None:
            self._nlib.cam_set_gain(self._nat, float(gain))
            return
        self.cam.set_controls({"AnalogueGain": max(0.0, float(gain))})

    def _apply_bild(self, kontrast: float, saettigung: float, hell: float) -> None:
        if self._nat is not None:
            self._nlib.cam_set_bild(self._nat, float(kontrast), float(saettigung), float(hell))
            return
        self.cam.set_controls({"Contrast": float(kontrast), "Saturation": float(saettigung), "Brightness": float(hell)})

    def _grab(self) -> bool:
        if self._nat is not None:
            return self._grab_native()
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
                       "crop": md.get("ScalerCrop"), "contrast": self.cfg.contrast, "saturation": self.cfg.saturation, "brightness": self.cfg.brightness,
                       "entzerrt": self._lens_id or False})
        return True

    def _suspend(self) -> None:
        if self._nat is not None:
            self._nlib.cam_stop(self._nat)
        else:
            self.cam.stop()
        log.info("PiCamera angehalten")

    def _resume_device(self) -> None:
        if self._nat is not None:
            if self._nlib.cam_start(self._nat) != 0:
                raise RuntimeError("cam_start scheitert")
        else:
            self.cam.start()
        log.info("PiCamera läuft wieder")

    def _close(self) -> None:
        if self._nat is not None:
            if self._thread is not None and self._thread.is_alive():
                log.warning("Kamera-Faden hängt noch in cam_get – Kamera bleibt bis Prozessende offen")
                return
            h, self._nat = self._nat, None
            self._nlib.cam_close(h)
            return
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
