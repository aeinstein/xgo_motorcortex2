"""YOLO (v8/v11 oder v5) asynchron im eigenen Thread, Backend NCNN oder onnxruntime.

Der Detektor ist langsamer als die Kamera. Deshalb:
* er bekommt bevorzugt *scharfe* Frames (Laplace-Varianz >= rel * Median),
  denn bei einem selbstbalancierenden Roboter ist ein Teil der Frames immer
  verwischt;
* Ergebnisse tragen die Frame-Nummer; die App verschiebt sie mit der
  Ego-Motion bis zum aktuellen Frame (Latenzkompensation).
"""
from __future__ import annotations

import logging
import os
import threading
import time
from dataclasses import dataclass, replace

import cv2
import numpy as np

from .affinity import ort_pool_affinities, pin_thread
from .config import DetectorCfg

log = logging.getLogger(__name__)

COCO = ["person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
        "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog",
        "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
        "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
        "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
        "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
        "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
        "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
        "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
        "teddy bear", "hair drier", "toothbrush"]


@dataclass
class Detection:
    box: tuple[float, float, float, float]  # x1,y1,x2,y2 in Vollbild-Pixeln
    cls: str
    conf: float


@dataclass
class DetectionBatch:
    frame_idx: int
    t: float
    dets: list[Detection]
    infer_ms: float


def is_ncnn_model(path: str) -> bool:
    """NCNN-Modell = Ultralytics-Exportordner (model.ncnn.param/.bin) oder direkt die .param."""
    return path.endswith(".param") or os.path.isfile(os.path.join(path, "model.ncnn.param"))


def model_exists(path: str) -> bool:
    if not path:
        return False
    if path.endswith(".param"):
        return os.path.isfile(path) and os.path.isfile(path[:-6] + ".bin")
    if os.path.isdir(path):
        return is_ncnn_model(path) and os.path.isfile(os.path.join(path, "model.ncnn.bin"))
    return os.path.isfile(path)


class _YoloBase:
    """Letterbox-Geometrie und Nachbearbeitung (v8/v11 bzw. v5), unabhängig vom Backend."""
    cfg: DetectorCfg
    names: list[str]
    allowed: set[int] | None
    size: int

    def _set_names(self, names: list[str] | None) -> None:
        self.names = names or COCO
        self.allowed = {self.names.index(c) for c in self.cfg.classes if c in self.names} or None

    def _letterbox(self, w0: int, h0: int):
        s = self.size
        r = min(s / w0, s / h0)
        nw, nh = int(round(w0 * r)), int(round(h0 * r))
        return r, nw, nh, (s - nw) // 2, (s - nh) // 2

    def _post(self, out: np.ndarray, r, px, py, w0, h0) -> list[Detection]:
        out = np.squeeze(out, 0)
        nc = len(self.names)
        thr = self.cfg.conf
        if out.shape[0] == 4 + nc:            # v8/v11: [4+nc, N]
            sc = out[4:]
            # Kandidaten zuerst nur über die erlaubten Klassen suchen (8 statt 80 Zeilen);
            # danach wie bisher: die beste Klasse über ALLE muss erlaubt sein und >= conf.
            if self.allowed is not None:
                idx = np.fromiter(sorted(self.allowed), np.intp)
                cand = np.flatnonzero(sc[idx].max(0) >= thr)
            else:
                cand = np.flatnonzero(sc.max(0) >= thr)
            if not cand.size:
                return []
            full = sc[:, cand]
            cls = full.argmax(0)
            conf = full[cls, np.arange(len(cand))]
            boxes = out[:4, cand].T
            if self.allowed is not None:
                m = np.isin(cls, idx)
                boxes, cls, conf = boxes[m], cls[m], conf[m]
        elif out.shape[1] == 5 + nc:          # v5: [N, 5+nc]
            boxes, scores = out[:, :4], out[:, 5:] * out[:, 4:5]
            cls = scores.argmax(1)
            conf = scores[np.arange(len(cls)), cls]
            m = conf >= thr
            if self.allowed is not None:
                m &= np.isin(cls, list(self.allowed))
            boxes, cls, conf = boxes[m], cls[m], conf[m]
        else:
            raise ValueError(f"Unbekanntes YOLO-Ausgabeformat {out.shape}")
        if not len(conf):
            return []
        xy = np.empty_like(boxes)
        xy[:, 0] = (boxes[:, 0] - boxes[:, 2] / 2 - px) / r
        xy[:, 1] = (boxes[:, 1] - boxes[:, 3] / 2 - py) / r
        xy[:, 2] = boxes[:, 2] / r
        xy[:, 3] = boxes[:, 3] / r
        # klassenweise NMS per Offset (läuft auch mit OpenCV 4.6 aus Bookworm)
        off = xy.copy()
        off[:, :2] += cls[:, None] * 4096.0
        keep = cv2.dnn.NMSBoxes(off.tolist(), conf.tolist(), self.cfg.conf, self.cfg.iou)
        res = []
        for i in np.array(keep).reshape(-1):
            x, y, w, h = xy[i]
            res.append(Detection((max(0.0, x), max(0.0, y), min(w0, x + w), min(h0, y + h)),
                                 self.names[int(cls[i])], float(conf[i])))
        return res


def _read_ultralytics_meta(path: str) -> tuple[list[str] | None, int | None]:
    """Klassennamen und imgsz aus metadata.yaml eines Ultralytics-Exports (ohne yaml-Modul)."""
    names: dict[int, str] = {}
    size = None
    try:
        with open(path, encoding="utf-8") as f:
            lines = f.read().splitlines()
    except OSError:
        return None, None
    in_names = False
    for i, ln in enumerate(lines):
        if ln.startswith("names:"):
            in_names = True
            continue
        if in_names:
            if ln.startswith("  ") and ":" in ln:
                k, v = ln.strip().split(":", 1)
                if k.strip().isdigit():
                    names[int(k)] = v.strip().strip("'\"")
                    continue
            in_names = False
        if ln.startswith("imgsz:"):
            nxt = ln.split(":", 1)[1].strip()
            if nxt.startswith("["):
                size = int(nxt.strip("[]").split(",")[0])
            elif i + 1 < len(lines) and lines[i + 1].lstrip().startswith("- "):
                size = int(lines[i + 1].lstrip()[2:])
    return ([names[i] for i in sorted(names)] if names else None), size


class YoloNcnn(_YoloBase):
    """NCNN-Backend (Ultralytics-Export format=ncnn). Am CM4 gemessen (2026-11-05, yolo11n 256,
    2 Threads): Inferenz 93 statt 112 ms (onnxruntime), Vorverarbeitung 2,7 statt 3,8 ms,
    identische Boxen. Die OpenMP-Threads entstehen beim ersten Lauf im Detektor-Thread und
    erben dessen Kerne – DetectorWorker pinnt ihn deshalb auf alle detector_cpus."""

    def __init__(self, cfg: DetectorCfg):
        import ncnn

        self.ncnn = ncnn
        self.cfg = cfg
        path = cfg.model
        if path.endswith(".param"):
            param, binf, meta = path, path[:-6] + ".bin", os.path.join(os.path.dirname(path), "metadata.yaml")
        else:
            param, binf = os.path.join(path, "model.ncnn.param"), os.path.join(path, "model.ncnn.bin")
            meta = os.path.join(path, "metadata.yaml")
        names, size = _read_ultralytics_meta(meta)
        self.size = size or cfg.input_size
        self._set_names(names)
        net = ncnn.Net()
        net.opt.num_threads = max(1, int(cfg.threads))
        net.opt.use_vulkan_compute = False
        net.opt.lightmode = True
        if net.load_param(param) != 0 or net.load_model(binf) != 0:
            raise RuntimeError(f"NCNN-Modell {path} nicht ladbar")
        self.net = net
        try:
            self.in_name, self.out_name = net.input_names()[0], net.output_names()[0]
        except Exception:
            self.in_name, self.out_name = "in0", "out0"   # Ultralytics/pnnx-Namen
        self._pix = ncnn.Mat.PixelType.PIXEL_BGR2RGB
        self._border = ncnn.BorderType.BORDER_CONSTANT
        self._norm = [1 / 255.0] * 3

    def __call__(self, bgr: np.ndarray) -> list[Detection]:
        ncnn = self.ncnn
        h0, w0 = bgr.shape[:2]
        r, nw, nh, px, py = self._letterbox(w0, h0)
        s = self.size
        # Größe ändern, BGR->RGB und Rand in C++ (gleiches Ergebnis wie cv2.resize + blobFromImage)
        m = ncnn.Mat.from_pixels_resize(np.ascontiguousarray(bgr), self._pix, w0, h0, nw, nh)
        m = ncnn.copy_make_border(m, py, s - nh - py, px, s - nw - px, self._border, 114.0)
        m.substract_mean_normalize([], self._norm)
        ex = self.net.create_extractor()
        ex.input(self.in_name, m)
        ret, out = ex.extract(self.out_name)
        if ret != 0:
            raise RuntimeError(f"NCNN extract fehlgeschlagen ({ret})")
        return self._post(np.asarray(out)[None], r, px, py, w0, h0)


class YoloOnnx(_YoloBase):
    def __init__(self, cfg: DetectorCfg, cpus=None):
        import onnxruntime as ort

        so = ort.SessionOptions()
        so.intra_op_num_threads = cfg.threads
        so.inter_op_num_threads = 1
        # Der Pool entsteht hier (im Hauptthread) und würde dessen Kerne erben: Pool-Threads
        # deshalb ausdrücklich auf die Detektor-Kerne legen. Den Aufrufer pinnt _run.
        aff = ort_pool_affinities(cpus, cfg.threads)
        if aff:
            so.add_session_config_entry("session.intra_op_thread_affinities", aff)
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.sess = ort.InferenceSession(cfg.model, so, providers=["CPUExecutionProvider"])
        self.inp = self.sess.get_inputs()[0]
        shape = self.inp.shape
        self.size = int(shape[2]) if isinstance(shape[2], int) else cfg.input_size
        self.cfg = cfg
        names = None
        meta = self.sess.get_modelmeta().custom_metadata_map
        if "names" in meta:  # Ultralytics schreibt die Klassennamen in die Metadaten
            try:
                import ast
                d = ast.literal_eval(meta["names"])
                names = [d[i] for i in sorted(d)]
            except Exception:
                pass
        self._set_names(names)

    def __call__(self, bgr: np.ndarray) -> list[Detection]:
        h0, w0 = bgr.shape[:2]
        r, nw, nh, px, py = self._letterbox(w0, h0)
        s = self.size
        img = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
        canvas = cv2.copyMakeBorder(img, py, s - nh - py, px, s - nw - px, cv2.BORDER_CONSTANT,
                                    value=(114, 114, 114))
        blob = cv2.dnn.blobFromImage(canvas, 1 / 255.0, swapRB=True)
        out = self.sess.run(None, {self.inp.name: blob})[0]
        return self._post(out, r, px, py, w0, h0)


class DetectorWorker:
    def __init__(self, cfg: DetectorCfg, cpus=None):
        self.cfg = cfg
        self.cpus = list(cpus or [])   # [cpu] detector_cpus: Aufrufer auf cpus[0], Pool auf den Rest
        self.model: _YoloBase | None = None
        self.backend = ""
        self._cond = threading.Condition()
        self._cand = None          # (frame, sharpness)
        self._cand_since = 0.0
        self._result: DetectionBatch | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.busy = False
        self.infer_ms = 0.0

    def start(self) -> bool:
        if not self.cfg.enabled:
            return False
        # Erst das Hauptmodell (NCNN oder ONNX), bei Fehlen/Fehler das Rückfallmodell.
        for path in dict.fromkeys(p for p in (self.cfg.model, self.cfg.fallback_model) if p):
            if not model_exists(path):
                log.warning("Modell %s fehlt (siehe tools/export_model.sh)", path)
                continue
            cfg = replace(self.cfg, model=path)
            try:
                if is_ncnn_model(path):
                    self.model, self.backend = YoloNcnn(cfg), "ncnn"
                else:
                    self.model, self.backend = YoloOnnx(cfg, self.cpus), "onnx"
                break
            except Exception as e:
                log.warning("Modell %s nicht nutzbar: %s", path, e)
        if self.model is None:
            log.warning("kein Detektor-Modell nutzbar – Objekterkennung aus")
            return False
        log.info("YOLO geladen: %s [%s] (%dpx, %d Klassen)", self.model.cfg.model, self.backend,
                 self.model.size, len(self.model.names))
        self._thread = threading.Thread(target=self._run, name="detector", daemon=True)
        self._thread.start()
        return True

    def close(self) -> None:
        self._stop.set()
        with self._cond:
            self._cond.notify_all()
        if self._thread:
            self._thread.join(timeout=2)

    def offer(self, frame, sharpness: float, median: float) -> None:
        """Pro Frame aufrufen. Behält den besten Kandidaten seit dem letzten Lauf."""
        if self.model is None:
            return
        with self._cond:
            now = time.monotonic()
            if self._cand is None:
                self._cand_since = now
            if self._cand is None or sharpness >= self._cand[1] or sharpness >= self.cfg.sharpness_rel * median:
                self._cand = (frame, sharpness, median)
            self._cond.notify()

    def poll(self) -> DetectionBatch | None:
        with self._cond:
            r, self._result = self._result, None
            return r

    def _ready(self) -> bool:
        if self._cand is None:
            return False
        _, sh, med = self._cand
        waited = time.monotonic() - self._cand_since
        return sh >= self.cfg.sharpness_rel * med or waited >= self.cfg.max_wait_s

    def _run(self) -> None:
        if self.cpus:
            # onnxruntime: Aufrufer auf cpus[0], Pool-Threads pinnt ORT selbst auf den Rest.
            # NCNN: OpenMP-Threads entstehen in diesem Thread und erben seine Kerne -> alle.
            pin_thread(self.cpus if self.backend == "ncnn" else self.cpus[:1], "Detektor")
        t_start = 0.0
        while not self._stop.is_set():
            # Höchstrate: frühestens 1/max_hz nach dem Start des letzten Laufs
            if self.cfg.max_hz > 0:
                rest = t_start + 1.0 / self.cfg.max_hz - time.monotonic()
                if rest > 0 and self._stop.wait(rest):
                    return
            with self._cond:
                while not self._stop.is_set() and not self._ready():
                    # submit/close wecken über notify; kurz warten muss er nur auf max_wait_s eines Kandidaten
                    self._cond.wait(0.02 if self._cand is not None else 1.0)
                if self._stop.is_set():
                    return
                frame, _, _ = self._cand
                self._cand = None
            self.busy = True
            t_start = time.monotonic()
            t0 = time.perf_counter()
            try:
                dets = self.model(frame.bgr)
            except Exception:
                log.exception("Inferenz fehlgeschlagen")
                dets = []
            ms = (time.perf_counter() - t0) * 1000
            self.infer_ms = 0.8 * self.infer_ms + 0.2 * ms if self.infer_ms else ms
            self.busy = False
            with self._cond:
                self._result = DetectionBatch(frame.idx, frame.t, dets, ms)
