"""YOLO (v8/v11 oder v5, ONNX-Export) asynchron im eigenen Thread.

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
from dataclasses import dataclass

import cv2
import numpy as np

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


class YoloOnnx:
    def __init__(self, cfg: DetectorCfg):
        import onnxruntime as ort

        so = ort.SessionOptions()
        so.intra_op_num_threads = cfg.threads
        so.inter_op_num_threads = 1
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.sess = ort.InferenceSession(cfg.model, so, providers=["CPUExecutionProvider"])
        self.inp = self.sess.get_inputs()[0]
        shape = self.inp.shape
        self.size = int(shape[2]) if isinstance(shape[2], int) else cfg.input_size
        self.cfg = cfg
        self.names = COCO
        meta = self.sess.get_modelmeta().custom_metadata_map
        if "names" in meta:  # Ultralytics schreibt die Klassennamen in die Metadaten
            try:
                import ast
                d = ast.literal_eval(meta["names"])
                self.names = [d[i] for i in sorted(d)]
            except Exception:
                pass
        self.allowed = {self.names.index(c) for c in cfg.classes if c in self.names} or None

    def __call__(self, bgr: np.ndarray) -> list[Detection]:
        h0, w0 = bgr.shape[:2]
        s = self.size
        r = min(s / w0, s / h0)
        nw, nh = int(round(w0 * r)), int(round(h0 * r))
        img = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
        px, py = (s - nw) // 2, (s - nh) // 2
        canvas = np.full((s, s, 3), 114, np.uint8)
        canvas[py:py + nh, px:px + nw] = img
        blob = cv2.dnn.blobFromImage(canvas, 1 / 255.0, swapRB=True)
        out = self.sess.run(None, {self.inp.name: blob})[0]
        return self._post(out, r, px, py, w0, h0)

    def _post(self, out: np.ndarray, r, px, py, w0, h0) -> list[Detection]:
        out = np.squeeze(out, 0)
        nc = len(self.names)
        if out.shape[0] == 4 + nc:            # v8/v11: [4+nc, N]
            out = out.T
            boxes, scores = out[:, :4], out[:, 4:]
        elif out.shape[1] == 5 + nc:          # v5: [N, 5+nc]
            boxes, scores = out[:, :4], out[:, 5:] * out[:, 4:5]
        else:
            raise ValueError(f"Unbekanntes YOLO-Ausgabeformat {out.shape}")
        cls = scores.argmax(1)
        conf = scores[np.arange(len(cls)), cls]
        m = conf >= self.cfg.conf
        if self.allowed is not None:
            m &= np.isin(cls, list(self.allowed))
        boxes, cls, conf = boxes[m], cls[m], conf[m]
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


class DetectorWorker:
    def __init__(self, cfg: DetectorCfg):
        self.cfg = cfg
        self.model: YoloOnnx | None = None
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
        if not os.path.exists(self.cfg.model):
            log.warning("Modell %s fehlt – Objekterkennung aus (siehe tools/export_model.sh)", self.cfg.model)
            return False
        self.model = YoloOnnx(self.cfg)
        log.info("YOLO geladen: %s (%dpx, %d Klassen)", self.cfg.model, self.model.size, len(self.model.names))
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
        while not self._stop.is_set():
            with self._cond:
                while not self._stop.is_set() and not self._ready():
                    self._cond.wait(0.02)
                if self._stop.is_set():
                    return
                frame, _, _ = self._cand
                self._cand = None
            self.busy = True
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
