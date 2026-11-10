"""xgonav – kamerabasierte Navigation für den XGO Rider.

Beispiele:
  python3 -m xgonav --mode follow                     # echter Rider, Person folgen
  python3 -m xgonav --mode explore
  python3 -m xgonav --mode approach --target chair
  python3 -m xgonav --mode home                        # zum Startpunkt der Raumkarte
  python3 -m xgonav --source test.mp4 --robot sim --mode follow   # Replay am PC
"""
from __future__ import annotations

import argparse
import logging
import signal
import sys
import threading
import time

from .affinity import pin_thread
from .app import App, render
from .config import Config
from .rawrec import RawRecorder
from .robot import make_robot


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="xgonav", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", help="TOML-Datei mit Overrides")
    ap.add_argument("--mode", choices=["idle", "follow", "explore", "approach", "home", "pet"])   # patrol nur über die Schnittstelle: braucht Bestätigung
    ap.add_argument("--target", help="COCO-Klasse für follow/approach (z. B. person, chair, dog)")
    ap.add_argument("--source", help="picam | Kameraindex | Videodatei")
    ap.add_argument("--robot", choices=["proc", "sim"], default=None,
                    help="proc = /proc/XGORider, sim = nur protokollieren (Default: proc bei picam, sonst sim)")
    ap.add_argument("--model", help="YOLO-ONNX-Datei")
    ap.add_argument("--no-detector", action="store_true")
    ap.add_argument("--web-port", type=int)
    ap.add_argument("--no-lcd", action="store_true")
    ap.add_argument("--record", help="Debug-Overlay als Video speichern (mp4)")
    ap.add_argument("--record-raw", metavar="DIR",
                    help="Rohaufzeichnung gleich einschalten (statt Knopf in der Oberfläche): JPEG + meta.jsonl, "
                         "nur während der Rider balanciert, je Aufnahme ein neues Unterverzeichnis von DIR")
    ap.add_argument("--duration", type=float, help="nach N Sekunden beenden")
    ap.add_argument("--fast", action="store_true", help="Videodatei so schnell wie möglich (nicht Echtzeit)")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args(argv)

    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s", datefmt="%H:%M:%S")
    cfg = Config.load(a.config)
    if a.mode:
        cfg.behavior.mode = a.mode
    if a.target:
        cfg.behavior.target_class = a.target
    if a.source:
        cfg.camera.source = a.source
    if a.model:
        cfg.detector.model = a.model
    if a.no_detector:
        cfg.detector.enabled = False
    if a.web_port is not None:
        cfg.output.web_port = a.web_port
    if a.no_lcd:
        cfg.output.lcd_fifo = ""
    if a.fast:
        cfg.camera.realtime_file = False
    kind = a.robot or ("proc" if cfg.camera.source == "picam" else "sim")
    # Pinning vor allen Threads: Kamera, libcamera, OpenCV-Pool, IMU und Web erben die Kerne.
    # Den Detektor legt DetectorWorker selbst auf [cpu] detector_cpus.
    main_cpus = pin_thread(cfg.cpu.main_cpus, "Hauptschleife")
    if main_cpus:
        import cv2
        cv2.setNumThreads(len(main_cpus))   # sonst 4 OpenCV-Worker auf 2 Kernen
        logging.getLogger("xgonav").info("OpenCV-Threads: %d", cv2.getNumThreads())

    robot = make_robot(kind, cfg.robot)
    app = App(cfg, robot)

    def on_sig(*_):
        app.stop()
    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    if cfg.output.web_port:
        from .web import serve
        serve(app, cfg.output.web_host, cfg.output.web_port, cfg.output.web_fps, cfg.cpu.render_cpus)
    if cfg.output.lcd_fifo and kind == "proc":
        from .lcd import LcdOut
        LcdOut(app, cfg.output.lcd_fifo, cfg.output.lcd_fps)
    if cfg.emotion.enabled and kind == "proc":
        from .emotion import Emotions
        app.emotions = Emotions(cfg.emotion, cfg.emotion.fifo)
    recs = []
    if a.record:
        recs.append(threading.Thread(target=_recorder, args=(app, a.record, cfg.camera.fps), daemon=True))
    for r in recs:
        r.start()
    app.rawrec = RawRecorder(app, a.record_raw or cfg.output.record_dir, cfg.output.record_min_free_mb,
                             cfg.cpu.render_cpus)
    if a.record_raw:
        app.rawrec.schalten(True)

    app.run(duration=a.duration)
    with app.snap_cond:
        app.snap_cond.notify_all()
    app.rawrec.beenden()
    for r in recs:
        r.join(timeout=5)
    return 0


def _recorder(app: App, path: str, fps: int) -> None:
    import cv2
    vw = None
    last = -1
    while not app.running:
        time.sleep(0.05)
    while app.running:
        with app.snap_cond:
            app.snap_cond.wait_for(lambda: app.snapshot.idx != last or not app.running, timeout=1.0)
            snap = app.snapshot
        if snap.idx == last:
            continue
        last = snap.idx
        img = render(snap)
        if img is None:
            continue
        if vw is None:
            vw = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (img.shape[1], img.shape[0]))
        vw.write(img)
    if vw is not None:
        vw.release()


if __name__ == "__main__":
    sys.exit(main())
