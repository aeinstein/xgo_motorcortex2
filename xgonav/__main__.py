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
from .robot import make_robot


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="xgonav", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", help="TOML-Datei mit Overrides")
    ap.add_argument("--mode", choices=["idle", "follow", "explore", "approach", "home"])
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
                    help="Rohbilder (JPEG) + Metadaten (meta.jsonl) für die Auswertung am PC speichern")
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
    if a.record_raw:
        recs.append(threading.Thread(target=_raw_recorder, args=(app, a.record_raw), daemon=True))
    for r in recs:
        r.start()

    app.run(duration=a.duration)
    with app.snap_cond:
        app.snap_cond.notify_all()
    for r in recs:
        r.join(timeout=5)
    return 0


def _raw_recorder(app: App, outdir: str) -> None:
    """Rohbilder + Metadaten je Frame. Läuft hinter der Hauptschleife her und nimmt
    immer den neuesten Snapshot; kommt es nicht nach, fehlen Frames (idx zeigt Lücken)."""
    import json
    import os
    import cv2
    os.makedirs(outdir, exist_ok=True)
    meta = open(os.path.join(outdir, "meta.jsonl"), "a", buffering=1)
    last = -1
    n = 0
    while not app.running:
        time.sleep(0.05)
    try:
        while app.running:
            with app.snap_cond:
                app.snap_cond.wait_for(lambda: app.snapshot.idx != last or not app.running, timeout=1.0)
                snap = app.snapshot
            if snap.idx == last or snap.frame is None:
                continue
            last = snap.idx
            name = "%06d.jpg" % snap.idx
            cv2.imwrite(os.path.join(outdir, name), snap.frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
            st = snap.stats
            meta.write(json.dumps({
                "idx": snap.idx, "t": round(snap.t, 4), "file": name,
                "pitch_dev": round(snap.pitch_dev, 6), "pitch_src": st.get("pitch_src"),
                "sent": [round(snap.sent[0], 4), round(snap.sent[1], 4)],
                "status": snap.cmd.status, "target": snap.cmd.target_id,
                "yaw_vis_deg": st.get("yaw_vis_deg"), "balancing": st.get("balancing"),
                # Kurs/Pose für die Auswertung von explore/home: Räder, IMU (Firmware), Raumkarte
                "mode": st.get("mode"), "yaw_deg": st.get("yaw_deg"), "yaw_src": st.get("yaw_src"),
                "yaw_drv": st.get("yaw_drv"), "odom": st.get("odom"),
                "pose": (st.get("grid") or {}).get("pose"), "start": (st.get("grid") or {}).get("start"),
                "tracks": [{"id": t.id, "cls": t.cls, "conf": round(t.conf, 3),
                            "box": [round(v, 1) for v in t.box]} for t in snap.tracks],
                "tracks_all": [{"id": t.id, "cls": t.cls, "hits": t.hits, "seen": round(snap.t - t.t_seen, 2),
                                "box": [round(v, 1) for v in t.box]} for t in snap.tracks_all],
                "fs": snap.fs_diag, "cam": snap.cam_meta,
            }) + "\n")
            n += 1
    finally:
        meta.close()
        logging.getLogger(__name__).info("Rohaufzeichnung: %d Frames in %s", n, outdir)


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
