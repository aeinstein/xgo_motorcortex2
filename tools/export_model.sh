#!/bin/bash
# YOLO-Modell für den CM4 exportieren (NCNN + ONNX als Rückfall). Auf einem PC/GPU-Server
# ausführen, NICHT auf dem Rider (torch/ultralytics sind für 2 GB RAM zu schwer).
#
#   ./tools/export_model.sh                 # yolo11n, 256 px (Default, CM4-tauglich)
#   ./tools/export_model.sh yolo11n 320     # genauer bei kleinen Objekten, ~2x langsamer
#
# Ergebnis: models/<name>_<size>_ncnn/ (model.ncnn.param/.bin, metadata.yaml) und
# models/<name>_<size>.onnx -> per scp nach /opt/xgonav/models/ auf den Rider.
# CM4 (2026-11-05, yolo11n 256, 2 Threads): NCNN 96 ms, onnxruntime 117 ms je Bild, gleiche Boxen.
set -euo pipefail
MODEL=${1:-yolo11n}
SIZE=${2:-256}
VENV=${VENV:-/tmp/yolo-export}      # vorhandenes venv mit ultralytics wird wiederverwendet
py_in_venv() { for c in "$VENV/bin/python" "$VENV/Scripts/python.exe"; do [ -x "$c" ] && { echo "$c"; return 0; }; done; return 0; }
PY=$(py_in_venv)
if [ -z "$PY" ]; then
    "${PYTHON:-python3}" -m venv "$VENV"     # Windows/Git-Bash: PYTHON=python
    PY=$(py_in_venv)
fi
"$PY" -m pip install -q "ultralytics>=8.3" onnx onnxslim pnnx ncnn
mkdir -p models
cd models
# opset 12 + simplify: läuft mit onnxruntime >= 1.14 auf aarch64
"$PY" -c "from ultralytics import YOLO; YOLO('${MODEL}.pt').export(format='onnx', imgsz=${SIZE}, opset=12, simplify=True, dynamic=False)"
mv "${MODEL}.onnx" "${MODEL}_${SIZE}.onnx"
"$PY" -c "from ultralytics import YOLO; YOLO('${MODEL}.pt').export(format='ncnn', imgsz=${SIZE})"
rm -rf "${MODEL}_${SIZE}_ncnn"
mv "${MODEL}_ncnn_model" "${MODEL}_${SIZE}_ncnn"
rm -rf "${MODEL}_${SIZE}_ncnn/__pycache__" "${MODEL}_${SIZE}_ncnn/model_ncnn.py"
echo "fertig: models/${MODEL}_${SIZE}_ncnn/ und models/${MODEL}_${SIZE}.onnx"
