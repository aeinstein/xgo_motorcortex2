#!/bin/bash
# YOLO-Modell als ONNX für den CM4 exportieren. Auf einem PC/GPU-Server ausführen,
# NICHT auf dem Rider (torch/ultralytics sind für 2 GB RAM zu schwer).
#
#   ./tools/export_model.sh                 # yolo11n, 256 px (Default, CM4-tauglich)
#   ./tools/export_model.sh yolo11n 320     # genauer bei kleinen Objekten, ~2x langsamer
#
# Ergebnis: models/<name>_<size>.onnx -> per scp nach ~/xgonav/models/ auf den Rider.
set -euo pipefail
MODEL=${1:-yolo11n}
SIZE=${2:-256}
python3 -m venv /tmp/yolo-export
. /tmp/yolo-export/bin/activate
pip install -q "ultralytics>=8.3" onnx onnxslim
mkdir -p models
cd models
# opset 12 + simplify: läuft mit onnxruntime >= 1.14 auf aarch64
yolo export model="${MODEL}.pt" format=onnx imgsz="${SIZE}" opset=12 simplify=True dynamic=False
mv "${MODEL}.onnx" "${MODEL}_${SIZE}.onnx"
echo "fertig: models/${MODEL}_${SIZE}.onnx"
