#!/bin/bash
# Modelle der Gesichtserkennung ([gesicht], xgonav/gesicht.py) holen und für den CM4 umwandeln. Am PC ausführen,
# NICHT auf dem Rider.
#
#   ./tools/export_face.sh
#
# Ergebnis (per deploy/install.sh an den Rider):
#   models/face_detection_yunet_2023mar.onnx   YuNet (opencv_zoo, MIT), läuft über cv2.FaceDetectorYN
#   models/mobilefacenet_ncnn/model.ncnn.*     MobileFaceNet w600k_mbf aus InsightFace buffalo_s, per pnnx nach ncnn
#                                              (Gewichte nur für nicht-kommerzielle Nutzung)
# Zum Schluss rechnet das Skript ein Testbild mit onnxruntime und ncnn und vergleicht die Merkmale.
set -euo pipefail
VENV=${VENV:-/tmp/face-export}
py_in_venv() { for c in "$VENV/bin/python" "$VENV/Scripts/python.exe"; do [ -x "$c" ] && { echo "$c"; return 0; }; done; return 0; }
PY=$(py_in_venv)
if [ -z "$PY" ]; then
    "${PYTHON:-python3}" -m venv "$VENV"     # Windows/Git-Bash: PYTHON=python
    PY=$(py_in_venv)
fi
"$PY" -m pip install -q pnnx ncnn onnx onnxruntime "numpy<3" opencv-python-headless
mkdir -p models
ARBEIT=$(mktemp -d)
trap 'rm -rf "$ARBEIT"' EXIT

YUNET=models/face_detection_yunet_2023mar.onnx
if [ ! -f "$YUNET" ]; then
    curl -fL -o "$YUNET" \
        https://media.githubusercontent.com/media/opencv/opencv_zoo/main/models/face_detection_yunet/face_detection_yunet_2023mar.onnx
fi

curl -fL -o "$ARBEIT/buffalo_s.zip" https://github.com/deepinsight/insightface/releases/download/v0.7/buffalo_s.zip
"$PY" -c "import zipfile,sys; z=zipfile.ZipFile(sys.argv[1]); n=[x for x in z.namelist() if x.endswith('w600k_mbf.onnx')][0]; open(sys.argv[2],'wb').write(z.read(n))" \
    "$ARBEIT/buffalo_s.zip" "$ARBEIT/w600k_mbf.onnx"
(cd "$ARBEIT" && "$PY" -m pnnx w600k_mbf.onnx "inputshape=[1,3,112,112]" >/dev/null 2>&1 \
    || { PNNX=$(dirname "$PY")/pnnx; "$PNNX" w600k_mbf.onnx "inputshape=[1,3,112,112]"; })
rm -rf models/mobilefacenet_ncnn && mkdir -p models/mobilefacenet_ncnn
mv "$ARBEIT/w600k_mbf.ncnn.param" models/mobilefacenet_ncnn/model.ncnn.param
mv "$ARBEIT/w600k_mbf.ncnn.bin" models/mobilefacenet_ncnn/model.ncnn.bin

# Abgleich onnxruntime gegen ncnn (gleiche Vorverarbeitung wie gesicht.Gesichter._merkmal)
"$PY" - "$ARBEIT/w600k_mbf.onnx" models/mobilefacenet_ncnn <<'EOF'
import sys
import ncnn
import numpy as np
import onnxruntime as ort

onnx, d = sys.argv[1], sys.argv[2]
rng = np.random.default_rng(1)
bgr = rng.integers(0, 256, (112, 112, 3), dtype=np.uint8)
x = ((bgr[:, :, ::-1].astype(np.float32) - 127.5) / 127.5).transpose(2, 0, 1)[None]
s = ort.InferenceSession(onnx, providers=["CPUExecutionProvider"])
a = s.run(None, {s.get_inputs()[0].name: x})[0].ravel()
net = ncnn.Net()
net.load_param(d + "/model.ncnn.param")
net.load_model(d + "/model.ncnn.bin")
m = ncnn.Mat.from_pixels(np.ascontiguousarray(bgr), ncnn.Mat.PixelType.PIXEL_BGR2RGB, 112, 112)
m.substract_mean_normalize([127.5] * 3, [1 / 127.5] * 3)
ex = net.create_extractor()
ex.input(net.input_names()[0], m)
_, out = ex.extract(net.output_names()[0])
b = np.asarray(out).ravel()
cos = float(a @ b / np.linalg.norm(a) / np.linalg.norm(b))
print(f"Merkmale: {a.size}, Kosinus onnxruntime/ncnn {cos:.6f}")
sys.exit(0 if a.size == 512 and cos > 0.999 else 1)
EOF
echo "fertig: $YUNET und models/mobilefacenet_ncnn/"
