#!/bin/bash
# Installation auf dem Rider (Raspberry Pi OS Bookworm 64-bit, CM4).
# Aufruf im entpackten xgonav-Verzeichnis:  sudo ./deploy/install.sh
set -euo pipefail
DEST=/opt/xgonav
USER_NAME=${SUDO_USER:-pi}

apt-get install -y python3-opencv python3-numpy python3-picamera2 python3-venv

# venv mit Systempaketen (picamera2/libcamera kommen nur aus apt)
python3 -m venv --system-site-packages "$DEST/venv"
"$DEST/venv/bin/pip" install -q "onnxruntime>=1.16"

mkdir -p "$DEST" "$DEST/models"
cp -r xgonav tools web "$DEST/"
[ -f "$DEST/config.toml" ] || cp deploy/config.example.toml "$DEST/config.toml"
cp -n models/*.onnx "$DEST/models/" 2>/dev/null || echo "Hinweis: kein Modell in models/ – siehe tools/export_model.sh"
chown -R "$USER_NAME":"$USER_NAME" "$DEST"
usermod -aG video "$USER_NAME"

sed "s/__USER__/$USER_NAME/" deploy/xgonav.service > /etc/systemd/system/xgonav.service
systemctl daemon-reload
echo
echo "Installiert nach $DEST."
echo "Test:      $DEST/venv/bin/python -m xgonav --config $DEST/config.toml --mode idle   (im Verzeichnis $DEST)"
echo "Dienst:    systemctl enable --now xgonav"
echo "Debug-Web: http://riderpi:8088/"
