#!/bin/bash
# Installation auf dem Rider (Raspberry Pi OS Bookworm 64-bit, CM4).
# Aufruf im entpackten xgonav-Verzeichnis:  sudo ./deploy/install.sh
set -euo pipefail
DEST=/opt/xgonav
USER_NAME=${SUDO_USER:-pi}

apt-get install -y python3-opencv python3-numpy python3-picamera2 python3-venv
# Bau des C++-Kerns (xgonav/native): gleiche OpenCV-Version wie python3-opencv
apt-get install -y g++ make pkg-config libopencv-dev

# venv mit Systempaketen (picamera2/libcamera kommen nur aus apt)
python3 -m venv --system-site-packages "$DEST/venv"
"$DEST/venv/bin/pip" install -q "onnxruntime>=1.16"
"$DEST/venv/bin/pip" install -q --no-deps ncnn   # Detektor-Backend (schneller als onnxruntime am CM4)

mkdir -p "$DEST" "$DEST/models"
cp -r xgonav tools web "$DEST/"
[ -f "$DEST/config.toml" ] || cp deploy/config.example.toml "$DEST/config.toml"
cp -n models/*.onnx "$DEST/models/" 2>/dev/null || echo "Hinweis: kein ONNX-Modell in models/ – siehe tools/export_model.sh"
cp -f models/floornet.bin "$DEST/models/" 2>/dev/null || true   # FloorNet, nur für [freespace] shadow_net
for d in models/*_ncnn; do [ -d "$d" ] && [ ! -e "$DEST/$d" ] && cp -r "$d" "$DEST/models/"; done
# C++-Kern der Ego-Motion bauen; scheitert das, rechnet xgonav in Python (langsamer)
make -C "$DEST/xgonav/native" || echo "Hinweis: C++-Kern nicht gebaut – Ego-Motion läuft in Python"
# Emotionen einmalig als RGB565 erzeugen (am Rider läuft davon nichts je Bild).
echo "Erzeuge Emotionen (einmalig) ..."
"$DEST/venv/bin/python" tools/eyes.py "$DEST/emotions" || echo "Hinweis: Emotionen nicht erzeugt"
# Geräusche (freudig/traurig bei Person erkannt/verloren), Wiedergabe per aplay
"$DEST/venv/bin/python" tools/sounds.py "$DEST/sounds"
# Augen des Modus pet (einmalig; am Rider läuft davon nichts je Bild)
"$DEST/venv/bin/python" tools/eyes.py --gaze "$DEST/eyes_gaze" || echo "Hinweis: Augen-Atlas nicht erzeugt – Modus pet zeigt das Overlay"
chown -R "$USER_NAME":"$USER_NAME" "$DEST"
usermod -aG video,audio "$USER_NAME"
# Dauerhafte Einstellungen aus dem Web (LED-Helligkeit): www-data schreibt, xgonav und rc.local lesen
install -d -m 2775 -g video /var/lib/xgonav
# Einstellungen im Web: www-data (Gruppe video) schreibt config.toml, Sicherungen nach /var/lib/xgonav/config-backup
chgrp video "$DEST/config.toml" && chmod 664 "$DEST/config.toml"
install -d -m 2775 -g video /var/lib/xgonav/config-backup

sed "s/__USER__/$USER_NAME/" deploy/xgonav.service > /etc/systemd/system/xgonav.service
systemctl daemon-reload
echo
echo "Installiert nach $DEST."
echo "Test:      $DEST/venv/bin/python -m xgonav --config $DEST/config.toml --mode idle   (im Verzeichnis $DEST)"
echo "Dienst:    systemctl enable --now xgonav"
echo "Oberfläche: https://<rider>/xgonav/ (nginx + php-fpm, siehe README; xgonav selbst nur 127.0.0.1:8088)"
