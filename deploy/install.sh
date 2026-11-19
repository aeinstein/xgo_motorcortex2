#!/bin/bash
# Installation auf dem Rider (Raspberry Pi OS Bookworm 64-bit, CM4).
# Aufruf im entpackten xgonav-Verzeichnis:  sudo ./deploy/install.sh
set -euo pipefail
DEST=/opt/xgonav
USER_NAME=${SUDO_USER:-pi}

apt-get install -y python3-opencv python3-numpy python3-picamera2 python3-venv
# Bau des C++-Kerns (xgonav/native): gleiche OpenCV-Version wie python3-opencv
apt-get install -y g++ make pkg-config libopencv-dev
# Kamera direkt über libcamera (xgonav/native cam); fehlt es, nimmt xgonav picamera2
apt-get install -y libcamera-dev || echo "Hinweis: libcamera-dev fehlt – Kamera über picamera2"

# venv mit Systempaketen (picamera2/libcamera kommen nur aus apt)
python3 -m venv --system-site-packages "$DEST/venv"
"$DEST/venv/bin/pip" install -q "onnxruntime>=1.16"
"$DEST/venv/bin/pip" install -q --no-deps ncnn   # Detektor-Backend (schneller als onnxruntime am CM4)

mkdir -p "$DEST" "$DEST/models"
cp -r xgonav tools web "$DEST/"
[ -f "$DEST/config.toml" ] || cp deploy/config.example.toml "$DEST/config.toml"
cp -n models/*.onnx "$DEST/models/" 2>/dev/null || echo "Hinweis: kein ONNX-Modell in models/ – siehe tools/export_model.sh"
# FloorNet ([freespace] net), ohne Netz ist der Rider blind. Wählbare Netze liegen in models/floornet/ ([freespace]
# net_dir, Auswahl im Web), models/floornet.bin ist ein Symlink auf das aktive. Nie mit cp auf den Symlink schreiben:
# das überschriebe das gewählte Netz. Das Netz aus dem Repo heißt dort p11a.bin.
install -d "$DEST/models/floornet"
if [ -f "$DEST/models/floornet.bin" ] && [ ! -L "$DEST/models/floornet.bin" ]; then
    mv "$DEST/models/floornet.bin" "$DEST/models/floornet/vorher-$(date +%Y%m%d-%H%M).bin"
    ln -s "floornet/$(basename "$(ls -t "$DEST"/models/floornet/vorher-*.bin | head -1)")" "$DEST/models/floornet.bin"
fi
[ -f models/floornet.bin ] && cp -n models/floornet.bin "$DEST/models/floornet/p11a.bin"
for f in models/floornet/*.bin; do [ -f "$f" ] && cp -n "$f" "$DEST/models/floornet/"; done
[ -e "$DEST/models/floornet.bin" ] || ln -sfn floornet/p11a.bin "$DEST/models/floornet.bin"
cp -r models/wechat_qr "$DEST/models/" 2>/dev/null || true   # WeChat-QR für die WLAN-Einrichtung ([wlan] qr_model_dir; github.com/WeChatCV/opencv_3rdparty, Apache-2.0)
for d in models/*_ncnn; do [ -d "$d" ] && [ ! -e "$DEST/$d" ] && cp -r "$d" "$DEST/models/"; done
# C++-Kern der Ego-Motion bauen; scheitert das, rechnet xgonav in Python (langsamer)
make -C "$DEST/xgonav/native" || echo "Hinweis: C++-Kern nicht gebaut – Ego-Motion läuft in Python"
# Kamera direkt über libcamera; nach jedem apt upgrade von libcamera neu bauen (keine stabile ABI)
make -C "$DEST/xgonav/native" cam || echo "Hinweis: libxgocam.so nicht gebaut – Kamera über picamera2"
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
echo "Oberfläche: https://<rider>/ (nginx + php-fpm, siehe README; xgonav selbst nur 127.0.0.1:8088)"
