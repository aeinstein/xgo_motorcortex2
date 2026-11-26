#!/usr/bin/env bash
# Simulator auf kimmy einspielen (vom PC aus, Git Bash):  ./tools/sim_kimmy.sh
# Kopiert xgonav/, tests/, web/ und die Werkzeuge nach ~/xgo_sim/repo, legt beim ersten Mal das venv an und startet
# die systemd-User-Dienste neu:
#   xgo-sim         Zentrale (tools/simserver.py, Port 8790; Läufe in ~/xgo_sim/laeufe bleiben erhalten)
#   xgo-rider       virtueller Rider (tools/sim_rider.py: xgonav 127.0.0.1:8793, 3D-Ansicht :8792)
#   xgo-rider-live  tools/live.php für ihn (127.0.0.1:8794)
# Dazu (einmalig per sudo) PHP, php-fpm-Pool xgo-rider und nginx-Site :8791 mit der echten Weboberfläche aus
# /srv/xgo_sim/web; /srv/xgo_sim/proc spielt /proc/XGORider (Gruppe www-data).
set -euo pipefail
cd "$(dirname "$0")/.."

ZIEL=${ZIEL:-aeinstein@kimmy.intern.itnox.de}
KEY=${KEY:-/c/Users/aeinstein.ITNOX/Documents/aeinstein-ed.pem}
PORT=${PORT:-8790}
SSH=(ssh -o IdentitiesOnly=yes -i "$KEY" "$ZIEL")

git ls-files -co --exclude-standard -- xgonav tests pytest.ini web \
    tools/sim3d.py tools/sim3d.html tools/simserver.py tools/simserver.html \
    tools/sim_rider.py tools/sim_rider.nginx.conf tools/sim_rider.fpm.conf tools/live.php tools/ws.inc.php tools/sprech.php \
  | grep -v -e '\.so$' -e '__pycache__' \
  | tar -czf - -T - \
  | "${SSH[@]}" "set -e; rm -rf ~/xgo_sim/repo.neu; mkdir -p ~/xgo_sim/repo.neu ~/xgo_sim/laeufe;
      tar -xzf - -C ~/xgo_sim/repo.neu; rm -rf ~/xgo_sim/repo; mv ~/xgo_sim/repo.neu ~/xgo_sim/repo"

"${SSH[@]}" bash -s "$PORT" <<'EOF'
set -e
PORT=$1
cd ~/xgo_sim
if [ ! -x venv/bin/python ]; then
  # kimmy hat kein python3-venv; wie die anderen Projekte dort über uv (Python 3.12)
  UV=$(command -v uv || echo ~/.local/bin/uv)
  rm -rf venv
  "$UV" venv -q --python 3.12 venv
  VIRTUAL_ENV=$PWD/venv "$UV" pip install -q "opencv-python-headless>=4.6" "numpy>=1.24" pytest
fi

# --- einmalig mit sudo: PHP, Verzeichnisse, php-fpm-Pool, nginx-Site (nur wenn etwas fehlt oder sich ändert)
if ! command -v php >/dev/null || ! ls /usr/sbin/php-fpm* >/dev/null 2>&1; then
  sudo -n apt-get install -y -q php-fpm php-cli >/dev/null
fi
PHPV=$(php -r 'echo PHP_MAJOR_VERSION.".".PHP_MINOR_VERSION;')
if [ ! -d /srv/xgo_sim/proc ]; then
  sudo -n mkdir -p /srv/xgo_sim/proc
  sudo -n chown "$USER":"$USER" /srv/xgo_sim
  sudo -n chown "$USER":www-data /srv/xgo_sim/proc
  sudo -n chmod 2775 /srv/xgo_sim/proc
fi
POOL=/etc/php/$PHPV/fpm/pool.d/xgo-rider.conf
if ! cmp -s repo/tools/sim_rider.fpm.conf "$POOL"; then
  sudo -n cp repo/tools/sim_rider.fpm.conf "$POOL"
  sudo -n systemctl restart "php$PHPV-fpm"
fi
SITE=/etc/nginx/sites-available/xgo-rider
if ! cmp -s repo/tools/sim_rider.nginx.conf "$SITE"; then
  sudo -n cp repo/tools/sim_rider.nginx.conf "$SITE"
  sudo -n ln -sf "$SITE" /etc/nginx/sites-enabled/xgo-rider
  sudo -n nginx -t -q
  sudo -n systemctl reload nginx
fi
# Weboberfläche: www-data liest /srv/xgo_sim/web (das Home ist für www-data nicht offen)
rm -rf /srv/xgo_sim/web.neu
cp -r repo/web /srv/xgo_sim/web.neu
rm -rf /srv/xgo_sim/web
mv /srv/xgo_sim/web.neu /srv/xgo_sim/web

mkdir -p ~/.config/systemd/user
cat > ~/.config/systemd/user/xgo-sim.service <<UNIT
[Unit]
Description=xgonav Simulator-Zentrale (Läufe starten, 3D-Ansicht, Port $PORT)

[Service]
WorkingDirectory=%h/xgo_sim/repo
ExecStart=%h/xgo_sim/venv/bin/python tools/simserver.py --host 0.0.0.0 --port $PORT --runs %h/xgo_sim/laeufe --max 4
Restart=on-failure
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=default.target
UNIT
cat > ~/.config/systemd/user/xgo-rider.service <<UNIT
[Unit]
Description=Virtueller Rider (tools/sim_rider.py, xgonav 127.0.0.1:8793, Weboberfläche :8791, 3D :8792)

[Service]
WorkingDirectory=%h/xgo_sim/repo
ExecStart=%h/xgo_sim/venv/bin/python tools/sim_rider.py --proc /srv/xgo_sim/proc --port 8793 --view-port 8792
Nice=10
UMask=0002
Restart=on-failure
RestartSec=3
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=default.target
UNIT
cat > ~/.config/systemd/user/xgo-rider-live.service <<UNIT
[Unit]
Description=Laufzeitdaten für den virtuellen Rider (tools/live.php, 127.0.0.1:8794)

[Service]
WorkingDirectory=%h/xgo_sim/repo
ExecStart=/usr/bin/php tools/live.php
Environment=XGO_PROC=/srv/xgo_sim/proc XGO_LIVE_PORT=8794 XGO_STATUS_URL=http://127.0.0.1:8793/status
Restart=on-failure
RestartSec=3

[Install]
WantedBy=default.target
UNIT
systemctl --user daemon-reload
for d in xgo-sim xgo-rider xgo-rider-live; do
  systemctl --user enable -q "$d.service"
  systemctl --user restart "$d.service"
done
sleep 2
systemctl --user is-active xgo-sim.service xgo-rider.service xgo-rider-live.service
EOF
echo "Simulator-Zentrale: http://192.168.53.236:$PORT/"
echo "Virtueller Rider:   http://192.168.53.236:8791/  (3D: http://192.168.53.236:8792/)"
