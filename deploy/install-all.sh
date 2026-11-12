#!/bin/bash
# Gesamtinstallation auf dem Rider (Raspberry Pi OS Bookworm 64-bit, CM4).
# Fasst README „Gesamtinstallation auf dem Rider“ zusammen: Treiber, Display-Daemon,
# rc.local, xgonav (deploy/install.sh), Weboberfläche (nginx mit TLS, php-fpm, sudoers).
#
# Aufruf im Repo-Root:  sudo ./deploy/install-all.sh [--no-driver] [--no-display] [--no-web]
#
# Bewusst NICHT gemacht:
#  - keine Dienste starten oder aktivieren, die den Rider bewegen (xgonav bleibt aus),
#  - Treiber nicht live neu laden (rmmod/modprobe) – greift erst nach dem Neustart,
#  - kein st7789_display.service: den Display-Daemon startet rc.local (nicht beides).
# Vorhandene Dateien in /etc werden vor dem Überschreiben als *.bak-<Zeitstempel> gesichert.
set -euo pipefail

cd "$(dirname "$0")/.."
REPO=$(pwd)
USER_NAME=${SUDO_USER:-pi}
STAMP=$(date +%Y%m%d-%H%M%S)
DO_DRIVER=1 DO_DISPLAY=1 DO_WEB=1

for arg in "$@"; do
    case "$arg" in
        --no-driver)  DO_DRIVER=0 ;;
        --no-display) DO_DISPLAY=0 ;;
        --no-web)     DO_WEB=0 ;;
        -h|--help)    sed -n 2,13p "$0"; exit 0 ;;
        *) echo "Unbekannte Option: $arg" >&2; exit 2 ;;
    esac
done

if [ "$(id -u)" -ne 0 ]; then
    echo "Bitte mit sudo aufrufen: sudo $0 $*" >&2
    exit 1
fi

WARN=()
schritt() { echo; echo "== $*"; }
warnung() { echo "WARNUNG: $*" >&2; WARN+=("$*"); }

# Datei nach Ziel kopieren; vorhandenes, abweichendes Ziel vorher sichern.
installiere() {   # installiere <quelle> <ziel> [modus]
    local src=$1 dst=$2 mode=${3:-0644}
    if [ -e "$dst" ] && ! cmp -s "$src" "$dst"; then
        cp -a "$dst" "$dst.bak-$STAMP"
        echo "  gesichert: $dst.bak-$STAMP"
    fi
    install -m "$mode" "$src" "$dst"
    echo "  $dst"
}

# Als Aufrufer bauen, nicht als root (Build-Artefakte gehören sonst root).
als_nutzer() { sudo -u "$USER_NAME" "$@"; }

# ---------------------------------------------------------------- Treiber
if [ "$DO_DRIVER" = 1 ]; then
    schritt "Treiber xgo-drv bauen und installieren"
    if [ ! -d "/lib/modules/$(uname -r)/build" ]; then
        echo "Kernel-Header für $(uname -r) fehlen (/lib/modules/$(uname -r)/build)." >&2
        echo "Auf Bookworm z. B.: sudo apt-get install linux-headers-$(uname -r) – oder mit --no-driver überspringen." >&2
        exit 1
    fi
    als_nutzer make -C kernel_module
    make -C kernel_module install
    echo "  Hinweis: läuft bereits ein xgo-drv, gilt der neue Treiber erst nach dem Neustart."
fi

# ---------------------------------------------------------------- Display
if [ "$DO_DISPLAY" = 1 ]; then
    schritt "Display-Daemon st7789_server bauen"
    als_nutzer make -C st7789/project/riderpi
    installiere st7789/project/riderpi/st7789_server /usr/bin/st7789_server 0755
    if systemctl is-enabled --quiet st7789_display.service 2>/dev/null; then
        warnung "st7789_display.service ist aktiviert, rc.local startet den Daemon ebenfalls – eins davon abschalten (systemctl disable st7789_display)."
    fi
fi

# ---------------------------------------------------------------- rc.local
schritt "rc.local (UART-Rohmodus, Treiber laden, Display-Daemon)"
installiere deploy/rc.local /etc/rc.local 0755
installiere deploy/rc-local.service /etc/systemd/system/rc-local.service
systemctl daemon-reload
# Kein "systemctl start": rc.local lädt den Treiber und startet den Daemon beim Booten.

# ---------------------------------------------------------------- xgonav
schritt "xgonav nach /opt/xgonav (deploy/install.sh)"
./deploy/install.sh

# ---------------------------------------------------------------- Web
if [ "$DO_WEB" = 1 ]; then
    schritt "Weboberfläche: nginx (TLS auf 443), php-fpm"
    apt-get install -y nginx php8.2-fpm php8.2-cli

    # php-fpm-Socket muss dem Benutzer gehören, unter dem nginx läuft.
    NGINX_USER=$(awk '$1 == "user" { sub(";", "", $2); print $2; exit }' /etc/nginx/nginx.conf)
    NGINX_USER=${NGINX_USER:-www-data}
    echo "  nginx läuft als: $NGINX_USER"
    tmp=$(mktemp)
    sed -e "s/^listen.owner = .*/listen.owner = $NGINX_USER/" \
        -e "s/^listen.group = .*/listen.group = $NGINX_USER/" \
        deploy/php-fpm-xgonav.conf > "$tmp"
    installiere "$tmp" /etc/php/8.2/fpm/pool.d/xgonav.conf
    rm -f "$tmp"

    # nginx macht TLS selbst; ohne Zertifikat/Schlüssel (Pfade in deploy/nginx-xgonav.conf)
    # würde nginx nicht starten, deshalb dann die Seite nicht aktivieren.
    CERT=$(awk '$1 == "ssl_certificate" { sub(";", "", $2); print $2 }' deploy/nginx-xgonav.conf)
    KEY=$(awk '$1 == "ssl_certificate_key" { sub(";", "", $2); print $2 }' deploy/nginx-xgonav.conf)
    # Einmalig umziehen: Zertifikat und Schlüssel lagen im Ordner der entfernten Fernsteuerung.
    TLS_ALT=/home/aeinstein/motorcortex/rc_server/riderpi
    if [ ! -f "$KEY" ] && [ -f "$TLS_ALT/privkey.key" ] && [ -f "$TLS_ALT/fullchain.crt" ]; then
        install -d -m 0755 "$(dirname "$CERT")"
        install -m 0644 "$TLS_ALT/fullchain.crt" "$CERT"
        install -m 0600 "$TLS_ALT/privkey.key" "$KEY"
        echo "  TLS: Zertifikat und Schlüssel nach $(dirname "$CERT") kopiert"
    fi
    installiere deploy/nginx-xgonav.conf /etc/nginx/sites-available/xgonav
    if [ -f "$CERT" ] && [ -f "$KEY" ]; then
        ln -sfn ../sites-available/xgonav /etc/nginx/sites-enabled/xgonav
        # Die Seite ist default_server auf 80 (Weiterleitung auf https), die Debian-Default-Site
        # würde damit kollidieren.
        if [ -e /etc/nginx/sites-enabled/default ]; then
            cp -a /etc/nginx/sites-enabled/default "/etc/nginx/sites-enabled-default.bak-$STAMP"
            rm /etc/nginx/sites-enabled/default
            echo "  sites-enabled/default entfernt (Sicherung /etc/nginx/sites-enabled-default.bak-$STAMP)"
        fi
        # Falls noch vorhanden: stunnel (früher TLS auf 443) und die entfernte Fernsteuerung.
        for dienst in stunnel4 rc-control; do
            if systemctl is-enabled --quiet "$dienst" 2>/dev/null || systemctl is-active --quiet "$dienst" 2>/dev/null; then
                systemctl disable --now "$dienst" >/dev/null 2>&1 && echo "  $dienst gestoppt und abgeschaltet"
            fi
        done
    else
        rm -f /etc/nginx/sites-enabled/xgonav
        warnung "Weboberfläche nicht aktiviert: Zertifikat/Schlüssel fehlen ($CERT, $KEY). Pfade in deploy/nginx-xgonav.conf anpassen und erneut aufrufen."
    fi

    # Rechte für www-data: Emotionen nach /tmp/lcd0, nur "systemctl poweroff", "xgo-ruhe on|off" und
    # "systemctl --no-block restart xgonav" (Einstellungen).
    usermod -aG video www-data
    installiere deploy/xgo-ruhe /usr/local/sbin/xgo-ruhe 0755
    for regel in poweroff ruhe restart; do
        if visudo -cf "deploy/sudoers-xgonav-$regel" >/dev/null; then
            installiere "deploy/sudoers-xgonav-$regel" "/etc/sudoers.d/xgonav-$regel" 0440
            visudo -c >/dev/null
        else
            warnung "deploy/sudoers-xgonav-$regel ist ungültig – nicht installiert."
        fi
    done

    nginx -t
    systemctl restart php8.2-fpm nginx

    # Laufzeitdaten per WebSocket (/xgonav/live): eigener PHP-CLI-Dienst statt Polling über php-fpm
    installiere deploy/xgonav-live.service /etc/systemd/system/xgonav-live.service
    systemctl daemon-reload
    systemctl enable xgonav-live >/dev/null 2>&1
    systemctl restart xgonav-live
fi

# ---------------------------------------------------------------- Ende
schritt "Fertig"
cat <<EOF
Installiert: $( [ "$DO_DRIVER" = 1 ] && echo -n "Treiber, " )$( [ "$DO_DISPLAY" = 1 ] && echo -n "Display, " )rc.local, xgonav$( [ "$DO_WEB" = 1 ] && echo -n ", Web" ).

Nächste Schritte:
  1. Neustart (lädt Treiber und Display-Daemon über rc.local):  sudo reboot
  2. Kontrolle:  cat /proc/XGORider/serial_stats; cat /proc/XGORider/imu
  3. Probelauf ohne Fahren:  cd /opt/xgonav && venv/bin/python -m xgonav --config config.toml --mode idle
  4. Dienst erst nach dem Probelauf:  sudo systemctl enable --now xgonav
  Oberfläche: https://<rider>/xgonav/
EOF
if [ ${#WARN[@]} -gt 0 ]; then
    echo
    echo "Offene Warnungen:"
    printf '  - %s\n' "${WARN[@]}"
fi
