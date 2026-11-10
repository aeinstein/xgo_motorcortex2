# Installation

Target: Raspberry Pi OS Bookworm 64-bit on the RiderPi CM4. Run from the repo root.

## One step

```
sudo ./deploy/install-all.sh [--no-driver] [--no-display] [--no-web]
```

| Step | Action |
|---|---|
| Driver | builds `kernel_module/` as the calling user, installs `xgo-drv`; needs `/lib/modules/$(uname -r)/build` (`linux-headers-$(uname -r)`) |
| Display | builds `st7789/project/riderpi`, installs `/usr/bin/st7789_server`; warns if `st7789_display.service` is enabled |
| rc.local | installs `/etc/rc.local` and `rc-local.service` |
| xgonav | runs `deploy/install.sh` |
| Web | `apt-get install nginx php8.2-fpm php8.2-cli`, php-fpm pool, nginx site, `www-data` → group `video`, sudoers rule, restarts php-fpm and nginx, installs and starts `xgonav-live.service` (`tools/live.php`) |

Behaviour:
- Files in `/etc` that differ are backed up as `*.bak-<timestamp>`.
- The nginx site is enabled only if certificate and key exist (`/etc/xgonav/tls/fullchain.crt`, `privkey.key`; copied once from the old `rc_server/riderpi` folder if only that exists). Then `sites-enabled/default` is removed and legacy `stunnel4`, `rc-control` are disabled if still present.
- Does **not** start xgonav, does **not** reload the driver. Both take effect after reboot / manual start.

## Kernel

| Item | Value |
|---|---|
| Version | stays on `6.6.74+rpt-rpi-v8` |
| Pin | `sudo cp deploy/apt-pin-kernel /etc/apt/preferences.d/xgonav-kernel` (blocks the `linux-*-rpi-*` meta packages and 6.12) |
| Why | on 6.12.115 `wm8960-soundcard` (DKMS) does not build, which aborts the kernel setup; `xgo-drv` is neither built nor tested there |
| After `apt upgrade` | check imports as the service user: `sudo -u aeinstein /opt/xgonav/venv/bin/python -c "import picamera2"` (the service also uses `~/.local`, root does not) |

## Manual steps

```
sudo make -C kernel_module && sudo make -C kernel_module install
make -C st7789/project/riderpi && sudo cp st7789/project/riderpi/st7789_server /usr/bin/
sudo cp deploy/rc.local /etc/rc.local && sudo cp deploy/rc-local.service /etc/systemd/system/
sudo ./deploy/install.sh
sudo apt-get install -y nginx php8.2-fpm php8.2-cli
sudo cp deploy/php-fpm-xgonav.conf /etc/php/8.2/fpm/pool.d/xgonav.conf    # listen.owner = nginx user
sudo cp deploy/nginx-xgonav.conf /etc/nginx/sites-available/xgonav
sudo ln -s ../sites-available/xgonav /etc/nginx/sites-enabled/
sudo rm /etc/nginx/sites-enabled/default
sudo usermod -aG video www-data
sudo install -m 0440 deploy/sudoers-xgonav-poweroff /etc/sudoers.d/xgonav-poweroff && sudo visudo -c
sudo install -m 0755 deploy/xgo-ruhe /usr/local/sbin/xgo-ruhe
sudo install -m 0440 deploy/sudoers-xgonav-ruhe /etc/sudoers.d/xgonav-ruhe && sudo visudo -c
sudo nginx -t && sudo systemctl restart php8.2-fpm nginx
sudo cp deploy/xgonav-live.service /etc/systemd/system/ && sudo systemctl daemon-reload && sudo systemctl enable --now xgonav-live
```

Reload driver without reboot (set UART flags first, see [driver.md](driver.md)):
```
sudo rmmod xgo-drv; sudo modprobe xgo-drv
cat /proc/XGORider/serial_stats
```

## `deploy/install.sh`

| Item | Value |
|---|---|
| apt packages | `python3-opencv python3-numpy python3-picamera2 python3-venv`; for the C++ core `g++ make pkg-config libopencv-dev` |
| venv | `/opt/xgonav/venv`, `--system-site-packages`, `onnxruntime>=1.16`, `ncnn` |
| Copied | `xgonav/`, `tools/`, `web/` → `/opt/xgonav/`; `models/*.onnx`, `models/*_ncnn/` (no overwrite) |
| C++ core | `make -C /opt/xgonav/xgonav/native` → `libxgofast.so` (failure → Python ego-motion, warning) |
| Config | `deploy/config.example.toml` → `/opt/xgonav/config.toml` only if missing |
| Emotions | `tools/eyes.py /opt/xgonav/emotions` |
| Sounds | `tools/sounds.py /opt/xgonav/sounds` |
| Eye atlas (mode `pet`) | `tools/eyes.py --gaze /opt/xgonav/eyes_gaze` |
| User | `$SUDO_USER` (default `pi`) owns `/opt/xgonav`, added to group `video` |
| Unit | `/etc/systemd/system/xgonav.service` (not enabled) |

## systemd unit `xgonav.service`

| Key | Value |
|---|---|
| ExecStart | `/opt/xgonav/venv/bin/python -m xgonav --config /opt/xgonav/config.toml` |
| User / Group | install user / `video` |
| Requires | `rc-local.service` |
| Restart | `on-failure`, 3 s |
| MemoryMax | 1200M |
| Nice | -5 |

```
sudo systemctl enable --now xgonav
```

## Web permissions (`www-data`)

| Need | Grant |
|---|---|
| Emotions → `/tmp/lcd0` | group `video` |
| Shutdown | `/etc/sudoers.d/xgonav-poweroff`: `www-data ALL=(root) NOPASSWD: /usr/bin/systemctl poweroff` |
| Sleep mode | `/usr/local/sbin/xgo-ruhe` (governor, display), `/etc/sudoers.d/xgonav-ruhe`: `www-data` may run only `xgo-ruhe on` / `xgo-ruhe off` |
| LEDs, actions, height, roll | `/proc/XGORider/*` is mode 0666 |

## nginx (`deploy/nginx-xgonav.conf`)

| Location | Target |
|---|---|
| `:80` | redirect to https |
| `/` | redirect to `/xgonav/` |
| `/xgonav/` | `/opt/xgonav/web/` (PHP via php-fpm) |
| `/xgonav/ws`, `/xgonav/frame`, `/xgonav/stream`, `/xgonav/map.png` | proxied to xgonav `127.0.0.1:8088` |
| `/xgonav/live` | WebSocket, proxied to `xgonav-live` `127.0.0.1:8089` |

## YOLO model (PC/GPU server only, not on the Pi)

```
./tools/export_model.sh [yolo11n] [256]    # -> models/yolo11n_256_ncnn/ + models/yolo11n_256.onnx (opset 12)
scp -r models/yolo11n_256_ncnn models/yolo11n_256.onnx root@<rider>:/opt/xgonav/models/
```

`VENV=<dir>` reuses an existing venv with ultralytics; on Windows/Git Bash set `PYTHON=python`.
Missing `model` → `fallback_model`; both missing → detector disabled (warning in log).

## Before driving

- Body height ≥ minimum + 10 mm (web UI: u8 ≥ 43).
