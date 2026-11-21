# Install

Target: Raspberry Pi OS Bookworm 64-bit on the RiderPi CM4, kernel **6.6.74**. All commands run on the Rider from the repo root unless marked *PC*.
Full reference: [Installation](../installation.md).

## 1. Prepare

| Step | Command |
|---|---|
| Pin the kernel (6.12 breaks the sound driver, `xgo-drv` untested there) | `sudo cp deploy/apt-pin-kernel /etc/apt/preferences.d/xgonav-kernel` |
| Kernel headers | `sudo apt install linux-headers-$(uname -r)` |
| Copy the repo to the Rider (*PC*) | `scp -r xgo_motorcortex2 root@<rider>:/home/` or `git clone` on the Rider |
| TLS certificate for the web UI | `/etc/xgonav/tls/fullchain.crt` + `privkey.key` (without them nginx stays off) |

!!! note "Windows clones"
    With `core.autocrlf=true`, shell scripts may arrive with CRLF and fail at boot. `.gitattributes` forces LF; check copied scripts with `file`.

## 2. Install everything

```bash
sudo ./deploy/install-all.sh            # driver, display, rc.local, xgonav -> /opt/xgonav, nginx + php-fpm
sudo reboot                             # driver and rc.local take effect after reboot
```

| Option | Skips |
|---|---|
| `--no-driver` | kernel module |
| `--no-display` | LCD daemon |
| `--no-web` | nginx, php-fpm, sudoers, `xgonav-live`, `xgonav-sprech` |

`install-all.sh` backs up every changed file in `/etc` as `*.bak-<timestamp>` and does **not** start xgonav.

## 3. What gets built

| Part | Built by | Without it |
|---|---|---|
| `xgo-drv` | `make -C kernel_module` | no hardware access |
| C++ core `libxgofast.so` | `make -C /opt/xgonav/xgonav/native` (in `install.sh`) | **no FloorNet = blind**, Rider does not drive by itself |
| Camera library `libxgocam.so` | `make -C /opt/xgonav/xgonav/native cam` | falls back to picamera2. Rebuild after every `apt upgrade` |
| `st7789_server` | `make -C st7789/project/riderpi` | no eyes on the LCD |

## 4. Models

| Model | Where it comes from |
|---|---|
| FloorNet `models/floornet.bin` | in the repo; `install.sh` copies it to `/opt/xgonav/models/floornet/` |
| YOLO (follow, approach, pet) | export on the PC/GPU machine only: `./tools/export_model.sh yolo11n 256`, then `scp -r models/yolo11n_256_ncnn models/yolo11n_256.onnx root@<rider>:/opt/xgonav/models/` |

Without YOLO the detector is off (warning in the log); manual driving, explore and home still work.

## 5. Start

```bash
sudo systemctl enable --now xgonav      # as a service (user aeinstein, group video)
journalctl -u xgonav -f                 # log (German)
```

Open `https://<rider>/`. No authentication: **LAN only**.

## 6. Check

| Check | Expect |
|---|---|
| `cat /proc/XGORider/serial_stats` | counters rising, few errors |
| `cat /proc/XGORider/imu` | roll/pitch/yaw change when you tilt the Rider |
| log after start | `[floornet] … geladen, Selbstprüfung …`; no `[floornet] … aus` |
| web header | battery, state, pipeline shown |

After `apt upgrade`: check imports as the service user, the service uses `~/.local` (OpenCV 4.10, numpy 1.26):
`sudo -u aeinstein /opt/xgonav/venv/bin/python -c "import cv2, numpy, picamera2"`.

## Update

```bash
git pull && sudo ./deploy/install.sh && sudo systemctl restart xgonav
```

Do not rebuild `.so` files while xgonav runs unless the build replaces them atomically (`.tmp` + `mv`, default since `d4f0943`).
