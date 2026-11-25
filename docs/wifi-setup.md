# Wi-Fi setup without a network

Two ways in, no existing connection needed: the **automatic hotspot** `RiderKerk` and the **buttons C+D** (Wi-Fi QR code to the camera). Bluetooth: [ble.md](ble.md). Code: root part `deploy/xgo-wlan` (NetworkManager), service `deploy/xgo-wlan-auto.service`, buttons `xgonav/wlan.py`, web form `web/wlan.php`. Automatic hotspot since 2026-11-21, not yet tested on the Rider.

## Automatic hotspot

| Item | Value |
|---|---|
| Trigger | `wlan0` not connected for 60 s (`/sys/class/net/wlan0/operstate` ≠ `up`): after boot without a reachable or configured network, or after a lost connection |
| Network | SSID `RiderKerk`, **open (no password)**, 2.4 GHz, NetworkManager profile `xgo-hotspot` (autoconnect off), Rider at `10.42.0.1` |
| Join | phone into `RiderKerk`, open `https://10.42.0.1/wlan.php` (self-signed TLS warning), pick network, enter password |
| Connect fails | hotspot back after about a minute; join again, `wlan.php` shows the error (`status` field `letzt`) |
| Back to known networks | every 5 min, only if a Wi-Fi profile exists and no device is joined: hotspot closed, `brcmfmac` reloaded, NetworkManager tries known networks (≤ 45 s), else hotspot again |
| No profile at all | hotspot stays |
| Independent of | xgonav (service runs as root, `After=NetworkManager.service`) |
| Security | open network: anyone in range reaches the web interface, MCP and driving while the hotspot is up |
| Display | xgonav shows `Hotspot RiderKerk: 10.42.0.1/wlan.php` only in the C+D screen; otherwise nothing |

## Buttons C+D

| Step | Action | Display |
|---|---|---|
| 1 | Hold **C+D** for 3 s (in sleep mode up to ~5 s) | eyes looking around with QR finder patterns as pupils (`QrEyes`, CM4 ~4.7 ms per frame at 20 fps ≈ 9 % of one core, only while searching), small camera image top left, Wi-Fi symbol with 60 s countdown ring (red for the last 10 s), "WLAN-QR vor die Kamera halten"; hotspot address if the automatic hotspot is up |
| 2 | Show a Wi-Fi QR code to the camera (Android: Wi-Fi → Share; iOS: Wi-Fi → (i) → share / Passwords app) | "Verbinde mit …", then "Verbunden: <ssid>" + IP (20 s) or the error (8 s, then back to 1) |
| 3 | No code within 60 s | "Kein WLAN-Code", setup ends (no hotspot from here since 2026-11-21) |
| Cancel | press **B**, hold C+D again, or `POST /wlan?on=0` | setup ends |

Also reachable from the web: `api.php?a=wlan&on=1`; `wlan.php` works in normal operation as "add Wi-Fi".

## Behaviour

| Item | Value |
|---|---|
| Trigger | `/proc/XGORider/buttons` (read by the robot thread anyway), `[wlan] buttons = 12` held `hold_s = 3`; cancel `cancel_buttons = 2` (B). A+B is shutdown in the driver |
| During setup | sleep mode ended, mode `idle`, `robot.stop()` (a balancing Rider stays standing), camera on, `_step` does not run |
| After | camera returns to its previous state, mode `idle` |
| QR | WeChat QR (opencv-contrib) if `qr_model_dir` exists, else `cv2.QRCodeDetector` on full frame, else 2× centre crop; ≤ `qr_hz = 5`; format `WIFI:T:..;S:..;P:..;H:..;;` with escapes, any order; WPA or open (WEP rejected) |
| Profile | keyfile `/etc/NetworkManager/system-connections/xgo-wlan-<uuid>.nmconnection` (0600), name `xgo-<ssid>`, `autoconnect-priority = 10`, `ipv4.may-fail = false` (no IPv4 = failure); password never on a command line; same-name `xgo-` profile replaced, others never deleted |
| Failure | new profile deleted, previous connection (or the hotspot) brought back |
| Locking | `flock /run/xgo-wlan.lock`: `connect`/`hotspot`/`hotspot-off` wait (≤ 90 s), an automatic step is skipped while one runs |
| Leaving the hotspot | `brcmfmac` reloaded (`chip_reset`, ~5 s) before reconnecting. Without it (measured 2026-11-19): frames to the Rider with WMM priority Voice are lost until reboot; NetworkManager's DHCPv4 uses TOS 0xc0, the router offers, nothing arrives → IPv6 only, unreachable via IPv4 |
| Files | `/run/xgo-wlan.hotspot` (hotspot up), `/run/xgo-wlan.prev` (previous connection), `/run/xgo-wlan.letzt` (last failed connect from the hotspot) |

## Cost on the Pi

| State | Cost |
|---|---|
| Connected | `xgo-wlan auto`: one shell `read` from `/sys` and one `sleep` every 10 s; no nmcli, no D-Bus |
| Hotspot | additionally `nmcli con show` + `iw station dump` every 5 min; a return attempt costs ~5 s chip reset and up to 45 s without hotspot |
| C+D setup | pipeline paused; QR decode ≤ 5 Hz; LCD image only on change; one `xgo-wlan status` at start |

## Config `[wlan]`

| Key | Default |
|---|---|
| `enabled` | true |
| `buttons`, `hold_s`, `cancel_buttons` | 12, 3.0, 2 |
| `qr_hz`, `qr_timeout_s` | 5.0, 60.0 |
| `qr_model_dir` | `models/wechat_qr` |
| `helper` | `/usr/local/sbin/xgo-wlan` |

`hotspot_ssid` and `hotspot_timeout_s` are gone (warning when still in `config.toml`). Hotspot timings are constants in `deploy/xgo-wlan` (`TAKT` 10 s, `FRIST` 60 s, `RUECK` 300 s).

## Vendor background

| Source | Method |
|---|---|
| ESP32 firmware (`xgorider2_rl`) | BLE is only a UART channel (fff0/fff1/fff2) for the remote app; no Wi-Fi provisioning, nothing forwarded to the Pi. Replaced by firmware patch 01-ble: own BLE service with a tunnel to the Pi, `wlan status/scan/rescan/connect` over Bluetooth with the same `xgo-wlan` ([ble.md](ble.md)) |
| Vendor Pi image (`RaspberryPi-CM4-XGO-Rider`) | display menu: `demos/network.py` (QR via pyzbar, overwrites `wpa_supplicant.conf`), `hotspot.py` (`nmcli` hotspot) |
