# Wi-Fi setup without a network

Buttons on the Rider start it; no existing connection needed. Code: `xgonav/wlan.py`, root part `deploy/xgo-wlan` (NetworkManager), web form `web/wlan.php`. Not yet tested on the Rider.

## Use

| Step | Action | Display |
|---|---|---|
| 1 | Hold **C+D** for 3 s (in sleep mode up to ~5 s) | eyes looking around with QR finder patterns as pupils (`QrEyes`, ~1.5 ms per frame, 20 fps), small camera image top left, Wi-Fi symbol with 60 s countdown ring (red for the last 10 s), "WLAN-QR vor die Kamera halten" |
| 2 | Show a Wi-Fi QR code to the camera (Android: Wi-Fi → Share; iOS: Wi-Fi → (i) → share / Passwords app) | "Verbinde mit …", then "Verbunden: <ssid>" + IP (20 s) or the error (8 s, then back to 1) |
| 3 | No code within 60 s → hotspot `xgo-rider`, random 8-char password | QR to join, SSID, password, `10.42.0.1/xgonav/wlan.php`, Wi-Fi symbol with 60 s countdown |
| 4 | Join the hotspot, open the URL (self-signed TLS warning), pick network, enter password | hotspot closes when switching; result on the display |
| Cancel | press **B**, hold C+D again, or `POST /wlan?on=0` | hotspot removed, previous connection restored |

Hotspot closes by itself after 60 s (countdown); each newly joined device restarts the countdown (`iw station dump`, polled every 3 s). Also reachable from the web: `api.php?a=wlan&on=1`; `wlan.php` works in normal operation as "add Wi-Fi".

## Behaviour

| Item | Value |
|---|---|
| Trigger | `/proc/XGORider/buttons` (read by the robot thread anyway), `[wlan] buttons = 12` held `hold_s = 3`; cancel `cancel_buttons = 2` (B). A+B is shutdown in the driver |
| During setup | sleep mode ended, mode `idle`, `robot.stop()` (a balancing Rider stays standing), camera on, `_step` does not run |
| After | camera returns to its previous state, mode `idle` |
| QR | `cv2.QRCodeDetector` on full frame, else 2× centre crop; ≤ `qr_hz = 5`; format `WIFI:T:..;S:..;P:..;H:..;;` with escapes, any order; WPA or open (WEP rejected) |
| Profile | keyfile `/etc/NetworkManager/system-connections/xgo-wlan-<uuid>.nmconnection` (0600), name `xgo-<ssid>`, `autoconnect-priority = 10`; password never on a command line; same-name `xgo-` profile replaced, others never deleted |
| Failure | new profile deleted, previous connection (or hotspot) brought back |
| Hotspot | `nmcli dev wifi hotspot`, profile `xgo-hotspot` (autoconnect off), previous connection stored in `/run/xgo-wlan.prev` |

## Cost on the Pi

| State | Cost |
|---|---|
| Idle | one integer compare per main-loop iteration; no thread, no process |
| Setup | pipeline paused; QR decode ≤ 5 Hz; LCD image only on change; `nmcli` status every 5 s in hotspot mode |

## Config `[wlan]`

| Key | Default |
|---|---|
| `enabled` | true |
| `buttons`, `hold_s`, `cancel_buttons` | 12, 3.0, 2 |
| `qr_hz`, `qr_timeout_s` | 5.0, 60.0 |
| `hotspot_ssid`, `hotspot_timeout_s` | `xgo-rider`, 60.0 |
| `helper` | `/usr/local/sbin/xgo-wlan` |

## Vendor background

| Source | Method |
|---|---|
| ESP32 firmware (`xgorider2_rl`) | BLE is only a UART channel (fff0/fff1/fff2) for the remote app; no Wi-Fi provisioning, nothing forwarded to the Pi |
| Vendor Pi image (`RaspberryPi-CM4-XGO-Rider`) | display menu: `demos/network.py` (QR via pyzbar, overwrites `wpa_supplicant.conf`), `hotspot.py` (`nmcli` hotspot) |
