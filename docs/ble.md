# Bluetooth (BLE)

Own GATT service of the ESP32 firmware (patch 0025). Replaces the vendor service `fff0`/`fff1`/`fff2`; the vendor app no longer finds the Rider.

| | |
|---|---|
| Device name | `RiderKerk` (fixed in the firmware; flash sector `0x320000` is no longer read or written) |
| Advertising | name + service UUID |
| Security | none: no pairing, no bonding, no encryption. Anyone in radio range can drive the Rider and set the Wi-Fi; the Wi-Fi password crosses the air in clear text |
| MTU | ESP32 offers 185; request ≥ 185 (Android: `requestMtu(247)`). Works with 23, large replies are then slow |
| Status | flashed and installed on the RiderPi 2026-11-20 ~19:00 (lying). Seen there: advertising as `RiderKerk` with the service UUID, register `0x18` = address of `wlan0`, one tunnel request from a BLE client answered (driver counters). Confirmed by the user with the Android app the same evening: connect, IP shown, web page opened. `CMD`, `CON`, `wlan connect` and throughput not yet checked |

## Service

Service `fdacc797-7e1d-4d87-9205-64ca79f74500`. Characteristics: same base, last byte as below. Every notify characteristic has a CCCD (`0x2902`).

| Characteristic | UUID … | Properties | Content |
|---|---|---|---|
| `CMD` | `…4501` | write, write without response, notify | binary frames of the serial protocol, replies by notify |
| `CON` | `…4502` | write, write without response, notify | text console of the firmware: line in, text out |
| `PI` | `…4503` | write, write without response, notify | byte tunnel to the Pi (Wi-Fi, xgonav) |
| `IP` | `…4504` | read, notify | IPv4 of the Pi, 4 bytes `a.b.c.d` |

Notifications are a byte stream cut at arbitrary points (MTU − 3 per packet): reassemble by frame length (`CMD`), line (`CON`) or reply header (`PI`). On disconnect the firmware sets forward and turn to neutral and drops unsent output.

## `IP`

| | |
|---|---|
| Source | driver `xgo-drv` writes the address of `wlan0` (`settings/ip_dev`) to register `0x18` on change and every 3 s |
| `0.0.0.0` | no address, or nothing from the Pi for 10 s (Pi off, driver not loaded) |
| Needs | driver only; works without `xgo-ble` and without xgonav |
| Also | read register `0x18` over `CMD`; `pi_ip` in `!status` |

## `CMD`: binary frames

Same protocol as the Pi UART (`docs/driver.md`, `firmware/README.md`).

```
55 00 LEN TYPE REG DATA… CHK 00 AA      LEN = bytes of DATA + 8 (≤ 32), CHK = ~(LEN + TYPE + REG + ΣDATA)
```

| TYPE | Meaning |
|---|---|
| `01` | write `REG` (no reply; result text in `!status` → `result`) |
| `02` | read `REG`, DATA = 1 byte (ignored); reply TYPE `12` with the register's fixed length |

| Example | Frame |
|---|---|
| read battery (`0x01`) | `55 00 09 02 01 01 F2 00 AA` → `55 00 09 12 01 57 8C 00 AA` (87 %) |
| stop / sit down (action 254) | `55 00 09 01 3E FE B9 00 AA` |
| LED 0 red (`0x69`) | `55 00 0B 01 69 FF 00 00 8B 00 AA` |

Not over BLE: register `0x11` (IMU stream, Pi only), telemetry stream `0x08`, registers `0x17`/`0x18` writes (Pi only). Removed with 0025: `0x13` (name suffix), `0xb3` (trace stream).

Action 254 written directly does not stop xgonav: send `nav POST /stop` first, otherwise mode `pet` makes it jump up again after a faint.

## `CON`: console

One command per line, terminated by `\n`, at most 95 characters. Same commands as on USB (`!status`, `!stop`, `!arm`, `!standup`, `!restart`, `!move F T`, `!height N`, `!led N`, `!balance N`, `!rgb I R G B`, `!fallguard …`, `!liftguard …`, `!gain X`, …; unknown line → command list). Output of `!trace dump` / `!trace usb on` still goes to USB.

| | |
|---|---|
| `!status` | one JSON line, ~3.5 kB |
| Output buffer | 4 kB; more is dropped |
| Commands without output | most setters; check `result` in `!status` |

## `PI`: tunnel to the Pi

Path: `PI` ↔ ESP32 ↔ UART ↔ `xgo-drv` (`/proc/XGORider/ble`) ↔ service `xgo-ble`. Needs driver and `xgo-ble`, not xgonav (except `nav`), not a network.

Request: one UTF-8 line, `\n` terminated, ≤ 1023 bytes. `<id>` = token chosen by the client (≤ 16 characters), echoed in the reply. One request at a time; replies come in request order.

```
<id> wlan status
<id> wlan scan
<id> wlan rescan
<id> wlan connect <WPA|NOPASS> <0|1> <ssid> [<password>]
<id> nav <GET|POST> <path?query>
```

Reply: header line, then exactly `n` bytes of body.

```
<id> <code> <n>\n<body>
```

| Request | `code` | Body |
|---|---|---|
| `wlan status` | exit code of `xgo-wlan` (0 = ok) | `con=`, `ssid=`, `ip=`, in hotspot mode (`RiderKerk`) `clients=` and `letzt=` (last failed connect); one `key=value` per line |
| `wlan scan` | 〃 | visible networks from the NetworkManager cache, `SIGNAL:SECURITY:SSID` per line (`:` inside an SSID is escaped as `\:`) |
| `wlan rescan` | 〃 | same after a fresh scan (several seconds) |
| `wlan connect` | 0 = connected, 1 = failed | on success as `status`; on failure the reason as last line. Replies after the attempt (up to ~60 s; client timeout ≥ 90 s). On failure the Pi returns to the previous connection |
| `nav` | HTTP status of xgonav; 599 = xgonav not running | reply body, cut at 16 384 bytes |
| any | 400 = not understood, 403 = path not allowed | text |

| `wlan connect` field | |
|---|---|
| `WPA` / `NOPASS` | WPA/WPA2/WPA3 personal with password (8–63 characters) / open network |
| `0` / `1` | 1 = hidden SSID |
| `ssid`, `password` | percent-encoded (space = `%20`, `%` = `%25`, non-ASCII as UTF-8 bytes); no line breaks |

`nav` passes the request to the xgonav HTTP interface on `127.0.0.1:8088` (`docs/web-interface.md`): `/status`, `/stop`, `/mode?name=…`, `/drive?v=…&w=…`, `/pipeline?on=…`, … Not allowed: `/ws`, `/frame`, `/map.png`. Movement rules are those of xgonav (`/drive` only in mode `manual`, modes need the floor net).

Example (phone → Rider, Rider → phone):

```
1 wlan status\n                         1 0 41\ncon=xgo-Heim\nssid=Heim\nip=192.168.53.110\n
2 wlan connect WPA 0 Mein%20Netz geheim123\n
                                        2 0 48\ncon=xgo-Mein Netz\nssid=Mein Netz\nip=10.10.0.110\n
3 nav POST /mode?name=idle\n            3 200 <n>\n<reply of xgonav>
```

After a successful `wlan connect` the new address also arrives as `IP` notification (within ~3 s). Web interface: `https://<ip>/` (self-signed certificate, no login; port 80 redirects).

Throughput (by design, not measured): Pi → phone ~4.6 kB/s, phone → Pi a little more (UART shared with the 100 Hz IMU stream). Disconnect: `xgo-ble` drops a half-received line; a running `wlan connect` finishes anyway.

## Parts

| Part | File | Role |
|---|---|---|
| Firmware | `firmware/patches/0025-ble-neu.patch` (`ble_transport.cpp`, `pi_tunnel.h`, `main.cpp`) | GATT service; tunnel frames type `0x15` register `0x17` (payload = kind + ≤ 23 bytes; kind 0 data, 1 connected, 2 disconnected); register `0x18` |
| Driver | `kernel_module/` | `/proc/XGORider/ble` (root, blocking read = one record, write = bytes to the client), `settings/ip_dev`, IP report; `serial_stats`: `tunnel_rx`, `tunnel_drop`, `tunnel_tx`, `pi_ip` |
| Pi service | `deploy/xgo-ble/xgo-ble.c`, `deploy/xgo-ble.service` | C program, root, blocks in `read`; runs `xgo-wlan`, talks HTTP to xgonav |
| Wi-Fi helper | `deploy/xgo-wlan` | NetworkManager; shared with buttons C+D and `wlan.php` (`docs/wifi-setup.md`) |
| PC tool | `tools/ble_test.py` (bleak) | check run and manual access to all four characteristics |

Cost on the Pi: idle none (sleeping process, one compare per serial frame, one 12-byte frame every 3 s); per request one process start or one local HTTP request. Nothing in the image pipeline.

## Tests

| Test | Covers |
|---|---|
| `firmware/tests/test_ble_tunnel.cpp` | tunnel frames, chunking, parser for `0x17`/`0x18`, read replies, output buffer, IP timeout, console line buffer |
| `driver-test/run.sh … --tunnel` | driver tunnel next to polling and stream: data from the emulator echoed completely and in order |
| `tests/test_xgo_ble.py` | `xgo-ble` against a fake `xgo-wlan` and a fake xgonav (Linux + gcc only) |
| `python tools/ble_test.py` | on the real Rider: IP, `CMD`, `CON`, `PI` |
