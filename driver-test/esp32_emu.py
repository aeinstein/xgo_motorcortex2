"""ESP32-Emulator am PTY: beantwortet XGO-Lesebefehle mit 115200-Baud-Timing und Fehlern.

python3 esp32_emu.py <harness> <sekunden> <imu_hz> [--faults] [--wheels --combo] [--stream[=HZ]|--stream-off]

--imu65: Register 0x65 (R-1.1.6: accel + roll/pitch/yaw, 24 Byte); --wheels: wird angenommen, wirkt aber nicht mehr
(0x0c beantwortet der Emulator wie die Firmware nicht); --combo: 0x0d (roll/pitch/yaw + Rad-Ticks + t_ms, 24 Byte).
--stream[=HZ]: Firmware-Patch 02-pi-interface. Der Emulator versteht das Schreiben auf 0x11 (Rate) und sendet dann selbst Pakete
Typ 0x14 (seq, t_us der Messung nach eigener, driftender ESP-Uhr, Winkel, Ticks) mit zufälligem Sendeverzug
(Firmware-loop); aus nach 5 s ohne Byte vom Host. Das Testprogramm bekommt HZ (Standard 100) als Strom-Rate.
--stream-off: wie --stream, aber die Firmware kennt 0x11 nicht (Rückfall auf Abfrage prüfen).
--vel: Firmware-Patch 08-velocity. Der Emulator nimmt Fahrrahmen (0x1a, 13 Byte) und Grenzen (0x1b, 14 Byte) an, prüft Länge und
Prüfsumme und beantwortet 0x1a lesen mit dem Zustand des letzten Rahmens; am Ende vergleicht er die empfangenen Bytes
mit denen, die das Testprogramm laut VELSENT/VELLIM geschickt hat, und dessen VELDIAG mit dem erwarteten Text.
--tunnel: Firmware-Patch 01-ble. Der Emulator schickt wie ein BLE-Client Daten durch den Tunnel (Rahmen Typ 0x15, Register
0x17, Kennung + höchstens 23 Byte, davor die Meldung "verbunden"); das Testprogramm schickt sie zurück (Schreiben auf
0x17). Geprüft wird, dass alles vollständig und in der Reihenfolge zurückkommt, neben Abfragen und Strom. Dazu wird
die auf 0x18 geschriebene IP des Pi ausgegeben.
Ohne diese Schalter antwortet der Emulator auf 0x65/0x0d gar nicht (wie R-1.1.3), auf 0x0c nie; der Treiber nutzt seit
2026-11-17 nur noch 0x0d und den Strom, --block und --imu65 prüfen nur noch, dass er sie nicht fälschlich annimmt.
"""
import math
import os
import pty
import random
import struct
import subprocess
import sys
import threading
import time

BYTE_S = 10 / 115200          # 8N1
args = sys.argv[1:]
harness, secs, hz = args[0], args[1], args[2]
BLOCK = "--block" in args
FAULTS = "--faults" in args
IMU65 = "--imu65" in args
COMBO = "--combo" in args
TUNNEL = "--tunnel" in args
VEL_CAPS = 3                # Fähigkeitsbyte (0x1a lesen, Byte 1): bit0 Krümmungsbetrieb, bit1 Kursziel
VEL = "--vel" in args   # Firmware-Patch 08-velocity: Fahrrahmen 0x1a (13 Byte) / Grenzen 0x1b (14 Byte), 0x1a lesen = Zustand
_st = [a for a in args if a.startswith("--stream")]
STREAM = bool(_st) and "--stream-off" not in args
STREAM_HZ = int(_st[0].split("=")[1]) if _st and "=" in _st[0] else 100
T0 = time.monotonic()
ESP_T0, ESP_PPM = T0 - 12.345, 40e-6       # ESP-Uhr: anderer Nullpunkt, 40 ppm schnell
TPS11, TPS21 = 400.0, -250.0     # Rad-Ticks/s (rechts rückwärts, damit das Vorzeichen geprüft wird)


def wheels_truth(t):
    return int(TPS11 * t) & 0xFFFFFFFF, int(TPS21 * t) & 0xFFFFFFFF
rng = random.Random(4)


def truth(t):  # Grad, t in s (CLOCK_MONOTONIC)
    return (2.0 * math.sin(2 * math.pi * 0.7 * t),
            5.0 * math.sin(2 * math.pi * 2.5 * t) + 1.5 * math.sin(2 * math.pi * 6.1 * t),
            (40.0 * t) % 360 - 180)


def frame(addr, data):
    ln = len(data) + 8
    chk = 255 - ((ln + 0x12 + addr + sum(data)) % 256)
    return bytes([0x55, 0x00, ln, 0x12, addr]) + bytes(data) + bytes([chk, 0x00, 0xAA])


def payload(addr, n, t):
    r, p, y = truth(t)
    f = lambda v: struct.pack("<f", v)
    if addr == 0x65:
        if not IMU65:
            return None
        return (f(0.1) + f(-0.2) + f(9.81) + f(r) + f(p) + f(y))[:n]
    if addr == 0x0d:
        if not COMBO:
            return None
        a, b = wheels_truth(t)
        return (f(r) + f(p) + f(y) + struct.pack("<III", a, b, int((t - T0) * 1000) & 0xFFFFFFFF))[:n]
    if addr == 0x1a and VEL:                # Zustand wie die Firmware (velocity_cmd.h fillDiag), ohne Rampe
        if not vel_frames:
            return bytes([8 if vel_limits else 0, VEL_CAPS]) + bytes(10)
        fl = vel_frames[-1]
        lim = vel_limits[-1] if vel_limits else bytes(6)
        flags = 1 | (2 if fl[0] & 1 else 0) | (8 if vel_limits else 0) | (16 if fl[0] & 2 else 0) | (32 if fl[0] & 4 else 0)
        return bytes([flags, VEL_CAPS]) + fl[1:3] + fl[7:9] + lim[0:6]
    if addr == 0x62 and n == 12:
        return (f(r) + f(p) + f(y)) if BLOCK else f(r)   # ohne Block-Support: nur 4 Byte
    table = {0x62: f(r), 0x63: f(p), 0x64: f(y), 0x01: bytes([87]), 0x02: bytes([1]),
             0x68: struct.pack(">h", int(y)), 0x07: b"R-1.4.2\0\0\0"}
    d = table.get(addr, bytes(n))
    return d[:n] if len(d) >= n else d + bytes(n - len(d))


master, slave = pty.openpty()
name = os.ttyname(slave)
stats = {"req": 0, "write": 0, "drop": 0, "corrupt": 0, "garbage": 0, "stream": 0, "stream_on": 0}
stop = False
wlock = threading.Lock()                  # Antworten und Strom-Pakete nicht ineinander schreiben
stream_rate = 0
last_rx = time.monotonic()
vel_frames, vel_limits = [], []          # --vel: empfangene Nutzdaten 0x1a / 0x1b


def send(out):
    with wlock:
        os.write(master, out)


def esp_us(t):
    return int((t - ESP_T0) * (1 + ESP_PPM) * 1e6) & 0xFFFFFFFF


def stream_frame(seq, t):
    r, p, y = truth(t)
    a, b = wheels_truth(t)
    data = struct.pack("<HIfffII", seq & 0xFFFF, esp_us(t), r, p, y, a, b)
    ln = len(data) + 8
    chk = 255 - ((ln + 0x14 + 0x11 + sum(data)) % 256)
    return bytes([0x55, 0x00, ln, 0x14, 0x11]) + data + bytes([chk, 0x00, 0xAA])


tunnel_sent = bytearray()
tunnel_back = bytearray()
pi_ip = None


def tunnel_frame(kind, data=b""):
    data = bytes([kind]) + bytes(data)
    ln = len(data) + 8
    chk = 255 - ((ln + 0x15 + 0x17 + sum(data)) % 256)
    return bytes([0x55, 0x00, ln, 0x15, 0x17]) + data + bytes([chk, 0x00, 0xAA])


def tunneler():
    """Wie die Firmware: Meldung verbunden, dann alle 0,4 s eine Anfrage in Stücken zu höchstens 23 Byte."""
    trng = random.Random(7)
    time.sleep(1.0)
    send(tunnel_frame(1))
    n = 0
    while not stop and time.monotonic() - T0 < float(secs) - 1.5:
        n += 1
        msg = ("%d wlan connect WPA 0 %s %s\n" % (n, "netz%%20" * trng.randrange(1, 5), "x" * trng.randrange(8, 64))).encode()
        tunnel_sent.extend(msg)
        for i in range(0, len(msg), 23):
            out = tunnel_frame(0, msg[i:i + 23])
            time.sleep(len(out) * BYTE_S)
            send(out)
        time.sleep(0.4)
    send(tunnel_frame(2))


def streamer():
    seq, nxt = 0, time.monotonic()
    while not stop:
        if not stream_rate or time.monotonic() - last_rx > 5.0:
            time.sleep(0.005)
            nxt = time.monotonic()
            continue
        nxt += 1.0 / stream_rate
        time.sleep(max(0.0, nxt - time.monotonic()))
        t = time.monotonic()                      # Messzeit
        seq += 1
        out = stream_frame(seq, t)
        time.sleep(rng.uniform(0.0, 0.008))       # Firmware-loop sendet später
        time.sleep(len(out) * BYTE_S)              # Übertragung: erst danach ist das Paket beim Pi vollständig
        if FAULTS and rng.random() < 0.02:        # Paket verworfen (UART-FIFO voll)
            continue
        send(out)
        stats["stream"] += 1


def serve():
    buf = bytearray()
    while not stop:
        try:
            chunk = os.read(master, 256)
        except OSError:
            return
        buf += chunk
        global last_rx, stream_rate
        if chunk:
            last_rx = time.monotonic()
        while True:
            i = buf.find(b"\x55\x00")
            if i < 0 or len(buf) < i + 3:
                break
            ln = buf[i + 2]
            if len(buf) < i + ln:
                break
            fr, buf = bytes(buf[i:i + ln]), buf[i + ln:]
            mode, addr = fr[3], fr[4]
            if mode == 0x01:
                stats["write"] += 1
                if addr == 0x11 and STREAM and len(fr) >= 6:
                    if fr[5] and not stream_rate:
                        stats["stream_on"] += 1
                    stream_rate = fr[5]
                if addr == 0x17 and len(fr) >= 9 and fr[5] == 0:
                    tunnel_back.extend(fr[6:ln - 3])
                if VEL and addr in (0x1a, 0x1b):
                    data = fr[5:ln - 3]
                    ok = fr[ln - 3] == 255 - ((ln + mode + addr + sum(data)) % 256)
                    want = (13,) if addr == 0x1a else (14,)   # 0x1b: 14 Byte
                    if not ok or len(data) not in want:
                        stats["vel_bad"] = stats.get("vel_bad", 0) + 1
                    else:
                        (vel_frames if addr == 0x1a else vel_limits).append(bytes(data))
                if addr == 0x18 and ln == 12:
                    global pi_ip
                    pi_ip = "%d.%d.%d.%d" % tuple(fr[5:9])
                continue
            if mode != 0x02:
                continue
            n = fr[5]
            stats["req"] += 1
            t = time.monotonic()
            data = payload(addr, n, t)
            if data is None:                    # Register gibt es nicht: keine Antwort
                continue
            out = frame(addr, data)
            time.sleep((9 + len(out)) * BYTE_S + 0.0003)  # Übertragung + Verarbeitung
            if FAULTS:
                x = rng.random()
                if x < 0.02:
                    stats["drop"] += 1
                    continue
                if x < 0.03:
                    stats["corrupt"] += 1
                    out = out[:-3] + bytes([out[-3] ^ 0x5A]) + out[-2:]
                elif x < 0.05:
                    stats["garbage"] += 1
                    out = bytes(rng.randrange(256) for _ in range(rng.randrange(1, 6))) + out
            if FAULTS and rng.random() < 0.1:   # Antwort in zwei Teilen
                k = rng.randrange(1, len(out))
                with wlock:
                    os.write(master, out[:k]); time.sleep(0.0005); os.write(master, out[k:])
            else:
                send(out)


th = threading.Thread(target=serve, daemon=True)
th.start()
threading.Thread(target=streamer, daemon=True).start()
if TUNNEL:
    threading.Thread(target=tunneler, daemon=True).start()
hargs = [harness, name, secs, hz] + ([str(STREAM_HZ)] if _st else [])
henv = dict(os.environ, XGO_TEST_VEL="1") if VEL else None
p = subprocess.run(hargs, capture_output=True, text=True, timeout=float(secs) + 30, env=henv)
stop = True
lines = p.stdout.splitlines()
samples = [tuple(map(int, l.split()[1:])) for l in lines if l.startswith("S ")]
wheels = [tuple(map(int, l.split()[1:])) for l in lines if l.startswith("W ")]
for l in lines:
    if not l.startswith(("S ", "W ")):
        print(l)
if wheels:
    wd = (wheels[-1][1] - wheels[0][1]) / 1e9
    wgaps = sum(1 for a, b in zip(wheels, wheels[1:]) if b[0] != a[0] + 1)
    # Ticks gegen die Wahrheit zum Zeitstempel (Mitte der Anfrage): Abweichung in Ticks
    wrap = lambda d: (d + 2**31) % 2**32 - 2**31          # Ticks laufen über (int32)
    e = sorted(max(abs(wrap(t11 - int(TPS11 * t / 1e9))), abs(wrap(t21 - int(TPS21 * t / 1e9))))
               for _, t, t11, t21, *_ in wheels)
    print("WHEELRATE %.1f Hz (%d Proben, %d Lücken), Abweichung median %d p95 %d max %d Ticks"
          % ((len(wheels) - 1) / wd if wd > 0 else 0, len(wheels), wgaps, e[len(e) // 2], e[int(len(e) * .95)], e[-1]))
print("EMU", stats)
if VEL:
    sent = [bytes.fromhex(l.split()[1]) for l in lines if l.startswith("VELSENT ")]
    lsent = [bytes.fromhex(l.split()[1]) for l in lines if l.startswith("VELLIM ")]
    diag = [l[len("VELDIAG "):] for l in lines if l.startswith("VELDIAG ")]
    want_diag = lines and next((l[len("VELWANT "):] for l in lines if l.startswith("VELWANT ")), None)
    ok = (sent and sent == vel_frames and lsent == vel_limits and not stats.get("vel_bad")
          and diag and diag[-1] == want_diag)
    print("VELTEST %s: %d Rahmen, %d Grenzen angekommen (gesendet %d/%d), Zustand '%s' (soll '%s')"
          % ("ok" if ok else "FEHLER", len(vel_frames), len(vel_limits), len(sent), len(lsent),
             diag[-1] if diag else "-", want_diag))
if TUNNEL:
    print("TUNNELTEST %s: %d Byte gesendet, %d zurück; IP des Pi %s"
          % ("ok" if tunnel_sent and bytes(tunnel_back) == bytes(tunnel_sent) else "FEHLER", len(tunnel_sent), len(tunnel_back), pi_ip))
if samples:
    ts = [s[1] for s in samples]
    dur = (ts[-1] - ts[0]) / 1e9
    seqs = [s[0] for s in samples]
    gaps = sum(1 for a, b in zip(seqs, seqs[1:]) if b != a + 1)
    errs = {k: [] for k in ("roll", "pitch", "yaw")}
    for seq, t, r, pi, y in samples:
        tr, tp, ty = truth(t / 1e9)
        errs["roll"].append(abs(r / 1000 - tr))
        errs["pitch"].append(abs(pi / 1000 - tp))
        dy = abs((y / 1000 - ty + 180) % 360 - 180)
        errs["yaw"].append(dy)
    print("RATE %.1f Hz (%d Proben in %.1f s, %d Lücken in seq)" % ((len(samples) - 1) / dur, len(samples), dur, gaps))
    for k, v in errs.items():
        v = sorted(v)
        print("ERR %-5s median %.3f° p95 %.3f° max %.3f°" % (k, v[len(v) // 2], v[int(len(v) * .95)], v[-1]))
    # Zeitfehler der Stempel: Nick-Abweichung / Nickrate, nur wo die Rate groß ist (> 40°/s); ab 2 s (Versatz eingeschwungen)
    terr = []
    for seq, t, r, pi, y in samples:
        ts_ = t / 1e9
        if ts_ - ts[0] / 1e9 < 2.0:
            continue
        rate = (truth(ts_ + 1e-4)[1] - truth(ts_ - 1e-4)[1]) / 2e-4
        if abs(rate) > 40:
            terr.append((pi / 1000 - truth(ts_)[1]) / rate * 1000)   # ms, + = Stempel zu früh
    if terr:
        a = sorted(terr)
        print("TIMEERR median %+.2f ms p5 %+.2f p95 %+.2f (n %d)" % (a[len(a) // 2], a[int(len(a) * .05)], a[int(len(a) * .95)], len(a)))
print("RC", p.returncode, p.stderr[-500:] if p.stderr else "")
