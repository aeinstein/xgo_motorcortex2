"""ESP32-Emulator am PTY: beantwortet XGO-Lesebefehle mit 115200-Baud-Timing und Fehlern.

python3 esp32_emu.py <harness> <sekunden> <imu_hz> [--block] [--faults]
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
    if addr == 0x62 and n == 12:
        return (f(r) + f(p) + f(y)) if BLOCK else f(r)   # ohne Block-Support: nur 4 Byte
    table = {0x62: f(r), 0x63: f(p), 0x64: f(y), 0x01: bytes([87]), 0x02: bytes([1]),
             0x68: struct.pack(">h", int(y)), 0x07: b"R-1.4.2\0\0\0"}
    d = table.get(addr, bytes(n))
    return d[:n] if len(d) >= n else d + bytes(n - len(d))


master, slave = pty.openpty()
name = os.ttyname(slave)
stats = {"req": 0, "write": 0, "drop": 0, "corrupt": 0, "garbage": 0}
stop = False


def serve():
    buf = bytearray()
    while not stop:
        try:
            chunk = os.read(master, 256)
        except OSError:
            return
        buf += chunk
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
                continue
            if mode != 0x02:
                continue
            n = fr[5]
            stats["req"] += 1
            t = time.monotonic()
            data = payload(addr, n, t)
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
                os.write(master, out[:k]); time.sleep(0.0005); os.write(master, out[k:])
            else:
                os.write(master, out)


th = threading.Thread(target=serve, daemon=True)
th.start()
p = subprocess.run([harness, name, secs, hz], capture_output=True, text=True, timeout=float(secs) + 30)
stop = True
lines = p.stdout.splitlines()
samples = [tuple(map(int, l.split()[1:])) for l in lines if l.startswith("S ")]
for l in lines:
    if not l.startswith("S "):
        print(l)
print("EMU", stats)
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
print("RC", p.returncode, p.stderr[-500:] if p.stderr else "")
