#!/usr/bin/env python3
"""Referenz der Lagefusion (Patch 04-attitude) und Erzeuger der Prüfdaten für test_lagefusion_replay.cpp.

    python firmware/tests/lagefusion_ref.py roh_liegend_60.csv roh_fahrt_163849.csv firmware/tests/data/lagefusion_163849.txt

Eingang: Mitschnitte von tools/rawdiag.py (Register 0x15: Mittel der Rohwerte zwischen zwei Lesungen, ~83 Hz).
Erst der liegende Mitschnitt (Startkalibrierung im Stillstand), dann 1 s künstliches Aufrichten in die Lage, mit
der der Fahrmitschnitt beginnt, dann die Fahrt. Je Zeile entstehen die 12 Rohbytes, wie updateImu sie bekommt
(auf Zählwerte gerundet), und die Lage, die diese Referenz daraus rechnet. Der Host-Test schickt dieselben
Zeilen durch updateImu und verlangt dieselbe Lage.

Die Klasse Fusion rechnet dasselbe wie updateImu (firmware/src/control/rider_leg_imu.cpp), in derselben
Reihenfolge; Änderungen immer in beiden machen.
"""
import csv
import math
import sys

import numpy as np

K_ACC = 19.6 / 32768.0
K_GYRO = 2000.0 / 32768.0
D2R, R2D = 0.017453292519943295, 57.29577951308232
# wie rider_leg_imu.h
kGyroNickGier, kGyroRollGier = -0.0124, 0.0099
kFusionTau, kFusionBiasTau, kBiasMaxError, kBiasLimit = 8.0, 40.0, 0.05, 5.0
kGravity, kAccelTrust, kRateTrust, kSpeedTau = 9.99, 0.5, 60.0, 0.05
kFusionTauSettle, kFusionBiasTauSettle, kBiasSettleTime = 2.0, 4.0, 30.0
kMeterPerTick = 3.14159265358979 * 0.060 / 1024.0
kCalGyroSpan, kCalAccelSpan, kCalMaxTries = 1.5, 0.5, 20
f32 = np.float32


class Fusion:
    def __init__(self):
        self.acc = np.zeros(3, f32); self.gyro = np.zeros(3, f32); self.bias = np.zeros(3, f32)
        self.up = np.array([0, 0, 1], f32); self.up_valid = False
        self.roll = self.pitch = self.yaw = f32(0)
        self.gpf = self.grf = f32(0)
        self.cal = False; self.cal_count = 0; self.cal_tries = 0; self.bias_from_boot = False
        self.gmin = np.zeros(3); self.gmax = np.zeros(3); self.amin = np.zeros(3); self.amax = np.zeros(3)
        self.last_us = 0; self.last_ticks = [0, 0]; self.ticks_seen = False; self.speed = f32(0)
        self.weight = 0.0; self.settle_s = 0.0

    def step(self, raw, now_us, uptime_ms, ticks, fresh):
        """raw: sechs Zählwerte in Sensorreihenfolge AX, AY, AZ, GX, GY, GZ."""
        scale = f32(0.000030517578125)
        def accel(r, sign):
            scaled = f32(f32(r) * f32(sign) * scale)
            g = float(scaled) * 9.8
            return f32(g + g)
        self.acc = np.array([accel(raw[0], -1), accel(raw[1], -1), accel(raw[2], 1)], f32)
        self.gyro = np.array([-f32(raw[4]) * scale * f32(2000), f32(raw[3]) * scale * f32(2000),
                              f32(raw[5]) * scale * f32(2000)], f32)
        if uptime_ms > 750 and not self.cal:
            if self.cal_count > 250:
                still = all(self.gmax[n] - self.gmin[n] < kCalGyroSpan and self.amax[n] - self.amin[n] < kCalAccelSpan
                            for n in range(3))
                if still or self.cal_tries >= kCalMaxTries:
                    self.cal = True; self.bias_from_boot = still
                    self.settle_s = 0.0 if still else kBiasSettleTime
                    if not still:
                        self.bias[0] = self.bias[1] = 0
                else:
                    self.cal_tries += 1; self.cal_count = 0; self.bias[:] = 0
            else:
                if abs(self.roll) < 4.0:
                    for n in range(3):
                        if self.cal_count == 0 or self.gyro[n] < self.gmin[n]: self.gmin[n] = self.gyro[n]
                        if self.cal_count == 0 or self.gyro[n] > self.gmax[n]: self.gmax[n] = self.gyro[n]
                        if self.cal_count == 0 or self.acc[n] < self.amin[n]: self.amin[n] = self.acc[n]
                        if self.cal_count == 0 or self.acc[n] > self.amax[n]: self.amax[n] = self.acc[n]
                    self.cal_count += 1
                    self.bias = (self.bias + self.gyro / f32(250.0)).astype(f32)
                else:
                    self.cal_count = 0; self.bias[:] = 0
        yaw_rate = f32(self.gyro[2] - self.bias[2])
        cal = self.cal
        w = np.array([-(float(self.gyro[1]) - (float(self.bias[1]) if cal else 0.0)) * D2R,
                      (float(self.gyro[0]) - (float(self.bias[0]) if cal else 0.0)) * D2R,
                      float(yaw_rate) * D2R])
        w[0] += kGyroNickGier * w[2]
        w[1] += kGyroRollGier * w[2]
        dt = ((now_us - self.last_us) & 0xFFFFFFFF) * 0.000001
        self.last_us = now_us
        ok_dt = 0.0 < dt < 0.05
        if fresh and self.ticks_seen and ok_dt:
            tk = 0.5 * ((ticks[0] - self.last_ticks[0]) + (ticks[1] - self.last_ticks[1]))
            k = dt / kSpeedTau if dt < kSpeedTau else 1.0
            self.speed = f32(float(self.speed) + (tk * kMeterPerTick / dt - float(self.speed)) * k)
        elif not fresh:
            self.speed = f32(0)
        self.ticks_seen = fresh
        self.last_ticks = list(ticks)
        a = np.array([float(self.acc[0]) - float(self.speed) * w[2], float(self.acc[1]), float(self.acc[2])])
        a_norm = math.sqrt(float(a @ a))
        u = self.up.astype(float)
        if not self.up_valid and a_norm > 1.0:
            u = a / a_norm; self.up_valid = True
        c1 = np.cross(w, u)
        if ok_dt:
            c2 = np.cross(w, c1)
            u = u - c1 * dt + 0.5 * c2 * dt * dt
            n = math.sqrt(float(u @ u))
            if n > 1e-9: u = u / n
        den_p = u[1] * u[1] + u[2] * u[2]; den_r = u[0] * u[0] + u[2] * u[2]
        pitch_gyro = f32((-c1[1] * u[2] + u[1] * c1[2]) / den_p * -R2D) if den_p > 1e-6 else f32(0)
        roll_gyro = f32((-c1[0] * u[2] + u[0] * c1[2]) / den_r * -R2D) if den_r > 1e-6 else f32(0)
        if abs(self.gpf - self.gyro[1]) < 1400.0:
            self.gpf = f32(float(pitch_gyro) * 0.7 + float(self.gpf) * 0.3)
        if abs(self.grf - self.gyro[0]) < 1800.0:
            self.grf = f32(float(roll_gyro) * 0.93 + float(self.grf) * 0.07)
        settle = cal and self.settle_s > 0.0
        if settle and ok_dt:
            self.settle_s = float(f32(f32(self.settle_s) - f32(dt)))
        weight = 0.0
        if a_norm > 1.0 and self.up_valid:
            alpha = 0.05
            if cal:
                am = (a_norm - kGravity) / kAccelTrust
                wm = math.sqrt(float(w @ w)) / (kRateTrust * D2R)
                weight = 1.0 / (1.0 + am * am) / (1.0 + wm * wm)
                alpha = weight * dt / (kFusionTauSettle if settle else kFusionTau) if ok_dt else 0.0
            d = a / a_norm - u
            err2 = float(d @ d)
            d = d * alpha
            if cal and err2 < kBiasMaxError * kBiasMaxError:
                tau_b = kFusionBiasTauSettle if settle else kFusionBiasTau
                bx = (u[1] * d[2] - u[2] * d[1]) / tau_b
                by = (u[2] * d[0] - u[0] * d[2]) / tau_b
                self.bias[1] = f32(min(kBiasLimit, max(-kBiasLimit, float(f32(float(self.bias[1]) - bx * R2D)))))
                self.bias[0] = f32(min(kBiasLimit, max(-kBiasLimit, float(f32(float(self.bias[0]) + by * R2D)))))
            u = u + d
            n = math.sqrt(float(u @ u))
            if n > 1e-9: u = u / n
        self.up = u.astype(f32)
        self.weight = weight
        self.pitch = f32(math.atan(u[1] / u[2]) * 57.2958)
        self.roll = f32(math.atan(u[0] / u[2]) * 57.2958)
        if self.cal and ok_dt:
            self.yaw = f32(float(self.yaw) + float(w @ u) * R2D * dt)
        return self.pitch, self.roll, self.yaw


def zeilen(pfad):
    R = [r for r in csv.DictReader(open(pfad)) if r.get("imu", "").count(";") == 4]
    out = []
    for r in R:
        ax, ay, az = float(r["ax"]), float(r["ay"]), float(r["az"])
        gr, gn, gg = float(r["g_roll"]), float(r["g_nick"]), float(r["g_gier"])
        # Körperachsen -> Sensorwörter: acc = [-AX, -AY, +AZ], gyro = [-GY, +GX, +GZ]
        raw = [round(-ax / K_ACC), round(-ay / K_ACC), round(az / K_ACC), round(gn / K_GYRO), round(-gr / K_GYRO),
               round(gg / K_GYRO)]
        out.append((int(r["t_us"]), raw, int(r["w11"]), int(r["w21"]), int(r["t_ns"])))
    return out


def aufrichten(raw_a, raw_b, bias_raw, sek=1.0, hz=333):
    """Künstliche Proben: Lage dreht in sek gleichmäßig von raw_a nach raw_b, Kreisel passend dazu."""
    def oben(raw):
        v = np.array([-raw[0], -raw[1], raw[2]], float)
        return v / np.linalg.norm(v), np.linalg.norm(v)
    ua, na = oben(raw_a); ub, nb = oben(raw_b)
    ang = math.acos(max(-1.0, min(1.0, float(ua @ ub))))
    n = int(sek * hz); out = []
    for k in range(1, n + 1):
        s = k / n
        u = (math.sin((1 - s) * ang) * ua + math.sin(s * ang) * ub) / math.sin(ang)
        s0 = (k - 1) / n
        u0 = (math.sin((1 - s0) * ang) * ua + math.sin(s0 * ang) * ub) / math.sin(ang)
        udot = (u - u0) * hz
        w = -np.cross(u, udot) * R2D                       # du/dt = -w x u
        g = na + (nb - na) * s
        acc = u * g                                         # Zählwerte, Körperachsen
        g_nick, g_roll = -w[0], w[1]
        raw = [round(-acc[0]), round(-acc[1]), round(acc[2]), round(g_nick / K_GYRO + bias_raw[0]),
               round(-g_roll / K_GYRO + bias_raw[1]), round(bias_raw[2])]
        out.append(raw)
    return out


def bauen(liegend, fahrt):
    L, F = zeilen(liegend), zeilen(fahrt)
    seq = []   # (now_us, raw, w11, w21, fresh, t_ns)
    t = 0
    for i, (t_us, raw, w11, w21, t_ns) in enumerate(L):
        t += 12000 if i == 0 else t_us - L[i - 1][0]
        seq.append((t, raw, 0, 0, 0, 0))
    mittel = lambda Z: [float(np.mean([z[1][k] for z in Z])) for k in range(6)]
    a, b = mittel(L[-80:]), mittel(F[:80])
    bias_raw = [mittel(L)[3], mittel(L)[4], mittel(L)[5]]
    for raw in aufrichten(a, b, bias_raw):
        t += 3000
        seq.append((t, raw, F[0][2], F[0][3], 0, 0))
    for i, (t_us, raw, w11, w21, t_ns) in enumerate(F):
        t += 12000 if i == 0 else t_us - F[i - 1][0]
        seq.append((t, raw, w11, w21, 1, t_ns))
    return seq


def rechnen(seq):
    f = Fusion(); out = []
    for now_us, raw, w11, w21, fresh, t_ns in seq:
        p, r, y = f.step(raw, now_us, now_us // 1000 + 1000, (w11, w21), bool(fresh))
        out.append((float(p), float(r), float(y), float(f.bias[0]), float(f.bias[1]), f.weight, float(f.speed)))
    return out, f


def main():
    liegend, fahrt, ziel = sys.argv[1:4]
    seq = bauen(liegend, fahrt)
    erg, f = rechnen(seq)
    with open(ziel, "w", newline="\n") as o:
        o.write("# now_us AX AY AZ GX GY GZ w11 w21 frisch | nick roll gier (Referenz lagefusion_ref.py)\n")
        for (now_us, raw, w11, w21, fresh, _), e in zip(seq, erg):
            o.write("%d %d %d %d %d %d %d %d %d %d %.4f %.4f %.3f\n" % (now_us, *raw, w11, w21, fresh, e[0], e[1], e[2]))
    print("%d Zeilen -> %s | kalibriert %s, Nullpunkte aus Start %s, am Ende Roll %.3f Nick %.3f °/s" % (
        len(seq), ziel, f.cal, f.bias_from_boot, f.bias[0], f.bias[1]))


if __name__ == "__main__":
    main()
