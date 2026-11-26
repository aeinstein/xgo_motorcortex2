#!/usr/bin/env python3
"""Stufe 6: Texturen der Modellflächen aus den Kamerabildern.

  python textur.py sammeln WELT.json INDEX.json PKT AUS run        # je Fahrt, parallel startbar
  python textur.py bauen WELT.json AUS KARTEN_DIR unten.npz        # Teilergebnisse zusammenführen -> Weltdatei

Je Bildpunkt mit bekanntem Raumpunkt (Punktwolke) wird geprüft, ob er auf einer Fläche des Modells liegt (Abstand zur
Ebene klein); die Farbe landet dort, wo der Sehstrahl die Fläche trifft (schärfer als der geschätzte Punkt). Nahe und
frontal gesehene Punkte zählen stärker. Tag (Start vor 17 Uhr) und Abend getrennt.
Flächen: je sichtbarem Teil vier Seiten (S, O, N, W), Name "<nr>:<seite>"; schräge Flächen "s<nr>".
"""
import json
import math
import os
import sys

import cv2
import numpy as np

TS = 50                 # Texel je Meter (2 cm)
OFF = (1.0, 5.0)        # Welt = gemeinsamer Bezug + OFF (plan.py --ursprung=-1,-5)
RMAX = 4.0


def flaechen(welt):
    """Liste (name, ax, ay, bx, by, z0, z1): von außen gesehen läuft a -> b von links nach rechts."""
    out = []
    for i, t in enumerate(welt["teile"]):
        if t.get("tex") != "foto" or not t.get("sichtbar", True):
            continue
        x1, y1, x2, y2 = t["x1"], t["y1"], t["x2"], t["y2"]
        for s, (a, b, c, d) in zip("SONW", ((x1, y1, x2, y1), (x2, y1, x2, y2), (x2, y2, x1, y2), (x1, y2, x1, y1))):
            out.append(("%d:%s" % (i, s), a, b, c, d, t["z0"], t["z1"]))
    for i, t in enumerate(welt.get("schraeg", [])):
        out.append(("s%d" % i, t["x1"], t["y1"], t["x2"], t["y2"], t["z0"], t["z1"]))
    return out


def sammeln(welt_pf, index_pf, pkt, aus, run, data=os.path.expanduser("~/floornet/data/rider/fahrt")):
    """Je Fläche und Texel die Farbe aus dem besten Einzelbild: Ein Bild, das die Fläche breit und frontal zeigt, schlägt
    alle anderen an jedem Texel, den es trifft (zusammenhängende Stücke statt Brei aus versetzten Bildern)."""
    welt = json.load(open(welt_pf))
    F = flaechen(welt)
    geo = []
    for name, ax, ay, bx, by, z0, z1 in F:
        L = math.hypot(bx - ax, by - ay)
        ex, ey = (bx - ax) / L, (by - ay) / L
        nx, ny = ey, -ex                         # rechte Normale = zur Sichtseite (Quader von außen)
        w, h = max(1, int(round(L * TS))), max(1, int(round((z1 - z0) * TS)))
        geo.append((name, ax, ay, ex, ey, nx, ny, L, z0, z1, w, h))
    wert = {g[0]: np.zeros(g[11] * g[10], np.float32) for g in geo}
    farbe = {g[0]: np.zeros((g[11] * g[10], 3), np.uint8) for g in geo}
    I = [e for e in json.load(open(index_pf)) if e[0] == run]
    liste = {e["idx"]: e for e in json.load(open(os.path.join(pkt, run, "liste.json")))}
    for _, idx, x0, y0, th in I:
        x0, y0 = x0 + OFF[0], y0 + OFF[1]
        z = np.load(os.path.join(pkt, run, "%06d.npz" % idx))
        Ph = z["xyz"].astype(np.float32)
        rh = np.linalg.norm(Ph - np.array([0.0, 0.0, 0.12], np.float32), axis=-1)
        kante = np.zeros(rh.shape, np.uint8)
        with np.errstate(invalid="ignore"):
            gx = np.abs(rh[:, 1:] - rh[:, :-1]) > 0.05 * rh[:, 1:]
            gy = np.abs(rh[1:] - rh[:-1]) > 0.05 * rh[1:]
        kante[:, 1:] |= gx
        kante[:, :-1] |= gx
        kante[1:] |= gy
        kante[:-1] |= gy
        bild = cv2.imread(os.path.join(data, run, "%06d.jpg" % idx))
        if bild is None:
            continue
        Hb, Wb = bild.shape[:2]
        P = cv2.resize(Ph, (Wb, Hb), interpolation=cv2.INTER_LINEAR)
        kf = cv2.resize(kante, (Wb, Hb), interpolation=cv2.INTER_NEAREST) > 0
        rr = np.linalg.norm(P - np.array([0.0, 0.0, 0.12], np.float32), axis=-1)
        ok = np.isfinite(P).all(-1) & (rr < RMAX) & ~kf & (P[..., 2] > 0.02)
        Q = P[ok]
        C = bild[ok]
        if len(Q) < 800:
            continue
        c, s = math.cos(th), math.sin(th)
        cam_h = liste[idx].get("cam_h") or 0.121
        cx, cy = x0 + c * 0.036, y0 + s * 0.036
        wx = x0 + c * Q[:, 0] - s * Q[:, 1]
        wy = y0 + s * Q[:, 0] + c * Q[:, 1]
        wz = Q[:, 2]
        dx, dy, dz = wx - cx, wy - cy, wz - cam_h
        rng = np.sqrt(dx * dx + dy * dy + dz * dz)
        for name, ax, ay, ex, ey, nx, ny, L, z0, z1, w, h in geo:
            seite = (cx - ax) * nx + (cy - ay) * ny
            if seite < 0.05:
                continue
            tpar = min(max((cx - ax) * ex + (cy - ay) * ey, 0.0), L)
            if math.hypot(cx - ax - tpar * ex, cy - ay - tpar * ey) > RMAX:
                continue
            d = (wx - ax) * nx + (wy - ay) * ny
            nah = np.abs(d) < 0.12 + 0.05 * rng
            if nah.sum() < 120:
                continue
            den = dx[nah] * nx + dy[nah] * ny
            gut = den < -1e-3
            if not gut.any():
                continue
            k = np.where(nah)[0][gut]
            t = -seite / den[gut]
            X, Y, Zz = cx + t * dx[k], cy + t * dy[k], cam_h + t * dz[k]
            u = (X - ax) * ex + (Y - ay) * ey
            v = Zz - z0
            m = (u >= 0) & (u < L) & (v >= 0) & (v < z1 - z0) & (t > 0.5) & (t < 1.6)
            if m.sum() < 80:
                continue
            k = k[m]
            ui = np.minimum((u[m] * TS).astype(np.int64), w - 1)
            vi = np.minimum((v[m] * TS).astype(np.int64), h - 1)
            flat = (h - 1 - vi) * w + ui                       # Zeile 0 = oben
            cosw = float(np.mean(-den[gut][m] / rng[k]))
            texel = len(np.unique(flat))
            # Bildwert: viele getroffene Texel, frontal, nicht zu weit (ab 2 m werden die Farben unscharf)
            sc = texel * cosw * cosw / (1.0 + max(0.0, float(np.mean(rng[k])) - 1.5))
            W_, F_ = wert[name], farbe[name]
            besser = sc > W_[flat]
            if besser.any():
                fb = flat[besser]
                F_[fb] = C[k[besser]]
                W_[fb] = sc
    os.makedirs(aus, exist_ok=True)
    out = {}
    for g in geo:
        n, w, h = g[0], g[10], g[11]
        if wert[n].any():
            out[n + "|w"] = wert[n].reshape(h, w)
            out[n + "|f"] = farbe[n].reshape(h, w, 3)
    np.savez_compressed(os.path.join(aus, run + ".npz"), **out)
    print(run, "Flächen mit Farbe:", len(out) // 2, "von", len(geo), flush=True)


def fuellen(img, hat):
    """Löcher schließen: erst mit dem nächsten bekannten Texel, große Flächen bleiben erkennbar glatt."""
    if hat.all():
        return img
    _, lab = cv2.distanceTransformWithLabels((~hat).astype(np.uint8), cv2.DIST_L2, 3, labelType=cv2.DIST_LABEL_PIXEL)
    ys, xs = np.where(hat)
    # Label -> Quelle: distanceTransformWithLabels nummeriert die Nullpixel (hier: bekannte Texel) fortlaufend
    quelle = np.zeros((lab.max() + 1, 2), np.int64)
    quelle[lab[ys, xs]] = np.stack([ys, xs], 1)
    q = quelle[lab]
    out = img[q[..., 0], q[..., 1]]
    return cv2.GaussianBlur(out, (0, 0), 1.0) * (~hat)[..., None] + img * hat[..., None]


def bauen(welt_pf, aus, karten, ziel):
    welt = json.load(open(welt_pf))
    F = flaechen(welt)
    best = {"tag": {}, "abend": {}}
    for f in sorted(os.listdir(aus)):
        if not f.endswith(".npz") or not f[0] == "2":
            continue
        B = best["tag" if int(f[9:11]) < 17 else "abend"]
        z = np.load(os.path.join(aus, f))
        for k in z.files:
            if not k.endswith("|w"):
                continue
            n = k[:-2]
            w_, f_ = z[k], z[n + "|f"]
            if n not in B:
                B[n] = [w_.copy(), f_.copy()]
            else:
                m = w_ > B[n][0]
                B[n][0][m] = w_[m]
                B[n][1][m] = f_[m]
    daten = {"modell": np.frombuffer(json.dumps(welt).encode("utf-8"), np.uint8)}
    bericht = []
    for licht in ("tag", "abend"):
        S = best[licht]
        n_ok = 0
        for name, ax, ay, bx, by, z0, z1 in F:
            if name not in S:
                continue
            hat = S[name][0] > 0
            if hat.mean() < 0.12 or hat.sum() < 40:
                continue
            img = S[name][1].astype(np.float64)
            # kleine Lücken mit dem Nachbarn, große mit der mittleren Farbe der Fläche (kein Schlierenbild)
            dist = cv2.distanceTransform((~hat).astype(np.uint8), cv2.DIST_L2, 3)
            voll = fuellen(img, hat)
            mittel = np.median(img[hat], axis=0)
            g = np.clip((dist - 6) / 10.0, 0, 1)[..., None]
            img = np.clip(voll * (1 - g) + mittel * g, 0, 255).astype(np.uint8)
            ok, buf = cv2.imencode(".png", img)
            daten["%s/%s" % (licht, name)] = buf
            n_ok += 1
            bericht.append((licht, name, round(float(hat.mean()), 2)))
        # Boden: Farbmosaik der Karten (2 cm), in Weltkoordinaten verschoben, Löcher gefüllt
        kz = np.load(os.path.join(karten, ("sonne" if licht == "tag" else "abend") + ".npz"))
        bn, bod = kz["bn"], kz["boden"]
        n = bn.shape[0]
        sx, sy = int(welt["size"] / 0.02), int(welt["size_y"] / 0.02)
        ox, oy = n // 2 - int(OFF[0] / 0.02), n // 2 - int(OFF[1] / 0.02)
        teil = bod[oy:oy + sy, ox:ox + sx].astype(np.float64)
        hat = (bn[oy:oy + sy, ox:ox + sx] > 6) & (kz["tn"][oy:oy + sy, ox:ox + sx] < 0.3 * (bn[oy:oy + sy, ox:ox + sx] + 1))
        teil = np.clip(fuellen(teil, hat), 0, 255).astype(np.uint8)
        daten["%s/boden" % licht] = cv2.imencode(".png", teil)[1]
        print(licht, "Texturen:", n_ok, "von", len(F), "| Boden bekannt %.0f %%" % (100 * hat.mean()))
    np.savez_compressed(ziel, **daten)
    json.dump(bericht, open(os.path.join(aus, "bericht.json"), "w"))
    print(ziel, "%.0f kB" % (os.path.getsize(ziel) / 1024))


if __name__ == "__main__":
    if sys.argv[1] == "sammeln":
        sammeln(*sys.argv[2:7])
    else:
        bauen(*sys.argv[2:6])
