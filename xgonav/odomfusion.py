"""Koppelnavigation: ein Kurs und eine Strecke je Regelschritt aus IMU, Rädern und Bild.

Verhalten, Merkkarte und Raumkarte bewegen sich alle mit diesem einen Ergebnis (vorher drei Quellen:
Verhalten Räder, Karten IMU). Beide Sensoren werden über dieselbe Zeitspanne verglichen (IMU-Kurs und
Radwege zur Bildzeit + [imu] time_offset_ms), so dient jeder dem anderen als Rückfall:

  Kurs     IMU, solange frisch und nicht im Zweifel; sonst Räder; sonst Bild (Ego-Motion).
  Strecke  Räder (Mittel beider); bei Schlupf das Rad, das zur IMU-Drehung passt und weniger zurücklegt;
           ohne Räder Fahrbefehl x vx_limit.

Widerspruch: Weicht der Radkurs über [odom] contra_window_s um mehr als contra_deg + contra_frac x Drehung
vom IMU-Kurs ab, hat einer unrecht. Ohne dritten Zeugen gilt das als Schlupf (Räder rutschen beim Drehen,
RiderPi 2026-11-06 bis 37° Unterschied, IMU lag richtig). Das Bild ist Zeuge, solange der Widerspruch besteht
und sein möglicher Fehler (cam_err_deg + cam_err_deg_m je gefahrenem Meter; bei Vorwärtsfahrt verfälscht,
2026-11-04: −8,9° auf 2 m) unter einem Viertel des Widerspruchs liegt. Liegt sein Kurs klar bei den Rädern, ist
die IMU imu_doubt_s lang zweifelhaft, der Kurs kommt aus den Rädern, und was die IMU im Fenster falsch gedreht
hat, wird nachgeholt (Rohfahrten: IMU-Kurs schlägt beim Umkippen um 40–140° aus, Räder und Bild ruhig).
Erkennbare Fehler fängt die Quelle selbst ab: IMU-Kurssprünge und Lücken (imu.py, yaw_jumps), Ticksprünge
und ESP-Neustarts (wheels.py), Alter (beide).

Nebenbei: Vorschlag für scale_left/scale_right/track_m aus Abschnitten ohne Schlupf gegen den IMU-Kurs
(wheels.calibrate_odom), nur zur Anzeige.

Kosten am Pi: je Regelschritt (24 Hz) einige Gleitkommaoperationen und ein kurzer Deque-Durchlauf
(< 0,05 ms); alle 10 s eine Ausgleichsrechnung über höchstens calib_n x 3 Werte (< 0,1 ms).
"""
from __future__ import annotations

import logging
import math
from collections import deque
from dataclasses import dataclass

from .config import OdomCfg
from .wheels import calibrate_odom

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class Schritt:
    ds: float                  # Strecke der Radmitte (m, vorwärts +)
    dyaw: float                # Drehung (rad, links +)
    kurs_src: str              # "imu", "räder", "kamera", "keiner"
    ds_src: str                # "räder", "schlupf" (ein Rad, IMU-Drehung), "befehl"


class OdomFusion:
    CALIB_EVERY_S = 10.0

    def __init__(self, cfg: OdomCfg):
        self.cfg = cfg
        self._t: float | None = None
        self._y: float | None = None                 # IMU-Kurs beim letzten Schritt (rad, entfaltet)
        self._s: tuple[float, float] | None = None   # Radwege beim letzten Schritt (m, mit Faktor)
        self._jumps = 0
        self._win: deque[tuple[float, float, float, float | None, float]] = deque()  # (t, di, dw, dv, |ds|)
        self.schlupf = False
        self._zweifel_bis = -math.inf
        self._nachholen = 0.0                        # Kurskorrektur nach einem Urteil der Kamera (rad)
        self.widersprueche = 0                       # Ereignisse (Übergang ok -> Widerspruch)
        self.imu_zweifel = 0                         # davon: Kamera gab den Rädern recht
        self.abw = 0.0                               # Radkurs minus IMU-Kurs im Fenster (rad)
        self.letzter: Schritt | None = None
        # Kalibriervorschlag
        self._seg = [0.0, 0.0, 0.0, 0.0]             # sl roh, sr roh, Drehung IMU, Dauer
        self._segs: deque[tuple[float, float, float]] = deque(maxlen=max(1, cfg.calib_n))
        self._calib_t = None
        self.vorschlag: dict | None = None

    def zweifel(self, t: float) -> bool:
        return t < self._zweifel_bis

    def step(self, t: float, yaw_imu: float | None, imu_jumps: int, wheels_s: tuple[float, float] | None,
             dyaw_vis: float | None, ds_cmd: float, balancing: bool = True) -> Schritt:
        """t: Zeit des Schritts (IMU-Zeit); yaw_imu: IMU-Kurs zu t (None = fehlt/zu alt); imu_jumps: Zähler
        verworfener Kurssprünge (ändert er sich, gilt die IMU-Drehung dieses Schritts nicht); wheels_s: Radwege
        zu t (None = keine frischen Raddaten); dyaw_vis: Bild-Kurs seit dem letzten Schritt (None = keiner);
        ds_cmd: Strecke laut Fahrbefehl (Rückfall)."""
        c = self.cfg
        di = None
        if yaw_imu is not None and self._y is not None and imu_jumps == self._jumps:
            di = yaw_imu - self._y
        self._y, self._jumps = yaw_imu, imu_jumps
        dl = dr = dw = dsw = None
        if wheels_s is not None and self._s is not None:
            dl, dr = wheels_s[0] - self._s[0], wheels_s[1] - self._s[1]
            dw, dsw = (dr - dl) / c.track_m, (dl + dr) / 2.0
        elif wheels_s is not None:
            dsw = 0.0                                 # erste Probe: Bezug, noch kein Weg
        self._s = wheels_s
        dt = t - self._t if self._t is not None else 0.0
        self._t = t

        self._pruefe(t, di, dw, dyaw_vis, dsw)
        zweifel = self.zweifel(t)

        if di is not None and not zweifel:
            dyaw, ksrc = di, "imu"
        elif dw is not None:
            dyaw, ksrc = dw, "räder"
        elif di is not None:
            dyaw, ksrc = di, "imu"
        elif dyaw_vis is not None:
            dyaw, ksrc = dyaw_vis, "kamera"
        else:
            dyaw, ksrc = 0.0, "keiner"

        if self._nachholen:
            dyaw, self._nachholen = dyaw + self._nachholen, 0.0

        if dsw is None:
            ds, dsrc = ds_cmd, "befehl"
        elif self.schlupf and ksrc == "imu" and dl is not None:
            # Differentialantrieb: Mitte = links + Spur/2 x Drehung = rechts − Spur/2 x Drehung. Das durchdrehende
            # Rad legt zu viel zurück, also das kleinere.
            cl, cr = dl + c.track_m / 2 * dyaw, dr - c.track_m / 2 * dyaw
            ds, dsrc = (cl if abs(cl) < abs(cr) else cr), "schlupf"
        else:
            ds, dsrc = dsw, "räder"

        if c.calib_seg_s > 0:
            self._kalib(t, dt, di, dl, dr, balancing and not zweifel and not self.schlupf)
        self.letzter = Schritt(ds, dyaw, ksrc, dsrc)
        return self.letzter

    def _pruefe(self, t: float, di, dw, dv, dsw) -> None:
        c, w = self.cfg, self._win
        if di is None or dw is None:
            # Ohne beide Kurse kein Vergleich; ein Fenster mit Lücke hieße Äpfel mit Birnen
            w.clear()
            self.schlupf, self.abw = False, 0.0
            return
        w.append((t, di, dw, dv, abs(dsw or 0.0)))
        while w and w[0][0] < t - c.contra_window_s:
            w.popleft()
        ei = sum(x[1] for x in w)
        ew = sum(x[2] for x in w)
        self.abw = ew - ei
        widerspruch = abs(ew - ei) > math.radians(c.contra_deg) + c.contra_frac * abs(ei)
        if widerspruch and self.zweifel(t):
            self._zweifel_bis = t + c.imu_doubt_s    # IMU liegt weiter daneben: Zweifel verlängern, kein neues Urteil
            w.clear()
            return
        if widerspruch:
            span = max(t - w[0][0], 1e-3)
            if not self.schlupf:
                self.widersprueche += 1
                log.info("Koppelnavigation: Schlupf? IMU %.1f°, Räder %.1f° in %.1f s", math.degrees(ei),
                         math.degrees(ew), span)
            # Die Kamera wird in jedem Schritt gefragt, solange der Widerspruch besteht. Sie zählt, wenn ihr möglicher
            # Fehler (cam_err_deg + cam_err_deg_m je gefahrenem Meter) unter einem Viertel des Widerspruchs liegt: im
            # Stand fast immer, bei Fahrt nur bei groben Fehlern (Fahrt 20261117-110738 bei 0,4 m/s: IMU −143°,
            # Räder +12°, Bild +3° in 1 s, der Rider kippte; die alte Regel „nur im Stand“ ließ das durch).
            err = math.radians(c.cam_err_deg + c.cam_err_deg_m * sum(x[4] for x in w))
            if err < 0.25 * abs(ew - ei) and all(x[3] is not None for x in w):
                ev = sum(x[3] for x in w)
                if abs(ev - ew) < 0.5 * abs(ev - ei):
                    self._zweifel_bis = t + c.imu_doubt_s
                    self.imu_zweifel += 1
                    # Die früheren Schritte im Fenster drehten mit der IMU; was die Räder dort anders sahen, holt dieser
                    # Schritt nach (er selbst nimmt schon die Räder)
                    self._nachholen = (ew - dw) - (ei - di)
                    log.warning("Koppelnavigation: IMU %.1f°, Räder %.1f°, Bild %.1f° – Bild gibt den Rädern recht, "
                                "Kurs %.0f s aus den Rädern", math.degrees(ei), math.degrees(ew), math.degrees(ev),
                                c.imu_doubt_s)
                    w.clear()
                    self.schlupf = False
                    return
        self.schlupf = widerspruch

    def _kalib(self, t: float, dt: float, di, dl, dr, ok: bool) -> None:
        c, seg = self.cfg, self._seg
        if not ok or di is None or dl is None:
            self._seg = [0.0, 0.0, 0.0, 0.0]        # Abschnitt verwerfen: Schlupf, Zweifel, Lücke, liegend
            return
        seg[0] += dl / c.scale_left
        seg[1] += dr / c.scale_right
        seg[2] += di
        seg[3] += dt
        if seg[3] >= c.calib_seg_s:
            if abs(seg[0]) + abs(seg[1]) > 0.02:     # im Stand bewegt sich nichts, sagt also nichts
                self._segs.append((seg[0], seg[1], seg[2]))
            self._seg = [0.0, 0.0, 0.0, 0.0]
        if self._calib_t is None:
            self._calib_t = t
        elif t - self._calib_t >= self.CALIB_EVERY_S:
            self._calib_t = t
            self._rechne_vorschlag()

    def _rechne_vorschlag(self) -> None:
        c, segs = self.cfg, list(self._segs)
        dreh = sum(abs(s[2]) for s in segs)
        weg = sum(abs(s[0] + s[1]) / 2 for s in segs)
        if len(segs) < 10 or dreh < 2 * math.pi or weg < 2.0:
            return                                    # zu wenig Drehung oder Strecke für Spur und Raddifferenz
        try:
            kl, kr, tr = calibrate_odom(segs, c.track_m, mean_scale=(c.scale_left + c.scale_right) / 2)
        except (ValueError, ArithmeticError):
            return
        self.vorschlag = {"scale_left": round(kl, 4), "scale_right": round(kr, 4), "track_m": round(tr, 4),
                          "n": len(segs), "dreh_deg": round(math.degrees(dreh)), "weg_m": round(weg, 1)}

    def info(self) -> dict:
        s = self.letzter
        return {"kurs": s.kurs_src if s else None, "strecke": s.ds_src if s else None,
                "abw_deg": round(math.degrees(self.abw), 1), "schlupf": self.schlupf,
                "widersprueche": self.widersprueche, "imu_zweifel": self.imu_zweifel,
                "zweifel": self._t is not None and self.zweifel(self._t), "vorschlag": self.vorschlag}
