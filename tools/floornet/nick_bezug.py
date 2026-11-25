"""Nick-Bezug nachträglich angleichen, nur für Auswertung und Labels am PC/kimmy (Nutzer 2026-11-19: „fürs Prüfen
darfst du die Werte anpassen“).

pitch_dev in meta.jsonl ist pitch_ref_deg − IMU-Nick mit dem pitch_ref_deg, der beim Fahren am Rider eingestellt war.
Mit der Lagefusion (Firmware-Patch 04-attitude, ab Fahrt 20261118-172730) passt nur −5,0 (Flurfahrten 2026-11-19, Flur 2,04 m); gefahren wurde
aber auch mit −4,23 und −6,23. Bezug je Fahrt aus den Daten selbst (pitch_dev + IMU-Nick der Bildzeit, Median,
Streuung ±0,1°). Fahrten davor bleiben, für die alte Firmware galt −4,23.
"""
from __future__ import annotations

import math

ZIEL_0021 = -5.0
GEFAHREN = {                      # Fahrt -> pitch_ref_deg beim Fahren (Lagefusion 04-attitude)
    "20261118-172730": -4.23,
    "20261118-175735": -6.23,
    "20261119-114557": -6.23,
    "20261119-120753": -6.23,
    "20261119-121128": -6.23,
    "20261119-122328": -6.23,
}


def versatz_rad(fahrt: str) -> float:
    """Zuschlag auf pitch_dev (rad) für diese Fahrt, 0 für alle nicht aufgeführten."""
    ref = GEFAHREN.get(fahrt)
    return 0.0 if ref is None else math.radians(ZIEL_0021 - ref)


def pitch_dev(fahrt: str, m: dict) -> float:
    """pitch_dev eines meta.jsonl-Eintrags mit angeglichenem Bezug (rad)."""
    return float(m.get("pitch_dev", 0.0)) + versatz_rad(fahrt)
