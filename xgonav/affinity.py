"""CPU-Pinning einzelner Threads (Linux). Ohne Konfiguration passiert nichts.

Am RiderPi (CM4, 4 Kerne) laufen Hauptschleife und Detektor auf getrennten Kernen,
damit YOLO die Hauptschleife nicht verdrängt (Schleifenspitzen). Siehe [cpu] in config.
"""
from __future__ import annotations

import logging
import os

log = logging.getLogger(__name__)


def available_cpus() -> set[int]:
    if hasattr(os, "sched_getaffinity"):
        try:
            return set(os.sched_getaffinity(0))
        except OSError:
            pass
    return set(range(os.cpu_count() or 1))


def pin_thread(cpus, name: str, quiet: bool = False) -> set[int]:
    """Den aufrufenden Thread auf die Kerne `cpus` legen (unter Linux gilt
    sched_setaffinity(0) nur für diesen Thread; danach erzeugte Threads erben es).
    Leere Liste = nichts tun. Nicht vorhandene Kerne werden mit Warnung verworfen.
    quiet: Erfolg nur auf DEBUG loggen (für kurzlebige Threads, z. B. je Web-Anfrage).
    Rückgabe: die gesetzte Menge (leer, wenn nichts gesetzt wurde)."""
    want = {int(c) for c in (cpus or [])}
    if not want or not hasattr(os, "sched_setaffinity"):
        return set()
    have = set(range(os.cpu_count() or 1))
    use = want & have
    if want - use:
        log.warning("%s: Kerne %s gibt es nicht, ignoriert", name, sorted(want - use))
    if not use:
        return set()
    try:
        os.sched_setaffinity(0, use)
    except OSError as e:
        log.warning("%s: CPU-Pinning fehlgeschlagen: %s", name, e)
        return set()
    (log.debug if quiet else log.info)("%s auf Kerne %s gepinnt", name, sorted(use))
    return use


def ort_pool_affinities(cpus, threads: int) -> str:
    """Wert für onnxruntime "session.intra_op_thread_affinities": ein Eintrag je
    Pool-Thread (threads - 1; der aufrufende Thread rechnet selbst mit), Kern-IDs
    1-basiert, Einträge mit ";" getrennt. Der Aufrufer bekommt cpus[0], die Pool-Threads
    die übrigen Kerne reihum. Leer, wenn nichts zu setzen ist."""
    cpus = [int(c) for c in (cpus or [])]
    n = int(threads) - 1
    if not cpus or n <= 0:
        return ""
    rest = cpus[1:] or cpus
    return ";".join(str(rest[i % len(rest)] + 1) for i in range(n))
