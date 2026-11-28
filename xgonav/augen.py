"""Augengenerator im Display-Daemon (st7789/project/riderpi/src/augen*.c, docs/display-emotions.md).

Gezeichnet wird im Daemon; xgonav schickt nur Textzeilen nach /tmp/lcd0.augen ([emotion] augen_fifo): Blick, Lid,
Abstand, Lidschlagrate, Farbe, Ausdruck. Die Augen leben dort von selbst (Lidschlag, Sakkaden, Atmen). Ohne
Daemon mit Generator (FIFO fehlt, kein Leser: ENXIO) ist `ok` False und das Display zeigt keine Augen (kein
Rückfallweg). Pi-Kosten hier: ein os.write von wenigen Byte bei Änderung.
"""
from __future__ import annotations

import errno
import logging
import os
import threading
import time

log = logging.getLogger(__name__)

STANDARD_FARBE = "tuerkis"           # Farbnamen kennt der Daemon (augen_leben.c, wie tools/eyes.py PALETTE)


class Augen:
    def __init__(self, fifo: str):
        self.fifo = fifo
        self._fd: int | None = None
        self._next_try = 0.0
        self._sent: dict[str, object] = {}
        self._lock = threading.Lock()
        self._gemeldet = False

    @property
    def ok(self) -> bool:
        with self._lock:
            return self._open()

    def _open(self) -> bool:
        if self._fd is not None:
            return True
        now = time.monotonic()
        if now < self._next_try:
            return False
        self._next_try = now + 2.0
        try:
            self._fd = os.open(self.fifo, os.O_WRONLY | getattr(os, "O_NONBLOCK", 0))
        except OSError as e:
            if e.errno not in (errno.ENOENT, errno.ENXIO) and not self._gemeldet:
                log.warning("Augen: %s", e)
                self._gemeldet = True
            return False
        self._sent.clear()
        log.info("Augen: Generator im Display-Daemon (%s)", self.fifo)
        return True

    def send(self, *zeilen: str) -> bool:
        """Zeilen in einem write (unter PIPE_BUF: atomar); volle Pipe: verworfen."""
        if not zeilen:
            return True
        with self._lock:
            if not self._open():
                return False
            try:
                os.write(self._fd, "".join(z + "\n" for z in zeilen).encode())
                return True
            except BlockingIOError:
                return False
            except OSError:                  # Daemon weg (EPIPE): später neu öffnen, alles neu senden
                os.close(self._fd)
                self._fd = None
                self._sent.clear()
                return False

    def _neu(self, key: str, wert, eps: float = 0.0) -> bool:
        alt = self._sent.get(key)
        if alt is not None and (alt == wert or (eps and isinstance(wert, tuple)
                                                 and max(abs(a - b) for a, b in zip(alt, wert)) < eps)):
            return False
        self._sent[key] = wert
        return True

    def leben(self, x: float, y: float, lid: float, nah: float, rate: float, farbe: str | None) -> bool:
        """Lebende Augen (Modus pet): nur Geändertes senden."""
        zeilen = []
        if self._neu("modus", "an"):
            zeilen.append("an")
        f = farbe or STANDARD_FARBE
        if self._neu("farbe", f):
            zeilen.append(f"farbe {f} 0.4")
        if self._neu("rate", (round(rate, 1),), 0.05):
            zeilen.append(f"rate {rate:.1f}")
        if self._neu("lid", (lid,), 0.02):
            zeilen.append(f"lid {lid:.3f}")
        if self._neu("nah", (nah,), 0.03):
            zeilen.append(f"nah {nah:.2f}")
        if self._neu("blick", (x, y), 0.02):
            zeilen.append(f"blick {x:.3f} {y:.3f}")
        ok = self.send(*zeilen)
        if not ok:
            self._sent.clear()               # beim nächsten Mal alles neu
        return ok

    def aus(self, farbe_zurueck: bool = True) -> None:
        """Lebende Augen aus (letztes Bild bleibt stehen), Farbe zurück auf Türkis."""
        zeilen = []
        if self._sent.get("modus") != "aus":
            zeilen.append("aus")
        if farbe_zurueck and self._sent.get("farbe") not in (None, STANDARD_FARBE):
            zeilen.append(f"farbe {STANDARD_FARBE}")
        if zeilen and self.send(*zeilen):
            self._sent["modus"] = "aus"
            if farbe_zurueck:
                self._sent["farbe"] = STANDARD_FARBE

    def ausdruck(self, name: str, farbe: str | None = None) -> bool:
        """Einen der Ausdrücke abspielen (tools/eyes.py EMOTIONS); False: kein Generator."""
        f = farbe or STANDARD_FARBE
        zeilen = [f"farbe {f} 0.2"] if self._neu("farbe", f) else []
        ok = self.send(*zeilen, f"ausdruck {name}")
        if not ok:
            self._sent.pop("farbe", None)
        return ok
