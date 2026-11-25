"""tools/fw_einheit.py: Register 0x0e lesen, Rad-Odometrie, Faktor aus ruhigen Fahrabschnitten (künstliche Daten)."""
import math
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
import fw_einheit as fe  # noqa: E402


def _zeile_0e(ms, v11, v21, fwd10, flags):
    b = bytearray(32)
    struct.pack_into("<Hhh", b, 0, ms, v11, v21)
    b[23] = flags
    struct.pack_into("<h", b, 28, fwd10)
    return "0x0e 32: " + " ".join("%02x" % x for x in b)


def test_diag_0e_felder():
    d = fe.diag_0e(_zeile_0e(1234, 32, -5, 321, 1))
    assert {k: d[k] for k in ("fw_ms", "fw_v11", "fw_v21", "fw_cmd", "faehrt")} == \
        {"fw_ms": 1234, "fw_v11": 32, "fw_v21": -5, "fw_cmd": 32.1, "faehrt": 1}
    assert {"nick_soll", "nick_ist", "balance", "halt_korr", "halt_modus"} <= d.keys()
    assert fe.imu_nick("56144 579050273938 51 -2244 174491") == -2.244
    assert fe.imu_nick("") is None
    assert fe.diag_0e("0x0e 32: Fehler -110") is None
    assert fe.diag_0e("0x15 32: " + " ".join(["00"] * 32)) is None


def test_odometrie_mittel_und_ueberlauf():
    o = fe.Odometrie(0.060, 0.3)
    mm_tick = math.pi * 60.0 / 1024
    start = 2**31 - 50                                   # läuft über int32
    v = None
    for i in range(40):                                  # 100 Hz, 10 Ticks je Probe = 1000 Ticks/s
        t = i * 10_000_000
        a = ((start + 10 * i + 2**31) % 2**32) - 2**31
        v = o.neu(t, a, a)
    assert abs(v - 1000 * mm_tick) < 1e-6


def _fahrt(faktor, soll, ist_fw, n=200):
    rows = []
    for i in range(n):
        an = 20 <= i < n - 20
        fw = ist_fw if an else 0
        rows.append({"t": i * 0.05, "fw_v11": fw, "fw_v21": fw, "fw_cmd": soll if an else 0.0,
                     "faehrt": 1, "odo_mms": fw * faktor if an else 0.0})
    return rows


def test_faktor_aus_ruhiger_fahrt():
    e = fe.auswerten(_fahrt(8.4, 32.1, 32))
    assert e["n"] > 100
    assert abs(e["faktor"] - 8.4) < 1e-9 and abs(e["faktor_gerade"] - 8.4) < 1e-9
    assert abs(e["ist_je_soll"] - 32 / 32.1) < 1e-9
    assert "8.40" in fe.bericht(e)


def test_anfahren_zaehlt_nicht_und_zu_wenig_daten():
    rows = _fahrt(8.4, 32.1, 32, n=30)                    # nur 10 Zeilen in Fahrt, alle am Rand
    assert fe.auswerten(rows)["faktor"] is None
    assert "zu wenig" in fe.bericht(fe.auswerten(rows))
