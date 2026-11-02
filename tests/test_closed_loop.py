"""Closed-Loop im Simulator (dauert je ~30 s). pytest -m slow"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sim_run import run  # noqa: E402


@pytest.mark.slow
def test_follow_person_with_shake():
    r = run("follow", 30.0)
    assert r["collisions"] == 0
    assert r["dist_min"] > 0.8          # nicht auffahren
    assert 1.1 < r["dist_mean"] < 2.4   # dranbleiben (Soll 1.2 m + Totzone, Person läuft)
    assert r["loop_ms"] < 30


@pytest.mark.slow
def test_explore_no_collisions():
    r = run("explore", 30.0)
    assert r["collisions"] == 0
    assert r["dist_m"] > 2.0            # bewegt sich tatsächlich


@pytest.mark.slow
def test_follow_with_imu_autosync():
    r = run("follow", 30.0, imu=True)
    ok, sign, off_ms, corr = r["imu_sync"]
    assert ok and sign == -1 and abs(off_ms + 8) <= 4   # Simulator: Vorzeichen -1, Versatz -8 ms
    assert r["collisions"] == 0
