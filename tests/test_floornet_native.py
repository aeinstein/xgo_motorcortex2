"""FloorNet: C++-Kern gegen PyTorch. Braucht torch und g++ (GPU-Rechner); ohne ARM rechnen die NEON-Befehle aus
tests/neon_shim. Das prüft das Rechenergebnis, nicht die Geschwindigkeit; am Rider folgt derselbe Vergleich."""
from __future__ import annotations

import platform
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

torch = pytest.importorskip("torch")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools" / "floornet"))

import export as X  # noqa: E402
import net as N  # noqa: E402
from xgonav import fastlib  # noqa: E402
from xgonav.floornet import SELFTEST_MAX, FloorNet  # noqa: E402

NATIVE = ROOT / "xgonav" / "native"


def _build(tmp: Path) -> Path:
    """Kern als eigene Bibliothek bauen, ohne ARM über Prüfkopien mit der Prüfhilfe."""
    cxx = shutil.which("g++")
    if cxx is None:
        pytest.skip("kein g++")
    so = tmp / "libxgofast.so"
    flags = ["-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC", "-o", str(so)]
    if platform.machine() == "aarch64":
        subprocess.run([cxx, *flags, "-mcpu=native", str(NATIVE / "floornet.cpp")], check=True)
        return so
    for name in ("floornet.cpp", "floornet_kern.h"):
        text = (NATIVE / name).read_text(encoding="utf-8").replace("defined(__aarch64__)", "1")
        (tmp / name).write_text(text, encoding="utf-8")
    subprocess.run([cxx, *flags, "-I" + str(ROOT / "tests" / "neon_shim"), str(tmp / "floornet.cpp")], check=True)
    return so


@pytest.fixture(scope="module")
def kern(tmp_path_factory, request):
    tmp = tmp_path_factory.mktemp("fn")
    so = _build(tmp)
    torch.manual_seed(1)
    m = N.FloorNet()
    m.train()
    for _ in range(4):
        m(torch.rand(4, 3, N.IN_H, N.IN_W) - 0.5)
    m.eval()
    binp = tmp / "floornet.bin"
    X.write(m, str(binp))
    old = fastlib.PATH
    fastlib.PATH = str(so)
    fastlib._cache.clear()
    net = FloorNet.load(str(binp))

    def zurueck():
        fastlib.PATH = old
        fastlib._cache.clear()

    request.addfinalizer(zurueck)
    assert net is not None
    return net, m, N.fold(m), binp


def _bilder():
    rng = np.random.default_rng(3)
    # Rauschen, glatter Verlauf, harte Kanten, Extreme
    noise = rng.integers(0, 256, (480, 640, 3), dtype=np.uint8)
    yy, xx = np.mgrid[0:480, 0:640]
    glatt = np.stack([xx * 255 // 639, yy * 255 // 479, (xx + yy) * 255 // 1118], axis=-1).astype(np.uint8)
    kante = np.where(yy[..., None] > 280, 200, 30).astype(np.uint8).repeat(3, axis=-1)
    return [noise, glatt, kante, np.zeros((480, 640, 3), np.uint8), np.full((480, 640, 3), 255, np.uint8), X.test_frame()]


def test_selbstpruefung_besteht(kern):
    net, *_ = kern
    assert net._lib.fn_selftest(net._h) < SELFTEST_MAX


@pytest.mark.parametrize("i", range(6))
def test_logits_und_zeilen_wie_pytorch(kern, i):
    net, _, ops, _ = kern
    img = _bilder()[i]
    rows, lg = net.logits(img)
    want = N.run_ops(ops, X.to_input(img))
    w = want[0].numpy()
    assert (np.abs(lg - w) / (1.0 + np.abs(w))).max() < SELFTEST_MAX
    assert np.abs(rows - N.decode(want)[0].numpy()).max() < 0.05


def test_gleiche_eingabe_gleiches_ergebnis(kern):
    net, *_ = kern
    a, b, img = _bilder()[0], _bilder()[1], _bilder()[2]
    first = net.run(img)
    net.run(a)
    net.run(b)
    assert np.array_equal(net.run(img), first)                               # kein Rest aus früheren Bildern


def test_zeilenabstand_wird_beachtet(kern):
    net, *_ = kern
    img = _bilder()[2]
    gross = np.zeros((480, 800, 3), np.uint8)
    gross[:, :640] = img
    want = net.run(img)
    got = np.empty(N.NCOLS, np.float32)
    net._lib.fn_run(net._h, gross.ctypes.data, gross.strides[0], got.ctypes.data, None)
    assert np.array_equal(want, got)


@pytest.mark.parametrize("wie", ["kurz", "kennung", "version", "lang"])
def test_kaputte_datei_wird_abgelehnt(kern, tmp_path, wie):
    *_, binp = kern
    d = bytearray(binp.read_bytes())
    if wie == "kurz":
        d = d[:-100]
    elif wie == "kennung":
        d[0] = 0
    elif wie == "version":
        d[4] = 99
    else:
        d += b"xx"
    p = tmp_path / "kaputt.bin"
    p.write_bytes(bytes(d))
    assert FloorNet.load(str(p)) is None


def test_falsche_erwartung_faellt_in_der_selbstpruefung_durch(kern, tmp_path):
    *_, binp = kern
    d = bytearray(binp.read_bytes())
    # Erwartete Logits verfälschen, das Netz selbst bleibt richtig
    pos = len(d) - 4 - 4 * N.NBINS * N.NCOLS
    d[pos:pos + 4] = np.array([1e6], "<f4").tobytes()
    p = tmp_path / "falsch.bin"
    p.write_bytes(bytes(d))
    assert FloorNet.load(str(p)) is None
