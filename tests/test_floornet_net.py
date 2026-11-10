"""FloorNet: Aufbau, Zeilenfächer, Verlust und das Einrechnen der BatchNorm. Braucht torch (nur auf dem GPU-Rechner)."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

torch = pytest.importorskip("torch")

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools" / "floornet"))

import net as N  # noqa: E402
import train as T  # noqa: E402


def _trained():
    torch.manual_seed(0)
    m = N.FloorNet()
    m.train()
    for _ in range(5):
        m(torch.rand(8, 3, N.IN_H, N.IN_W) - 0.5)
    return m.eval()


def test_form_und_groesse_wie_gemessen():
    m = N.FloorNet().eval()
    assert m(torch.zeros(2, 3, N.IN_H, N.IN_W)).shape == (2, N.NBINS, N.NCOLS)
    # Messung am CM4 (bench_kern, f2_s): ~30 kParam; ohne BatchNorm und mit 64 statt 67 Ausgängen etwas weniger
    plain = sum(p.numel() for k, p in m.named_parameters() if ".bn" not in k and not k.startswith("bn_"))
    assert 25_000 < plain < 31_000


def test_faecher_steigen_und_treffen_die_raender():
    e = N.bin_edges()
    assert len(e) == N.NBINS + 1 and (np.diff(e) > 0).all()
    assert e[0] == pytest.approx(N.ROW_TOP) and e[-1] == pytest.approx(N.ROW_BOTTOM)
    assert np.diff(e)[0] < 2.0 and 8.0 < np.diff(e)[-1] < 12.0


def test_decode_trifft_das_fach_und_parabel_verschiebt_es():
    c = N.bin_centers()
    lo = torch.full((1, N.NBINS, N.NCOLS), -10.0)
    lo[0, 30, :] = 5.0
    assert N.decode(lo)[0, 0].item() == pytest.approx(c[30], abs=1e-4)
    lo[0, 31, :] = 4.0
    lo[0, 29, :] = 0.0
    assert c[30] < N.decode(lo)[0, 0].item() < c[31]


def _zacke():
    """Alle Spalten Gipfel in Fach 30; Spalte 40 höher in Fach 50 mit zweitem Gipfel in 30, Spalte 20 nur in 50."""
    lo = torch.zeros(1, N.NBINS, N.NCOLS)
    lo[0, 30, :] = 10.0
    lo[0, 50, 40], lo[0, 30, 40] = 10.0, 9.5
    lo[0, 50, 20], lo[0, 30, 20] = 10.0, 0.0
    return lo


def test_decode_laesst_zacken_ohne_auftrag_stehen():
    c = N.bin_centers()
    assert c[50] - c[30] > N.OUTLIER_PX
    got = N.decode(_zacke())[0]
    assert got[40].item() == pytest.approx(c[50], abs=1e-3) and got[20].item() == pytest.approx(c[50], abs=1e-3)


def test_decode_nimmt_zacke_mit_zweitem_gipfel_heraus():
    c = N.bin_centers()
    roh = N.decode(_zacke())[0]
    got = N.decode(_zacke(), N.OUTLIER_PX, N.OUTLIER_RATIO)[0]
    assert got[40].item() == pytest.approx(c[30], abs=1e-3)                  # zweiter Gipfel beim Nachbarwert: dorthin
    assert got[20].item() == pytest.approx(c[50], abs=1e-3)                  # nichts beim Nachbarwert: bleibt (Hindernis)
    andere = [x for x in range(N.NCOLS) if x not in (20, 40)]
    assert torch.equal(got[andere], roh[andere])


def test_decode_laesst_breite_stufe_stehen():
    lo = torch.zeros(1, N.NBINS, N.NCOLS)
    lo[0, 30, :40], lo[0, 50, 40:] = 10.0, 10.0
    lo[0, 30, 40:], lo[0, 50, :40] = 9.5, 9.5                               # zweiter Gipfel überall, aber kein Ausreißer
    assert torch.equal(N.decode(lo, N.OUTLIER_PX, N.OUTLIER_RATIO), N.decode(lo))


@pytest.mark.parametrize("b", [0, N.NBINS - 1])
def test_decode_am_rand_bleibt_im_bereich(b):
    lo = torch.zeros((1, N.NBINS, N.NCOLS))
    lo[0, b, :] = 5.0
    assert N.decode(lo)[0, 0].item() == pytest.approx(N.bin_centers()[b], abs=1e-4)


def test_fold_rechnet_dasselbe():
    m = _trained()
    x = torch.rand(3, 3, N.IN_H, N.IN_W) - 0.5
    with torch.no_grad():
        want = m(x)
    got = N.run_ops(N.fold(m), x)
    assert torch.abs(want - got).max().item() < 1e-3


def test_variante_mit_dilatation_und_zusatzblock():
    """Standardnetz bleibt, wie es ist; fold() rechnet auch die Variante richtig."""
    assert N.FloorNet().variant == {"dil": [1, 1], "extra": 0}
    std = sum(p.numel() for p in N.FloorNet().parameters())
    torch.manual_seed(0)
    m = N.FloorNet(dil=(2, 3), extra=1)
    assert sum(p.numel() for p in N.FloorNet(dil=(2, 3)).parameters()) == std     # Dilatation kostet nichts
    assert sum(p.numel() for p in m.parameters()) > std
    m.train()
    for _ in range(5):
        m(torch.rand(8, 3, N.IN_H, N.IN_W) - 0.5)
    m.eval()
    x = torch.rand(2, 3, N.IN_H, N.IN_W) - 0.5
    with torch.no_grad():
        want = m(x)
    assert want.shape == (2, N.NBINS, N.NCOLS)
    assert torch.abs(want - N.run_ops(N.fold(m), x)).max().item() < 1e-3


def test_fold_haelt_die_vorgaben_des_kerns_ein():
    ops = N.fold(_trained())
    for op in ops:
        if op["kind"] == "dw":
            assert op["b"] is None and not op["relu"]
        if op["kind"] == "pw" and op is not ops[-1]:
            assert op["w"].shape[0] % 8 == 0


def _centers():
    return torch.as_tensor(N.bin_centers(), dtype=torch.float32)


def _peak(row):
    i = int(np.argmin(np.abs(N.bin_centers() - row)))
    lo = torch.full((1, N.NBINS, N.NCOLS), -8.0)
    lo[0, i, :] = 8.0
    return lo


def test_verlust_bestraft_falsche_grenze_und_fernes_mehr_als_nahes():
    row = torch.full((1, N.NCOLS), 300.0)
    kind = torch.full((1, N.NCOLS), T.EXACT)
    right = T.column_loss(_peak(300.0), row, kind, _centers())
    # Gleich viele Fächer daneben: fälschlich frei (kleinere Zeile) kostet mehr als fälschlich blockiert
    i = int(np.argmin(np.abs(N.bin_centers() - 300.0)))
    too_far = T.column_loss(_peak(N.bin_centers()[i - 12]), row, kind, _centers())
    too_near = T.column_loss(_peak(N.bin_centers()[i + 12]), row, kind, _centers())
    assert right < too_near < too_far


def test_mindestens_ist_gleichgueltig_nach_oben_und_ignore_zaehlt_nicht():
    row = torch.full((1, N.NCOLS), 300.0)
    al = torch.full((1, N.NCOLS), T.ATLEAST)
    weiter = T.column_loss(_peak(250.0), row, al, _centers())
    naeher = T.column_loss(_peak(400.0), row, al, _centers())
    assert weiter < 1e-3 < naeher
    ign = torch.full((1, N.NCOLS), T.IGNORE)
    assert T.column_loss(_peak(400.0), row, ign, _centers()).item() == 0.0


def test_augment_verschiebt_labels_mit_dem_bild():
    # Helle Zeile bei Vollbildzeile 301 ausgeschnitten: Label und Bild müssen nach dem Verschieben zusammenpassen
    x = torch.zeros(16, 3, N.IN_H + T.MARGIN, N.IN_W, dtype=torch.uint8)
    top = N.CROP_Y - T.MARGIN * N.DOWN
    y0 = (301 - top) // N.DOWN
    x[:, :, y0, :] = 255
    row = torch.full((16, N.NCOLS), 301.0)
    kind = torch.full((16, N.NCOLS), T.EXACT)
    torch.manual_seed(3)
    xa, ra, _ = T.augment(x, row, kind)
    got = xa.mean(dim=(1, 3)).argmax(dim=1)                                  # Eingabezeile der hellen Zeile
    want = (ra[:, 0] - N.CROP_Y) / N.DOWN
    assert torch.abs(got.float() - want).max().item() <= 1.0


def test_normierung_gleicht_helligkeit_aus_und_begrenzt_den_faktor():
    x = torch.rand(2, 3, N.IN_H, N.IN_W) * 0.1 + 0.3
    a = N.prep(x, True)
    b = N.prep(x * 0.25, True)                                                # vierfach dunkler: gleiche Eingabe
    assert torch.abs(a - b).max().item() < 1e-3
    assert N.prep(x, False).max().item() <= 0.5
    schwarz = N.prep(torch.full((1, 3, N.IN_H, N.IN_W), 0.002), True)         # Faktor bleibt begrenzt
    assert schwarz.max().item() <= 0.002 * N.NORM_MAX - 0.5 + 1e-5


def _run_mit_labels(row, kind):
    g = {"W": 640, "H": 480, "f": 328.5, "cam_h": 0.128, "tilt_deg": 5.18}
    return {"geom": g, "pitch": np.zeros(1, np.float32), "row": np.array([row], np.float32), "kind": np.array([kind], np.uint8)}


def _row_at(d):
    return 240 + 328.5 * np.tan(np.arctan(0.128 / d) + np.radians(5.18))


def test_ferne_grenze_wird_zu_mindestens_nahe_bleibt():
    r = _run_mit_labels([_row_at(0.8), _row_at(2.0)], [T.EXACT, T.EXACT])
    T.soften(r, far_atleast=1.25)
    assert r["kind"][0].tolist() == [T.EXACT, T.ATLEAST]
    assert r["row"][0, 1] == pytest.approx(_row_at(2.0))


def test_mindestens_behauptet_nur_noch_bis_zur_grenze():
    r = _run_mit_labels([_row_at(2.0), _row_at(0.4), _row_at(2.0)], [T.ATLEAST, T.ATLEAST, T.EXACT])
    T.soften(r, cap_atleast=0.6)
    assert r["row"][0, 0] == pytest.approx(_row_at(0.6), abs=0.01)        # Anspruch gekürzt
    assert r["row"][0, 1] == pytest.approx(_row_at(0.4))                   # war schon kürzer
    assert r["row"][0, 2] == pytest.approx(_row_at(2.0))                   # exakt bleibt
    assert r["kind"][0].tolist() == [T.ATLEAST, T.ATLEAST, T.EXACT]


def test_ohne_angaben_aendert_soften_nichts():
    r = _run_mit_labels([_row_at(2.0)], [T.ATLEAST])
    alt = r["row"].copy()
    T.soften(r)
    assert np.array_equal(r["row"], alt) and r["kind"][0, 0] == T.ATLEAST


def test_band_bericht_erkennt_fernes_und_nahes():
    g = {"W": 640, "H": 480, "f": 328.5, "cam_h": 0.128, "tilt_deg": 5.18}
    pitch = np.zeros(4)
    hz = 240 + 328.5 * np.tan(np.radians(5.18))
    row_at = lambda d: 240 + 328.5 * np.tan(np.arctan(0.128 / d) + np.radians(5.18))  # noqa: E731
    label = np.full((4, 40), row_at(0.75), np.float32)
    kind = np.full((4, 40), T.EXACT, np.uint8)
    frei = T.band_report(np.full((4, 40), hz - 5.0), label, kind, pitch, g)["0.5-1.0"]        # über dem Horizont
    assert frei["frei"] == 1.0 and frei["block"] == 0.0
    block = T.band_report(np.full((4, 40), row_at(0.4)), label, kind, pitch, g)["0.5-1.0"]
    assert block["block"] == 1.0 and block["frei"] == 0.0
    gleich = T.band_report(label, label, kind, pitch, g)["0.5-1.0"]
    assert gleich["frei"] == 0.0 and gleich["block"] == 0.0 and gleich["med_cm"] < 0.1


def test_near_frames_by_share_of_close_columns():
    geo = {"H": 480, "f": 328.49, "tilt_deg": 5.18, "cam_h": 0.128}
    nah = T.row_dist(geo, np.array([[440.0]], np.float32), np.zeros(1, np.float32))[0, 0]
    assert nah < 0.5
    row = np.full((3, 80), 260.0, np.float32)
    row[1, :30] = 440.0     # 37,5 % der Spalten nah
    row[2, :10] = 440.0     # 12,5 %
    run = {"geom": geo, "row": row, "pitch": np.zeros(3, np.float32), "kind": np.full((3, 80), T.EXACT, np.uint8)}
    assert T.near_frames(run, 0.5, 0.3).tolist() == [False, True, False]
    # Spalten ohne Label zählen nicht
    run["kind"][1, :30] = T.IGNORE
    assert T.near_frames(run, 0.5, 0.3).tolist() == [False, False, False]


def test_smooth_loss_ignores_jumps_the_label_has():
    centers = torch.as_tensor(N.bin_centers(), dtype=torch.float32)
    k = len(centers)
    ziel = torch.full((1, 80), float(centers[10]))
    ziel[0, 40:] = float(centers[50])
    kind = torch.full((1, 80), T.EXACT, dtype=torch.uint8)

    def logits_for(rows):
        i = torch.as_tensor([int(torch.argmin((centers - r).abs())) for r in rows[0]])
        lg = torch.full((1, k, 80), -50.0)
        lg[0, i, torch.arange(80)] = 50.0
        return lg

    assert float(T.smooth_loss(logits_for(ziel), ziel, kind, centers)) < 1e-3
    glatt = torch.full((1, 80), float(centers[10]))
    spitze = glatt.clone()
    spitze[0, 20:25] = float(centers[50])
    assert float(T.smooth_loss(logits_for(spitze), glatt, kind, centers)) > 0.05
    # Sprung über eine Spalte ohne Label zählt nicht
    kind2 = kind.clone()
    kind2[0, 20:25] = T.IGNORE
    assert float(T.smooth_loss(logits_for(spitze), glatt, kind2, centers)) < 1e-3
