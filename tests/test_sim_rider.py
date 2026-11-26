"""Virtueller Rider: SimRobot mit /proc-Spiegel, Logzeilen für tools/live.php, begrenzter Hub der 3D-Ansicht."""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from xgonav.config import Config  # noqa: E402
from xgonav.robot import SimRobot  # noqa: E402
from tools.sim3d import Hub  # noqa: E402
from tools.sim_rider import imu_zeile, rad_zeile  # noqa: E402


def lies(d, name):
    with open(os.path.join(d, name)) as f:
        return f.read().strip()


def von_aussen(d, name, text):
    """Wie api.php: Datei direkt überschreiben (mtime ändert sich auch bei gleichem Text)."""
    p = os.path.join(d, name)
    with open(p, "w") as f:
        f.write(text)
    st = os.stat(p)
    os.utime(p, ns=(st.st_atime_ns, st.st_mtime_ns + 1_000_000))


def test_proc_dateien_angelegt(tmp_path):
    cfg = Config.neutral()
    SimRobot(cfg.robot, proc_dir=str(tmp_path))
    assert lies(tmp_path, "state") == "1" and lies(tmp_path, "battery") == "100"
    assert os.path.isdir(tmp_path / "leds") and os.path.isdir(tmp_path / "settings")


def test_hoehe_eigen_und_von_aussen(tmp_path):
    cfg = Config.neutral()
    r = SimRobot(cfg.robot, proc_dir=str(tmp_path))
    r.set_height(100.0)
    u = lies(tmp_path, "height")
    assert u.isdigit() and abs(r.height_u8_to_mm(int(u)) - r.height_mm) < 0.01
    r._poll_state()
    assert abs(r.height_mm - r.height_u8_to_mm(int(u))) < 0.01          # eigener Wert, keine Änderung
    von_aussen(tmp_path, "height", "180")
    r._poll_state()
    assert abs(r.height_mm - r.height_u8_to_mm(180)) < 0.01


def test_absetzen_aufstehen_von_aussen(tmp_path):
    cfg = Config.neutral()
    r = SimRobot(cfg.robot, proc_dir=str(tmp_path))
    von_aussen(tmp_path, "action", "254")
    r._poll_state()
    assert not r.state.balancing and lies(tmp_path, "state") == "254"
    von_aussen(tmp_path, "action", "253")
    r._poll_state()
    assert not r.state.balancing                                          # springt erst nach ~1 s auf
    r._up_at = time.monotonic() - 0.01
    r._poll_state()
    assert r.state.balancing and lies(tmp_path, "state") == "1"


def test_ohne_proc_dir_wie_bisher(tmp_path):
    cfg = Config.neutral()
    r = SimRobot(cfg.robot)
    r.set_height(100.0)
    r._poll_state()
    assert r.proc_dir is None and not os.listdir(tmp_path)


def test_logzeilen_format_live_php():
    # live.php nimmt nur Zeilen mit genau 5 ganzzahligen Feldern (seq t_ns roll pitch yaw / seq t_ns l r fw_ms)
    z = imu_zeile(7, 123456789, 0.0349, -90.5).split()
    assert len(z) == 5 and all(s.lstrip("-").isdigit() for s in z)
    assert int(z[3]) == -2000 and int(z[4]) == -90500                   # Nick + = vorn tief, Milligrad
    w = rad_zeile(7, 123456789, 1024, -3, 500).split()
    assert len(w) == 5 and w[2:] == ["1024", "-3", "500"]


def test_hub_begrenzt_kopf_bleibt():
    h = Hub(max_lines=3)
    for s in ["kopf", "a", "b", "c", "d"]:
        h.add(s)
    assert h.n == 5 and h.lines == ["b", "c", "d"]
    neu, i = h.ab(0)
    assert neu == ["kopf", "b", "c", "d"] and i == 5
    h.add("e")
    neu, i = h.ab(i)
    assert neu == ["e"] and i == 6
    voll = Hub()
    for s in ["k", "x"]:
        voll.add(s)
    assert voll.ab(0) == (["k", "x"], 2)
