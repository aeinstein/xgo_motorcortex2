"""deploy/xgo-wlan auto: offener Hotspot RiderKerk ohne WLAN, Rückkehr zu bekannten Netzen.

Ein Schritt je Aufruf (XGO_WLAN_EINMAL=1) gegen falsche nmcli/iw/modprobe/sleep im PATH; deren Zustand liegt als
Dateien in einem Verzeichnis (verb, ip, profile, gaeste, rueck) und jeder Aufruf steht in calls.
"""
import os
import shutil
import subprocess
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

SKRIPT = os.path.join(os.path.dirname(__file__), "..", "deploy", "xgo-wlan")
SH = shutil.which("sh")
pytestmark = pytest.mark.skipif(SH is None, reason="kein sh")

FAKE_NMCLI = r"""#!/bin/sh
ST="$XGO_TEST_ST"
echo "nmcli $*" >> "$ST/calls"
case "$*" in
    *"GENERAL.CONNECTION dev show"*) cat "$ST/verb" 2>/dev/null ;;
    *"GENERAL.STATE dev show"*) echo "100 (verbunden)" ;;
    *"IP4.ADDRESS dev show"*) cat "$ST/ip" 2>/dev/null ;;
    *"TYPE,NAME con show"*) cat "$ST/profile" 2>/dev/null ;;
    *"802-11-wireless.ssid con show"*) [ "$(cat "$ST/verb" 2>/dev/null)" = xgo-hotspot ] && echo RiderKerk ;;
    "con add "*) [ -e "$ST/add_fehler" ] && { echo "Fehler: kein AP"; exit 1; }; : ;;
    *"con up id xgo-hotspot"*) echo xgo-hotspot > "$ST/verb"; echo 10.42.0.1/24 > "$ST/ip"; echo up > "$ST/operstate" ;;
    *"con down id xgo-hotspot"*|*"con delete id xgo-hotspot"*)
        if [ "$(cat "$ST/verb" 2>/dev/null)" = xgo-hotspot ]; then : > "$ST/verb"; : > "$ST/ip"; echo down > "$ST/operstate"; fi ;;
    *"dev connect wlan0"*)
        if [ -s "$ST/rueck" ]; then cp "$ST/rueck" "$ST/verb"; echo 192.168.1.9/24 > "$ST/ip"; echo up > "$ST/operstate"
        else exit 4; fi ;;
esac
exit 0
"""
FAKE_IW = r"""#!/bin/sh
echo "iw $*" >> "$XGO_TEST_ST/calls"
n=$(cat "$XGO_TEST_ST/gaeste" 2>/dev/null || echo 0)
i=0; while [ "$i" -lt "$n" ]; do echo "Station aa:bb:cc:dd:ee:0$i (on wlan0)"; i=$((i+1)); done
"""
FAKE_EINFACH = r"""#!/bin/sh
echo "$(basename "$0") $*" >> "$XGO_TEST_ST/calls"
exit 0
"""


def schreib(pfad, text):
    """Immer LF (sh liest sonst "up\\r")."""
    with open(pfad, "w", newline="\n") as f:
        f.write(text)


@pytest.fixture
def umg(tmp_path):
    bin_ = tmp_path / "bin"
    st = tmp_path / "st"
    run = tmp_path / "run"
    net = tmp_path / "net" / "wlan0"
    for d in (bin_, st, run, net):
        d.mkdir(parents=True)
    for name, text in (("nmcli", FAKE_NMCLI), ("iw", FAKE_IW), ("modprobe", FAKE_EINFACH),
                       ("sleep", FAKE_EINFACH)):
        p = bin_ / name
        schreib(p, text)
        p.chmod(0o755)
    schreib((st / "operstate"), "down\n")

    def lauf(**env_extra):
        schreib((net / "operstate"), (st / "operstate").read_text())
        env = dict(os.environ)
        env.update({"XGO_TEST_ST": st.as_posix(), "XGO_WLAN_RUN": run.as_posix(),
                    "XGO_WLAN_SYS": (tmp_path / "net").as_posix(), "XGO_WLAN_EINMAL": "1"})
        env["PATH"] = str(bin_) + os.pathsep + env["PATH"]
        env.update({k: str(v) for k, v in env_extra.items()})
        schreib((st / "calls"), "")
        p = subprocess.run([SH, SKRIPT, "auto"], env=env, capture_output=True, text=True, timeout=60)
        assert p.returncode == 0, p.stdout + p.stderr
        return p.stdout, (st / "calls").read_text().splitlines()

    lauf.st, lauf.run = st, run
    return lauf


def _hotspot_auf(calls):
    return any(c.startswith("nmcli con add") for c in calls)


def test_verbunden_tut_nichts(umg):
    schreib((umg.st / "operstate"), "up\n")
    schreib((umg.st / "verb"), "xgo-Heim\n")
    out, calls = umg(XGO_WLAN_WEG_S=1000)
    assert not _hotspot_auf(calls)
    assert [c for c in calls if not c.startswith("nmcli -g GENERAL.CONNECTION")] == []   # nur der Start-Check


def test_ohne_wlan_erst_nach_frist(umg):
    out, calls = umg(XGO_WLAN_WEG_S=40)          # 40 + 10 < 60
    assert not _hotspot_auf(calls)
    out, calls = umg(XGO_WLAN_WEG_S=50)          # 50 + 10 = 60: Hotspot
    add = [c for c in calls if c.startswith("nmcli con add")]
    assert len(add) == 1
    assert "ssid RiderKerk" in add[0] and "802-11-wireless.mode ap" in add[0] and "autoconnect no" in add[0]
    assert "security" not in add[0] and "psk" not in add[0]          # offen (Nutzerentscheidung)
    assert "nmcli --wait 30 con up id xgo-hotspot" in calls
    assert (umg.run / "xgo-wlan.hotspot").exists()


def test_hotspot_fehlgeschlagen_setzt_zurueck(umg):
    schreib((umg.st / "add_fehler"), "1")
    out, calls = umg(XGO_WLAN_WEG_S=50)
    assert "Hotspot fehlgeschlagen" in out
    assert any(c.startswith("modprobe -r") for c in calls)           # Chip-Reset
    assert not (umg.run / "xgo-wlan.hotspot").exists()


def _im_hotspot(umg):
    schreib((umg.st / "verb"), "xgo-hotspot\n")
    schreib((umg.st / "ip"), "10.42.0.1/24\n")
    schreib((umg.st / "operstate"), "up\n")
    schreib((umg.run / "xgo-wlan.hotspot"), "")


def test_im_hotspot_vor_ablauf_nichts(umg):
    _im_hotspot(umg)
    schreib((umg.st / "profile"), "802-11-wireless:xgo-Heim\n802-11-wireless:xgo-hotspot\n")
    out, calls = umg(XGO_WLAN_HS_S=100)
    assert not any("dev connect" in c or c.startswith("iw") for c in calls)


def test_im_hotspot_ohne_profil_bleibt(umg):
    _im_hotspot(umg)
    schreib((umg.st / "profile"), "802-11-wireless:xgo-hotspot\n802-3-ethernet:Kabel\n")
    out, calls = umg(XGO_WLAN_HS_S=290)
    assert not any("dev connect" in c or "con delete" in c for c in calls)


def test_im_hotspot_mit_gast_bleibt(umg):
    _im_hotspot(umg)
    schreib((umg.st / "profile"), "802-11-wireless:xgo-Heim\n802-11-wireless:xgo-hotspot\n")
    schreib((umg.st / "gaeste"), "1")
    out, calls = umg(XGO_WLAN_HS_S=290)
    assert "bleibt" in out
    assert not any("dev connect" in c or "con delete" in c for c in calls)


def test_rueckkehr_gelingt(umg):
    _im_hotspot(umg)
    schreib((umg.st / "profile"), "802-11-wireless:xgo-Heim\n802-11-wireless:xgo-hotspot\n")
    schreib((umg.st / "rueck"), "xgo-Heim\n")
    out, calls = umg(XGO_WLAN_HS_S=290)
    assert "verbunden mit xgo-Heim" in out
    assert any(c.startswith("modprobe -r") for c in calls)           # Chip-Reset nach dem AP-Modus
    assert not _hotspot_auf(calls)
    assert not (umg.run / "xgo-wlan.hotspot").exists()


def test_rueckkehr_misslingt_hotspot_wieder(umg):
    _im_hotspot(umg)
    schreib((umg.st / "profile"), "802-11-wireless:xgo-Heim\n802-11-wireless:xgo-hotspot\n")
    out, calls = umg(XGO_WLAN_HS_S=290)
    assert "wieder auf" in out
    i = calls.index("nmcli --wait 45 dev connect wlan0")
    assert any(c.startswith("nmcli con add") for c in calls[i:])
    assert (umg.run / "xgo-wlan.hotspot").exists()


def test_hotspot_verschwunden_neu(umg):
    schreib(umg.run / "xgo-wlan.hotspot", "")    # Merkdatei da, aber wlan0 nicht oben
    out, calls = umg()
    assert "ist weg" in out and _hotspot_auf(calls)


def test_ohne_wlan0_nichts(umg, tmp_path):
    shutil.rmtree(tmp_path / "net" / "wlan0")
    (tmp_path / "net" / "wlan0").mkdir()
    env_net = tmp_path / "leer"
    env_net.mkdir()
    out, calls = umg(XGO_WLAN_SYS=env_net.as_posix(), XGO_WLAN_WEG_S=1000)
    assert "kein wlan0" in out and not _hotspot_auf(calls)


def test_status_zeigt_letzten_fehler(umg):
    _im_hotspot(umg)
    schreib((umg.run / "xgo-wlan.letzt"), "Heim: Secrets were required\n")
    env = dict(os.environ, XGO_TEST_ST=umg.st.as_posix(), XGO_WLAN_RUN=umg.run.as_posix())
    env["PATH"] = str(umg.st.parent / "bin") + os.pathsep + env["PATH"]
    p = subprocess.run([SH, SKRIPT, "status"], env=env, capture_output=True, text=True, timeout=30)
    kv = dict(z.split("=", 1) for z in p.stdout.splitlines() if "=" in z)
    assert kv["con"] == "xgo-hotspot" and kv["ssid"] == "RiderKerk" and kv["clients"] == "0"
    assert kv["letzt"] == "Heim: Secrets were required"
