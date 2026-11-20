#!/usr/bin/env python3
"""BLE-Dienst des Riders vom PC aus prüfen und bedienen (Firmware-Patch 0025, docs/ble.md).

Läuft am PC mit Bluetooth, nicht am Rider. Braucht `pip install bleak`.

    python tools/ble_test.py                      # Prüflauf: IP, Akku/Version (CMD), !status (CON), wlan status + nav (PI)
    python tools/ble_test.py ip                   # IP des Pi
    python tools/ble_test.py con "!status"        # Zeile an die Konsole der Firmware
    python tools/ble_test.py read 0x07            # Register lesen (Binärrahmen)
    python tools/ble_test.py write 0x69 255 0 0   # Register schreiben, hier LED 0 rot
    python tools/ble_test.py pi "wlan scan"       # Anfrage an den Pi (die laufende Nummer setzt das Werkzeug davor)
    python tools/ble_test.py pi "nav GET /status"
    python tools/ble_test.py wlan SSID [PASSWORT] # WLAN des Pi setzen (stellt das Netz um!)

Bewegt den Rider nicht von selbst; `write 0x30 ...`, `con "!move ..."` oder `pi "nav POST /mode..."` tun das sehr wohl.
"""
import argparse
import asyncio
import sys
from urllib.parse import quote

BASIS = "fdacc797-7e1d-4d87-9205-64ca79f745%02x"
SERVICE, CMD, CON, PI, IP = (BASIS % n for n in range(5))
NAME = "RiderKerk"


def rahmen(typ, reg, daten=b""):
    ln = len(daten) + 8
    chk = 255 - ((ln + typ + reg + sum(daten)) % 256)
    return bytes([0x55, 0x00, ln, typ, reg]) + bytes(daten) + bytes([chk, 0x00, 0xAA])


def rahmen_lesen(puffer):
    """Ersten vollständigen Rahmen aus dem Puffer: (typ, reg, daten) oder None."""
    i = puffer.find(b"\x55\x00")
    if i < 0 or len(puffer) < i + 3:
        return None
    ln = puffer[i + 2]
    if ln < 8 or len(puffer) < i + ln:
        return None
    f = bytes(puffer[i:i + ln])
    del puffer[:i + ln]
    if f[-2:] != b"\x00\xaa" or f[-3] != 255 - (sum(f[2:-3]) % 256):
        return None
    return f[3], f[4], f[5:-3]


class Rider:
    def __init__(self, client):
        self.c = client
        self.puffer = {CMD: bytearray(), CON: bytearray(), PI: bytearray()}
        self.neu = asyncio.Event()
        self.nummer = 0

    async def start(self):
        for u in (CMD, CON, PI):
            await self.c.start_notify(u, lambda _, d, u=u: (self.puffer[u].extend(d), self.neu.set()))

    async def _warte(self, fertig, timeout):
        async def lauf():
            while True:
                r = fertig()
                if r is not None:
                    return r
                self.neu.clear()
                await self.neu.wait()
        return await asyncio.wait_for(lauf(), timeout)

    async def ip(self):
        return ".".join(str(b) for b in await self.c.read_gatt_char(IP))

    async def lesen(self, reg, n=1, timeout=3.0):
        self.puffer[CMD].clear()
        await self.c.write_gatt_char(CMD, rahmen(0x02, reg, bytes([n])), response=True)
        typ, r, daten = await self._warte(lambda: rahmen_lesen(self.puffer[CMD]), timeout)
        return daten

    async def schreiben(self, reg, daten):
        await self.c.write_gatt_char(CMD, rahmen(0x01, reg, bytes(daten)), response=True)

    async def konsole(self, zeile, ruhe=0.6, timeout=5.0):
        """Zeile senden, Ausgabe sammeln, bis `ruhe` Sekunden nichts mehr kommt (manche Befehle antworten nicht)."""
        self.puffer[CON].clear()
        await self.c.write_gatt_char(CON, (zeile.strip() + "\n").encode(), response=True)
        ende = asyncio.get_running_loop().time() + timeout
        n = -1
        while asyncio.get_running_loop().time() < ende and n != len(self.puffer[CON]):
            n = len(self.puffer[CON])
            await asyncio.sleep(ruhe)
        return self.puffer[CON].decode(errors="replace")

    async def pi(self, anfrage, timeout=15.0):
        """Anfrage an xgo-ble: Rückgabe (code, rumpf)."""
        self.nummer += 1
        ident = str(self.nummer)
        self.puffer[PI].clear()
        daten = ("%s %s\n" % (ident, anfrage)).encode()
        for i in range(0, len(daten), 160):
            await self.c.write_gatt_char(PI, daten[i:i + 160], response=True)

        def fertig():
            p = self.puffer[PI]
            k = p.find(b"\n")
            if k < 0:
                return None
            kopf = bytes(p[:k]).split()
            if len(kopf) != 3:
                raise RuntimeError("Antwort nicht verstanden: %r" % bytes(p[:k]))
            n = int(kopf[2])
            if len(p) < k + 1 + n:
                return None
            if kopf[0].decode() != ident:
                raise RuntimeError("Antwort auf %s statt %s" % (kopf[0].decode(), ident))
            return int(kopf[1]), bytes(p[k + 1:k + 1 + n])
        return await self._warte(fertig, timeout)


async def pruefen(r):
    ok = True

    def zeile(gut, text):
        nonlocal ok
        ok = ok and gut
        print("%s %s" % ("ok    " if gut else "FEHLER", text))

    ip = await r.ip()
    zeile(True, "IP des Pi: %s%s" % (ip, "  (0.0.0.0 = kein Netz oder der Treiber meldet nichts)" if ip == "0.0.0.0" else ""))
    try:
        akku = (await r.lesen(0x01, 1))[0]
        version = (await r.lesen(0x07, 10)).split(b"\0")[0].decode()
        ip2 = ".".join(str(b) for b in await r.lesen(0x18, 4))
        zeile(version.startswith("R-") and ip2 == ip, "CMD: Version %s, Akku %d %%, IP aus Register 0x18 %s" % (version, akku, ip2))
    except asyncio.TimeoutError:
        zeile(False, "CMD: keine Antwort auf Leserahmen")
    status = await r.konsole("!status")
    zeile(status.lstrip().startswith("{") and status.rstrip().endswith("}"),
          "CON: !status, %d Zeichen%s" % (len(status), "" if status.rstrip().endswith("}") else " (unvollständig)"))
    try:
        code, rumpf = await r.pi("wlan status")
        zeile(code == 0, "PI: wlan status -> %d  %s" % (code, rumpf.decode(errors="replace").strip().replace("\n", ", ")))
        code, rumpf = await r.pi("nav GET /status")
        zeile(code in (200, 599), "PI: nav GET /status -> %d, %d Byte%s" % (code, len(rumpf), " (xgonav läuft nicht)" if code == 599 else ""))
    except asyncio.TimeoutError:
        zeile(False, "PI: keine Antwort (läuft xgo-ble, ist der Treiber mit /proc/XGORider/ble geladen?)")
    return ok


async def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("befehl", nargs="?", default="check", choices=["check", "ip", "con", "read", "write", "pi", "wlan"])
    ap.add_argument("args", nargs="*")
    ap.add_argument("--name", default=NAME, help="Gerätename (Default RiderKerk)")
    ap.add_argument("--adresse", help="BLE-Adresse statt Suche nach dem Namen")
    a = ap.parse_args()
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError:
        sys.exit("bleak fehlt: pip install bleak")

    ziel = a.adresse
    if not ziel:
        print("suche %s ..." % a.name)
        ziel = await BleakScanner.find_device_by_name(a.name, timeout=15.0)
        if ziel is None:
            sys.exit("kein Gerät %r gefunden" % a.name)
    async with BleakClient(ziel) as client:
        if not any(str(s.uuid).lower() == SERVICE for s in client.services):
            sys.exit("Dienst %s fehlt: alte Firmware?" % SERVICE)
        r = Rider(client)
        await r.start()
        if a.befehl == "check":
            sys.exit(0 if await pruefen(r) else 1)
        if a.befehl == "ip":
            print(await r.ip())
        elif a.befehl == "con":
            print(await r.konsole(" ".join(a.args)), end="")
        elif a.befehl == "read":
            reg = int(a.args[0], 0)
            n = int(a.args[1], 0) if len(a.args) > 1 else 1
            print((await r.lesen(reg, n)).hex(" "))
        elif a.befehl == "write":
            await r.schreiben(int(a.args[0], 0), [int(x, 0) for x in a.args[1:]])
        elif a.befehl == "pi":
            code, rumpf = await r.pi(" ".join(a.args), timeout=90.0)
            print(code)
            print(rumpf.decode(errors="replace"), end="")
        elif a.befehl == "wlan":
            ssid = quote(a.args[0], safe="")
            pw = quote(a.args[1], safe="") if len(a.args) > 1 else ""
            code, rumpf = await r.pi("wlan connect %s 0 %s %s" % ("WPA" if pw else "NOPASS", ssid, pw), timeout=150.0)
            print(code)
            print(rumpf.decode(errors="replace"), end="")
            await asyncio.sleep(4.0)
            print("IP:", await r.ip())


if __name__ == "__main__":
    asyncio.run(main())
