#!/usr/bin/env python3
"""Schema aller Einstellungen für die Weboberfläche (web/config-schema.json) aus xgonav/config.py.

Läuft am PC; die JSON wird mit web/ eingespielt. Am Rider rechnet dafür nichts.
Je Feld: Abschnitt, Schlüssel, Typ, Default, Hilfetext (Kommentare aus config.py).

    python tools/config_schema.py            # schreibt web/config-schema.json
    python tools/config_schema.py --check    # nur prüfen, ob die Datei aktuell ist (Rückgabe 1 sonst)
"""
from __future__ import annotations

import ast
import dataclasses
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from xgonav.config import Config  # noqa: E402

CONFIG_PY = ROOT / "xgonav" / "config.py"
ZIEL = ROOT / "web" / "config-schema.json"
OBEN = "allgemein"          # Pseudo-Abschnitt für Schlüssel auf oberster Ebene (control_hz)

KATEGORIEN = [
    ("Hardware", ["robot", "camera", "imu", "odom", "cpu", OBEN]),
    ("Wahrnehmung", ["ego", "detector", "tracker", "freespace", "horizon"]),
    ("Karten", ["map", "grid", "reloc"]),
    ("Verhalten", ["behavior", "explore", "home", "patrol", "pet", "emotion", "ruhe"]),
    ("Ausgabe", ["output"]),
]


def _kommentare(quelle: str) -> tuple[dict[str, dict[str, str]], dict[str, str]]:
    """Hilfetexte je Klasse und Feld (Kommentarblock darüber + Inline-Kommentar) und Klassen-Docstrings."""
    zeilen = quelle.splitlines()
    felder: dict[str, dict[str, str]] = {}
    docs: dict[str, str] = {}
    for knoten in ast.parse(quelle).body:
        if not isinstance(knoten, ast.ClassDef):
            continue
        docs[knoten.name] = " ".join((ast.get_docstring(knoten) or "").split())
        je = felder.setdefault(knoten.name, {})
        for st in knoten.body:
            if not (isinstance(st, ast.AnnAssign) and isinstance(st.target, ast.Name)):
                continue
            teile = []
            i = st.lineno - 2                      # Zeile direkt darüber (0-basiert)
            block = []
            while i >= 0 and zeilen[i].strip().startswith("#"):
                block.append(zeilen[i].strip().lstrip("#").strip())
                i -= 1
            teile += reversed(block)
            ende = zeilen[st.end_lineno - 1]
            if "#" in ende:
                # Inline-Kommentar: erstes # außerhalb von Strings
                tok = _inline(ende)
                if tok:
                    teile.append(tok)
            je[st.target.id] = " ".join(" ".join(teile).split())
    return felder, docs


def _inline(zeile: str) -> str:
    q = None
    for i, c in enumerate(zeile):
        if q:
            if c == q:
                q = None
        elif c in "\"'":
            q = c
        elif c == "#":
            return zeile[i + 1:].strip()
    return ""


def _typ(annot: str, default) -> dict:
    a = annot.replace(" ", "")
    nullbar = a.endswith("|None")
    a = a.removesuffix("|None")
    if a in ("bool", "int", "float", "str"):
        t = {"typ": a}
    elif a.startswith("list"):
        elem = "str" if a == "list[str]" else "zahl"
        t = {"typ": "liste", "elem": elem}
    else:
        raise ValueError(f"Typ {annot} nicht unterstützt")
    if nullbar:
        t["nullbar"] = True
    return t


def schema() -> dict:
    felder, docs = _kommentare(CONFIG_PY.read_text(encoding="utf-8"))
    cfg = Config()
    abschnitte: dict[str, dict] = {}
    oben = {"name": OBEN, "doc": "Schlüssel auf oberster Ebene (ohne Abschnitt)", "felder": []}
    for f in dataclasses.fields(cfg):
        wert = getattr(cfg, f.name)
        if dataclasses.is_dataclass(wert):
            cls = type(wert).__name__
            liste = []
            for g in dataclasses.fields(wert):
                d = getattr(wert, g.name)
                liste.append({"key": g.name, **_typ(str(g.type), d), "default": d,
                              "hilfe": felder.get(cls, {}).get(g.name, "")})
            abschnitte[f.name] = {"name": f.name, "doc": docs.get(cls, ""), "felder": liste}
        else:
            oben["felder"].append({"key": f.name, **_typ(str(f.type), wert), "default": wert,
                                   "hilfe": felder.get("Config", {}).get(f.name, "")})
    abschnitte[OBEN] = oben
    vergeben = {a for _, liste in KATEGORIEN for a in liste}
    fehlt = set(abschnitte) - vergeben
    if fehlt:
        raise SystemExit(f"Abschnitte ohne Kategorie: {sorted(fehlt)} – KATEGORIEN ergänzen")
    return {"oben": OBEN,
            "kategorien": [{"name": n, "abschnitte": [abschnitte[a] for a in liste]} for n, liste in KATEGORIEN]}


def text() -> str:
    return json.dumps(schema(), ensure_ascii=False, indent=1) + "\n"


def main() -> int:
    neu = text()
    if "--check" in sys.argv:
        alt = ZIEL.read_text(encoding="utf-8") if ZIEL.exists() else ""
        if alt != neu:
            print(f"{ZIEL} veraltet: python tools/config_schema.py")
            return 1
        return 0
    ZIEL.write_text(neu, encoding="utf-8", newline="\n")
    n = sum(len(a["felder"]) for k in schema()["kategorien"] for a in k["abschnitte"])
    print(f"{ZIEL}: {n} Einstellungen")
    return 0


if __name__ == "__main__":
    sys.exit(main())
