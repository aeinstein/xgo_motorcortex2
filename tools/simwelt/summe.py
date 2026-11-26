#!/usr/bin/env python3
"""Karten mehrerer Fahrten (schon im gemeinsamen Bezug) aufsummieren: python summe.py DIR NAME run1 run2 ..."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import karte

d, name, runs = sys.argv[1], sys.argv[2], sys.argv[3:]
acc = None
for r in runs:
    z = np.load(os.path.join(d, r + ".npz"))
    e = {"bod": z["boden"].astype(np.float64) * z["bn"][..., None], "bn": z["bn"].astype(np.float64), "band": z["band"].astype(np.float64),
         "top": z["top"].astype(np.float64) * z["tn"][..., None], "tn": z["tn"].astype(np.float64), "hist": z["hist"].astype(np.float32), "bahn": z["bahn"]}
    if acc is None:
        acc = e
    else:
        for k in ("bod", "bn", "band", "top", "tn", "hist"):
            acc[k] += e[k]
        acc["bahn"] = np.concatenate([acc["bahn"], e["bahn"]])
np.savez_compressed(os.path.join(d, name + ".npz"), boden=(acc["bod"] / np.maximum(acc["bn"], 1)[..., None]).astype(np.uint8),
                    bn=acc["bn"].astype(np.float32), band=acc["band"].astype(np.float32),
                    top=(acc["top"] / np.maximum(acc["tn"], 1)[..., None]).astype(np.uint8), tn=acc["tn"].astype(np.float32),
                    hist=np.minimum(acc["hist"], 65535).astype(np.uint16), bahn=acc["bahn"])
karte.bild(d, name, gross=3)
