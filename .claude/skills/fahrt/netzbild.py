"""FloorNet-Grenze eines Laufs auf Bildern einer Fahrt zeichnen (auf kimmy, ~/floornet/sweep/netzbild.py):
  cd ~/floornet && CUDA_VISIBLE_DEVICES= .venv/bin/python sweep/netzbild.py lauf_rider <fahrt> /tmp/out.jpg <bildnr> ..."""
import sys, os
sys.path.insert(0, os.path.expanduser("~/floornet/repo/tools/floornet"))
import cv2, numpy as np, torch
from net import load_net, prep, decode, CROP_Y, CROP_H, IN_W, IN_H
lauf, fahrt, out = sys.argv[1:4]
nrs = sys.argv[4:]
net = load_net(lauf).eval()
src = os.path.expanduser("~/floornet/data/rider/fahrt/" + fahrt)
tiles = []
for n in nrs:
    img = cv2.imread("%s/%06d.jpg" % (src, int(n)))
    x = cv2.resize(img[CROP_Y:CROP_Y + CROP_H], (IN_W, IN_H), interpolation=cv2.INTER_AREA)
    t = torch.as_tensor(x[None]).permute(0, 3, 1, 2).float() / 255.0
    with torch.no_grad():
        row = decode(net(prep(t, False)), outlier_px=25.0, row_top=net.row_top)[0].numpy()
    v = img.copy()
    pts = np.array([[(c + 0.5) * 8, r] for c, r in enumerate(row)], np.int32)
    cv2.polylines(v, [pts], False, (0, 255, 0), 3)
    cv2.putText(v, "%s %s" % (n, os.path.basename(lauf.rstrip("/"))), (8, 30), 0, 0.9, (0, 255, 255), 2)
    tiles.append(cv2.resize(v, (400, 300)))
while len(tiles) % 4:
    tiles.append(np.zeros_like(tiles[0]))
cv2.imwrite(out, np.vstack([np.hstack(tiles[i:i + 4]) for i in range(0, len(tiles), 4)]))
print("ok", len(nrs))
