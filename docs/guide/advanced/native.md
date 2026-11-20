# C++ core

Everything that runs per frame on the CM4 is C++ (`xgonav/native/`), loaded by ctypes in `xgonav/fastlib.py`. Python only wires it together. Each function group has an ABI number and, where possible, a bit-identical Python path for tests.

| Library / file | Content |
|---|---|
| `libxgofast.so` | `egofast.cpp` (ego-motion), `floornet.cpp` (FloorNet, NEON), `gridfast.cpp` (room map), `navfast.cpp` (A*, frontier, local map, path check), `linsefast.cpp` (undistortion), `jpegfast.cpp` (hardware JPEG), `ortfast.cpp` (sound direction), `stimmfast.cpp` (voice samples) |
| `libxgocam.so` | `camfast.cpp`: libcamera directly, undistorts from the DMA buffer |

```bash
make -C xgonav/native        # on the Rider (libopencv-dev)
make -C xgonav/native cam    # camera library (libcamera-dev); rebuild after apt upgrade
```

| Rule | Why |
|---|---|
| change C++ and Python together | tests compare both bit for bit (`tests/test_navfast.py`, `test_*_native_matches_*`) |
| bump the ABI number when a struct changes | old `.so` is refused instead of crashing |
| replace `.so` atomically (`.tmp` + `mv`) | overwriting a loaded library crashes the running xgonav |

PC: `XGONAV_FASTLIB=navfast.so python -m pytest -q tests/` runs the OpenCV-free parts through C++ ([Testing](../../testing.md)). Reference: [Architecture](../../architecture.md).
