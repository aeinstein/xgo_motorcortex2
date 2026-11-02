#!/bin/bash
# Treibercode im Userspace gegen einen ESP32-Emulator testen (kein Modul-Laden nötig).
#   driver-test/run.sh kernel_module [sekunden] [imu_hz] [--block] [--faults]
set -euo pipefail
KM=$(realpath "$1"); shift
HERE=$(cd "$(dirname "$0")" && pwd)
cp "$KM/xgo-serial.c" "$KM/xgo-serial.h" "$HERE/km/"
cp "$KM/../constants.h" "$HERE/constants.h"
cat > "$HERE/km/xgo-drv.h" <<'H'
#pragma once
#include "../kshim.h"
static int verbose = 0;
#include "../constants.h"
H
python3 - "$KM/xgo-drv.c" "$HERE/km/imu_extract.c" <<'PY'
import sys
src = open(sys.argv[1]).read()
a = src.index("static bool imu_read(struct imu_sample *s) {")
b = src.index("// Wird bei jedem speed_x/speed_z-Write")
open(sys.argv[2], "w").write(src[a:b].replace("abs(", "abs_s32("))
PY
gcc -O2 -Wall -Wno-pointer-sign -Wno-unused-function -Wno-unused-variable -I"$HERE/shim" \
    -o "$HERE/km/harness" "$HERE/km/test_main.c" -lpthread -lm
python3 "$HERE/esp32_emu.py" "$HERE/km/harness" "${1:-6}" "${2:-50}" "${@:3}"
