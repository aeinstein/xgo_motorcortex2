#!/bin/bash
# Host-Tests der Firmware-Patches (g++, ohne ESP32). Nach firmware/build.sh, aus dem Repo-Root:
#   firmware/tests/run.sh            # alle
#   firmware/tests/run.sh velocity   # nur die Tests, deren Name oder Patch passt
set -uo pipefail
cd "$(dirname "$0")/../.."
S=firmware/build/xgorider2_rl/firmware/src
[ -d "$S" ] || { echo "Quellen fehlen: erst firmware/build.sh" >&2; exit 2; }
CXX=${CXX:-g++}
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT

# Patch | Test | Quellen
TESTS="
01-ble|test_ble_tunnel|command_protocol.cpp
02-pi-interface|test_imu_stream|command_protocol.cpp
03-diagnostics|test_raw_diag|control/rider_leg_imu.cpp command_protocol.cpp
03-diagnostics|test_lift_log|action_engine.cpp
04-attitude|test_lagefusion_replay|control/rider_leg_imu.cpp
04-attitude|test_imu_yaw_coupling|control/rider_leg_imu.cpp
04-attitude|test_pitch_fusion|control/rider_leg_imu.cpp
04-attitude|test_yaw_bias|control/rider_leg_imu.cpp
04-attitude|test_gyro_boot_cal|control/rider_leg_imu.cpp
05-actions|test_action_table|action_engine.cpp
05-actions|test_battery_recover|board_support.cpp
06-leds|test_led_calibrating|board_support.cpp
07-balance|test_turn_hold|control/rider_motion.cpp control/pid.cpp
07-balance|test_roll_trim|control/rider_leg_imu.cpp
07-balance|test_drive_integral|control/rider_motion.cpp control/pid.cpp
07-balance|test_roll_hold|motion_controller.cpp control/rider_motion.cpp control/pid.cpp control/rider_leg_imu.cpp board_support.cpp
07-balance|test_curve_lean|motion_controller.cpp control/rider_motion.cpp control/pid.cpp control/rider_leg_imu.cpp
08-velocity|test_velocity_cmd|control/rider_motion.cpp control/pid.cpp command_protocol.cpp
08-velocity|test_velocity_ctrl|control/rider_motion.cpp control/pid.cpp
08-velocity|test_turn_ctrl|control/rider_motion.cpp control/pid.cpp
"
fail=0; n=0
while IFS='|' read -r patch name srcs; do
    [ -z "$patch" ] && continue
    [ $# -gt 0 ] && [[ "$patch $name" != *"$1"* ]] && continue
    files=(); for s in $srcs; do files+=("$S/$s"); done
    n=$((n + 1))
    if ! "$CXX" -std=c++17 -ffp-contract=off -I "$S" "firmware/tests/$name.cpp" "${files[@]}" -o "$OUT/$name" 2> "$OUT/$name.log"; then
        echo "FEHLER $patch $name: übersetzt nicht"; sed -n 1,20p "$OUT/$name.log"; fail=1; continue
    fi
    if "$OUT/$name" > "$OUT/$name.out" 2>&1; then echo "ok     $patch $name"
    else echo "FEHLER $patch $name"; grep -v "^ok " "$OUT/$name.out" | head -20; fail=1; fi
done <<< "$TESTS"
echo "$n Tests, $([ $fail = 0 ] && echo alle grün || echo FEHLER)"
exit $fail
