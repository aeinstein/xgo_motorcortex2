# Calibration

Defaults in `xgonav/config.py` are the values of one RiderPi. Another unit needs at least the steps below. Run on the Rider in `/opt/xgonav`, write results into `config.toml` (or via *Einstellungen*). Order matters. Full procedure: [Calibration](../../calibration.md).

| # | What | Key | How |
|---|---|---|---|
| 0 | horizontal FOV | `[camera] hfov_deg` | turn ≥ 360° on the spot, compare image rotation with IMU heading |
| 1 | turn direction and scale | `[robot] vyaw_limit_dps` | `python tools/calibrate.py turn` |
| 2 | camera height and tilt | `[camera] mount_offset_mm`, `tilt_deg`, `[imu] pitch_ref_deg` | floor grid / folding rule |
| 3 | IMU time offset and sign | `[imu] time_offset_ms`, `pitch_sign` | fixed on the RiderPi; `Config.neutral()` = automatic |
| 4 | wheel odometry | `[odom] scale_left`, `scale_right`, `track_m` | tape measure, full turns |
| 5 | drive gains | `[robot] reverse_gain`, `manual_forward_gain` | |

Positioning helpers (balancing Rider): `python tools/nudge.py turn <deg>`, `python tools/nudge.py move <s>`.

!!! danger "Leg calibration is locked"
    Saving leg zero points with slack legs overwrote them in the ESP32 flash once. The web buttons are disabled; see [Calibration](../../calibration.md#locked-leg-calibration).
