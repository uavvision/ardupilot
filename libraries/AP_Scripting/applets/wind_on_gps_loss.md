# Wind on GPS loss

`wind_on_gps_loss.lua` gives EKF3 a wind when the GPS is lost and keeps it while the EKF dead reckons. Without
it the EKF learns the wind slowly, or not at all in a GPS-denied flight (svitlich flight: it held about 2 m/s
while the real wind was about 8 m/s, and the aircraft ended 8.2 km from the landing site).

## Needs

The firmware patch `ahrs:set_wind(north, east, sigma)` (branch `Plane-4.5.4-uavnav-setwind`: `NavEKF3::setWind`,
`AP_AHRS::set_wind`, Lua binding, Replay record RSW3). Without it the script prints an error and stops.
EKF3 accepts the wind only while dead reckoning (no GPS, external nav or optical flow fused for about 1 s, no
odometry for 5 s) and with the wind states active (in flight). The script therefore keeps offering the wind in
steady flight; the first accepted call marks the start of dead reckoning.

## What it does

- Wind source (`SWND_SRC`): 0 = the wind set before flight (`SWND_SPD`, `SWND_DIR`, the direction it blows FROM,
  as in forecasts); 1 = auto: keep the wind the EKF learned from real GPS in flight (at least `SWND_GPS_S`
  seconds), unless a forecast is set and the learned wind is more than `SWND_SRC_TOL` from it; otherwise the
  forecast. A wind set before flight but changed in flight before the GPS loss does not override a learned wind;
  a change during the hold does.
- Real GPS: the primary receiver has a 3D fix, more than 5 m/s, and its position (within `SWND_GPS_TOL`) and
  speed (within 3 m/s) agree with the EKF's, so the EKF is using it. A position typed in on the ground (speed 0)
  and a spoofed receiver the EKF rejects do not count, either as GPS time or as "GPS back".
- Steady flight only: flying for `SWND_DELAY` s, not in TAKEOFF mode, not on an AUTO `NAV_TAKEOFF`, airspeed
  above `SWND_AS_MIN`. A wind given 0.3 s into a catapult launch locked a wrong wind in the Replay of the
  svitlich flight.
- Hold (`SWND_HOLD` = 1): after acceptance `EK3_WIND_P_NSE` is set to `SWND_HOLD_NSE` (0.01) and
  `EK3_WIND_PSCALE` to 0 (RAM only), so the airspeed fusion does not walk the wind away. They hold the wind
  approximately, not exactly (svitlich Replay: 7.3 to 8.05 m/s east in the first seconds, then steady). Both are
  restored when real GPS is back for 5 s, on disarm, or when disabled. If scripting restarts during the hold,
  the script finds the hold values at start and sets 0.1 / 1.0 (this aircraft's values, the firmware defaults).
- Changing `SWND_SPD` or `SWND_DIR` while dead reckoning re-sends that wind (operator update).
- Log: message `SWND` (Ev 1 accepted, 2 re-sent, 3 GPS back, 4 disarmed, 5 disabled; WN, WE, Sig, Src) and
  `[SWND]` text.

## Parameters (table key 171)

| Name | Default | Meaning |
|---|---|---|
| SWND_ENABLE | 1 | enable |
| SWND_SPD | -1 | wind speed set before flight, m/s (negative = none) |
| SWND_DIR | 0 | direction the wind comes from, deg true |
| SWND_SIGMA | 0.5 | uncertainty given to the EKF, m/s (0.5 to 20) |
| SWND_SRC | 1 | 0 set before flight, 1 auto |
| SWND_GPS_S | 30 | seconds of real GPS in flight to trust the learned wind |
| SWND_DELAY | 20 | seconds flying before a wind is given |
| SWND_AS_MIN | 12 | least airspeed, m/s (0 = no check) |
| SWND_HOLD | 1 | hold the wind while dead reckoning |
| SWND_HOLD_NSE | 0.01 | EK3_WIND_P_NSE while holding |
| SWND_GPS_TOL | 100 | m; GPS position must agree with the EKF's to count as real GPS |
| SWND_SRC_TOL | 3 | m/s; largest learned-vs-forecast difference for the learned wind to be used |

## SITL results (2026-11-19)

ArduPlane SITL, speedup 3, wind from the west (SIM_WIND_SPD 8, SIM_WIND_DIR 270, SIM_WIND_TURB 0; in SITL the
direction is the one the wind comes FROM), airspeed fused (ARSPD_USE 1). AUTO mission: takeoff, then legs of
2.5 km on a hexagon (all directions). Error = EKF position (XKF1 core 0 PN/PE) against the SIM2 true position.
Runs A to C used the first version of the script (commit 3f7b0ac08d); the GPS agreement test, the forecast
check, the AUTO take-off exclusion and the restart check were added after them and are not run in SITL.

Runs A and B: GPS disabled (SIM_GPS_DISABLE 1) at 11.6 m height, flown 651 s after that.

| Time after GPS loss | A no script, error m | A, EKF wind E m/s | B script (SRC 0, 8 m/s from 270), error m | B, EKF wind E m/s |
|---|---|---|---|---|
| 60 s | 282 | 1.5 | 42 | 7.9 |
| 120 s | 626 | 2.0 | 59 | 8.0 |
| 300 s | 1653 | 2.1 | 35 | 8.0 |
| 600 s | 3476 | 2.1 | 36 | 8.05 |
| end (651 s) | 3769 | 2.1 | 38 | 8.05 |
| max | 3769 | | 74 (at 106 s) | |

In B the wind was accepted 16.3 s after the GPS loss (first steady flight after takeoff); the EKF wind jumped
from 0.8 to 8.0 m/s east and the velocity east by the same 7.2 m/s in one step.

Run C: `SWND_SRC` 1, forecast 8 m/s from 250 (20 deg off), GPS kept for 60 s of flight, then lost. The script
gave the learned wind (N -0.9, E 6.2: the EKF had not converged to 8 in 60 s); error 50 / 117 / 204 m at
60 / 120 / 180 s. At 180 s the operator set `SWND_DIR` 270: re-sent N 0, E 8; error 218 / 247 / 272 m at
240 / 300 / 360 s (the error already built up stays). GPS back at 361 s: after 5 s the parameters returned to
0.1 / 1.0, error 0.7 m at 480 s. Disarm restore and a second outage were not run.
