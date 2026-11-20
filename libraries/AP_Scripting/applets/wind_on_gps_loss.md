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

## Setting up the aircraft (once)

1. Flash ArduPlane built from `Plane-4.5.4-uavnav-setwind`.
2. Set `SCR_ENABLE` 1 and reboot.
3. Copy `wind_on_gps_loss.lua` to `APM/scripts/` on the SD card (in SITL: `scripts/` in the directory SITL runs
   in) and reboot. At boot the GCS shows `[SWND] Wind on GPS loss, version ...` and the `SWND_` parameters
   appear. `[SWND] firmware has no ahrs:set_wind, script stopped` means the firmware is not from this branch.
4. For Replay of the flight, set `LOG_REPLAY` 1 and `LOG_DISARMED` 1. The accepted wind is then logged as
   `RSW3`, and Replay applies it at the same EKF frame. A wind accepted before logging starts is not logged.
5. Choose the wind source, `SWND_SRC`. 1 (auto) uses the wind the EKF learned from GPS if it had at least
   `SWND_GPS_S` seconds of GPS in flight, otherwise the forecast; 0 always uses the forecast.
6. Leave `EK3_WIND_P_NSE` and `EK3_WIND_PSCALE` at the aircraft's normal values. The script changes them in RAM
   only while it holds the wind.

All the SITL results below were obtained with an airspeed sensor fused (`ARSPD_USE` 1); without one this has not
been tried.

## Before each flight

- Set `SWND_SPD` and `SWND_DIR` to the forecast or measured wind at flight altitude. `SWND_DIR` is the direction
  the wind blows FROM, degrees true, as in forecasts: 8 m/s from the west is `SWND_SPD` 8, `SWND_DIR` 270.
  `SWND_SPD` -1 means no forecast; with `SWND_SRC` 0 the script then gives nothing.
- `SWND_ENABLE` 1.

## In flight

Nothing happens while the GPS works. The script waits for steady flight: `SWND_DELAY` s after the start of the
flight, not in TAKEOFF mode, not on an AUTO `NAV_TAKEOFF`, airspeed above `SWND_AS_MIN`. It then offers the
wind every 0.5 s, and EKF3 accepts it about 1 s after it stops fusing GPS. Messages:

| Message | Meaning |
|---|---|
| `[SWND] wind set N n E e m/s given (set before flight)` or `(learned from GPS)` | accepted; the wind is held from now on |
| `[SWND] learned wind N n E e far from forecast, using forecast` | the learned wind is more than `SWND_SRC_TOL` from the forecast |
| `[SWND] wind re-sent N n E e m/s given (...)` | you changed `SWND_SPD` or `SWND_DIR` while dead reckoning; the new wind was given |
| `[SWND] GPS back, wind params restored` | GPS in use again for 5 s; `EK3_WIND_P_NSE`, `EK3_WIND_PSCALE` restored, ready for the next loss |
| `[SWND] disarmed, ...` / `[SWND] disabled, ...` | restored on disarm, or on `SWND_ENABLE` 0 |

- **Operator update:** during the outage, a better wind estimate can be sent by changing `SWND_SPD` and
  `SWND_DIR` (both in the FROM convention). The new wind is re-sent at once and replaces the held one. It does not
  undo the position error built up so far.
- **Stop holding:** set `SWND_ENABLE` 0. The EKF parameters are restored and the EKF learns the wind from airspeed
  again.

## After the flight: the log

- `SWND`: Ev 1 accepted, 2 re-sent, 3 GPS back, 4 disarmed, 5 disabled; WN, WE (m/s, the direction the air
  moves TO), Sig, Src (0 set before flight, 1 learned).
- `RSW3` (only with `LOG_REPLAY` 1): the wind EKF3 accepted, WN, WE, Sigma.
- `XKF2` VWN, VWE: the EKF wind per core. It steps to the given wind at the `SWND` Ev 1 time.
- `XKF1` PN, PE: the EKF position. In SITL, compare it with the true position in `SIM2`.

## Replay

Build once: `./waf configure --board sitl` and `./waf replay`, giving `build/sitl/tool/Replay`. Replay writes
`logs/00000001.BIN` into the current directory, so run it in an empty one. The replayed EKF cores are logged with
`C` 100 and 101, next to the flown `C` 0 and 1.

- `Replay flight.BIN`: replays the EKF with the logged `RSW3`. Check that `XKF2` `C`100 matches `C`0.
- `Replay --set-wind=T,N,E,SIGMA flight.BIN`: from log time T (seconds since boot, as `TimeUS`), offers wind N,
  E (m/s, TO convention) with SIGMA every frame until EKF3 accepts it, then prints `set-wind accepted at ... s`.
  Use it to try a forecast on a flight that was flown without one, for example `--set-wind=1975,0,8,0.5`. A
  logged `RSW3` is applied as well, so for a clean comparison use a log flown without the script.

Checked on a SITL log (2026-11-20): plain Replay reproduced the flown wind step at the same 88.0 s
(VWE 7.88 → 8.05 m/s), and `--set-wind=150,0,4,0.5` was accepted at 150.005 s, with `C`100 VWE going from 8.24
to 4.00 m/s.

## Calling `ahrs:set_wind` from another script

`ahrs:set_wind(north, east, sigma)` returns true if at least one EKF3 core accepted the wind. Rules:

- North and east are the direction the air moves TO, in m/s. From a forecast: `north = -spd * cos(dir)`,
  `east = -spd * sin(dir)`.
- It returns false and changes nothing unless EKF3 is dead reckoning. So call it repeatedly, for example every
  0.5 s, until it returns true, and only in steady flight, not during a launch.
- Arguments outside the binding's ranges (north and east -100 to 100, sigma 0.5 to 20) raise a Lua error that
  stops the calling script. Clamp them, or call it through `pcall(ahrs.set_wind, ahrs, n, e, s)`.
- On acceptance the EKF ground velocity changes by the same amount as the wind (in SITL, 8 m/s east for a
  wind change of 8 m/s east). Position does not jump.
- The airspeed fusion keeps adjusting the wind. To hold it, set `EK3_WIND_P_NSE` 0.01 and `EK3_WIND_PSCALE` 0
  while dead reckoning and restore them when GPS returns, as this applet does.

## Tests

Two ArduPlane autotests on this branch, run in the `ardupilot:Plane-4.5.4` image:
`Tools/autotest/autotest.py --no-clean test.Plane.SetWindEKF3 test.Plane.WindOnGPSLoss`.

- `SetWindEKF3` uses `libraries/AP_Scripting/tests/set_wind_test.lua` and checks the binding:
  - refused on the ground and in flight with GPS;
  - accepted about 5 s after the GPS is disabled, with wind and ground velocity stepping together;
  - a Lua error for out-of-range arguments;
  - refused again once GPS is back.
- `WindOnGPSLoss` flies this applet in SITL wind of 8 m/s from 270:
  - wind given at the GPS loss and held (EKF3 wind within 1.5 m/s for 120 s);
  - EKF within 150 m of the true position;
  - hold parameters set, then restored when GPS returns;
  - `RSW3` logged.

  It passed with a largest error of 29 m. The EKF had already learned 7.9 m/s from 25 s of GPS flight before the
  loss, so this checks the applet's behaviour, not its benefit; for the benefit see runs A and B below.

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
