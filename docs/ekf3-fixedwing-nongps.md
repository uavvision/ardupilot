# EKF3: Fixed-Wing Non-GPS Navigation with Visual Aiding

Date: 2026-10-17
Branch: `Plane-4.5.4-uavnav-estimator`
Scope: `libraries/AP_NavEKF3/`

This document records what was observed while inspecting the EKF3 state estimator,
and proposes upgrades to support **fixed-wing (Plane), non-GPS** flight aided by
**visual helper systems (VIO, optical flow)**.

---

## 1. Observed: Current Architecture

### 1.1 State vector (24 states)

Defined in `AP_NavEKF3_core.h::state_elements`.

| Index | State | Notes |
|---|---|---|
| 0–3 | Quaternion | NED → body rotation |
| 4–6 | Velocity | NED (m/s) |
| 7–9 | Position | NED (m) |
| 10–12 | Gyro bias | body frame (rad) |
| 13–15 | Accel bias | body frame (m/s) |
| 16–18 | Earth magnetic field | Gauss |
| 19–21 | Body magnetic field | Gauss |
| 22–23 | Wind velocity | horizontal N/E only (m/s) |

Prediction runs at ~83 Hz (`EKF_TARGET_DT = 12 ms`); measurements are buffered and
fused at the delayed fusion-time horizon with an output observer propagating the
solution to the current time (`calcOutputStates()`).

### 1.2 Aiding modes

`NavEKF3_core::PV_AidingMode`:

| Mode | Meaning |
|---|---|
| `AID_NONE` | No external aiding; synthetic constant-position / zero-velocity fusion constrains tilt drift |
| `AID_RELATIVE` | Optical flow or body-frame odometry; velocity bounded, position drifts |
| `AID_ABSOLUTE` | GPS, range beacons, or external nav (EXTNAV) position |

Transitions are managed in `setAidingMode()` and are largely **binary** — sources are
switched rather than blended, and source changes trigger position/velocity resets.

### 1.3 Configurable sources

`AP_NavEKF_Source` (3 source sets via `EK3_SRCx_*`):

- `posxy` / `velxy`: `NONE, GPS, BEACON, OPTFLOW, EXTNAV, WHEEL_ENCODER`
- `posz` / `velz`: `NONE, BARO, RANGEFINDER, GPS, BEACON, EXTNAV`
- `yaw`: `NONE, COMPASS, GPS, GPS_COMPASS_FALLBACK, EXTNAV, GSF`

### 1.4 Fixed-wing awareness already present

- `assume_zero_sideslip()` = `fly_forward && vehicle_class != GROUND`.
- `FuseAirspeed()` fuses TAS magnitude against `|V_ground − V_wind|`, primarily
  observing the 2-D wind states.
- `FuseSideslip()` fuses a synthetic β = 0 observation.
- `status.flags.dead_reckoning` (wind-relative nav, `doingWindRelNav`) is reported.
- A launch-without-magnetometer special case clamps NE velocity during launch accel
  in `SelectVelPosFusion()`.
- `EKFGSF_yaw` (independent yaw estimator) uses IMU + GPS NE velocity; TAS is used
  **only** for centripetal-acceleration compensation in coordinated turns.

---

## 2. Observed: Visual sensor paths and their limitations

### 2.1 Optical flow — hover-oriented

- `SelectFlowFusion()` / `EstimateTerrainOffset()` assume a **downward-facing** sensor
  (`prevTnb.c.z > DCM33FlowMin` tilt gate).
- The terrain-offset estimator refuses flow fusion when `!gpsIsInUse`,
  `PV_AidingMode == AID_RELATIVE`, or `velHorizSq < 25` (i.e. speed < 5 m/s).
- `getEkfControlLimits()` derives a ground-speed limit of
  `(maxFlowRate − 1.0) · heightAGL` and de-rates nav velocity gains above 5 m.
- Plane default is `FLOW_USE_DEFAULT = 2` (terrain-only, **not** nav).

**Consequence:** optical flow cannot aid a cruising fixed-wing; it is only usable in
slow/low flight such as an approach or flare.

### 2.2 VIO position + yaw (`writeExtNavData`)

- Fuses absolute NED position and quaternion with **spherical** `posErr` / `angErr`.
- `FuseVelPosNED()` contains `// To-Do: this posErr should come from external nav when
  fusing external nav position`.
- VIO yaw is weakly observable and handled without a fixed-wing-specific strategy.

**Consequence:** monocular-VIO scale drift translates directly into position drift at
fixed-wing speeds; the coarse spherical noise model cannot weight axes independently.

### 2.3 VIO velocity (`writeExtNavVelData`)

- Fuses raw NED velocity with a scalar error (`err`).
- No use of the wind model to separate VIO drift from true wind; no scale learning.

### 2.4 Body-frame odometry (`writeBodyFrameOdom`, `FuseBodyVel`)

- Converts body-frame `delPos` to body velocity and fuses velocity only.
- `delAng` is discarded except as an angular-rate correction.
- Gated by `EK3_FEATURE_BODY_ODOM` and wired for rover wheel-odometry semantics.

### 2.5 GSF yaw (`EKFGSF_yaw`)

- Innovation is GPS NE velocity only; TAS only compensates centripetal accel.
- No GPS ⇒ no GSF yaw estimate.

---

## 3. Observed: Gap summary

| Sensor | Current design assumption | Fixed-wing mismatch |
|---|---|---|
| Optical flow | Downward, slow, low, hover | Terrain estimator requires GPS or >5 m/s; speed limit scales with AGL; plane default is terrain-only |
| VIO position/yaw | Absolute + spherical noise | Scale drift → position drift at speed; coarse noise model; weak yaw |
| VIO velocity | Raw NED + scalar noise | No wind separation, no scale observability |
| Body odom | Rover wheel model, velocity only | Rotation delta unused; no fixed-wing wind-relative interpretation |
| GSF yaw | GPS velocity innovation | Unavailable without GPS |

Net effect: a fixed-wing without GPS can only dead-reckon on airspeed + wind (which is
unobservable without ground truth), and the visual sensors it would carry are wired for
hover, not cruise.

---

## 4. Proposed upgrades

### U1 — Wind-relative visual velocity fusion (key enabler)

Fuse VIO/flow velocity in the wind frame. Since
`V_ground = V_air + V_wind` and `V_air` is observable from airspeed + attitude:

- Treat visual velocity as an observation of `(V_N − w_N, V_E − w_E, V_D)`.
- The redundancy with airspeed is what makes **VIO scale error observable**.
- Add a forward-facing optical-flow model (planar scene along body X) so a forward
  camera provides a speed/drift measurement in cruise, replacing the hover-only
  `getEkfControlLimits()` logic.

### U2 — VIO scale / drift state (state augmentation)

Add opt-in states behind a feature flag (e.g. `EK3_FEATURE_EXTNAV_SCALE`):

- A metric **scale** state linking VIO deltas to metres, observable whenever airspeed
  and visual velocity are both available.
- Optionally a slow **position drift-rate** state.

Converts monocular VIO from a position source into a bounded-error velocity /
short-baseline source.

### U3 — Non-GPS heading: extend GSF-yaw with airspeed + visual velocity

Extend `EKFGSF_yaw::fuseVelData()` to accept:

- **Wind-relative velocity** from airspeed + zero-sideslip + wind estimate.
- **Visual velocity** as a second, GPS-independent horizontal-velocity source.

Restores yaw initialization/recovery (`EK3_GSF_USE`,
`EKFGSF_resetMainFilterYaw`) without GPS — critical at launch and after visual dropout.

### U4 — Full 6-DOF covariance from VIO

Replace spherical noise with per-axis position error, per-axis velocity error, and
optionally full pose covariance. Let the filter weight VIO height/velocity/position
independently, and exploit the strong-rotation / weak-translation characteristic of
visual odometry.

### U5 — Body-frame odometry for fixed-wing (attitude aiding)

In `FuseBodyVel`, also fuse the VIO rotation delta (`delAng`) as a relative
attitude/heading observation. Provides a yaw-rate/heading-rate constraint independent
of compass and GPS, naturally compatible with `assume_zero_sideslip()`.

### U6 — Graceful aiding-source blending and fallback

- VIO absolute → VIO velocity + airspeed wind (dead-reckon) → airspeed-only, with
  position covariance growing continuously rather than binary timeout / `AID_NONE`
  transitions.
- Add an "EXTNAV velocity-only, no position" source option so VIO can be fused as a
  velocity source without committing to its drifting position.
- Make `setAidingMode()` transitions less binary for fixed-wing.

### U7 — Camera/topographic aiding in the terminal phase

Reuse the terrain-offset estimator + rangefinder for landing/flare, where down-facing
flow becomes valid again (slow, low). Add a fixed-wing switch that enables down-flow
terrain/velocity fusion only when speed < threshold and AGL < threshold during approach.

---

## 5. Suggested sequencing

1. **U1 + U3** — largest capability gain, no state-vector surgery.
2. **U6** — "EXTNAV velocity-only" source mode + continuous-covariance fallback.
3. **U2 + U4** — scale state and per-axis VIO covariance.
4. **U5 + U7** — attitude aid and landing-phase flow.

---

## 6. Risks

- **Unobservability:** wind and VIO scale are only jointly observable during
  manoeuvres (turns, climbs); gate learning on excitation, similar to
  `setWindMagStateLearningMode()`.
- **Sideslip assumption:** `assume_zero_sideslip()` is violated in gusts/slips; VIO
  velocity can help relax it, so structure upgrades to eventually drop the
  zero-sideslip assumption.
- **VIO yaw:** never treat as absolute; prefer relative/rate aiding (U5) and
  wind-relative heading (U3).
- **Compute:** added states and full covariance raise 24-state matrix cost; gate behind
  `EK3_FEATURE_*` flags as already done for body odom / external nav.
