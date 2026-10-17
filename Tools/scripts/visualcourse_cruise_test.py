#!/usr/bin/env python3
"""
SITL regression test for ArduPlane visual course-hold (non-GPS CRUISE).

Measures the ground-course deviation while CRUISE holds a course locked by the
forward-flow crab estimate (AP_VisualCourse). Prints one greppable metrics line
and exits 0 (PASS) or 1 (FAIL).

Ground truth is GPS course. The visual loop is what actually steers because the
test forces VC_FORCE=1 (use visual course even though GPS is available); GPS is
only used as the measurement reference.

Usage (SITL must already be running):
    build/sitl/bin/arduplane -S -I1 --model plane --speedup 5 \
        --home=-35.363261,149.165230,584,353

    python3 Tools/scripts/visualcourse_cruise_test.py --port 5770

Options:
    --port 5770         TCP port of SITL instance (I1 -> 5770)
    --kp 0.8            VC_GAIN_P
    --kd 0.0            VC_GAIN_D
    --wind-spd 8        SIM_WIND_SPD (m/s)
    --wind-dir 90       SIM_WIND_DIR (deg, wind FROM this heading)
    --duration 18       measurement window (s)
    --max-dev 15.0      pass threshold on max course deviation (deg)
    --autotune          run the relay autotune (VC_AUTOTUNE=1) before measuring
    --takeoff-alt 70    takeoff target altitude (m)
    --no-cal            skip the accelerometer calibration (faster re-runs)
    --gps-off           R3: disable GPS before CRUISE (VC_FORCE=0, real fallback path);
                        course truth comes from SIMSTATE, also reports AHRS yaw error
                        and EKF position drift
    --timeout 240       overall wall-clock timeout (s)

Exit codes: 0 = PASS, 1 = FAIL, 2 = setup error.
"""
import argparse
import math
import statistics
import sys
import time

from pymavlink import mavutil

MODE_GUIDED = 15
MODE_CRUISE = 7
MODE_TAKEOFF = 13
MSG_SIMSTATE = 164
MSG_GLOBAL_POSITION_INT = 33
M_PER_DEG = 111319.5

MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
MAV_MODE_FLAG_SAFETY_ARMED = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
MAV_PARAM_TYPE_REAL32 = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
MAV_PARAM_TYPE_INT8 = mavutil.mavlink.MAV_PARAM_TYPE_INT8


def wrap_180(d):
    return (d + 180.0) % 360.0 - 180.0


class Test:
    def __init__(self, args):
        self.args = args
        self.m = None
        self.hit_ground = False

    def _check_ground_hit(self, msg):
        if msg.get_type() == "STATUSTEXT" and "Hit ground" in msg.text:
            self.hit_ground = True

    def log(self, msg):
        print(f"[vc-test] {msg}", flush=True)

    def connect(self):
        self.m = mavutil.mavlink_connection(f"tcp:127.0.0.1:{self.args.port}")
        self.m.wait_heartbeat(timeout=10)
        self.log("connected")

    def set_mode(self, num):
        self.m.mav.set_mode_send(self.m.target_system,
                                 MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, num)
        time.sleep(0.5)

    def param(self, name, val, ptype=MAV_PARAM_TYPE_REAL32):
        self.m.mav.param_set_send(self.m.target_system, self.m.target_component,
                                  name.encode(), float(val), ptype)

    def wait_gps_fix(self, timeout=40):
        end = time.time() + timeout
        while time.time() < end:
            msg = self.m.recv_match(type="GPS_RAW_INT", blocking=True, timeout=0.2)
            if msg is not None and msg.fix_type >= 3:
                return msg
        return None

    def arm(self, timeout=90):
        end = time.time() + timeout
        while time.time() < end:
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0)
            t0 = time.time()
            while time.time() - t0 < 2.0:
                msg = self.m.recv_match(blocking=True, timeout=0.2)
                if (msg is not None and msg.get_type() == "HEARTBEAT"
                        and msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED):
                    return True
        return False

    def _rel_alt_state(self, timeout):
        """Yield (rel_alt_m, climb, airspeed) updates until timeout.
        VFR_HUD.alt is AMSL, so altitude comes from GLOBAL_POSITION_INT.relative_alt."""
        end = time.time() + timeout
        rel_alt = None
        while time.time() < end:
            msg = self.m.recv_match(type=["VFR_HUD", "GLOBAL_POSITION_INT"],
                                    blocking=True, timeout=0.2)
            if msg is None:
                continue
            if msg.get_type() == "GLOBAL_POSITION_INT":
                rel_alt = msg.relative_alt * 1e-3
            elif rel_alt is not None:
                yield rel_alt, msg.climb, msg.airspeed

    def wait_airborne(self, timeout=90):
        for rel_alt, _, aspd in self._rel_alt_state(timeout):
            if rel_alt > 20 and aspd > 8:
                return True
        return False

    def wait_stable_altitude(self, target_alt, timeout=90):
        """Wait for the climb to finish and airspeed to build so the run starts level."""
        stable_since = 0.0
        for rel_alt, climb, aspd in self._rel_alt_state(timeout):
            if rel_alt > target_alt - 10 and abs(climb) < 1.5 and aspd >= 12:
                if stable_since == 0.0:
                    stable_since = time.time()
                elif time.time() - stable_since > 2.0:
                    return True
            else:
                stable_since = 0.0
        return False

    def wait_course_settled(self, timeout=40):
        """Wait until the GPS course is stable (~3 s within 3 deg) so the
        measurement excludes the post-lock transient."""
        end = time.time() + timeout
        window = []
        while time.time() < end:
            msg = self.m.recv_match(type="GPS_RAW_INT", blocking=True, timeout=0.2)
            if msg is None or msg.fix_type < 3 or msg.vel <= 300:
                continue
            window.append(msg.cog * 0.01)
            if len(window) < 30:
                continue
            window = window[-30:]
            mean = sum(window) / len(window)
            devs = [abs(wrap_180(c - mean)) for c in window]
            if max(devs) <= 3.0:
                return True
        return False

    def wait_autotune_done(self, timeout=90):
        end = time.time() + timeout
        while time.time() < end:
            msg = self.m.recv_match(type="STATUSTEXT", blocking=True, timeout=0.2)
            if msg is None:
                continue
            if "autotune" in msg.text.lower():
                print(f"[vc-test] {msg.text}")
            if "autotune done" in msg.text:
                return True
            if "autotune aborted" in msg.text:
                return False
        return False

    def measure(self, duration):
        courses = []
        end = time.time() + duration
        while time.time() < end:
            msg = self.m.recv_match(type=["GPS_RAW_INT", "STATUSTEXT"], blocking=True, timeout=0.2)
            if msg is None:
                continue
            self._check_ground_hit(msg)
            if msg.get_type() == "GPS_RAW_INT" and msg.fix_type >= 3 and msg.vel > 300:
                courses.append(msg.cog * 0.01)
        return courses

    def wait_gps_lost(self, timeout=15):
        end = time.time() + timeout
        while time.time() < end:
            msg = self.m.recv_match(type="GPS_RAW_INT", blocking=True, timeout=0.2)
            if msg is not None and msg.fix_type < 2:
                return True
        return False

    def measure_truth(self, duration):
        """GPS-denied measurement from SIMSTATE truth.
        Returns (courses_deg, yaw_errs_deg, max_drift_m, modes_seen)."""
        positions = []
        yaw_errs = []
        max_drift = 0.0
        att_yaw = None
        ekf_pos = None
        modes = set()
        while self.m.recv_match(blocking=False) is not None:
            pass  # drop messages queued before the mode change
        end = time.time() + duration
        while time.time() < end:
            msg = self.m.recv_match(
                type=["SIMSTATE", "ATTITUDE", "GLOBAL_POSITION_INT", "HEARTBEAT", "STATUSTEXT"],
                blocking=True, timeout=0.2)
            if msg is None or msg.get_srcSystem() != self.m.target_system:
                continue
            self._check_ground_hit(msg)
            t = msg.get_type()
            if t == "ATTITUDE":
                att_yaw = msg.yaw
            elif t == "GLOBAL_POSITION_INT":
                ekf_pos = (msg.lat * 1e-7, msg.lon * 1e-7)
            elif t == "HEARTBEAT":
                modes.add(msg.custom_mode)
            elif t == "SIMSTATE":
                lat, lng = msg.lat * 1e-7, msg.lng * 1e-7
                positions.append((time.time(), lat, lng))
                if att_yaw is not None:
                    yaw_errs.append(wrap_180(math.degrees(att_yaw - msg.yaw)))
                if ekf_pos is not None:
                    dn = (ekf_pos[0] - lat) * M_PER_DEG
                    de = (ekf_pos[1] - lng) * M_PER_DEG * math.cos(math.radians(lat))
                    max_drift = max(max_drift, math.hypot(dn, de))

        # true course between truth fixes ~0.5 s (wall) apart
        courses = []
        last = None
        for p in positions:
            if last is None or p[0] - last[0] >= 0.5:
                if last is not None:
                    dn = (p[1] - last[1]) * M_PER_DEG
                    de = (p[2] - last[2]) * M_PER_DEG * math.cos(math.radians(p[1]))
                    if math.hypot(dn, de) > 1.0:
                        courses.append(math.degrees(math.atan2(de, dn)) % 360.0)
                last = p
        return courses, yaw_errs, max_drift, modes

    def run(self):
        self.connect()
        a = self.args

        # streaming
        for mid, hz in ((24, 10), (30, 10), (74, 10), (MSG_SIMSTATE, 10),
                        (MSG_GLOBAL_POSITION_INT, 10)):
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                mid, int(1e6 / hz), 0, 0, 0, 0, 0)
            time.sleep(0.2)

        # make sure GPS is the truth source
        self.param("SIM_GPS_DISABLE", 0, MAV_PARAM_TYPE_INT8)

        if self.wait_gps_fix() is None:
            self.log("ERROR: no GPS fix")
            return 2

        if not a.no_cal:
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                0, 0, 0, 0, 4, 0, 0)
            time.sleep(5)

        # visual-course setup + environment
        self.param("VC_ENABLE", 1, MAV_PARAM_TYPE_INT8)
        self.param("VC_FORCE", 1, MAV_PARAM_TYPE_INT8)
        self.param("VC_GAIN_P", a.kp)
        self.param("VC_GAIN_D", a.kd)
        self.param("VC_AUTOTUNE", 1 if a.autotune else 0, MAV_PARAM_TYPE_INT8)
        self.param("SIM_WIND_SPD", a.wind_spd)
        self.param("SIM_WIND_DIR", a.wind_dir)
        self.param("LOG_DISARMED", 1, MAV_PARAM_TYPE_INT8)
        self.param("TKOFF_ALT", a.takeoff_alt)

        # Plane does not climb on NAV_TAKEOFF in GUIDED; TAKEOFF mode climbs to TKOFF_ALT
        self.set_mode(MODE_TAKEOFF)
        if not self.arm():
            self.log("ERROR: arm failed")
            return 2

        if not self.wait_airborne():
            self.log("ERROR: not airborne")
            return 2
        self.log("airborne")
        if not self.wait_stable_altitude(a.takeoff_alt):
            self.log("ERROR: altitude did not stabilise")
            return 2

        if a.gps_off:
            return self.run_gps_off()

        self.set_mode(MODE_CRUISE)

        if a.autotune:
            self.log("waiting for autotune to finish ...")
            if not self.wait_autotune_done():
                self.log("ERROR: autotune did not finish")
                return 2
            self.log("autotune done")

        # wait for the course-hold to converge so the metric is steady-state
        if not self.wait_course_settled():
            self.log("ERROR: course did not settle")
            return 2
        courses = self.measure(a.duration)

        if self.hit_ground:
            self.log("ERROR: aircraft hit the ground during measurement (invalid run)")
            return 2
        if len(courses) < 20:
            self.log("ERROR: not enough samples")
            return 2

        lock = courses[0]
        devs = [wrap_180(c - lock) for c in courses]
        max_dev = max(abs(d) for d in devs)
        std = statistics.stdev(devs)
        mean_abs = sum(abs(d) for d in devs) / len(devs)

        print(f"METRICS lock={lock:.1f} max_dev={max_dev:.2f} std={std:.2f} "
              f"mean_abs={mean_abs:.2f} n={len(courses)}")
        print(f"CONFIG kp={a.kp} kd={a.kd} wind_spd={a.wind_spd} "
              f"wind_dir={a.wind_dir} duration={a.duration} autotune={a.autotune}")

        ok = max_dev <= a.max_dev
        print(f"RESULT: {'PASS' if ok else 'FAIL'} "
              f"(max_dev {max_dev:.2f} vs threshold {a.max_dev:.2f})")
        return 0 if ok else 1

    def run_gps_off(self):
        a = self.args
        # exercise the real fallback path: GPS preferred, visual only because GPS is gone
        self.param("VC_FORCE", 0, MAV_PARAM_TYPE_INT8)
        # without GPS the synthetic airspeed freezes, so fly on the (simulated) pitot
        self.param("ARSPD_USE", 1, MAV_PARAM_TYPE_INT8)
        # mid throttle -> mid airspeed target; at AIRSPEED_MIN a headwind drops ground speed
        # below the SITL flow cutoff (3 m/s) and the visual course drops out
        self.param("RC_OVERRIDE_TIME", -1)
        ignore = 65535
        self.m.mav.rc_channels_override_send(
            self.m.target_system, self.m.target_component,
            ignore, ignore, 1500, ignore, ignore, ignore, ignore, ignore)
        time.sleep(3.0)
        self.param("SIM_GPS_DISABLE", 1, MAV_PARAM_TYPE_INT8)
        if not self.wait_gps_lost():
            self.log("WARNING: GPS_RAW_INT still reports a fix")
        self.log("GPS disabled")

        self.set_mode(MODE_CRUISE)
        time.sleep(5.0)  # lock + settle (wall time)

        courses, yaw_errs, drift, modes = self.measure_truth(a.duration)
        if self.hit_ground:
            self.log("ERROR: aircraft hit the ground during measurement (invalid run)")
            return 2
        if len(courses) < 20:
            self.log("ERROR: not enough truth samples (is SIMSTATE streamed?)")
            return 2

        lock = courses[0]
        devs = [wrap_180(c - lock) for c in courses]
        max_dev = max(abs(d) for d in devs)
        std = statistics.stdev(devs)
        mean_abs = sum(abs(d) for d in devs) / len(devs)
        yaw_max = max(abs(e) for e in yaw_errs) if yaw_errs else float("nan")
        stayed_cruise = modes == {MODE_CRUISE}

        print(f"METRICS lock={lock:.1f} max_dev={max_dev:.2f} std={std:.2f} "
              f"mean_abs={mean_abs:.2f} n={len(courses)} "
              f"yaw_err_max={yaw_max:.2f} ekf_drift_max={drift:.0f}m modes={sorted(modes)}")
        print(f"CONFIG gps_off=True kp={a.kp} kd={a.kd} wind_spd={a.wind_spd} "
              f"wind_dir={a.wind_dir} duration={a.duration}")

        ok = max_dev <= a.max_dev and stayed_cruise
        print(f"RESULT: {'PASS' if ok else 'FAIL'} "
              f"(max_dev {max_dev:.2f} vs threshold {a.max_dev:.2f}, "
              f"stayed_cruise={stayed_cruise})")
        return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5770)
    ap.add_argument("--kp", type=float, default=0.8)
    ap.add_argument("--kd", type=float, default=0.0)
    ap.add_argument("--wind-spd", type=float, default=8.0)
    ap.add_argument("--wind-dir", type=float, default=90.0)
    ap.add_argument("--duration", type=float, default=18.0)
    ap.add_argument("--max-dev", type=float, default=15.0)
    ap.add_argument("--autotune", action="store_true")
    ap.add_argument("--takeoff-alt", type=float, default=70.0)
    ap.add_argument("--no-cal", action="store_true")
    ap.add_argument("--gps-off", action="store_true")
    args = ap.parse_args()

    t = Test(args)
    try:
        return t.run()
    except KeyboardInterrupt:
        return 130
    except Exception as exc:  # noqa: BLE001
        print(f"[vc-test] ERROR: {exc}", flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())
