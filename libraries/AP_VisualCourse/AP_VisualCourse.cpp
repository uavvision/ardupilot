#include "AP_VisualCourse.h"

#if AP_VISUALCOURSE_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#endif

#include "LogStructure.h"

extern const AP_HAL::HAL& hal;

AP_VisualCourse *AP_VisualCourse::_singleton;

const AP_Param::GroupInfo AP_VisualCourse::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Visual course-hold enable
    // @Description: Enable non-GPS course-hold using a forward-facing optical flow sensor
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_VisualCourse, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: GAIN_P
    // @DisplayName: Visual course proportional gain
    // @Description: Roll command (rad) per radian of crab angle
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("GAIN_P", 1, AP_VisualCourse, _gain_p, 1.0f),

    // @Param: GAIN_D
    // @DisplayName: Visual course derivative gain
    // @Description: Roll command (rad) per rad/s of crab rate
    // @Range: 0 5
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("GAIN_D", 2, AP_VisualCourse, _gain_d, 0.2f),

    // @Param: QUAL_MIN
    // @DisplayName: Visual course minimum flow quality
    // @Description: Minimum optical flow quality (0..255) required to be healthy
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("QUAL_MIN", 3, AP_VisualCourse, _qual_min, 100),

    // @Param: TIMEOUT_MS
    // @DisplayName: Visual course data timeout
    // @Description: Maximum age of flow data (ms) before declaring unhealthy
    // @Units: ms
    // @Range: 100 2000
    // @User: Advanced
    AP_GROUPINFO("TIMEOUT_MS", 4, AP_VisualCourse, _timeout_ms, 500),

    // @Param: FORCE
    // @DisplayName: Visual course force enable
    // @Description: When enabled, CRUISE mode uses the visual course even if GPS is available
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("FORCE", 5, AP_VisualCourse, _force, 0),

    // @Param: AUTOTUNE
    // @DisplayName: Visual course autotune
    // @Description: When enabled, CRUISE runs a relay experiment on the visual
    //   course loop and saves the resulting VC_GAIN_P/VC_GAIN_D, then disables itself.
    // @Values: 0:Disabled,1:Run
    // @User: Advanced
    AP_GROUPINFO("AUTOTUNE", 6, AP_VisualCourse, _autotune, 0),

    // @Param: AUTOTUNE_AMP
    // @DisplayName: Visual course autotune relay amplitude
    // @Description: Roll amplitude used by the autotune relay excitation
    // @Units: deg
    // @Range: 5 30
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("AUTOTUNE_AMP", 7, AP_VisualCourse, _autotune_amp, 20.0f),

    // @Param: AS_REF
    // @DisplayName: Visual course gain-scheduling reference airspeed
    // @Description: Airspeed (m/s) the VC_GAIN_P/D were tuned at. When nonzero, the
    //   CRUISE servo scales the gains by airspeed / AS_REF (clamped 0.5..2). The
    //   autotuner saves the tune airspeed here automatically. 0 disables scheduling.
    // @Units: m/s
    // @Range: 0 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("AS_REF", 8, AP_VisualCourse, _as_ref, 0.0f),

    // @Param: SYNC_MS
    // @DisplayName: Visual course timestamp sync tolerance
    // @Description: Maximum amount a companion time_usec may lead the FC clock. Stamps further
    //   ahead, zero, or more than 10 s behind are treated as unsynced (no TIMESYNC) and the
    //   receipt time is used instead.
    // @Units: ms
    // @Range: 1 200
    // @User: Advanced
    AP_GROUPINFO("SYNC_MS", 9, AP_VisualCourse, _sync_ms, 10),

    AP_GROUPEND
};

// a stamp this far behind the receipt time is in a different timebase, not just late
#define VC_MAX_PLAUSIBLE_LATENCY_US 10000000LL

AP_VisualCourse::AP_VisualCourse() :
    _crab(0.0f),
    _crab_rate(0.0f),
    _healthy(false)
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
    memset(&_latest, 0, sizeof(_latest));
    _last_msg_us = 0;
    _prev_sample_us = 0;
    _prev_crab = 0.0f;
    _synced = true;
    _sync_text_ms = 0;
    _unsync_text_ms = 0;
    _atune_active = false;
    _atune_relay = 0;
    _atune_periods = 0;
    _atune_period_sum = 0.0f;
    _atune_amp_sum = 0.0f;
    _atune_cycle_start_ms = 0;
    _atune_alt0 = 0.0f;
    _log_course_error = 0.0f;
    _log_roll_cmd = 0.0f;
}

void AP_VisualCourse::handle_msg(const mavlink_message_t &msg)
{
    if (!enabled() || (msg.msgid != MAVLINK_MSG_ID_OPTICAL_FLOW)) {
        return;
    }

    mavlink_optical_flow_t packet;
    mavlink_msg_optical_flow_decode(&msg, &packet);

    // the companion sends the FOE-derived crab angle in flow_comp_m_x (rad)
    handle_crab_sample(packet.flow_comp_m_x, packet.quality, packet.time_usec);
}

void AP_VisualCourse::reset()
{
    _crab_rate = 0.0f;
}

// airspeed gain-scheduling scale: V / VC_AS_REF, clamped; 1.0 when disabled
float AP_VisualCourse::gain_scale() const
{
    const float ref = _as_ref.get();
    if (ref <= 0.0f) {
        return 1.0f;   // scheduling disabled
    }
    float aspd;
    if (!AP::ahrs().airspeed_estimate(aspd) || aspd < 1.0f) {
        return 1.0f;
    }
    return constrain_float(aspd / ref, 0.5f, 2.0f);
}

// record the last servo inputs for the VC log
void AP_VisualCourse::set_servo_state(float course_error, float roll_cmd)
{
    _log_course_error = course_error;
    _log_roll_cmd = roll_cmd;
}

// store a crab sample into _latest (shared by handle_msg and the SITL backend)
void AP_VisualCourse::handle_crab_sample(float crab, uint8_t quality, uint64_t time_us)
{
    const uint64_t rx_us = AP_HAL::micros64();
    const int64_t latency_us = int64_t(rx_us) - int64_t(time_us);
    const bool synced = (time_us != 0) &&
                        (latency_us >= -int64_t(_sync_ms.get()) * 1000LL) &&
                        (latency_us <= VC_MAX_PLAUSIBLE_LATENCY_US);

    const uint32_t now_ms = AP_HAL::millis();
    if (synced != _synced) {
        uint32_t &last_ms = synced ? _sync_text_ms : _unsync_text_ms;
        if (now_ms - last_ms > 5000) {
            last_ms = now_ms;
            if (synced) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "VisualCourse: timestamps synced");
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "VisualCourse: unsynced time_usec, using rx");
            }
        }
    }
    _synced = synced;

    _latest.crab = crab;
    _latest.quality = quality;
    _latest.time_us = synced ? time_us : rx_us;
    _last_msg_us = rx_us;
    _latest.fresh = true;
}

// filter a new crab sample; rate comes from the spacing of sample timestamps
void AP_VisualCourse::process_new_sample(float crab, uint64_t sample_us)
{
    if (_prev_sample_us != 0 && sample_us <= _prev_sample_us) {
        return;   // duplicate or out-of-order sample
    }
    const float dt = (_prev_sample_us == 0) ? 0.0f : (sample_us - _prev_sample_us) * 1.0e-6f;

    if (dt <= 0.0f || dt * 1000.0f > _timeout_ms.get()) {
        // first sample or after a gap: re-initialise instead of filtering across it
        _crab = crab;
        _crab_rate = 0.0f;
    } else {
        const float rate_meas = wrap_PI(crab - _prev_crab) / dt;
        _crab_rate += (rate_meas - _crab_rate) * (dt / (dt + 0.1f));
        _crab += wrap_PI(crab - _crab) * (dt / (dt + 0.05f));
    }
    _prev_crab = crab;
    _prev_sample_us = sample_us;
    _healthy = true;
}

// relay autotune for the course-hold PD gains (Ziegler-Nichols on a relay limit cycle)
bool AP_VisualCourse::autotune_update(float course_error, float &roll_cmd)
{
    if (!autotune_enabled()) {
        _atune_active = false;
        return false;
    }

    const float amp = radians(constrain_float(_autotune_amp.get(), 5.0f, 30.0f));

    // aileron-authority gate: the relay needs enough airspeed for a meaningful
    // limit cycle, otherwise the roll response saturates and the cycle diverges
    float aspd;
    if (!AP::ahrs().airspeed_estimate(aspd)) {
        return false;                       // no airspeed yet: wait (PD runs meanwhile)
    }
    if (!_atune_active && aspd < 12.0f) {
        return false;                       // not at cruise speed yet: wait
    }
    if (_atune_active && aspd < 10.0f) {
        autotune_abort("airspeed too low");
        return false;
    }

    if (!_atune_active) {
        _atune_active = true;
        _atune_min = course_error;
        _atune_max = course_error;
        _atune_relay = (course_error >= 0.0f) ? 1 : -1;
        _atune_cycle_start_ms = 0;   // first rising switch only starts the timer
        _atune_period_sum = 0.0f;
        _atune_amp_sum = 0.0f;
        _atune_periods = 0;
        float posD;
        AP::ahrs().get_relative_position_D_home(posD);
        _atune_alt0 = -posD;   // altitude above home
    }

    // abort on course excursion (a healthy relay keeps |error| bounded by ~2*amp)
    if (fabsf(course_error) > MAX(radians(45.0f), 2.0f * amp)) {
        autotune_abort("course excursion");
        return false;
    }
    // abort on excessive bank angle
    if (fabsf(AP::ahrs().get_roll()) > radians(50.0f)) {
        autotune_abort("roll excursion");
        return false;
    }
    // abort on altitude excursion from the experiment start
    float posD;
    AP::ahrs().get_relative_position_D_home(posD);
    if (fabsf(-posD - _atune_alt0) > 30.0f) {
        autotune_abort("altitude excursion");
        return false;
    }

    _atune_min = MIN(_atune_min, course_error);
    _atune_max = MAX(_atune_max, course_error);

    // relay with hysteresis: switch only when the error leaves the +/- band
    const float hyst = radians(1.0f);
    int8_t sign;
    if (course_error > hyst) {
        sign = 1;
    } else if (course_error < -hyst) {
        sign = -1;
    } else {
        sign = _atune_relay;   // hold the last output inside the band
    }

    // a rising relay switch (-1 -> +1) delimits one full oscillation cycle
    if (_atune_relay < 0 && sign > 0) {
        const float period = 1.0e-3f * (AP_HAL::millis() - _atune_cycle_start_ms);
        const float amplitude = 0.5f * (_atune_max - _atune_min);
        if (_atune_cycle_start_ms != 0 && period > 0.5f && amplitude > radians(1.0f)) {
            _atune_period_sum += period;
            _atune_amp_sum += amplitude;
            _atune_periods++;
        }
        _atune_min = course_error;
        _atune_max = course_error;
        _atune_cycle_start_ms = AP_HAL::millis();
    }
    _atune_relay = sign;

    // relay output (actual desired roll, + = right); inverted for negative feedback
    roll_cmd = -sign * amp;

    if (_atune_periods >= 3) {
        const float Tu = _atune_period_sum / _atune_periods;
        const float A = _atune_amp_sum / _atune_periods;
        const float Ku = 4.0f * amp / (M_PI * MAX(A, radians(0.5f)));
        // clamp the learned gains to a safe band before saving
        const float Kp_zn = 0.6f * Ku;
        const float Kp = constrain_float(Kp_zn, 0.05f, 5.0f);
        const float Kd_zn = Kp * Tu * 0.125f;
        const float Kd = constrain_float(Kd_zn, 0.0f, 3.0f);
        const bool clamped = !is_equal(Kp, Kp_zn) || !is_equal(Kd, Kd_zn);
        _gain_p.set_and_save(Kp);
        _gain_d.set_and_save(Kd);
        // remember the tune airspeed for gain scheduling (R9)
        if (aspd > 1.0f) {
            _as_ref.set_and_save(aspd);
        }
        _autotune.set_and_save(0);
        _atune_active = false;
        _atune_periods = 0;
        _atune_period_sum = 0.0f;
        _atune_amp_sum = 0.0f;
        GCS_SEND_TEXT(clamped ? MAV_SEVERITY_WARNING : MAV_SEVERITY_INFO,
                      "VC autotune done: Kp=%.2f Kd=%.2f%s", (double)Kp, (double)Kd,
                      clamped ? " CLAMPED" : "");
        return false;
    }

    return true;
}

// discard partial relay cycles without disabling VC_AUTOTUNE (e.g. on unlock or dropout)
void AP_VisualCourse::autotune_reset()
{
    _atune_active = false;
    _atune_periods = 0;
    _atune_period_sum = 0.0f;
    _atune_amp_sum = 0.0f;
}

// abandon the experiment and disable VC_AUTOTUNE (unsafe conditions)
void AP_VisualCourse::autotune_abort(const char *reason)
{
    if (_atune_active || autotune_enabled()) {
        _autotune.set_and_save(0);
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "VisualCourse autotune aborted: %s", reason);
    }
    _atune_active = false;
    _atune_periods = 0;
    _atune_period_sum = 0.0f;
    _atune_amp_sum = 0.0f;
}

void AP_VisualCourse::update()
{
    if (!enabled()) {
        _healthy = false;
        _latest.fresh = false;
        return;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    update_sitl();
#endif

    // signed: a synced stamp may lead the FC clock by up to VC_SYNC_MS
    const int64_t age_us = int64_t(AP_HAL::micros64()) - int64_t(_latest.time_us);
    const bool fresh = (_latest.time_us != 0) &&
                       (age_us < int64_t(_timeout_ms.get()) * 1000LL);

    if (_latest.fresh) {
        _latest.fresh = false;
        if (!fresh || (_latest.quality < _qual_min.get())) {
            _healthy = false;
        } else {
            process_new_sample(_latest.crab, _latest.time_us);
        }
    } else {
        // no new message: health decays with message age only
        _healthy = fresh && (_latest.quality >= _qual_min.get());
    }

    write_log();
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
void AP_VisualCourse::update_sitl()
{
    // SITL has no companion sending OPTICAL_FLOW; synthesise the crab angle from
    // the true simulated ground velocity, pack it as a real OPTICAL_FLOW message,
    // and feed it through handle_msg() exactly like a real message (P5.2).
    const SITL::SIM *sitl = AP::sitl();
    if (sitl == nullptr) {
        return;
    }
    const float speed_h = sqrtf(sq(sitl->state.speedN) + sq(sitl->state.speedE));
    if (speed_h < 3.0f) {
        return;
    }
    const float course = atan2f(sitl->state.speedE, sitl->state.speedN);
    const float crab = wrap_PI(course - radians(sitl->state.yawDeg));

    // what a perfect forward camera's FOE would report (companion side, body frame)
    mavlink_optical_flow_t packet{};
    packet.time_usec = AP_HAL::micros64();
    packet.flow_comp_m_x = crab;   // crab angle, rad, + = drifting right
    packet.quality = 255;

    mavlink_message_t msg;
    mavlink_msg_optical_flow_encode(0, 0, &msg, &packet);
    handle_msg(msg);
}
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL

void AP_VisualCourse::write_log(void)
{
    const struct log_VisualCourse pkt {
        LOG_PACKET_HEADER_INIT(LOG_VC_MSG),
        time_us       : AP_HAL::micros64(),
        healthy       : _healthy,
        crab          : _crab,
        crab_rate     : _crab_rate,
        quality       : _latest.quality,
        course_error  : _log_course_error,
        roll_cmd      : _log_roll_cmd,
        atune_active  : _atune_active,
        atune_periods : _atune_periods,
        synced        : _synced
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

namespace AP {
    AP_VisualCourse *visualcourse() {
        return AP_VisualCourse::get_singleton();
    }
}

#endif  // AP_VISUALCOURSE_ENABLED
