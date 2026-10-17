#pragma once

#include "AP_VisualCourse_config.h"

#if AP_VISUALCOURSE_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

/*
  AP_VisualCourse - non-GPS course-hold using a forward-facing optical
  flow sensor fed via the MAVLink OPTICAL_FLOW message.

  The companion computer tracks features, estimates the focus of expansion,
  and sends the FOE-derived crab angle (rad, body frame) in the
  flow_comp_m_x field. The flight controller filters it into a crab angle
  and rate; no mount rotation, gyro compensation or depth are required on
  the FC.

  The crab angle is the angle between the ground-velocity direction and
  the body X axis (positive = drifting right). CRUISE mode drives the ground
  course (heading + crab) to the locked course with a PD roll servo.
*/
class AP_VisualCourse
{
public:
    AP_VisualCourse();

    CLASS_NO_COPY(AP_VisualCourse);

    // singleton
    static AP_VisualCourse *get_singleton() { return _singleton; }

    // consume an OPTICAL_FLOW mavlink message
    void handle_msg(const mavlink_message_t &msg);

    // update filter + health, called at ~50Hz
    void update();

    // reset the crab-rate filter (call when the course locks)
    void reset();

    bool enabled() const { return _enable.get() > 0; }
    bool healthy() const { return _healthy; }

    // crab angle: ground velocity direction relative to body X (rad, + = right drift)
    float crab_angle() const { return _crab; }
    // crab rate (rad/s)
    float crab_rate() const { return _crab_rate; }

    // PD gains for the CRUISE roll servo
    float gain_p() const { return _gain_p.get(); }
    float gain_d() const { return _gain_d.get(); }
    // airspeed gain-scheduling scale (V / VC_AS_REF, clamped; 1.0 when disabled)
    float gain_scale() const;
    // record the last servo inputs (course error, roll command) for the VC log
    void set_servo_state(float course_error, float roll_cmd);

    // force visual course-hold even when GPS is available (test/validation)
    bool force() const { return _force.get() > 0; }

    // course-hold autotune (relay experiment, tunes VC_GAIN_P/D in flight)
    bool autotune_enabled() const { return _autotune.get() > 0; }
    // run one autotune step; returns true while running (roll_cmd = desired roll, rad, + = right).
    bool autotune_update(float course_error, float &roll_cmd);
    // discard partial relay cycles without disabling VC_AUTOTUNE (unlock / dropout)
    void autotune_reset();
    // abandon the experiment and disable VC_AUTOTUNE (unsafe conditions)
    void autotune_abort(const char *reason);

    static const struct AP_Param::GroupInfo var_info[];

private:
    static AP_VisualCourse *_singleton;

    // parameters
    AP_Int8  _enable;
    AP_Float _gain_p;      // roll (rad) per rad of crab
    AP_Float _gain_d;      // roll (rad) per rad/s of crab rate
    AP_Int16 _qual_min;    // minimum flow quality 0..255
    AP_Int16 _timeout_ms;  // max message age (ms)
    AP_Int8  _force;       // force visual course even with GPS
    AP_Int8  _autotune;    // run relay autotune
    AP_Float _autotune_amp;// relay roll amplitude (deg)
    AP_Float _as_ref;      // reference airspeed for gain scheduling (m/s, 0 = off)
    AP_Int16 _sync_ms;     // max lead of time_usec over the FC clock (ms)

    // latest message state
    struct {
        bool     fresh;
        uint64_t time_us;   // sample time in the FC timebase (stamp if synced, else receipt)
        float    crab;      // crab angle (rad, body frame, + = right drift)
        uint8_t  quality;
    } _latest;

    // filter state
    float _crab;             // rad
    float _crab_rate;        // rad/s
    uint64_t _last_msg_us;   // receipt time of last message (us)
    uint64_t _prev_sample_us;
    float _prev_crab;
    bool _synced;            // last stamp was in the FC timebase
    uint32_t _sync_text_ms;
    uint32_t _unsync_text_ms;
    bool _healthy;

    // servo logging (set by CRUISE each tick, written by write_log)
    float _log_course_error; // rad, course error vs locked course
    float _log_roll_cmd;     // rad, commanded roll

    // autotune state
    bool _atune_active;
    float _atune_min, _atune_max;
    int8_t _atune_relay;         // last relay output sign (for hysteresis)
    uint32_t _atune_cycle_start_ms;
    float _atune_period_sum, _atune_amp_sum;
    uint8_t _atune_periods;
    float _atune_alt0;           // altitude above home at experiment start (m)

    void handle_crab_sample(float crab, uint8_t quality, uint64_t time_us);
    void process_new_sample(float crab, uint64_t sample_us);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    void update_sitl();
#endif
    void write_log(void);
};

namespace AP {
    AP_VisualCourse *visualcourse();
}

#endif  // AP_VISUALCOURSE_ENABLED
