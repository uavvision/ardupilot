#include "Plane.h"
#include <cmath>

const AP_Param::GroupInfo KAM_Targeting::var_info[] = {
    AP_GROUPINFO("P_X",          1,  KAM_Targeting, _p_x,          0.8f),
    AP_GROUPINFO("P_Y",          2,  KAM_Targeting, _p_y,          0.774f),
    AP_GROUPINFO("I_X",          3,  KAM_Targeting, _i_x,          0.044f),
    AP_GROUPINFO("I_Y",          4,  KAM_Targeting, _i_y,          0.04f),
    AP_GROUPINFO("D_X",          5,  KAM_Targeting, _d_x,          1.0f),
    AP_GROUPINFO("D_Y",          6,  KAM_Targeting, _d_y,          0.0f),
    AP_GROUPINFO("THR",          7,  KAM_Targeting, _thr,          90.0f),
    AP_GROUPINFO("P_V",          8,  KAM_Targeting, _p_v,          0.004f),
    AP_GROUPINFO("I_V",          9,  KAM_Targeting, _i_v,          0.03f),
    AP_GROUPINFO("D_V",          10, KAM_Targeting, _d_v,          0.001f),
    AP_GROUPINFO("VEL",          11, KAM_Targeting, _vel,          110.0f),
    AP_GROUPINFO("LA",           12, KAM_Targeting, _la,           0.95f),
    AP_GROUPINFO("P_O",          13, KAM_Targeting, _p_o,          0.0f),
    AP_GROUPINFO("R_O",          14, KAM_Targeting, _r_o,          0.0f),
    AP_GROUPINFO("PRX",          15, KAM_Targeting, _prx,          0.0f),
    AP_GROUPINFO("PRY",          16, KAM_Targeting, _pry,          0.0f),
    AP_GROUPINFO("M_USE",        17, KAM_Targeting, _m_use,        0.0f),
    AP_GROUPINFO("STAB",         18, KAM_Targeting, _stab,         1.0f),
    AP_GROUPINFO("RDR",          19, KAM_Targeting, _rdr,          0.3f),
    AP_GROUPINFO("X_IMAX",       20, KAM_Targeting, _x_imax,       10.0f),
    AP_GROUPINFO("Y_IMAX",       21, KAM_Targeting, _y_imax,       10.0f),
    AP_GROUPINFO("HUD_MASK",     22, KAM_Targeting, _hud_mask,     0x00FFFFFF),
    AP_GROUPINFO("EV",           23, KAM_Targeting, _ev,           0.0f),
    AP_GROUPINFO("AE_M_MODE",    24, KAM_Targeting, _ae_m_mode,    1.0f),
    AP_GROUPINFO("AE_C_MODE",    25, KAM_Targeting, _ae_c_mode,    0.0f),
    AP_GROUPINFO("P_LIM_DEG",    26, KAM_Targeting, _p_lim_deg,    45.0f),
    AP_GROUPINFO("R_LIM_DEG",    27, KAM_Targeting, _r_lim_deg,    45.0f),
    AP_GROUPINFO("THR_SLEW",     28, KAM_Targeting, _thr_slew,     12.0f),
    AP_GROUPINFO("CALIB",        29, KAM_Targeting, _calib,        0.0f),
    AP_GROUPINFO("RESTART",      30, KAM_Targeting, _restart,      0.0f),
    AP_GROUPINFO("PROX_SCALE",   31, KAM_Targeting, _prox_scale,   1.25f),
    AP_GROUPINFO("PROX_TIME_S",  32, KAM_Targeting, _prox_time_s,  0.0f),
    AP_GROUPINFO("PROX_TIME_E",  33, KAM_Targeting, _prox_time_e,  0.0f),
    AP_GROUPINFO("PROX_OPT",     34, KAM_Targeting, _prox_opt,     0.0f),
    // slot 35 reserved
    AP_GROUPINFO("PITCH_T_LIM",  36, KAM_Targeting, _pitch_t_lim,  60.0f),
    AP_GROUPINFO("D_FCUT",       37, KAM_Targeting, _d_fcut,       2.5f),
    AP_GROUPINFO("CORR_MAX",     38, KAM_Targeting, _corr_max,     0.0f),
    AP_GROUPINFO("RX_TMO",       39, KAM_Targeting, _rx_tmo,       250.0f),
    AP_GROUPINFO("FOV_GUARD",    40, KAM_Targeting, _fov_guard,    1.0f),
    AP_GROUPINFO("P_SLEW",       41, KAM_Targeting, _p_slew,       15.0f),
    AP_GROUPINFO("CAM_LAG_MS",   42, KAM_Targeting, _cam_lag_ms,   75.0f),
    // slot 43 reserved
    AP_GROUPINFO("TERM_DEP_DEG", 44, KAM_Targeting, _term_dep_deg, 12.0f),
    AP_GROUPINFO("DEP_REF_DEG",  45, KAM_Targeting, _dep_ref_deg,  8.0f),
    AP_GROUPINFO("D_FREEZE_DPS", 46, KAM_Targeting, _d_freeze_dps, 45.0f),
    AP_GROUPINFO("TURN_COORD",   47, KAM_Targeting, _turn_coord,   1.0f),
    AP_GROUPINFO("LEAD_T",       48, KAM_Targeting, _lead_t,       0.8f),
    AP_GROUPINFO("DEBUG",        49, KAM_Targeting, _debug,        1.0f),
    AP_GROUPINFO("LOS_GAIN",     50, KAM_Targeting, _los_gain,     0.0f),
    AP_GROUPINFO("BH_EN",        51, KAM_Targeting, _bh_en,        1.0f),
    AP_GROUPINFO("BH_P",         52, KAM_Targeting, _bh_p,         1.0f),
    AP_GROUPINFO("BH_I",         53, KAM_Targeting, _bh_i,         0.22f),
    AP_GROUPINFO("BH_MAX",       54, KAM_Targeting, _bh_max,       32.0f),
    AP_GROUPINFO("BH_SLEW",      55, KAM_Targeting, _bh_slew,      30.0f),
    AP_GROUPINFO("BH_D",         56, KAM_Targeting, _bh_d,         0.35f),
    AP_GROUPINFO("HARD",         57, KAM_Targeting, _hard,         0),
    AP_GROUPEND
};

namespace {

inline float sign_f(float x)
{
    return x >= 0.0f ? 1.0f : -1.0f;
}

inline float kmh_to_ms(float kmh)
{
    return kmh * (1000.0f / 3600.0f);
}

inline float map_range(float in_min, float in_max, float out_min, float out_max, float value)
{
    const float lo = MIN(in_min, in_max);
    const float hi = MAX(in_min, in_max);
    const float clamped = constrain_float(value, lo, hi);
    const float alpha_val = (clamped - in_min) / (in_max - in_min);
    return out_min + alpha_val * (out_max - out_min);
}

inline float wrap_pi_safe(float rad)
{
    if (!std::isfinite(rad)) {
        return 0.0f;
    }
    return wrap_PI(rad);
}

void body_angles_to_ned(float roll, float pitch, float yaw, float raw_ax, float raw_ay,
                        float &lambda_ned, float &elev_ned, float &ax_horiz)
{
    (void)roll;
    const float cy_el = cosf(raw_ay);
    const float x_u = cy_el * cosf(raw_ax);
    const float y_u = cy_el * sinf(raw_ax);
    const float z_u = -sinf(raw_ay);

    const float cp = cosf(pitch);
    const float sp = sinf(pitch);
    const float x_horiz = x_u * cp + z_u * sp;
    const float y_horiz = y_u;
    const float z_down  = -x_u * sp + z_u * cp;

    ax_horiz = atan2f(y_horiz, x_horiz);
    lambda_ned = wrap_pi_safe(yaw + ax_horiz);

    const float horiz_norm = sqrtf(x_horiz * x_horiz + y_horiz * y_horiz);
    elev_ned = atan2f(-z_down, MAX(horiz_norm, 1e-6f));
}

bool is_valid_landing_target(const mavlink_landing_target_t &msg)
{
    return std::isfinite(msg.angle_x) && std::isfinite(msg.angle_y)
        && std::isfinite(msg.x) && std::isfinite(msg.y) && std::isfinite(msg.z)
        && std::isfinite(msg.size_x) && std::isfinite(msg.size_y);
}

} // namespace

KAM_Targeting::KAM_Targeting()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void KAM_Targeting::ParamRef::bind(const char *name)
{
    ptr = AP_Param::find(name, &type);
}

float KAM_Targeting::ParamRef::get(float def) const
{
    if (ptr == nullptr || type == AP_PARAM_NONE || type > AP_PARAM_FLOAT) {
        return def;
    }
    return ptr->cast_to_float(type);
}

void KAM_Targeting::PID::init(float p, float i, float d, float imax, float d_cut, bool decay)
{
    kP = p;
    kI = i;
    kD = d;
    iMax = imax;
    has_iMax = true;
    d_max = 0.0f;
    d_cutoff_hz = d_cut > 0.0f ? d_cut : 4.0f;
    i_decay = decay;
    has_last_t = false;
    last_t = 0.0;
    has_last_err = false;
    last_err = 0.0f;
    has_last_d_t = false;
    last_d_t = 0.0;
    I = 0.0f;
    P = 0.0f;
    D = 0.0f;
    total = 0.0f;
    err = 0.0f;
    d_filtered = 0.0f;
}

float KAM_Targeting::PID::update(float error, bool inhibit_i, bool new_sample, float d_gain,
                                 float err_d, bool has_sample_dt, float sample_dt_s, double now_s)
{
    if (!has_last_t) {
        last_t = now_s;
        has_last_t = true;
    }
    const float dt = float(now_s - last_t);
    last_t = now_s;

    const float p_out = kP * error;
    const bool dt_ok = (dt > 1e-4f) && (dt < 0.5f);
    if (!dt_ok) {
        has_last_err = false;
        has_last_d_t = false;
    }

    if (dt_ok && !inhibit_i) {
        if (i_decay && !is_equal(sign_f(I), sign_f(error)) && fabsf(error) > 0.01f && fabsf(I) > 0.01f) {
            I = I * expf(-2.0f * dt);
        } else {
            I = I + kI * error * dt;
        }
    }
    if (has_iMax) {
        I = constrain_float(I, -iMax, iMax);
    }
    float d_val = d_filtered;
    if (dt_ok && new_sample) {
        if (has_last_err) {
            float dt_d;
            if (!has_sample_dt) {
                dt_d = has_last_d_t ? constrain_float(float(now_s - last_d_t), 0.02f, 0.5f) : dt;
            } else {
                dt_d = constrain_float(sample_dt_s, 0.015f, 0.5f);
            }
            float diff = err_d - last_err;
            if (diff > 180.0f) {
                diff -= 360.0f;
            } else if (diff < -180.0f) {
                diff += 360.0f;
            }
            const float a = calc_lowpass_alpha_dt(dt_d, d_cutoff_hz);
            d_filtered = (1.0f - a) * d_filtered + a * (kD * diff / dt_d);
            d_val = d_filtered;
        }
        last_err = err_d;
        has_last_err = true;
        last_d_t = now_s;
        has_last_d_t = true;
    }

    float d_out = d_val * d_gain;
    if (d_max > 0.0f) {
        d_out = constrain_float(d_out, -d_max, d_max);
    }
    const float ret = p_out + I + d_out;
    P = p_out;
    D = d_out;
    err = error;
    total = ret;
    return ret;
}

void KAM_Targeting::PID::reset(float integrator)
{
    I = integrator;
}

void KAM_Targeting::PID::log(const char *name) const
{
#if HAL_LOGGING_ENABLED
    AP::logger().Write(name, "TimeUS,Err,P,I,D,Total", "s-----", "F-----", "Qfffff",
                       AP_HAL::micros64(), (double)err, (double)P, (double)I, (double)D, (double)total);
#endif
}

void KAM_Targeting::LowPass::init(float a)
{
    alpha = a;
    prev = 0.0f;
    has_prev = false;
}

float KAM_Targeting::LowPass::update(float value)
{
    if (!has_prev) {
        prev = value;
        has_prev = true;
        return value;
    }
    prev = (1.0f - alpha) * prev + alpha * value;
    return prev;
}

void KAM_Targeting::LowPass::reset_empty()
{
    prev = 0.0f;
    has_prev = false;
}

void KAM_Targeting::LowPass::reset(float value)
{
    prev = value;
    has_prev = true;
}

void KAM_Targeting::SlewRate::init(float r)
{
    rate = r;
    prev = 0.0f;
    last_t = 0.0;
    has_prev = false;
    has_last_t = false;
}

float KAM_Targeting::SlewRate::update(float value, double now_s)
{
    if (has_prev && has_last_t) {
        const float max_delta = rate * float(now_s - last_t);
        value = constrain_float(value, prev - max_delta, prev + max_delta);
    }
    prev = value;
    has_prev = true;
    last_t = now_s;
    has_last_t = true;
    return value;
}

void KAM_Targeting::SlewRate::reset(float value, double now_s)
{
    prev = value;
    has_prev = true;
    last_t = now_s;
    has_last_t = true;
}

void KAM_Targeting::bind_ext_params()
{
    if (_ext_params_bound) {
        return;
    }
    _ap_roll_tau.bind("RLL2SRV_TCONST");
    _ap_pitch_tau.bind("PTCH2SRV_TCONST");
    _ap_rmax.bind("RLL2SRV_RMAX");
    _ap_rmax_up.bind("PTCH2SRV_RMAX_UP");
    _ap_rmax_dn.bind("PTCH2SRV_RMAX_DN");
    _ap_rll_ff.bind("PTCH2SRV_RLL");
    _ap_roll_lim.bind("ROLL_LIMIT_DEG");
    _scr_user1.bind("SCR_USER1");
    _ext_params_bound = true;
}

void KAM_Targeting::handle_landing_target(const mavlink_landing_target_t &packet)
{
    if (_hard.get() == 0) {
        return;
    }
    if (!is_valid_landing_target(packet)) {
        return;
    }
    const double rx_ms = double(AP_HAL::millis());
    if (!_has_last_target_rx_ms || rx_ms >= _last_target_rx_ms) {
        if (!_has_pending_target || rx_ms >= _pending_rx_ms) {
            _pending_target = packet;
            _pending_rx_ms = rx_ms;
            _has_pending_target = true;
        }
    }
}

bool KAM_Targeting::terminal_state(float dep_raw_deg, double now_ms, float &dep_deg_out, bool &has_dep_out)
{
    if (std::isfinite(dep_raw_deg)) {
        if (!_has_dep_filt) {
            _dep_filt = dep_raw_deg;
            _has_dep_filt = true;
        } else {
            const float dt_dep = constrain_float(float(now_ms - _dep_ms) * 0.001f, 0.005f, 0.5f);
            const float a_dep = calc_lowpass_alpha_dt(dt_dep, 1.5f);
            _dep_filt = (1.0f - a_dep) * _dep_filt + a_dep * dep_raw_deg;
        }
        _dep_ms = now_ms;
        const float dep_deg = _dep_filt;
        const float gate_deg = fabsf(_term_dep_deg.get());
        if (gate_deg <= 0.0f) {
            _term_latched = false;
        } else if (dep_deg >= gate_deg) {
            _term_latched = true;
        } else if (dep_deg < MAX(0.0f, gate_deg - 1.5f)) {
            _term_latched = false;
        }
        dep_deg_out = dep_deg;
        has_dep_out = true;
        return _term_latched;
    }
    dep_deg_out = 0.0f;
    has_dep_out = false;
    return false;
}

float KAM_Targeting::get_roll_servo_out(float angle_err_deg) const
{
    float gain_tau = _ap_roll_tau.get(0.5f);
    if (gain_tau < 0.05f) {
        gain_tau = 0.05f;
    }
    float desired_rate = angle_err_deg / gain_tau;
    const float rmax = _ap_rmax.get(0.0f);
    if (rmax > 0.0f) {
        desired_rate = constrain_float(desired_rate, -rmax, rmax);
    }
    return desired_rate;
}

float KAM_Targeting::get_pitch_servo_out(float angle_err_deg) const
{
    float gain_tau = _ap_pitch_tau.get(0.5f);
    if (gain_tau < 0.05f) {
        gain_tau = 0.05f;
    }
    float desired_rate = angle_err_deg / gain_tau;
    const float rmax_dn = _ap_rmax_dn.get(0.0f);
    const float rmax_up = _ap_rmax_up.get(0.0f);
    if (rmax_dn > 0.0f && desired_rate < -rmax_dn) {
        desired_rate = -rmax_dn;
    } else if (rmax_up > 0.0f && desired_rate > rmax_up) {
        desired_rate = rmax_up;
    }
    return desired_rate;
}

float KAM_Targeting::get_coordination_rate_offset() const
{
    const float roll_rad = AP::ahrs().get_roll();
    const float pitch_rad = AP::ahrs().get_pitch();
    float airspeed = 0.0f;
    if (!AP::ahrs().airspeed_estimate(airspeed) || airspeed < 1.0f) {
        return 0.0f;
    }
    const float rll_ff = _ap_rll_ff.get(1.0f);
    const float rate_rad_s = cosf(pitch_rad) * (9.80665f / airspeed)
        * tanf(roll_rad) * sinf(roll_rad);
    return degrees(rate_rad_s) * rll_ff;
}

void KAM_Targeting::reset_bh_state(bool has_engage_ms, double engage_ms)
{
    _has_lambda_0 = false;
    _lambda_0 = 0.0f;
    _lambda_init_sin = 0.0f;
    _lambda_init_cos = 0.0f;
    _lambda_init_cnt = 0;
    _bh_integral = 0.0f;
    _has_bh_corr_deg = true;
    _bh_corr_deg = 0.0f;
    _has_last_bh_calc_ms = false;
    _has_bh_engage_ms = has_engage_ms;
    _bh_engage_ms = engage_ms;
    _term_latched = false;
    _vtrim_armed = false;
    _has_el_0 = false;
    _el_0 = 0.0f;
    _has_el_filt = false;
    _el_filt = 0.0f;
    _el_prev = 0.0f;
    _el_rate_dps = 0.0f;
    _has_last_el_ms = false;
    _el_rate_slow = 0.0f;
    _el_integral = 0.0f;
    _el_corr_deg = 0.0f;
    _el_drift_deg = 0.0f;
    _has_last_el_calc_ms = false;
    _has_el_engage_ms = has_engage_ms;
    _el_engage_ms = engage_ms;
    _has_lambda_filt = false;
    _lambda_filt = 0.0f;
    _lambda_prev = 0.0f;
    _lambda_rate_filt = 0.0f;
    _drift_deg = 0.0f;
    _has_lambda_deg = true;
    _lambda_deg = 0.0f;
    _has_last_bh_ms = false;
    _has_last_bh_target_ms = false;
    if (!has_engage_ms) {
        _has_last_target_rx_ms = false;
        _has_last_pitch_cmd = false;
        _has_last_raw_angles = false;
        _has_last_corr_x = false;
        _has_dep_filt = false;
        _dep_ms = 0.0;
        _has_last_frame_ms = false;
        _cam_lag_ms_val = 0.0f;
    }
}

float KAM_Targeting::update_vertical_bh(float dep_raw_deg, float curr_roll_deg, float curr_pitch_deg,
                                          float raw_ay_deg, double now_ms)
{
    if (!_has_el_filt) {
        return 0.0f;
    }
    if (!_has_el_engage_ms) {
        _el_engage_ms = now_ms;
        _has_el_engage_ms = true;
        _vtrim_armed = false;
    }
    const float wait_s = float(now_ms - _el_engage_ms) * 0.001f;
    if (!_vtrim_armed) {
        if ((wait_s >= 1.5f && dep_raw_deg >= 0.5f && fabsf(curr_roll_deg) <= 12.0f
                && fabsf(raw_ay_deg) <= 4.0f && fabsf(_el_rate_dps) <= 1.5f)
            || (wait_s >= 2.2f && fabsf(curr_roll_deg) <= 15.0f)) {
            _vtrim_armed = true;
            _el_0 = _el_filt;
            _has_el_0 = true;
            _el_prev = _el_filt;
            _el_rate_dps = 0.0f;
            _el_rate_slow = 0.0f;
        }
        _el_drift_deg = 0.0f;
        _last_el_calc_ms = now_ms;
        _has_last_el_calc_ms = true;
        return _el_integral;
    }

    if (wait_s < 4.5f && (!_has_el_0 || _el_filt < _el_0)) {
        _el_0 = _el_filt;
        _has_el_0 = true;
    }

    const float dt_s = constrain_float(float(now_ms - (_has_last_el_calc_ms ? _last_el_calc_ms : now_ms)) * 0.001f, 0.005f, 0.1f);
    _last_el_calc_ms = now_ms;
    _has_last_el_calc_ms = true;
    const float a_s = calc_lowpass_alpha_dt(dt_s, 0.15f);
    _el_rate_slow = (1.0f - a_s) * _el_rate_slow + a_s * constrain_float(_el_rate_dps, -10.0f, 10.0f);
    const float el_ref = _has_el_0 ? _el_0 : _el_filt;
    const float el_drift_rad = constrain_float(_el_filt - el_ref, radians(-12.0f), radians(12.0f));
    _el_drift_deg = degrees(el_drift_rad);
    const float k_rate = (_el_drift_deg > 0.8f && _el_rate_slow < 0.0f) ? 0.45f : 1.00f;
    float k_drift = el_drift_rad > 0.0f ? 0.22f : 0.20f;
    if ((el_drift_rad > 0.0f && _el_rate_slow < -0.05f)
        || (el_drift_rad < 0.0f && _el_rate_slow > 0.05f)) {
        k_drift = 0.0f;
    }
    float step_rad_s = k_rate * radians(_el_rate_slow) + k_drift * el_drift_rad;
    if (step_rad_s > radians(1.5f)) {
        step_rad_s = radians(1.5f);
    }

    const bool sat_windup = _pitch_sat
        && ((step_rad_s > 0.0f && _el_integral > 0.0f) || (step_rad_s < 0.0f && _el_integral < 0.0f));
    if (step_rad_s < 0.0f && (dep_raw_deg >= 24.0f || fabsf(curr_roll_deg) > 15.0f)) {
        step_rad_s = 0.0f;
    }
    const float el_up_cap_deg = map_range(-1.0f, 3.0f, 8.0f, 3.5f, curr_pitch_deg);
    const bool i_up_ok = ((raw_ay_deg <= 1.2f) && (curr_pitch_deg < 3.0f)
        && (degrees(_el_integral) < el_up_cap_deg)) || (_el_integral <= 0.0f);
    const float dep_min = step_rad_s > 0.0f ? -2.0f : 0.5f;
    if (!sat_windup && dep_raw_deg >= dep_min && dep_raw_deg < 24.0f
        && fabsf(curr_roll_deg) <= 15.0f && fabsf(raw_ay_deg) < 15.0f
        && (step_rad_s <= 0.0f || i_up_ok)) {
        _el_integral = constrain_float(_el_integral + step_rad_s * dt_s, radians(-12.0f), radians(8.0f));
    }

    _el_corr_deg = degrees(_el_integral);
    return _el_integral;
}

void KAM_Targeting::refresh_tunables()
{
    _max_throttle = _thr.get();
    _pitch_offset_rad = radians(_p_o.get());
    _roll_offset_rad = radians(_r_o.get());

    const float px = _p_x.get(), ix = _i_x.get(), dx = _d_x.get(), ximax = _x_imax.get();
    if (!is_equal(px, _tuned.px) || !is_equal(ix, _tuned.ix) || !is_equal(dx, _tuned.dx) || !is_equal(ximax, _tuned.ximax)) {
        _tuned.px = px; _tuned.ix = ix; _tuned.dx = dx; _tuned.ximax = ximax;
        _pid_x.kP = px; _pid_x.kI = ix; _pid_x.kD = dx; _pid_x.iMax = ximax;
    }

    const float py = _p_y.get(), iy = _i_y.get(), dy = _d_y.get(), yimax = _y_imax.get();
    if (!is_equal(py, _tuned.py) || !is_equal(iy, _tuned.iy) || !is_equal(dy, _tuned.dy) || !is_equal(yimax, _tuned.yimax)) {
        _tuned.py = py; _tuned.iy = iy; _tuned.dy = dy; _tuned.yimax = yimax;
        _pid_y.kP = py; _pid_y.kI = iy; _pid_y.kD = dy; _pid_y.iMax = yimax;
    }

    const float pv = _p_v.get(), iv = _i_v.get(), dv = _d_v.get();
    if (!is_equal(pv, _tuned.pv) || !is_equal(iv, _tuned.iv) || !is_equal(dv, _tuned.dv)) {
        _tuned.pv = pv; _tuned.iv = iv; _tuned.dv = dv;
        _pid_v.kP = pv; _pid_v.kI = iv; _pid_v.kD = dv;
    }

    const float dcut = _d_fcut.get();
    if (!is_equal(dcut, _tuned.dcut) && dcut > 0.0f) {
        _tuned.dcut = dcut;
        _pid_x.d_cutoff_hz = dcut;
        _pid_y.d_cutoff_hz = dcut;
        _pid_v.d_cutoff_hz = dcut;
    }

    _pid_x.d_max = 0.6f * fabsf(_r_lim_deg.get());

    const float la = _la.get();
    if (!is_equal(la, _tuned.la) && la > 0.0f && la < 1.0f) {
        _tuned.la = la;
        _angle_x_filter.alpha = la;
        _angle_y_filter.alpha = la;
    }

    const float slew = _thr_slew.get();
    if (!is_equal(slew, _tuned.slew) && slew > 0.0f) {
        _tuned.slew = slew;
        _throttle_slew.rate = slew;
    }
}

void KAM_Targeting::reset_pid(bool x, bool y, bool v)
{
    const float dcut = _d_fcut.get();
    if (x) {
        _pid_x.init(_p_x.get(), _i_x.get(), _d_x.get(), _x_imax.get(), dcut, true);
    }
    if (y) {
        _pid_y.init(_p_y.get(), _i_y.get(), _d_y.get(), _y_imax.get(), dcut, true);
    }
    if (v) {
        _pid_v.init(_p_v.get(), _i_v.get(), _d_v.get(), 1.0f, dcut, false);
    }
}

void KAM_Targeting::set_message_interval()
{
    static const struct {
        uint32_t id;
        int32_t interval_us;
    } iv[] = {
        {163, -1}, {178, -1}, {193, -1}, {285, -1}, {124, -1}, {24, -1},
        {32, 33333}, {152, -1}, {62, -1}, {125, -1}, {27, -1}, {116, -1},
        {29, -1}, {36, -1}, {241, -1}, {74, 66666}, {226, 200000},
        {30, 33333}, {33, 33333}, {65, 100000},
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(iv); i++) {
        gcs().set_message_interval(2, iv[i].id, iv[i].interval_us);
    }
}

void KAM_Targeting::deactivate_targeting(const char *msg_text)
{
#if AP_SCRIPTING_ENABLED
    plane.nav_scripting_enable(255);
#endif
    _targeting_activated = false;
    reset_bh_state(false, 0.0);
    _hud_mask.set(_hud_mask_cached);
    gcs().send_text(MAV_SEVERITY_INFO, "%s", msg_text);
}

void KAM_Targeting::send_debug_floats(float ax_deg, float ay_deg, float ax_err_deg, float cxr_deg,
                                      float roll_t, float pitch_t, bool has_dep, float dep_deg,
                                      bool terminal, float yaw_cmd, float roll_deg, float thr) const
{
    auto &g = gcs();
    g.send_named_float("CRAB", _crab_deg);
    g.send_named_float("BNOSE", _b_nose);
    g.send_named_float("BVEL", _b_vel);
    g.send_named_float("AX", ax_deg);
    g.send_named_float("AY", ay_deg);
    g.send_named_float("AXERR", ax_err_deg);
    g.send_named_float("CXR", cxr_deg);
    g.send_named_float("CORRU", _bh_corr_deg);
    g.send_named_float("ELCORR", _el_corr_deg);
    g.send_named_float("ROLLT", roll_t);
    g.send_named_float("PITHT", pitch_t);
    g.send_named_float("DEP", has_dep ? dep_deg : -999.0f);
    g.send_named_float("GATE", terminal ? 1.0f : 0.0f);
    g.send_named_float("DRATE", _ax_rate);
    g.send_named_float("FOVG", _fov_coef);
    g.send_named_float("TURN", _turn_dps);
    g.send_named_float("LEAD", _lead_deg);
    g.send_named_float("DRIFT", _drift_deg);
    g.send_named_float("LOSR", _los_rate_x);
    g.send_named_float("YAWCMD", yaw_cmd);
    g.send_named_float("ROLL", roll_deg);
    g.send_named_float("THR", thr);
}

float KAM_Targeting::update_bh_x(float lambda_raw, float angle_x, float shape_x, float cur_dep, float curr_roll_deg,
                                 bool bh_on, bool target_sample_new, bool has_dt_cam, float dt_cam_s, double now_ms)
{
    if (!bh_on) {
        _bh_corr_deg = 0.0f;
        _has_bh_corr_deg = true;
        _drift_deg = 0.0f;
        return 0.0f;
    }
    const float bh_p = constrain_float(_bh_p.get(), 0.0f, 3.0f);
    if (!_has_lambda_filt) {
        _lambda_filt = lambda_raw;
        _lambda_prev = lambda_raw;
        _lambda_rate_filt = 0.0f;
        _has_lambda_filt = true;
    } else {
        const float dt_f = constrain_float(float(now_ms - (_has_last_bh_ms ? _last_bh_ms : now_ms)) * 0.001f, 0.005f, 0.1f);
        const float a_l = calc_lowpass_alpha_dt(dt_f, 5.0f);
        _lambda_filt = wrap_pi_safe(_lambda_filt + a_l * wrap_pi_safe(lambda_raw - _lambda_filt));
        if (target_sample_new) {
            const float d_lam = wrap_pi_safe(_lambda_filt - _lambda_prev);
            const float dt_l = has_dt_cam ? dt_cam_s : MAX(dt_f, 0.02f);
            const float a_lr = calc_lowpass_alpha_dt(dt_l, 0.45f);
            _lambda_rate_filt = (1.0f - a_lr) * _lambda_rate_filt
                + a_lr * constrain_float(d_lam / dt_l, radians(-25.0f), radians(25.0f));
            _lambda_prev = _lambda_filt;
        }
    }
    _last_bh_ms = now_ms;
    _has_last_bh_ms = true;
    _lambda_deg = degrees(_lambda_filt);
    _has_lambda_deg = true;

    if (_has_last_bh_target_ms && (now_ms - _last_bh_target_ms) > 2000.0) {
        _has_lambda_0 = false;
        _lambda_init_sin = 0.0f;
        _lambda_init_cos = 0.0f;
        _lambda_init_cnt = 0;
        _has_last_bh_calc_ms = false;
        _bh_engage_ms = now_ms;
        _has_bh_engage_ms = true;
    }
    if (!_has_bh_engage_ms) {
        _bh_engage_ms = now_ms;
        _has_bh_engage_ms = true;
    }
    _last_bh_target_ms = now_ms;
    _has_last_bh_target_ms = true;

    if (!_has_lambda_0) {
        const float bh_wait_s = float(now_ms - _bh_engage_ms) * 0.001f;
        const bool bh_ready = (fabsf(curr_roll_deg) <= 14.0f && fabsf(degrees(angle_x)) <= 6.0f)
            || (bh_wait_s >= 2.8f && fabsf(curr_roll_deg) <= 22.0f)
            || (bh_wait_s >= 4.0f);
        if (bh_ready) {
            if (target_sample_new) {
                _lambda_init_sin += sinf(_lambda_filt);
                _lambda_init_cos += cosf(_lambda_filt);
                _lambda_init_cnt++;
                if (_lambda_init_cnt >= 3) {
                    _lambda_0 = atan2f(_lambda_init_sin, _lambda_init_cos);
                    _has_lambda_0 = true;
                }
            }
        } else if (fabsf(curr_roll_deg) > 20.0f) {
            _lambda_init_sin = 0.0f;
            _lambda_init_cos = 0.0f;
            _lambda_init_cnt = 0;
        }
        _bh_corr_deg = degrees(_bh_integral);
        _has_bh_corr_deg = true;
        _drift_deg = 0.0f;
        _last_bh_calc_ms = now_ms;
        _has_last_bh_calc_ms = true;
        return _bh_corr_deg;
    }

    const float dt_s = constrain_float(float(now_ms - (_has_last_bh_calc_ms ? _last_bh_calc_ms : now_ms)) * 0.001f, 0.005f, 0.1f);
    _last_bh_calc_ms = now_ms;
    _has_last_bh_calc_ms = true;
    float drift_rad = wrap_pi_safe(_lambda_filt - _lambda_0);
    _drift_deg = degrees(drift_rad);

    const float bl_i = constrain_float(_bh_i.get(), 0.0f, 2.0f);
    const float bl_max = radians(constrain_float(fabsf(_bh_max.get()), 0.0f, 40.0f));
    const float bl_slew_dps = fabsf(_bh_slew.get());
    const float lrate_rad = _lambda_rate_filt;
    const float lrate_dps = fabsf(degrees(lrate_rad));
    const float term_fade = map_range(22.0f, 32.0f, 1.0f, 0.35f, cur_dep);
    const float d_fade = map_range(2.5f, 5.0f, 1.0f, 0.0f, lrate_dps);

    if (cur_dep >= 16.0f && lrate_dps >= 4.0f && _has_bh_corr_deg) {
        return _bh_corr_deg;
    }
    const float eff_p = bh_p;
    const float eff_d = constrain_float(_bh_d.get(), 0.0f, 5.0f);
    const bool sat_windup = _roll_sat && ((drift_rad > 0.0f && _bh_integral > 0.0f) || (drift_rad < 0.0f && _bh_integral < 0.0f));
    if (!sat_windup && cur_dep < 24.0f
        && fabsf(curr_roll_deg) <= 22.0f && fabsf(degrees(angle_x)) < 30.0f) {
        const float d_int = bl_i * drift_rad * dt_s;
        if (drift_rad * lrate_rad <= 0.0f || cur_dep >= 5.0f) {
            _lambda_0 = (eff_p > 1e-6f) ? wrap_pi_safe(_lambda_0 + d_int / eff_p) : 0.0f;
            drift_rad = wrap_pi_safe(_lambda_filt - _lambda_0);
            _drift_deg = degrees(drift_rad);
        }
        if (drift_rad * lrate_rad >= 0.0f) {
            _bh_integral = constrain_float(_bh_integral + d_int, -bl_max, bl_max);
        }
    }

    const float pd_drift_rad = ((eff_p * term_fade) * drift_rad
        + (eff_d * term_fade * d_fade) * lrate_rad) / shape_x
        + _bh_integral;
    const float raw_corr_deg = degrees(constrain_float(pd_drift_rad, -bl_max, bl_max));
    if (!_has_bh_corr_deg) {
        _bh_corr_deg = raw_corr_deg;
        _has_bh_corr_deg = true;
    } else {
        const float max_step_deg = bl_slew_dps * dt_s;
        _bh_corr_deg = _bh_corr_deg + constrain_float(raw_corr_deg - _bh_corr_deg, -max_step_deg, max_step_deg);
    }
    return _bh_corr_deg;
}

void KAM_Targeting::drive_outputs(float roll_curr, float curr_pitch_deg, float curr_roll_deg,
                                  float error_pitch_deg, float error_roll_deg, bool terminal,
                                  float size_x, float size_y, double now_ms,
                                  float &target_roll_deg_out, float &target_pitch_deg_out,
                                  float &roll_rate_out, float &pitch_rate_out, float &yaw_rate_out, float &throttle_out)
{
    const float raw_target_pitch_deg = curr_pitch_deg + error_pitch_deg;
    const float stab_weight = constrain_float(_stab.get(), 0.0f, 1.0f);
    float roll_limit_deg = _r_lim_deg.get();
    const float ap_roll_limit = _ap_roll_lim.get(0.0f);
    if (ap_roll_limit > 0.0f && ap_roll_limit < roll_limit_deg) {
        roll_limit_deg = ap_roll_limit;
    }

    float raw_target_roll_deg = (1.0f - stab_weight) * (curr_roll_deg + error_roll_deg) + stab_weight * error_roll_deg;
    const float los_gain = _los_gain.get();
    if (fabsf(los_gain) > 0.01f) {
        _los_rate_x = _ax_rate_signed + _yaw_rate_dps;
        raw_target_roll_deg += los_gain * _los_rate_x;
    } else {
        _los_rate_x = 0.0f;
    }

    const float target_roll_deg = constrain_float(raw_target_roll_deg, -roll_limit_deg, roll_limit_deg);
    float pitch_limit_deg = _p_lim_deg.get();
    if (terminal) {
        pitch_limit_deg = MAX(pitch_limit_deg, _pitch_t_lim.get());
    }
    float target_pitch_deg = constrain_float(raw_target_pitch_deg, -pitch_limit_deg, pitch_limit_deg);

    const float p_slew_dps = fabsf(_p_slew.get());
    if (!_has_last_pitch_cmd) {
        _last_pitch_cmd_deg = curr_pitch_deg;
        _last_pitch_cmd_ms = now_ms;
        _has_last_pitch_cmd = true;
    }
    if (p_slew_dps > 0.1f) {
        const float dt_s = MAX(float(now_ms - _last_pitch_cmd_ms) * 0.001f, 0.001f);
        const float max_step = MIN(p_slew_dps * dt_s, p_slew_dps * 0.04f);
        const float step = target_pitch_deg - _last_pitch_cmd_deg;
        if (step > max_step) {
            target_pitch_deg = _last_pitch_cmd_deg + max_step;
        } else if (step < -max_step) {
            target_pitch_deg = _last_pitch_cmd_deg - max_step;
        }
    }
    _last_pitch_cmd_deg = target_pitch_deg;
    _last_pitch_cmd_ms = now_ms;

    _pitch_sat = !is_equal(raw_target_pitch_deg, target_pitch_deg);
    _roll_sat = !is_equal(raw_target_roll_deg, target_roll_deg);

    const float pitch_rate_h = get_pitch_servo_out(target_pitch_deg - curr_pitch_deg);
    float roll_rate = get_roll_servo_out(target_roll_deg - curr_roll_deg);
    float pitch_rate = pitch_rate_h * cosf(roll_curr) + get_coordination_rate_offset();

    const bool fov_guard_on = (_fov_guard.get() >= 0.5f);
    _fov_coef = 1.0f;
    if (fov_guard_on && !terminal) {
        if (fabsf(size_y) > 0.8f && fabsf(size_y) <= 1.0f
            && fabsf(target_pitch_deg) >= pitch_limit_deg - 0.1f && is_equal(sign_f(target_pitch_deg), sign_f(size_y))) {
            const float fc = map_range(0.8f, 1.0f, 1.0f, 0.0f, fabsf(size_y));
            pitch_rate *= fc;
            _fov_coef = MIN(_fov_coef, fc);
        }
        if (!is_zero(roll_rate) && fabsf(size_x) > 0.8f && fabsf(size_x) <= 1.0f
            && fabsf(target_roll_deg) >= roll_limit_deg - 0.1f && is_equal(sign_f(target_roll_deg), sign_f(size_x))) {
            const float fc = map_range(0.8f, 1.0f, 1.0f, 0.0f, fabsf(size_x));
            roll_rate *= fc;
            _fov_coef = MIN(_fov_coef, fc);
        }
    }

    float airspeed_est = 0.0f;
    const bool has_airspeed = AP::ahrs().airspeed_estimate(airspeed_est);
    _turn_dps = 0.0f;
    const float tc_gain = _turn_coord.get();
    if (tc_gain > 0.01f && has_airspeed && airspeed_est > 5.0f) {
        _turn_dps = tc_gain * degrees(9.80665f * tanf(roll_curr) / airspeed_est);
    }
    const float yaw_rate = roll_rate * _rdr.get() - pitch_rate_h * sinf(roll_curr) + _turn_dps;

    const double now_s = now_ms * 0.001;
    float throttle = float(gcs().get_hud_throttle()) / 100.0f;
    if (has_airspeed) {
        const float err_v = kmh_to_ms(_vel.get()) - airspeed_est;
        throttle = _pid_v.update(err_v, false, true, 1.0f, err_v, false, 0.0f, now_s);
    }
    throttle = _throttle_slew.update(constrain_float(_throttle_filter.update(throttle) * 100.0f, 0.0f, _max_throttle), now_s);

#if AP_SCRIPTING_ENABLED
    plane.set_rudder_offset(0.0f, true);
    plane.set_target_throttle_rate_rpy(throttle, roll_rate, pitch_rate, yaw_rate);
#endif

    _pid_y.log("PLNY");
    _pid_x.log("PLNX");
    _pid_v.log("PLNV");

    target_roll_deg_out = target_roll_deg;
    target_pitch_deg_out = target_pitch_deg;
    roll_rate_out = roll_rate;
    pitch_rate_out = pitch_rate;
    yaw_rate_out = yaw_rate;
    throttle_out = throttle;
}

void KAM_Targeting::update_gyro_yaw()
{
    if (_hard.get() == 0) {
        _gyro_yaw_valid = false;
        _gyro_yaw = 0.0f;
        return;
    }
    const auto &ins_ref = AP::ins();
    const auto &ahrs_ref = AP::ahrs();
    uint8_t gyro_idx = ahrs_ref.get_primary_gyro_index();
    if (!ins_ref.get_gyro_health(gyro_idx)) {
        gyro_idx = ins_ref.get_primary_gyro();
    }
    Vector3f d_ang;
    float d_dt = 0.0f;
    if (!ins_ref.get_delta_angle(gyro_idx, d_ang, d_dt) || d_dt <= 0.0f) {
        return;
    }
    const Vector3f d_ang_vb = ahrs_ref.get_rotation_autopilot_body_to_vehicle_body() * d_ang;
    const float roll = ahrs_ref.get_roll();
    const float pitch = ahrs_ref.get_pitch();
    if (!std::isfinite(roll) || !std::isfinite(pitch)) {
        return;
    }
    const float cos_p = cosf(pitch);
    if (fabsf(cos_p) > 0.15f) {
        const float d_psi = (d_ang_vb.y * sinf(roll) + d_ang_vb.z * cosf(roll)) / cos_p;
        if (std::isfinite(d_psi)) {
            _gyro_yaw = wrap_pi_safe(_gyro_yaw + d_psi);
        }
    }
    _gyro_yaw_valid = true;
}

void KAM_Targeting::update()
{
    if (_hard.get() == 0) {
        if (_targeting_activated) {
            deactivate_targeting("Targeting hard OFF (KAM_HARD=0)");
        }
        _hard_announced = false;
        _has_pending_target = false;
        return;
    }

    bind_ext_params();
    if (!_hard_announced) {
        _hard_announced = true;
        _elapsed_ms = 0;
        gcs().send_text(MAV_SEVERITY_INFO, "Targeting C++ v1.9 active");
    }

    if (_elapsed_ms <= 0) {
        set_message_interval();
        _elapsed_ms = 10000;
    }
    _elapsed_ms -= 20;

    const double now_ms = double(AP_HAL::millis());
    const double now_s = now_ms * 0.001;
    const float roll_curr = AP::ahrs().get_roll();
    const float pitch_curr = AP::ahrs().get_pitch();
    const float yaw_curr = _gyro_yaw_valid ? _gyro_yaw : 0.0f;

    if (std::isfinite(roll_curr) && std::isfinite(pitch_curr) && std::isfinite(yaw_curr)) {
        _att_hist_idx = (_att_hist_idx + 1) % 20;
        _att_hist_t[_att_hist_idx] = now_ms;
        _att_hist_r[_att_hist_idx] = roll_curr;
        _att_hist_p[_att_hist_idx] = pitch_curr;
        _att_hist_y[_att_hist_idx] = yaw_curr;
    }

#if AP_SCRIPTING_ENABLED
    uint8_t m_use_raw = 0;
    if (rc().get_aux_cached((RC_Channel::aux_func_t)304, m_use_raw)) {
        float m_use = float(m_use_raw);
        if (!is_equal(m_use, _last_m_use)) {
            m_use = constrain_float(m_use, 0.0f, 1.0f);
            if (m_use > 0.0f) {
                rc().run_aux_function((RC_Channel::AUX_FUNC)304, RC_Channel::AuxSwitchPos::LOW, RC_Channel::AuxFuncTriggerSource::SCRIPTING);
                gcs().send_text(MAV_SEVERITY_NOTICE, "Matcher Enabled KAM_M_USE: %g", (double)m_use);
                _m_use.set(m_use);
            }
        }
        _last_m_use = m_use;
    }
#endif

    const bool reset_pid_x = _prx.get() > 0.0f;
    const bool reset_pid_y = _pry.get() > 0.0f;
    if (reset_pid_x || reset_pid_y) {
        _prx.set(0.0f);
        _pry.set(0.0f);
        reset_pid(reset_pid_x, reset_pid_y, false);
    }

    if (_scr_user1.ptr != nullptr) {
        const float sw_pos = _scr_user1.get(0.0f);
        if (sw_pos <= 0.0f && _targeting_activated) {
            deactivate_targeting("Targeting hard OFF ");
        } else if (sw_pos > 0.0f && !_targeting_activated && plane.get_mode() == (uint8_t)Mode::Number::ACRO) {
            reset_bh_state(false, 0.0);
#if AP_SCRIPTING_ENABLED
            if (!plane.nav_scripting_enable((uint8_t)Mode::Number::ACRO)) {
                gcs().send_text(MAV_SEVERITY_NOTICE, "Targeting: nav scripting enable refused");
                return;
            }
            plane.set_rudder_offset(0.0f, true);
#endif
            const float la = _la.get();
            _angle_x_filter.init(la);
            _angle_y_filter.init(la);
            _pitch_offset_rad = radians(_p_o.get());
            _roll_offset_rad = radians(_r_o.get());
            _max_throttle = _thr.get();
            _pitch_sat = false;
            _roll_sat = false;

            const float init_thr_pct = constrain_float(float(gcs().get_hud_throttle()), 0.0f, _max_throttle);
            const float init_thr_norm = init_thr_pct / 100.0f;
            reset_pid(true, true, true);
            _pid_v.reset(init_thr_norm);
            _throttle_filter.init(0.2f);
            _throttle_filter.reset(init_thr_norm);
            _throttle_slew.init(_thr_slew.get());
            _throttle_slew.reset(init_thr_pct, now_s);
            _targeting_activated = true;
            _hud_mask_cached = _hud_mask.get();
            _hud_mask.set(_hud_mask_cached | (1 << 1));
            gcs().send_text(MAV_SEVERITY_INFO, "Targeting hard ON ");
        }
    }

    if (_targeting_activated && plane.get_mode() != (uint8_t)Mode::Number::ACRO) {
        deactivate_targeting("Targeting hard OFF (mode changed)");
    }

    if (!_targeting_activated) {
        return;
    }

    refresh_tunables();
#if HAL_LOGGING_ENABLED
    AP::logger().Write("GAIN", "TimeUS,Px,Ix,Dx,La,Slew,Rmax,Rlim,RSat", "s--------", "F--------", "Qffffffff",
                       AP_HAL::micros64(), (double)_tuned.px, (double)_tuned.ix, (double)_tuned.dx,
                       (double)_tuned.la, (double)_tuned.slew, (double)_ap_rmax.get(0.0f),
                       (double)_r_lim_deg.get(), _roll_sat ? 1.0 : 0.0);
#endif

    const float target_timeout_ms = _rx_tmo.get();
    if (!_has_pending_target) {
        return;
    }
    const mavlink_landing_target_t parsed_msg = _pending_target;
    const double parsed_rx_ms = _pending_rx_ms;
    _has_pending_target = false;

    const double gap_ms = _has_last_target_rx_ms ? (parsed_rx_ms - _last_target_rx_ms) : 0.0;
    _last_target_rx_ms = parsed_rx_ms;
    _has_last_target_rx_ms = true;
    if ((now_ms - _last_target_rx_ms) > double(target_timeout_ms)) {
        return;
    }

    if (gap_ms > 2000.0) {
        reset_bh_state(false, 0.0);
        _angle_x_filter.reset_empty();
        _angle_y_filter.reset_empty();
        _pid_x.reset(0.0f);
        _pid_y.reset(0.0f);
        _last_target_rx_ms = parsed_rx_ms;
        _has_last_target_rx_ms = true;
        gcs().send_text(MAV_SEVERITY_NOTICE, "New target approach: Integrals reset");
    } else if (_has_last_raw_angles
               && ((_has_dep_filt ? _dep_filt : 0.0f) < 15.0f || fabsf(degrees(parsed_msg.angle_x)) > 15.0f)) {
        const float dx = degrees(parsed_msg.angle_x - _last_raw_angle_x);
        const float dy = degrees(parsed_msg.angle_y - _last_raw_angle_y);
        const float jump_deg = sqrtf(dx * dx + dy * dy);
        if (jump_deg > 8.0f && jump_deg < 120.0f) {
            reset_bh_state(true, now_ms);
            _angle_x_filter.reset_empty();
            _angle_y_filter.reset_empty();
            _pid_x.reset(0.0f);
            _pid_y.reset(0.0f);
            gcs().send_text(MAV_SEVERITY_NOTICE, "Target jump %.1f deg: Integrals reset", (double)jump_deg);
        }
    }

    if (std::isfinite(parsed_msg.angle_y)) {
        _angle_y_filter.update(parsed_msg.angle_y + _pitch_offset_rad);
    }
    if (std::isfinite(parsed_msg.angle_x)) {
        _angle_x_filter.update(parsed_msg.angle_x + _roll_offset_rad);
    }

    const float raw_ay_f = _angle_y_filter.get();
    const float raw_ax_f = _angle_x_filter.get();

    const double frame_ms = double(parsed_msg.time_usec / 1000ULL);
    bool target_sample_new = false;
    bool has_dt_cam = false;
    float dt_cam_s = 0.0f;

    if (std::isfinite(frame_ms) && frame_ms > 0.0) {
        if (!_has_last_frame_ms) {
            target_sample_new = true;
            _last_frame_ms = frame_ms;
            _has_last_frame_ms = true;
        } else if (frame_ms > _last_frame_ms) {
            target_sample_new = true;
            dt_cam_s = constrain_float(float(frame_ms - _last_frame_ms) * 0.001f, 0.015f, 0.5f);
            has_dt_cam = true;
            _last_frame_ms = frame_ms;
        } else {
            target_sample_new = false;
        }
    } else {
        target_sample_new = !_has_last_raw_angles
            || !is_equal(parsed_msg.angle_x, _last_raw_angle_x)
            || !is_equal(parsed_msg.angle_y, _last_raw_angle_y);
    }

    _last_raw_angle_x = parsed_msg.angle_x;
    _last_raw_angle_y = parsed_msg.angle_y;
    _has_last_raw_angles = true;

    const float cam_lag_ms = constrain_float(_cam_lag_ms.get(), 0.0f, 350.0f);
    double target_att_ms = now_ms - double(cam_lag_ms);
    if (std::isfinite(frame_ms) && frame_ms > 0.0) {
        const double rx_lag = now_ms - frame_ms;
        if (rx_lag >= 0.0 && rx_lag <= 400.0) {
            target_att_ms = frame_ms - double(cam_lag_ms);
        }
    }
    _cam_lag_ms_val = constrain_float(float(now_ms - target_att_ms), 0.0f, 400.0f);

    float roll_cam = roll_curr;
    float pitch_cam = pitch_curr;
    float yaw_cam = yaw_curr;
    int8_t i_lo = -1, i_hi = -1;
    double t_lo = -1e12, t_hi = 1e12;
    for (int8_t i = 0; i < 20; i++) {
        const double t_ms = _att_hist_t[i];
        if (t_ms > 0.0) {
            if (t_ms <= target_att_ms && t_ms > t_lo) {
                t_lo = t_ms;
                i_lo = i;
            }
            if (t_ms >= target_att_ms && t_ms < t_hi) {
                t_hi = t_ms;
                i_hi = i;
            }
        }
    }
    if (i_lo >= 0 && i_hi >= 0 && t_hi > t_lo) {
        const float u = float((target_att_ms - t_lo) / (t_hi - t_lo));
        roll_cam = _att_hist_r[i_lo] + u * (_att_hist_r[i_hi] - _att_hist_r[i_lo]);
        pitch_cam = _att_hist_p[i_lo] + u * (_att_hist_p[i_hi] - _att_hist_p[i_lo]);
        yaw_cam = wrap_pi_safe(_att_hist_y[i_lo] + u * wrap_pi_safe(_att_hist_y[i_hi] - _att_hist_y[i_lo]));
    } else if (i_lo >= 0) {
        roll_cam = _att_hist_r[i_lo];
        pitch_cam = _att_hist_p[i_lo];
        yaw_cam = _att_hist_y[i_lo];
    } else if (i_hi >= 0) {
        roll_cam = _att_hist_r[i_hi];
        pitch_cam = _att_hist_p[i_hi];
        yaw_cam = _att_hist_y[i_hi];
    }

    float lambda_raw = 0.0f, elev_ned = 0.0f, angle_x = 0.0f;
    body_angles_to_ned(roll_cam, pitch_cam, yaw_cam, raw_ax_f, raw_ay_f, lambda_raw, elev_ned, angle_x);
    const float dep_raw_deg = -degrees(elev_ned);

    if (!_has_el_filt) {
        _el_filt = elev_ned;
        _el_prev = elev_ned;
        _has_el_filt = true;
        _last_el_ms = now_ms;
        _has_last_el_ms = true;
    } else if (target_sample_new) {
        const float dt_el = has_dt_cam ? dt_cam_s : constrain_float(float(now_ms - (_has_last_el_ms ? _last_el_ms : now_ms)) * 0.001f, 0.02f, 0.5f);
        const float a_el = calc_lowpass_alpha_dt(dt_el, 2.0f);
        _el_filt = _el_filt + a_el * (elev_ned - _el_filt);
        const float raw_el_rate = degrees(_el_filt - _el_prev) / dt_el;
        const float a_er = calc_lowpass_alpha_dt(dt_el, 0.65f);
        _el_rate_dps = (1.0f - a_er) * _el_rate_dps + a_er * constrain_float(raw_el_rate, -25.0f, 25.0f);
        _el_prev = _el_filt;
        _last_el_ms = now_ms;
        _has_last_el_ms = true;
    }

    const float curr_pitch_deg = degrees(pitch_curr);
    const float curr_roll_deg = degrees(roll_curr);
    const float el_rate_val = _el_rate_dps;
    const float bank_flat = constrain_float(1.0f - (fabsf(curr_roll_deg) - 15.0f) / 15.0f, 0.25f, 1.0f);
    const float el_gain = (el_rate_val < 0.0f && dep_raw_deg > 7.5f)
        ? map_range(7.5f, 18.0f, 1.0f, 1.55f, dep_raw_deg) : 0.85f;
    const float dep_w = map_range(0.5f, 2.5f, 0.0f, 1.0f, dep_raw_deg);
    const float gl_up_lim = map_range(-1.5f, 2.0f, 4.5f, 1.0f, curr_pitch_deg);
    const float angle_y = (elev_ned - pitch_curr)
        + radians(constrain_float(el_gain * el_rate_val * bank_flat * dep_w, -4.0f, gl_up_lim))
        + update_vertical_bh(dep_raw_deg, curr_roll_deg, curr_pitch_deg, degrees(raw_ay_f), now_ms);

    const float corr_max_rad = radians(fabsf(_corr_max.get()));
    const float corr_y = constrain_float(parsed_msg.y, -corr_max_rad, corr_max_rad);
    const float corr_x = constrain_float(parsed_msg.x, -corr_max_rad, corr_max_rad);

    _crab_deg = 0.0f;
    Vector3f vel_ned;
    if (AP::ahrs().get_velocity_NED(vel_ned)) {
        const float ahrs_yaw = AP::ahrs().get_yaw();
        if (std::isfinite(ahrs_yaw) && std::isfinite(vel_ned.x) && std::isfinite(vel_ned.y)
            && (vel_ned.x * vel_ned.x + vel_ned.y * vel_ned.y) > 25.0f) {
            _crab_deg = degrees(wrap_pi_safe(ahrs_yaw - atan2f(vel_ned.y, vel_ned.x)));
        }
    }
    const float ax_deg = degrees(angle_x);
    _sx_ang = degrees(atanf(parsed_msg.size_x * 0.829f));
    _b_nose = _sx_ang - ax_deg;
    _b_vel = _sx_ang + _crab_deg - ax_deg;

    float dep_deg = 0.0f;
    bool has_dep = false;
    const bool terminal = terminal_state(dep_raw_deg, now_ms, dep_deg, has_dep);
    float corr_x_used = corr_x;
    if (_has_last_corr_x) {
        const float corr_step_max = radians(25.0f) * constrain_float(float(now_ms - _last_corr_ms) * 0.001f, 0.005f, 0.2f);
        corr_x_used = _last_corr_x_used + constrain_float(corr_x - _last_corr_x_used, -corr_step_max, corr_step_max);
    }
    _last_corr_x_used = corr_x_used;
    _last_corr_ms = now_ms;
    _has_last_corr_x = true;

    if (target_sample_new) {
        if (_has_last_ax) {
            const float dt_ax = has_dt_cam ? dt_cam_s : constrain_float(float(now_ms - _last_ax_ms) * 0.001f, 0.02f, 0.5f);
            const float diff_ax = degrees(angle_x - _last_ax);
            const float a_ax = calc_lowpass_alpha_dt(dt_ax, 2.0f);
            _ax_rate = (1.0f - a_ax) * _ax_rate + a_ax * (fabsf(diff_ax) / dt_ax);
            _ax_rate_signed = (1.0f - a_ax) * _ax_rate_signed + a_ax * (diff_ax / dt_ax);
        }
        _last_ax = angle_x;
        _last_ax_ms = now_ms;
        _has_last_ax = true;
    }

    float shape_f = 1.0f;
    const float dep_ref_deg = fabsf(_dep_ref_deg.get());
    if (has_dep && dep_ref_deg > 0.5f) {
        shape_f = tanf(radians(dep_deg)) / tanf(radians(dep_ref_deg));
    }
    shape_f = constrain_float(shape_f, 1.0f, 2.5f);

    const float shape_y = constrain_float(1.0f + 0.25f * (shape_f - 1.0f), 1.0f, 1.35f);
    const float error_pitch_deg = _pid_y.update(
        degrees(angle_y + corr_y) * shape_y,
        _pitch_sat || (fabsf(curr_roll_deg) > 15.0f),
        target_sample_new, 1.0f, degrees(elev_ned), has_dt_cam, dt_cam_s, now_s);

    const auto &ins_ref = AP::ins();
    const auto &ahrs_ref = AP::ahrs();
    uint8_t gyro_idx = ahrs_ref.get_primary_gyro_index();
    if (!ins_ref.get_gyro_health(gyro_idx)) {
        gyro_idx = ins_ref.get_primary_gyro();
    }
    const Vector3f gyro_vb = ahrs_ref.get_rotation_autopilot_body_to_vehicle_body() * ins_ref.get_gyro(gyro_idx);
    const float cos_p_curr = cosf(pitch_curr);
    const float yaw_rate_rads = (fabsf(cos_p_curr) > 0.15f)
        ? (gyro_vb.y * sinf(roll_curr) + gyro_vb.z * cosf(roll_curr)) / cos_p_curr
        : 0.0f;
    const float yaw_rate_dps = std::isfinite(yaw_rate_rads) ? degrees(yaw_rate_rads) : 0.0f;
    _yaw_rate_dps = yaw_rate_dps;
    const float roll_lim = MAX(10.0f, _r_lim_deg.get());
    _lead_deg = _lead_t.get() * yaw_rate_dps * constrain_float(fabsf(curr_roll_deg) / roll_lim, 0.0f, 1.0f);

    const float shape_x = constrain_float(1.0f + 0.5f * (shape_f - 1.0f), 1.0f, 1.6f);
    const bool bh_on = (_bh_en.get() >= 0.5f);
    const float cur_dep = has_dep ? dep_deg : dep_raw_deg;
    const float bh_corr_deg = update_bh_x(
        lambda_raw, angle_x, shape_x, cur_dep,
        curr_roll_deg, bh_on, target_sample_new, has_dt_cam, dt_cam_s, now_ms);

    const float ax_err_deg = (ax_deg + bh_corr_deg) * shape_x + degrees(corr_x_used) - _lead_deg;
    float d_gain = 1.0f;
    const float freeze_dps = fabsf(_d_freeze_dps.get());
    if (freeze_dps > 1.0f) {
        d_gain = constrain_float(1.0f - (_ax_rate - freeze_dps) / freeze_dps, 0.0f, 1.0f);
    }
    const float error_roll_deg = _pid_x.update(
        ax_err_deg, _roll_sat || bh_on, target_sample_new, d_gain, ax_deg, has_dt_cam, dt_cam_s, now_s);

    float target_roll_deg = 0.0f, target_pitch_deg = 0.0f;
    float roll_rate = 0.0f, pitch_rate = 0.0f, yaw_rate = 0.0f, throttle = 0.0f;
    drive_outputs(roll_curr, curr_pitch_deg, curr_roll_deg, error_pitch_deg, error_roll_deg,
                  terminal, parsed_msg.size_x, parsed_msg.size_y, now_ms,
                  target_roll_deg, target_pitch_deg, roll_rate, pitch_rate, yaw_rate, throttle);

#if HAL_LOGGING_ENABLED
    const uint64_t now_us = AP_HAL::micros64();
    auto &log = AP::logger();
    log.Write("PRTS", "TimeUS,RollRate,PitchRate,YawRate,PrxScale", "s----", "F----", "Qffff",
              now_us, (double)roll_rate, (double)pitch_rate, (double)yaw_rate, 1.0);
    log.Write("TGTM", "TimeUS,DepDeg,LagMs,ProxMax,Term", "s----", "F----", "Qffff",
              now_us, has_dep ? (double)dep_deg : -999.0, (double)_cam_lag_ms_val, 1.0, terminal ? 1.0 : 0.0);
    log.Write("BHLD", "TimeUS,KfLamb,KfRate,Lamb0,Drift", "s----", "F----", "Qffff",
              now_us, _has_lambda_deg ? (double)_lambda_deg : -999.0, (double)_bh_corr_deg,
              _has_lambda_0 ? (double)degrees(_lambda_0) : -999.0, (double)_drift_deg);
    log.Write("TERM", "TimeUS,Dep,Gamma,Gate,DRate", "s----", "F----", "Qffff",
              now_us, has_dep ? (double)dep_deg : -999.0, (double)curr_pitch_deg,
              terminal ? 1.0 : 0.0, (double)_ax_rate);
    log.Write("FRAM", "TimeUS,Crab,SxAng,Ax,CxRaw,CxUsed,Bn,Bv", "s-------", "F-------", "Qfffffff",
              now_us, (double)_crab_deg, (double)_sx_ang, (double)ax_deg,
              (double)degrees(parsed_msg.x), (double)degrees(corr_x), (double)_b_nose, (double)_b_vel);
    log.Write("PERS", "TimeUS,ScrErrX,ScrErrY,AngleX,AngleY,Ss,Dsc", "s------", "F------", "Qffffff",
              now_us, (double)parsed_msg.size_x, (double)parsed_msg.size_y,
              (double)parsed_msg.angle_x, (double)parsed_msg.angle_y, (double)corr_x, (double)corr_y);
#endif

    if (_debug.get() >= 0.5f) {
        _tx_div++;
        if (_tx_div >= 2) {
            _tx_div = 0;
            send_debug_floats(ax_deg, degrees(angle_y), ax_err_deg,
                              degrees(parsed_msg.x), target_roll_deg, target_pitch_deg,
                              has_dep, dep_deg, terminal, yaw_rate, curr_roll_deg, throttle);
        }
    }
}
