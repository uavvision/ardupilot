#pragma once

#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

class KAM_Targeting {
public:
    KAM_Targeting();

    CLASS_NO_COPY(KAM_Targeting);

    static const struct AP_Param::GroupInfo var_info[];

    void update();
    void update_gyro_yaw();
    void handle_landing_target(const mavlink_landing_target_t &packet);

private:
    struct PID {
        float kP = 0.0f;
        float kI = 0.0f;
        float kD = 0.0f;
        float iMax = 0.0f;
        bool has_iMax = false;
        float d_max = 0.0f;
        float d_cutoff_hz = 4.0f;
        bool i_decay = true;

        bool has_last_t = false;
        double last_t = 0.0;
        bool has_last_err = false;
        float last_err = 0.0f;
        bool has_last_d_t = false;
        double last_d_t = 0.0;

        float I = 0.0f;
        float P = 0.0f;
        float D = 0.0f;
        float total = 0.0f;
        float err = 0.0f;
        float d_filtered = 0.0f;

        void init(float p, float i, float d, float imax, float d_cut, bool decay = true);
        float update(float error, bool inhibit_i, bool new_sample, float d_gain,
                     float err_d, bool has_sample_dt, float sample_dt_s, double now_s);
        void reset(float integrator);
        void log(const char *name) const;
    };

    struct LowPass {
        float alpha = 1.0f;
        float prev = 0.0f;
        bool has_prev = false;

        void init(float a);
        float update(float value);
        void reset_empty();
        void reset(float value);
        float get() const { return prev; }
        bool valid() const { return has_prev; }
    };

    struct SlewRate {
        float rate = 0.0f;
        float prev = 0.0f;
        double last_t = 0.0;
        bool has_prev = false;
        bool has_last_t = false;

        void init(float r);
        float update(float value, double now_s);
        void reset(float value, double now_s);
    };

    struct ParamRef {
        AP_Param *ptr = nullptr;
        enum ap_var_type type = AP_PARAM_NONE;
        void bind(const char *name);
        float get(float def) const;
    };

    // Parameters (table 139 equivalent, KAM_ prefix)
    AP_Float _p_x;
    AP_Float _p_y;
    AP_Float _i_x;
    AP_Float _i_y;
    AP_Float _d_x;
    AP_Float _d_y;
    AP_Float _thr;
    AP_Float _p_v;
    AP_Float _i_v;
    AP_Float _d_v;
    AP_Float _vel;
    AP_Float _la;
    AP_Float _p_o;
    AP_Float _r_o;
    AP_Float _prx;
    AP_Float _pry;
    AP_Float _m_use;
    AP_Float _stab;
    AP_Float _rdr;
    AP_Float _x_imax;
    AP_Float _y_imax;
    AP_Int32 _hud_mask;
    AP_Float _ev;
    AP_Float _ae_m_mode;
    AP_Float _ae_c_mode;
    AP_Float _p_lim_deg;
    AP_Float _r_lim_deg;
    AP_Float _thr_slew;
    AP_Float _calib;
    AP_Float _restart;
    AP_Float _prox_scale;
    AP_Float _prox_time_s;
    AP_Float _prox_time_e;
    AP_Float _prox_opt;
    AP_Float _pitch_t_lim;
    AP_Float _d_fcut;
    AP_Float _corr_max;
    AP_Float _rx_tmo;
    AP_Float _fov_guard;
    AP_Float _p_slew;
    AP_Float _cam_lag_ms;
    AP_Float _term_dep_deg;
    AP_Float _dep_ref_deg;
    AP_Float _d_freeze_dps;
    AP_Float _turn_coord;
    AP_Float _lead_t;
    AP_Float _debug;
    AP_Float _los_gain;
    AP_Float _bh_en;
    AP_Float _bh_p;
    AP_Float _bh_i;
    AP_Float _bh_max;
    AP_Float _bh_slew;
    AP_Float _bh_d;
    AP_Int8  _hard;

    // Cached external parameters
    ParamRef _ap_roll_tau;
    ParamRef _ap_pitch_tau;
    ParamRef _ap_rmax;
    ParamRef _ap_rmax_up;
    ParamRef _ap_rmax_dn;
    ParamRef _ap_rll_ff;
    ParamRef _ap_roll_lim;
    ParamRef _scr_user1;
    bool _ext_params_bound = false;

    // Pending LANDING_TARGET message
    mavlink_landing_target_t _pending_target{};
    double _pending_rx_ms = 0.0;
    bool _has_pending_target = false;

    // Controllers & filters
    PID _pid_y;
    PID _pid_x;
    PID _pid_v;
    LowPass _angle_x_filter;
    LowPass _angle_y_filter;
    LowPass _throttle_filter;
    SlewRate _throttle_slew;

    // Tuned cache
    struct {
        float px = -1.0f, ix = -1.0f, dx = -1.0f, ximax = -1.0f;
        float py = -1.0f, iy = -1.0f, dy = -1.0f, yimax = -1.0f;
        float pv = -1.0f, iv = -1.0f, dv = -1.0f;
        float la = -1.0f, slew = -1.0f, dcut = -1.0f;
    } _tuned;

    float _max_throttle = 0.0f;
    float _pitch_offset_rad = 0.0f;
    float _roll_offset_rad = 0.0f;

    // Runtime state (mirrors Lua ST)
    bool _targeting_activated = false;
    bool _hard_announced = false;
    int32_t _hud_mask_cached = 0x00FFFFFF;
    int32_t _elapsed_ms = 0;
    float _last_m_use = 0.0f;
    uint8_t _tx_div = 0;

    bool _has_last_target_rx_ms = false;
    double _last_target_rx_ms = 0.0;
    bool _has_last_pitch_cmd = false;
    float _last_pitch_cmd_deg = 0.0f;
    double _last_pitch_cmd_ms = 0.0;
    bool _has_last_raw_angles = false;
    float _last_raw_angle_x = 0.0f;
    float _last_raw_angle_y = 0.0f;
    bool _has_last_corr_x = false;
    float _last_corr_x_used = 0.0f;
    double _last_corr_ms = 0.0;

    float _ax_rate = 0.0f;
    float _ax_rate_signed = 0.0f;
    float _los_rate_x = 0.0f;
    bool _has_last_ax = false;
    float _last_ax = 0.0f;
    double _last_ax_ms = 0.0;
    bool _pitch_sat = false;
    bool _roll_sat = false;
    float _turn_dps = 0.0f;
    float _crab_deg = 0.0f;
    float _sx_ang = 0.0f;
    float _b_nose = 0.0f;
    float _b_vel = 0.0f;
    float _fov_coef = 1.0f;
    float _yaw_rate_dps = 0.0f;
    float _lead_deg = 0.0f;

    // Vertical Bearing Hold
    bool _has_el_filt = false;
    float _el_filt = 0.0f;
    float _el_prev = 0.0f;
    float _el_rate_dps = 0.0f;
    float _el_rate_slow = 0.0f;
    bool _has_last_el_ms = false;
    double _last_el_ms = 0.0;
    bool _has_el_0 = false;
    float _el_0 = 0.0f;
    bool _vtrim_armed = false;
    float _el_integral = 0.0f;
    float _el_corr_deg = 0.0f;
    float _el_drift_deg = 0.0f;
    bool _has_last_el_calc_ms = false;
    double _last_el_calc_ms = 0.0;
    bool _has_el_engage_ms = false;
    double _el_engage_ms = 0.0;

    // Depression & terminal gate
    bool _has_dep_filt = false;
    float _dep_filt = 0.0f;
    double _dep_ms = 0.0;
    bool _term_latched = false;

    // AHRS 20-sample ring buffer
    double _att_hist_t[20]{};
    float _att_hist_r[20]{};
    float _att_hist_p[20]{};
    float _att_hist_y[20]{};
    uint8_t _att_hist_idx = 0;
    bool _has_last_frame_ms = false;
    double _last_frame_ms = 0.0;
    float _cam_lag_ms_val = 0.0f;

    // Horizontal Bearing Hold
    bool _has_lambda_0 = false;
    float _lambda_0 = 0.0f;
    float _lambda_init_sin = 0.0f;
    float _lambda_init_cos = 0.0f;
    uint16_t _lambda_init_cnt = 0;
    bool _has_lambda_filt = false;
    float _lambda_filt = 0.0f;
    float _lambda_prev = 0.0f;
    float _lambda_rate_filt = 0.0f;
    float _bh_integral = 0.0f;
    bool _has_bh_corr_deg = false;
    float _bh_corr_deg = 0.0f;
    float _drift_deg = 0.0f;
    bool _has_lambda_deg = false;
    float _lambda_deg = 0.0f;
    bool _has_last_bh_ms = false;
    double _last_bh_ms = 0.0;
    bool _has_last_bh_calc_ms = false;
    double _last_bh_calc_ms = 0.0;
    bool _has_last_bh_target_ms = false;
    double _last_bh_target_ms = 0.0;
    bool _has_bh_engage_ms = false;
    double _bh_engage_ms = 0.0;

    // Raw IMU gyro-integrated yaw (compass/EKF-independent)
    float _gyro_yaw = 0.0f;
    bool _gyro_yaw_valid = false;

    void bind_ext_params();
    void reset_bh_state(bool has_engage_ms, double engage_ms);
    void reset_pid(bool x, bool y, bool v);
    void refresh_tunables();
    void set_message_interval();
    void deactivate_targeting(const char *msg_text);
    bool terminal_state(float dep_raw_deg, double now_ms, float &dep_deg_out, bool &has_dep_out);
    float update_vertical_bh(float dep_raw_deg, float curr_roll_deg, float curr_pitch_deg, float raw_ay_deg, double now_ms);
    float update_bh_x(float lambda_raw, float angle_x, float shape_x, float cur_dep, float curr_roll_deg,
                      bool bh_on, bool target_sample_new, bool has_dt_cam, float dt_cam_s, double now_ms);
    float get_roll_servo_out(float angle_err_deg) const;
    float get_pitch_servo_out(float angle_err_deg) const;
    float get_coordination_rate_offset() const;
    void drive_outputs(float roll_curr, float curr_pitch_deg, float curr_roll_deg,
                       float error_pitch_deg, float error_roll_deg, bool terminal,
                       float size_x, float size_y, double now_ms,
                       float &target_roll_deg_out, float &target_pitch_deg_out,
                       float &roll_rate_out, float &pitch_rate_out, float &yaw_rate_out, float &throttle_out);
    void send_debug_floats(float ax_deg, float ay_deg, float ax_err_deg, float cxr_deg,
                           float roll_t, float pitch_t, bool has_dep, float dep_deg,
                           bool terminal, float yaw_cmd, float roll_deg, float thr) const;
};
