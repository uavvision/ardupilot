#include "mode.h"
#include "Plane.h"

bool ModeCruise::_enter()
{
    locked_heading = false;
    using_visual_course = false;
    lock_timer_ms = 0;
    visual_dropout_start_ms = 0;

#if HAL_SOARING_ENABLED
    // for ArduSoar soaring_controller
    plane.g2.soaring_controller.init_cruising();
#endif

    plane.set_target_altitude_current();

    return true;
}

void ModeCruise::update()
{
    /*
      in CRUISE mode we use the navigation code to control
      roll when heading is locked. Heading becomes unlocked on
      any aileron or rudder input
    */
    if (plane.channel_roll->get_control_in() != 0 || plane.channel_rudder->get_control_in() != 0) {
        locked_heading = false;
        using_visual_course = false;
        lock_timer_ms = 0;
        visual_dropout_start_ms = 0;
#if AP_VISUALCOURSE_ENABLED
        plane.visual_course.autotune_reset();
#endif
    }

#if AP_SCRIPTING_ENABLED
    if (plane.nav_scripting_active()) {
        // while a trick is running unlock heading and zero altitude offset
        locked_heading = false;
        lock_timer_ms = 0;
        visual_dropout_start_ms = 0;
        plane.set_target_altitude_current();
#if AP_VISUALCOURSE_ENABLED
        plane.visual_course.autotune_reset();
#endif
    }
#endif
    
    if (!locked_heading) {
        plane.nav_roll_cd = plane.channel_roll->norm_input() * plane.roll_limit_cd;
        plane.update_load_factor();
    } else {
#if AP_VISUALCOURSE_ENABLED
        // handover between GPS and visual while locked (R11)
        if (using_visual_course) {
            // switch back to GPS when it returns, unless the visual course is forced
            if (!plane.visual_course.force() &&
                plane.gps.status() >= AP_GPS::GPS_OK_FIX_2D &&
                plane.gps.ground_speed() >= 3) {
                using_visual_course = false;
                visual_dropout_start_ms = 0;
                locked_heading_cd = plane.gps.ground_course_cd();
                plane.prev_WP_loc = plane.current_loc;
                gcs().send_text(MAV_SEVERITY_INFO, "VisualCourse: GPS restored, using GPS course");
            }
        } else {
            // fall back to visual when GPS is lost while locked
            if (plane.gps.status() < AP_GPS::GPS_OK_FIX_2D &&
                plane.visual_course.enabled() && plane.visual_course.healthy()) {
                using_visual_course = true;
                visual_dropout_start_ms = 0;
                const float locked_course = plane.ahrs.get_yaw() + plane.visual_course.crab_angle();
                locked_heading_cd = degrees(locked_course) * 100.0f;
                plane.visual_course.reset();
                gcs().send_text(MAV_SEVERITY_INFO, "VisualCourse: GPS lost, using visual course");
            }
        }

        if (using_visual_course) {
            // visual course-hold: hold the ground course (heading + crab) at the
            // locked value.
            AP_VisualCourse &vc = plane.visual_course;
            if (vc.healthy()) {
                visual_dropout_start_ms = 0;

                const float course = plane.ahrs.get_yaw() + vc.crab_angle();
                const float locked_course = radians(locked_heading_cd * 0.01f);
                const float course_error = wrap_PI(course - locked_course);
                const float course_rate = plane.ahrs.get_yaw_rate_earth() + vc.crab_rate();

                float roll_cmd;
                if (!vc.autotune_enabled() || !vc.autotune_update(course_error, roll_cmd)) {
                    // positive course error (track right of lock) needs roll left;
                    // also the fallback when autotune finishes or aborts this tick.
                    // Gains are airspeed-scheduled (R9).
                    const float gs = vc.gain_scale();
                    roll_cmd = -gs * (vc.gain_p() * course_error + vc.gain_d() * course_rate);
                }
                plane.nav_roll_cd = constrain_int32(degrees(roll_cmd) * 100.0f,
                                                    -plane.roll_limit_cd, plane.roll_limit_cd);
                vc.set_servo_state(course_error, roll_cmd);
            } else {
                // visual dropout: level the wings, and after 1 s unlock (R10)
                vc.autotune_reset();
                plane.nav_roll_cd = 0;
                if (visual_dropout_start_ms == 0) {
                    visual_dropout_start_ms = AP_HAL::millis();
                } else if (AP_HAL::millis() - visual_dropout_start_ms > 1000) {
                    locked_heading = false;
                    using_visual_course = false;
                    lock_timer_ms = 0;
                    visual_dropout_start_ms = 0;
                    gcs().send_text(MAV_SEVERITY_WARNING, "VisualCourse: dropped out, heading unlocked");
                }
            }
            plane.update_load_factor();
        } else
#endif
        {
            plane.calc_nav_roll();
        }
    }
    plane.update_fbwb_speed_height();
}

/*
  handle CRUISE mode, locking heading to GPS course when we have
  sufficient ground speed, and no aileron or rudder input
 */
void ModeCruise::navigate()
{
#if AP_SCRIPTING_ENABLED
    if (plane.nav_scripting_active()) {
        // don't try to navigate while running trick
        return;
    }
#endif

    // check if we are moving in the direction of the front of the vehicle
    const int32_t ground_course_cd = plane.gps.ground_course_cd();
    const bool moving_forwards = fabsf(wrap_PI(radians(ground_course_cd * 0.01) - plane.ahrs.get_yaw())) < M_PI_2;

    const bool have_gps = (plane.gps.status() >= AP_GPS::GPS_OK_FIX_2D);

#if AP_VISUALCOURSE_ENABLED
    // lock to the visual (FOE-centred) course when GPS is unavailable, or when
    // the visual course is forced (VC_FORCE) for test/validation
    const AP_VisualCourse &vc = plane.visual_course;
    const bool visual_ok = vc.enabled() && vc.healthy();
    const bool use_visual = visual_ok && (vc.force() || !have_gps);
    float airspeed = 0.0f;
    const bool have_airspeed = plane.ahrs.airspeed_estimate(airspeed) &&
                               (airspeed >= plane.aparm.airspeed_min);
#else
    const bool use_visual = false;
    const bool have_airspeed = false;
#endif

    if (!locked_heading &&
        plane.channel_roll->get_control_in() == 0 &&
        plane.rudder_input() == 0 &&
        lock_timer_ms == 0 &&
        ((use_visual && have_airspeed) ||
         (!use_visual && have_gps && plane.gps.ground_speed() >= 3 && moving_forwards))) {
        // user wants to lock the heading - start the timer
        lock_timer_ms = millis();
    }
    if (lock_timer_ms != 0 &&
        (millis() - lock_timer_ms) > 500) {
        // lock the heading after 0.5 seconds of zero heading input
        // from user
        locked_heading = true;
        lock_timer_ms = 0;
        if (use_visual) {
            // lock to the current ground course (heading + crab) and hold it with flow
            using_visual_course = true;
#if AP_VISUALCOURSE_ENABLED
            const float locked_course = plane.ahrs.get_yaw() + plane.visual_course.crab_angle();
            locked_heading_cd = degrees(locked_course) * 100.0f;
            plane.visual_course.reset();
#else
            locked_heading_cd = plane.ahrs.yaw_sensor;
#endif
        } else {
            // GPS course (existing behaviour)
            using_visual_course = false;
            locked_heading_cd = ground_course_cd;
        }
        plane.prev_WP_loc = plane.current_loc;
    }
    if (locked_heading && !using_visual_course) {
        plane.next_WP_loc = plane.prev_WP_loc;
        // always look 1km ahead
        plane.next_WP_loc.offset_bearing(locked_heading_cd*0.01f, plane.prev_WP_loc.get_distance(plane.current_loc) + 1000);
        plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);
    }
}

bool ModeCruise::get_target_heading_cd(int32_t &target_heading) const
{
    // locked_heading_cd holds the ground course (not the yaw heading) for both the
    // GPS and visual paths, so this reports the locked course (R12).
    target_heading = locked_heading_cd;
    return locked_heading;
}
