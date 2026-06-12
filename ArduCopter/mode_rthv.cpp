#include "Copter.h"

#if MODE_RTHV_ENABLED

const AP_Param::GroupInfo ModeRTHV::var_info[] = {

    // @Param: TIME_GAIN
    // @DisplayName: RTHV return time gain
    // @Description: Multiplier applied to accumulated outbound flight time to calculate RTHV return duration.
    // @Units: scalar
    // @Range: 0.5 5.0
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("TIME_GAIN", 1, ModeRTHV, _time_gain, 1.00f),

    // @Param: TIME_MAX
    // @DisplayName: RTHV maximum return time
    // @Description: Maximum time RTHV is allowed to fly the return vector before executing the configured action.
    // @Units: s
    // @Range: 1 600
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TIME_MAX", 2, ModeRTHV, _time_max, 600.00f),

    // @Param: TIME_MIN
    // @DisplayName: RTHV minimum return time
    // @Description: Minimum time RTHV will fly the return vector even if accumulated outbound time is small.
    // @Units: s
    // @Range: 0 120
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TIME_MIN", 3, ModeRTHV, _time_min, 10.00f),

    // @Param: PTCH_RATE
    // @DisplayName: RTHV pitch ramp rate
    // @Description: Rate at which RTHV increases forward pitch command during return vector flight.
    // @Units: cdeg/s
    // @Range: 10 1000
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("PTCH_RATE", 4, ModeRTHV, _pitch_rate, 100.00f),

    // @Param: ALT_SLEW
    // @DisplayName: RTHV climb rate
    // @Description: Vertical climb rate command used until the RTHV target altitude is reached.
    // @Units: cm/s
    // @Range: 0 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("ALT_SLEW", 5, ModeRTHV, _alt_slew, 100.0f),

    // @Param: ALT_TRGT
    // @DisplayName: RTHV target altitude
    // @Description: Altitude target used by RTHV climb logic, in centimeters above EKF origin.
    // @Units: cm
    // @Range: 0 30000
    // @Increment: 100
    // @User: Standard
    AP_GROUPINFO("ALT_TRGT", 6, ModeRTHV, _alt_target, 10000.00f),

    // @Param: ACTION
    // @DisplayName: RTHV action after return time
    // @Description: Action to execute after the RTHV return time expires.
    // @Values: 0:None,1:Land,2:RTL,3:AltHold
    // @User: Standard
    AP_GROUPINFO("ACTION", 7, ModeRTHV, _action, 1.00f),

    // @Param: WIND_SPD
    // @DisplayName: RTHV estimated wind speed
    // @Description: Estimated wind speed used for yaw wind compensation. This is a user-provided estimate, not a measured value.
    // @Units: m/s
    // @Range: 0 30
    // @Increment: 0.5
    // @User: Standard
    AP_GROUPINFO("YAW_COMP", 8, ModeRTHV, _yaw_comp, 0.00f),

AP_GROUPEND
};

enum RTHVAction
{
    RTHVAction_None,
    RTHVAction_LAND,
    RTHVAction_RTL,
    RTHVAction_HOLD,
};

/*
 * Init and run calls for althold, flight mode
 */

// rthv_init - initialise rthv controller
bool ModeRTHV::init(bool ignore_checks)
{
    if (!pos_control->is_active_z())
    {
        pos_control->init_z_controller();
    }

    pos_control->set_max_speed_accel_z(
        -get_pilot_speed_dn(),
        g.pilot_speed_up,
        g.pilot_accel_z);

    pos_control->set_correction_speed_accel_z(
        -get_pilot_speed_dn(),
        g.pilot_speed_up,
        g.pilot_accel_z);

    _target_yaw_cd = wrap_360_cd(ahrs.yaw_sensor + 18000+_yaw_comp.get()*100.0f);

    _rthv_pitch_cd = 0.0f;
    _last_update_ms = AP_HAL::millis();
    _rthv_start_ms = _last_update_ms;

    return_time_done = false;
    action_notify = true;

    _rthv_return_time_ms = constrain_float(
        copter._rthv_outbound_time_ms *
            _time_gain.get(),
        _time_min.get() * 1000.0f,
        _time_max.get() * 1000.0f);


    gcs().send_text(
        MAV_SEVERITY_WARNING,
        "RTHV: Return time: %lu",
        (unsigned long)_rthv_return_time_ms);

    return true;
}
// rthv_run - runs the rthv controller
// should be called at 100hz or more
void ModeRTHV::run()
{
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    const uint32_t now = AP_HAL::millis();

    if ((now - _rthv_start_ms) >= _rthv_return_time_ms && return_time_done == false)
    {
        // Time to return home has been reached, set flag to stop horizontal movement and start vertical descent
        gcs().send_text(MAV_SEVERITY_WARNING, "RTHV: Return time reached");
        return_time_done = true;
    }

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    float target_roll = 0.0f;

    const float dt = (now - _last_update_ms) * 0.001f;
    _last_update_ms = now;

    const float pitch_rate_cd_s = _pitch_rate.get();

    const float max_pitch_cd =
        attitude_control->get_althold_lean_angle_max_cd();
    // обмеження: не більше ніж -15°

    _rthv_pitch_cd -= pitch_rate_cd_s * dt;
    _rthv_pitch_cd = constrain_float(
        _rthv_pitch_cd,
        -max_pitch_cd,
        0.0f);

    if (return_time_done)
    {
        if (action_notify)
        {
            gcs().send_text(MAV_SEVERITY_WARNING, "RTHV: Executing action %d", (int)_action.get());
            action_notify = false;
        }
        switch ((RTHVAction)(int(_action.get())))
        {
        case RTHVAction_LAND:
            // do nothing, just let the vehicle descend with the current horizontal position hold settings
            set_mode(Mode::Number::LAND, ModeReason::RETURN_HOME_VECTOR);
            break;
        case RTHVAction_RTL:
            // set the horizontal position hold target to the home location to achieve return to launch
            if (!copter.position_ok())
            {
                _rthv_pitch_cd = 0.0f;
                break;
            }
            copter._spoof_check_suppress_until_ms = AP_HAL::millis() + 25000;
            if (!set_mode(Mode::Number::RTL, ModeReason::RETURN_HOME_VECTOR))
            {
                gcs().send_text(MAV_SEVERITY_WARNING, "RTHV: RTL failed");
                _rthv_pitch_cd = 0.0f;
            }
            break;
        case RTHVAction_HOLD:
            // set the horizontal position hold target to the current location to achieve a hold in place
            set_mode(Mode::Number::ALT_HOLD, ModeReason::RETURN_HOME_VECTOR);
            break;
        case RTHVAction_None:
            // do nothing, just hold position with the current horizontal position hold settings
            _rthv_pitch_cd = 0.0f;
            break;
        }
    }
    const float current_alt_cm = inertial_nav.get_position_z_up_cm();
    const float target_alt_cm = _alt_target.get();

    float target_climb_rate = 0.0f;

    if (current_alt_cm < target_alt_cm)
    {
        target_climb_rate = _alt_slew.get(); // наприклад 100 cm/s
    }

    target_climb_rate = constrain_float(
        target_climb_rate,
        0.0f,
        g.pilot_speed_up);

    // Alt Hold State Machine Determination
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

    // Alt Hold State Machine
    switch (althold_state)
    {

    case AltHoldModeState::MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->relax_z_controller(0.0f); // forces throttle output to decay to zero
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_z_controller(0.0f); // forces throttle output to decay to zero
        break;

    case AltHoldModeState::Takeoff:
        // initiate take-off
        if (!takeoff.running())
        {
            takeoff.start(constrain_float(g.pilot_takeoff_alt, 0.0f, 1000.0f));
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate);
        break;

    case AltHoldModeState::Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

#if AP_RANGEFINDER_ENABLED
        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();
#endif

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
        break;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_yaw(
        target_roll,
        _rthv_pitch_cd,
        _target_yaw_cd,
        true);
    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}
#endif