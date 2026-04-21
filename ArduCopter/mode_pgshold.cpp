#include "Copter.h"

#if MODE_PGSHOLD_ENABLED

/*
 * PGSHold flight mode
 *
 * AltHold with gentle automatic position correction using external nav (odometry).
 * When the pilot releases the sticks the mode holds the last known position by
 * applying a PD lean-angle correction derived from the EKF position estimate.
 * The pilot can always override by moving the sticks; on stick release the
 * hold target is re-captured at the current position.
 *
 * If the EKF loses its position or velocity estimate mid-flight the correction
 * is silently disabled and the hold target is invalidated so it re-latches
 * cleanly on recovery.
 *
 * Required EKF source parameters:
 *   EK3_SRC1_POSXY = 6  (EXTNAV)
 *   EK3_SRC1_VELXY = 6  (EXTNAV)
 *   EK3_SRC1_VELZ  = 6  (EXTNAV)
 *
 * Tunable parameters (GCS / MAVLink):
 *   PGSH_P    – position P gain in cd/cm           (default 0.30)
 *   PGSH_D    – velocity D gain in cd/(cm/s)        (default 3.0)
 *   PGSH_MAX  – maximum correction angle in cdeg    (default 300 = 3°)
 */

const AP_Param::GroupInfo ModePGSHold::var_info[] = {

    // @Param: _P
    // @DisplayName: PGSHold position P gain
    // @Description: Position proportional gain. Higher values result in faster position correction but can cause oscillation. Set PGSH_D to zero first when tuning this.
    // @Units: cdeg/cm
    // @Range: 0.05 2.0
    // @Increment: 0.05
    // @User: Standard
    AP_GROUPINFO("_P",   1, ModePGSHold, _pos_p,       0.30f),

    // @Param: _D
    // @DisplayName: PGSHold velocity D gain
    // @Description: Optional velocity derivative gain for damping. Reduces overshoot but amplifies velocity noise. Set to zero to make the mode position-dominant.
    // @Units: cdeg/(cm/s)
    // @Range: 0.0 20.0
    // @Increment: 0.5
    // @User: Standard
    AP_GROUPINFO("_D",   2, ModePGSHold, _vel_d,       0.00f),

    // @Param: _MAX
    // @DisplayName: PGSHold maximum correction angle
    // @Description: Hard limit on the lean-angle correction applied by the position hold algorithm. Keeps the correction authority small relative to pilot authority.
    // @Units: cdeg
    // @Range: 50 1000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("_MAX", 3, ModePGSHold, _max_corr_cd, 300.0f),

    // @Param: _LAT
    // @DisplayName: PGSHold sensor latency
    // @Description: Known end-to-end latency of the external nav position and velocity data. When non-zero, position error is predicted forward by this amount using the EKF horizontal velocity before the PD correction is computed. Set to zero to disable prediction.
    // @Units: s
    // @Range: 0.0 1.0
    // @Increment: 0.02
    // @User: Advanced
    AP_GROUPINFO("_LAT", 4, ModePGSHold, _latency,     0.00f),

    AP_GROUPEND
};

ModePGSHold::ModePGSHold(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

// pgshold_init - initialise PGSHold controller
bool ModePGSHold::init(bool ignore_checks)
{
    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // reset position hold target so it is captured fresh on the first Flying iteration
    _pos_target_set = false;

    return true;
}

// pgshold_run - runs the PGSHold controller
// should be called at 100hz or more
void ModePGSHold::run()
{
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max_cd());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate();

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // State Machine Determination (shared with AltHold)
    AltHoldModeState pgshold_state = get_alt_hold_state(target_climb_rate);

    // State Machine
    switch (pgshold_state) {

    case AltHoldModeState::MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->relax_z_controller(0.0f);
        _pos_target_set = false;
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        _pos_target_set = false;
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_z_controller(0.0f);
        break;

    case AltHoldModeState::Takeoff:
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt, 0.0f, 1000.0f));
        }
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);
        takeoff.do_pilot_takeoff(target_climb_rate);
        _pos_target_set = false;
        break;

    case AltHoldModeState::Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if AP_AVOIDANCE_ENABLED
        copter.avoid.adjust_roll_pitch(target_roll, target_pitch, copter.aparm.angle_max);
#endif

        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

#if AP_RANGEFINDER_ENABLED
        copter.surface_tracking.update_surface_offset();
#endif

        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);

        // -------------------------------------------------------
        // Gentle XY position correction using EKF / odometry data
        // -------------------------------------------------------
        {
            const bool pilot_has_input = !is_zero(target_roll) || !is_zero(target_pitch);

            // Query EKF for position — gates the entire correction block.
            // get_relative_position_NE_origin() returns false when the EKF
            // does not have a valid position estimate (e.g. EXTNAV dropped).
            Vector2f pos_ne_m;
            if (!ahrs.get_relative_position_NE_origin(pos_ne_m)) {
                // No valid position — invalidate hold target so it re-latches on recovery.
                _pos_target_set = false;
            } else {
                const Vector2f curr_pos_cm = pos_ne_m * 100.0f;

                if (pilot_has_input || !_pos_target_set) {
                    // Pilot is steering, or this is the first Flying iteration:
                    // track current position as the hold target.
                    _pos_target_cm  = curr_pos_cm;
                    _pos_target_set = true;
                } else {
                    // Sticks are centered and position is valid — apply PD correction.

                    // position error in earth frame (cm)
                    Vector2f pos_err_cm = _pos_target_cm - curr_pos_cm;

                    // Velocity is optional and only used for derivative damping.
                    // If D is disabled or velocity is unavailable, the controller
                    // runs as position-only with zero damping input.
                    const float kD = _vel_d.get();
                    float vel_n_cms = 0.0f;
                    float vel_e_cms = 0.0f;
                    if (kD > 0.0f) {
                        Vector3f vel_ned_ms;
                        if (ahrs.get_velocity_NED(vel_ned_ms)) {
                            vel_n_cms = vel_ned_ms.x * 100.0f;
                            vel_e_cms = vel_ned_ms.y * 100.0f;
                        }
                    }

                    // If PGSH_LAT > 0, predict position error forward using
                    // horizontal velocity only. This is less aggressive than
                    // IMU acceleration feed-forward and tends to be more robust
                    // to EXTNAV timing error and measurement noise.
                    const float latency_s = _latency.get();
                    if (latency_s > 0.001f) {
                        pos_err_cm.x += vel_n_cms * latency_s;
                        pos_err_cm.y += vel_e_cms * latency_s;
                    }

                    // read tunable gains
                    const float kP          = _pos_p.get();
                    const float max_corr_cd = _max_corr_cd.get();

                    // earth-frame PD output (North and East components)
                    const float out_N = kP * pos_err_cm.x - kD * vel_n_cms;
                    const float out_E = kP * pos_err_cm.y - kD * vel_e_cms;

                    // rotate earth-frame output into body-frame roll / pitch.
                    // Convention follows PosHold get_wind_comp_lean_angles():
                    //   roll  = -N·sin(yaw) + E·cos(yaw)
                    //   pitch = -(N·cos(yaw) + E·sin(yaw))
                    target_roll  += constrain_float(
                        -out_N * ahrs.sin_yaw() + out_E * ahrs.cos_yaw(),
                        -max_corr_cd, max_corr_cd);
                    target_pitch += constrain_float(
                        -(out_N * ahrs.cos_yaw() + out_E * ahrs.sin_yaw()),
                        -max_corr_cd, max_corr_cd);
                }
            }

            // Clamp total lean angle to the vehicle's configured limit so that
            // the sum of pilot input + correction never exceeds angle_max.
            target_roll  = constrain_float(target_roll,  -(float)copter.aparm.angle_max, (float)copter.aparm.angle_max);
            target_pitch = constrain_float(target_pitch, -(float)copter.aparm.angle_max, (float)copter.aparm.angle_max);
        }
        break;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}

#endif  // MODE_PGSHOLD_ENABLED
