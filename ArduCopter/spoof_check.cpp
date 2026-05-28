#include "Copter.h"
#include <AP_NavEKF3/AP_NavEKF3_Spoof.h>

void Copter::spoof_check()
{

    if (gps_spoof_clear_requested()) {
        gps_spoof_clear_request_reset();
        failsafe_spoof_off_event();
        return;
    }
    if (gps_spoof_trigger_requested()) {
        gps_spoof_trigger_request_reset();
        failsafe_spoof_event();
        return;
    }
    if (!motors->armed()) {
        return;
    }

    if (failsafe.spoof) {
        return;
    }
    if (gps_spoof_is_latched()) {
        return;
    }

    nav_filter_status filt_status = inertial_nav.get_filter_status();

    const bool spoof_suspected =
        filt_status.flags.gps_glitching ||
        (
            filt_status.flags.using_gps &&
            !filt_status.flags.gps_quality_good
        );

    if (spoof_suspected) {
        failsafe_spoof_event();
    }
}

void Copter::failsafe_spoof_event()
{
    if (failsafe.spoof) {
        return;
    }

    failsafe.spoof = true;

    if (g.fs_spoof_action == FS_SPOOF_ACTION_DO_NOTHING) {
        gps_spoof_set_latched(false);
    }
    else {
        gps_spoof_set_latched(true);
    }
    LOGGER_WRITE_ERROR(LogErrorSubsystem::GPS, LogErrorCode::GPS_GLITCH);
    gcs().send_text(MAV_SEVERITY_CRITICAL, "GPS spoof suspected: GPS fusion blocked");

    if (!motors->armed()) {
        return;
    }

    // take action based on fs_ekf_action parameter
    switch (g.fs_spoof_action) {
        case FS_SPOOF_ACTION_ALTHOLD:
            // AltHold
            if (!set_mode(Mode::Number::ALT_HOLD, ModeReason::SPOOF_FAILSAFE)) {
                set_mode_land_with_pause(ModeReason::SPOOF_FAILSAFE);
            }
            break;
        case FS_SPOOF_ACTION_LAND:
                set_mode_land_with_pause(ModeReason::SPOOF_FAILSAFE);
                break;
        case FS_SPOOF_ACTION_LAND_EVEN_STABILIZE:
            // Land even if in Stabilize or another manual mode
             if (!set_mode(Mode::Number::LAND, ModeReason::SPOOF_FAILSAFE)) {
                set_mode_land_with_pause(ModeReason::SPOOF_FAILSAFE);
            }
            break;
        case FS_SPOOF_ACTION_PGS:
            if (!set_mode(Mode::Number::PGSHOLD, ModeReason::SPOOF_FAILSAFE)) {
                set_mode_land_with_pause(ModeReason::SPOOF_FAILSAFE);
            }
            break;
        case FS_SPOOF_ACTION_DO_NOTHING:
             // Do nothing, but log the event
             break;
        default:
            set_mode_land_with_pause(ModeReason::SPOOF_FAILSAFE);
            break;
    }


    gcs().send_text(MAV_SEVERITY_CRITICAL,
                    "Spoof failsafe: changed to %s",
                    flightmode->name());
}

void Copter::failsafe_spoof_off_event()
{
    failsafe.spoof = false;

    // Тільки ручне/контрольоване очищення
    gps_spoof_set_latched(false);

    gcs().send_text(MAV_SEVERITY_CRITICAL, "GPS spoof failsafe cleared");
}

void Copter::failsafe_spoof_recheck()
{
    if (!failsafe.spoof) {
        return;
    }

    if (!motors->armed()) {
        return;
    }

    if (flightmode->requires_GPS()) {
        if (!set_mode(Mode::Number::ALT_HOLD, ModeReason::SPOOF_FAILSAFE)) {
            set_mode_land_with_pause(ModeReason::SPOOF_FAILSAFE);
        }
    }
}