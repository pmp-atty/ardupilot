#include "AP_NavEKF3_Spoof.h"

static bool gps_spoof_latched = false;
static bool gps_spoof_clear_req = false;
static bool gps_spoof_trigger_req = false;

void gps_spoof_set_latched(bool v)
{
    gps_spoof_latched = v;
}

bool gps_spoof_is_latched()
{
    return gps_spoof_latched;
}

void gps_spoof_request_clear()
{
    gps_spoof_clear_req = true;
}

bool gps_spoof_clear_requested()
{
    return gps_spoof_clear_req;
}

void gps_spoof_clear_request_reset()
{
    gps_spoof_clear_req = false;
}

void gps_spoof_trigger_request_set()
{
    gps_spoof_trigger_req = true;
}

void gps_spoof_trigger_request_reset()
{
    gps_spoof_trigger_req = false;
}
bool gps_spoof_trigger_requested()
{
    return gps_spoof_trigger_req;
}
