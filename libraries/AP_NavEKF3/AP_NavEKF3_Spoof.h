#pragma once

extern bool g_gps_spoof_latched;

void gps_spoof_set_latched(bool v);
bool gps_spoof_is_latched();

void gps_spoof_request_clear();
bool gps_spoof_clear_requested();
void gps_spoof_clear_request_reset();
void gps_spoof_trigger_request_set();
bool gps_spoof_trigger_requested();
void gps_spoof_trigger_request_reset();