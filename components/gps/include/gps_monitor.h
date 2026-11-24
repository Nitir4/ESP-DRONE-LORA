#pragma once
#include "gps_nmea.h"
typedef struct {
    gps_nmea_t nmea;
    uint32_t bytes, age_ms;
    bool enabled, uart_ok, data, fix;
} gps_snapshot_t;
void gps_monitor_start(void);
// A short locked copy; callers never perform UART reads or wait for GPS fixes.
void gps_monitor_snapshot(gps_snapshot_t *out);
