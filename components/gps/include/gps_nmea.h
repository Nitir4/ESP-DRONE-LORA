#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t sentences, bad_checksum, last_nmea_ms, last_fix_ms;
    bool fix;
    int quality, satellites, in_view;
    double latitude, longitude, altitude;
    char utc[16];
} gps_nmea_t;

// Input is one NMEA sentence, without CR/LF. Never accepts unchecked data.
bool gps_nmea_parse(gps_nmea_t *state, const char *line, uint32_t now_ms);
