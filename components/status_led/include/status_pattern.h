#pragma once
#include <stdbool.h>
#include <stdint.h>
enum { STATUS_CONTROLLER,STATUS_DRONE };
enum { STATUS_ROLL=1,STATUS_PITCH=2,STATUS_YAW=4,STATUS_THROTTLE=8 };
enum { STATUS_DRONE_LINK_MS=300,STATUS_CONTROLLER_LINK_MS=500 };
typedef struct {
    int role;
    bool radio_known,radio_ok,have_rx,have_tx,have_gps;
    bool mpu_known,mpu_ok,gps_known,gps_data,gps_fix,gps_stream_ok,armed;
    uint8_t axes;
    uint32_t rx_ms,tx_ms,gps_ms;
} status_input_t;
typedef struct { uint8_t mask,fault,axis; bool linked; } status_output_t;
status_output_t status_pattern(const status_input_t *state,uint32_t now);
