#pragma once
#include "status_pattern.h"
void status_led_init(int role,int activity_gpio,int state_gpio,int fault_gpio,bool active_low);
void status_led_radio(bool okay);
void status_led_health(bool mpu_known,bool mpu_ok,bool gps_known,bool gps_data,bool gps_fix,bool gps_stream_ok,bool armed);
void status_led_control_rx(uint32_t now,uint8_t axes);
void status_led_control_tx(uint32_t now,uint8_t axes);
void status_led_reply_rx(uint32_t now,bool gps_packet);
void status_led_gps_tx(uint32_t now);
void status_led_tick(uint32_t now);
status_output_t status_led_snapshot(uint32_t now);
extern uint8_t status_led_mask;
extern uint32_t status_led_edges[3];
