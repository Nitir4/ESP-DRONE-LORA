#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "lora_pair.h"

enum { LR_CONTROL=0x10, LR_STATUS=0x20, LR_CRTP=0x30, LR_GPS=0x40, LR_FULL_CONTROL=0x50 };
enum { LR_ARM=1, LR_KILL=2, LR_REPLY=4 };
enum { LR_READY=1, LR_ARMED=2, LR_FAILSAFE=4 };
enum { LR_CONTROL_SIZE=8, LR_MAX_FRAME=35 };
enum { LR_CRTP_COMMAND_SIZE=15, LR_FULL_CONTROL_SIZE=19 };
typedef struct { uint8_t raw[LR_CRTP_COMMAND_SIZE], flags, seq; } lr_full_control_t;
enum { LR_GPS_SIZE=18, LR_GPS_DATA=1, LR_GPS_FIX=2, LR_GPS_MPU_OK=4, LR_GPS_READY=8 };
typedef struct {
    int32_t latitude_e7,longitude_e7;
    int16_t altitude_m;
    uint16_t age_ms;
    uint8_t satellites,in_view,flags,seq;
} lr_gps_t;
void lr_encode_gps(uint8_t out[LR_GPS_SIZE],uint16_t pair,const lr_gps_t *gps);
bool lr_decode_gps(const uint8_t *data,size_t size,uint16_t pair,lr_gps_t *gps);
typedef struct { int8_t roll, pitch, yaw; uint8_t throttle, flags, seq; } lr_control_t;
typedef struct {
    bool synced, armed, arm_requested, have_control;
    uint8_t last_seq;
    uint32_t last_ms;
} lr_guard_t;
typedef enum { LR_ACCEPT, LR_BAD_FRAME, LR_WRONG_PAIR, LR_STALE } lr_result_t;

// Original CRTP header + little-endian <fffH>. Forward raw bytes, not repacked values.
void lr_crtp_pack(uint8_t raw[LR_CRTP_COMMAND_SIZE], float roll, float pitch, float yaw, uint16_t thrust);
bool lr_crtp_read(const uint8_t raw[LR_CRTP_COMMAND_SIZE], float *roll, float *pitch, float *yaw, uint16_t *thrust);
void lr_encode_full_control(uint8_t out[LR_FULL_CONTROL_SIZE], uint16_t pair, const lr_full_control_t *control);
lr_result_t lr_decode_full_control(const uint8_t *data, size_t size, uint16_t pair, lr_full_control_t *out);
lr_result_t lr_guard_accept_full(lr_guard_t *guard, const lr_full_control_t *control, uint32_t now_ms);
void lr_full_control_forward(uint8_t raw[LR_CRTP_COMMAND_SIZE], const lr_full_control_t *control, bool armed);

void lr_header(uint8_t *out, uint16_t pair, uint8_t seq, uint8_t kind_flags);
bool lr_pair_matches(const uint8_t *data, size_t size, uint16_t pair);
void lr_encode_control(uint8_t out[8], uint16_t pair, const lr_control_t *control);
lr_result_t lr_decode_control(const uint8_t *data, size_t size, uint16_t pair, lr_control_t *out);
lr_result_t lr_guard_accept(lr_guard_t *guard, const lr_control_t *control, uint32_t now_ms);
bool lr_guard_expire(lr_guard_t *guard, uint32_t now_ms);
void lr_expand_control(const lr_control_t *control, bool armed, float *roll, float *pitch,
                       float *yaw, uint16_t *thrust);
