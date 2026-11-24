#include "lora_protocol.h"
#include <float.h>
#include <math.h>
#include <string.h>

_Static_assert(sizeof(float)==4 && FLT_RADIX==2 && FLT_MANT_DIG==24 && FLT_MAX_EXP==128,
               "CRTP requires IEEE-754 binary32 floats");

static void put32(uint8_t *p,uint32_t v){for(int i=0;i<4;i++)p[i]=(uint8_t)(v>>(8*i));}
static uint32_t get32(const uint8_t *p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}

void lr_crtp_pack(uint8_t raw[LR_CRTP_COMMAND_SIZE],float roll,float pitch,float yaw,uint16_t thrust)
{
    raw[0]=0x30; // CRTP port 3, channel 0, matching ESP-Drone's CRTP_HEADER.
    float values[]={roll,pitch,yaw};
    for(unsigned i=0;i<3;i++){uint32_t bits;memcpy(&bits,&values[i],4);put32(raw+1+i*4,bits);}
    raw[13]=(uint8_t)thrust;raw[14]=(uint8_t)(thrust>>8);
}

bool lr_crtp_read(const uint8_t raw[LR_CRTP_COMMAND_SIZE],float *roll,float *pitch,float *yaw,uint16_t *thrust)
{
    // Preserve reserved header bits; cflib may set them to 1 (0x3c).
    if((raw[0]&0xf3)!=0x30)return false;
    float values[3];
    for(unsigned i=0;i<3;i++){uint32_t bits=get32(raw+1+i*4);memcpy(&values[i],&bits,4);if(!isfinite(values[i]))return false;}
    *roll=values[0];*pitch=values[1];*yaw=values[2];
    *thrust=(uint16_t)raw[13]|((uint16_t)raw[14]<<8);
    return true;
}

void lr_encode_full_control(uint8_t out[LR_FULL_CONTROL_SIZE],uint16_t pair,const lr_full_control_t *c)
{lr_header(out,pair,c->seq,LR_FULL_CONTROL|c->flags);memcpy(out+4,c->raw,LR_CRTP_COMMAND_SIZE);}

lr_result_t lr_decode_full_control(const uint8_t *data,size_t size,uint16_t pair,lr_full_control_t *out)
{
    if(size!=LR_FULL_CONTROL_SIZE || (data[3]&0xf8)!=LR_FULL_CONTROL)return LR_BAD_FRAME;
    if(!lr_pair_matches(data,size,pair))return LR_WRONG_PAIR;
    float r,p,y;uint16_t t;
    if(!lr_crtp_read(data+4,&r,&p,&y,&t))return LR_BAD_FRAME;
    memcpy(out->raw,data+4,LR_CRTP_COMMAND_SIZE);out->seq=data[2];out->flags=data[3]&7;
    return LR_ACCEPT;
}

void lr_full_control_forward(uint8_t raw[LR_CRTP_COMMAND_SIZE],const lr_full_control_t *c,bool armed)
{
    memcpy(raw,c->raw,LR_CRTP_COMMAND_SIZE);
    // Protocol preservation does not bypass arming/kill; only active controls pass unchanged.
    if(!armed || (c->flags&LR_KILL))memset(raw+1,0,LR_CRTP_COMMAND_SIZE-1);
}

lr_result_t lr_guard_accept_full(lr_guard_t *guard,const lr_full_control_t *c,uint32_t now_ms)
{
    float r,p,y;uint16_t thrust;
    if(!lr_crtp_read(c->raw,&r,&p,&y,&thrust))return LR_BAD_FRAME;
    // Share the existing guard using a zero/nonzero predicate, never quantize the actual thrust.
    lr_control_t gate={.seq=c->seq,.flags=c->flags,.throttle=thrust!=0};
    return lr_guard_accept(guard,&gate,now_ms);
}

void lr_encode_gps(uint8_t out[LR_GPS_SIZE],uint16_t pair,const lr_gps_t *g)
{
    lr_header(out,pair,g->seq,LR_GPS|g->flags);
    put32(out+4,(uint32_t)g->latitude_e7);put32(out+8,(uint32_t)g->longitude_e7);
    out[12]=(uint8_t)g->altitude_m;out[13]=(uint8_t)((uint16_t)g->altitude_m>>8);
    out[14]=g->satellites;out[15]=g->in_view;
    out[16]=(uint8_t)g->age_ms;out[17]=(uint8_t)(g->age_ms>>8);
}

bool lr_decode_gps(const uint8_t *p,size_t size,uint16_t pair,lr_gps_t *g)
{
    if(size!=LR_GPS_SIZE || !lr_pair_matches(p,size,pair) || (p[3]&0xf0)!=LR_GPS)return false;
    lr_gps_t value={.seq=p[2],.flags=p[3]&15,.latitude_e7=(int32_t)get32(p+4),
        .longitude_e7=(int32_t)get32(p+8),.altitude_m=(int16_t)((uint16_t)p[12]|((uint16_t)p[13]<<8)),
        .satellites=p[14],.in_view=p[15],.age_ms=(uint16_t)p[16]|((uint16_t)p[17]<<8)};
    if(value.latitude_e7< -900000000 || value.latitude_e7>900000000 ||
       value.longitude_e7< -1800000000 || value.longitude_e7>1800000000 ||
       ((value.flags&LR_GPS_FIX) && !(value.flags&LR_GPS_DATA)))return false;
    *g=value;return true;
}

void lr_header(uint8_t *out, uint16_t pair, uint8_t seq, uint8_t kind_flags)
{
    out[0]=(uint8_t)pair; out[1]=(uint8_t)(pair>>8); out[2]=seq; out[3]=kind_flags;
}

bool lr_pair_matches(const uint8_t *data, size_t size, uint16_t pair)
{
    return size>=4 && data[0]==(uint8_t)pair && data[1]==(uint8_t)(pair>>8);
}

void lr_encode_control(uint8_t out[8], uint16_t pair, const lr_control_t *control)
{
    lr_header(out,pair,control->seq,LR_CONTROL|control->flags);
    out[4]=(uint8_t)control->roll; out[5]=(uint8_t)control->pitch;
    out[6]=(uint8_t)control->yaw; out[7]=control->throttle;
}

lr_result_t lr_decode_control(const uint8_t *data, size_t size, uint16_t pair, lr_control_t *out)
{
    if(size!=8 || (data[3]&0xf8)!=LR_CONTROL) return LR_BAD_FRAME;
    if(!lr_pair_matches(data,size,pair)) return LR_WRONG_PAIR;
    // -128 is reserved so all three signed channels have symmetric ranges.
    if(data[4]==128 || data[5]==128 || data[6]==128) return LR_BAD_FRAME;
    *out=(lr_control_t){.seq=data[2],.flags=data[3]&7,.roll=(int8_t)data[4],
                       .pitch=(int8_t)data[5],.yaw=(int8_t)data[6],.throttle=data[7]};
    return LR_ACCEPT;
}

bool lr_guard_expire(lr_guard_t *guard, uint32_t now_ms)
{
    if(guard->have_control && (uint32_t)(now_ms-guard->last_ms)>LORA_LINK_TIMEOUT_MS) {
        guard->have_control=false; guard->synced=false;
        guard->armed=false; guard->arm_requested=false;
        return true;
    }
    return false;
}

lr_result_t lr_guard_accept(lr_guard_t *guard, const lr_control_t *control, uint32_t now_ms)
{
    lr_guard_expire(guard,now_ms);
    if(!guard->synced) {
        // A fresh session must begin disarmed, with throttle exactly zero.
        if(control->throttle || (control->flags&LR_ARM)) return LR_STALE;
    } else {
        uint8_t delta=(uint8_t)(control->seq-guard->last_seq);
        if(delta==0 || delta>=128) return LR_STALE;
    }
    guard->synced=true; guard->have_control=true;
    guard->last_seq=control->seq; guard->last_ms=now_ms;
    bool wants_arm=(control->flags&LR_ARM)!=0;
    if((control->flags&LR_KILL) || !wants_arm) guard->armed=false;
    // Explicit low-throttle rising edge: cannot auto-rearm after a timeout.
    else if(!guard->arm_requested && control->throttle==0) guard->armed=true;
    guard->arm_requested=wants_arm && !(control->flags&LR_KILL);
    return LR_ACCEPT;
}

void lr_expand_control(const lr_control_t *c, bool armed, float *roll, float *pitch,
                       float *yaw, uint16_t *thrust)
{
    bool active=armed && !(c->flags&LR_KILL);
    *roll=active ? c->roll*.25f : 0;
    *pitch=active ? c->pitch*.25f : 0;
    *yaw=active ? c->yaw*2.f : 0;
    *thrust=active ? (uint16_t)(((uint32_t)c->throttle*60000u+127u)/255u) : 0;
}
