#include "lora_protocol.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    uint8_t bytes[8];lr_control_t c={.roll=120,.pitch=-120,.yaw=90,.throttle=255,.flags=LR_ARM|LR_REPLY,.seq=255},d;
    lr_encode_control(bytes,0x1234,&c);
    assert(sizeof(bytes)==8 && bytes[0]==0x34 && bytes[1]==0x12);
    assert(lr_decode_control(bytes,8,0x1234,&d)==LR_ACCEPT);
    assert(d.roll==120 && d.pitch==-120 && d.yaw==90 && d.throttle==255 && d.flags==5 && d.seq==255);
    assert(lr_decode_control(bytes,8,0x1235,&d)==LR_WRONG_PAIR);
    for(size_t n=0;n<8;n++)assert(lr_decode_control(bytes,n,0x1234,&d)==LR_BAD_FRAME);
    bytes[3]|=8;assert(lr_decode_control(bytes,8,0x1234,&d)==LR_BAD_FRAME);bytes[3]&=~8;
    bytes[4]=128;assert(lr_decode_control(bytes,8,0x1234,&d)==LR_BAD_FRAME);bytes[4]=120;
    float r,p,y;uint16_t thrust;
    lr_expand_control(&c,true,&r,&p,&y,&thrust);
    assert(r==30 && p== -30 && y==180 && thrust==60000);
    lr_expand_control(&c,false,&r,&p,&y,&thrust);assert(r==0 && p==0 && y==0 && thrust==0);
    c.flags|=LR_KILL;lr_expand_control(&c,true,&r,&p,&y,&thrust);assert(thrust==0);

    lr_guard_t guard={0};
    c=(lr_control_t){.seq=254,.flags=LR_ARM,.throttle=0};
    assert(lr_guard_accept(&guard,&c,0)==LR_STALE); // no arm on initial acquisition
    c.flags=0;c.throttle=1;assert(lr_guard_accept(&guard,&c,0)==LR_STALE);
    c.throttle=0;assert(lr_guard_accept(&guard,&c,0)==LR_ACCEPT);
    assert(!guard.armed);
    c.seq=255;c.flags=LR_ARM;assert(lr_guard_accept(&guard,&c,50)==LR_ACCEPT && guard.armed);
    c.seq=0;c.throttle=128;assert(lr_guard_accept(&guard,&c,100)==LR_ACCEPT && guard.armed);
    uint32_t last=guard.last_ms;
    assert(lr_guard_accept(&guard,&c,120)==LR_STALE && guard.last_ms==last);
    c.seq=255;assert(lr_guard_accept(&guard,&c,150)==LR_STALE && guard.last_ms==last);
    assert(!lr_guard_expire(&guard,400));
    assert(lr_guard_expire(&guard,401) && !guard.armed && !guard.synced);
    c.seq=1;assert(lr_guard_accept(&guard,&c,450)==LR_STALE); // held arm must not reacquire
    c.flags=0;c.throttle=0;assert(lr_guard_accept(&guard,&c,500)==LR_ACCEPT);
    c.seq=2;c.flags=LR_ARM;c.throttle=100;
    assert(lr_guard_accept(&guard,&c,550)==LR_ACCEPT && !guard.armed);
    c.seq=3;c.throttle=0;assert(lr_guard_accept(&guard,&c,600)==LR_ACCEPT && !guard.armed);
    c.seq=4;c.flags=0;assert(lr_guard_accept(&guard,&c,650)==LR_ACCEPT);
    c.seq=5;c.flags=LR_ARM;assert(lr_guard_accept(&guard,&c,700)==LR_ACCEPT && guard.armed);
    c.seq=6;c.flags=LR_KILL|LR_ARM;c.throttle=255;
    assert(lr_guard_accept(&guard,&c,750)==LR_ACCEPT && !guard.armed);
    guard=(lr_guard_t){0};c=(lr_control_t){0};
    assert(lr_guard_accept(&guard,&c,0xffffff00u)==LR_ACCEPT);
    assert(!lr_guard_expire(&guard,0x20));assert(lr_guard_expire(&guard,0x30)); // millis rollover
    for(int v=-127;v<=127;v++) {
        c=(lr_control_t){.roll=v,.pitch=v,.yaw=v,.seq=(uint8_t)v,.throttle=(uint8_t)(v+127)};
        lr_encode_control(bytes,0x1234,&c);assert(lr_decode_control(bytes,8,0x1234,&d)==LR_ACCEPT);
        assert(c.roll==d.roll && c.pitch==d.pitch && c.yaw==d.yaw && c.throttle==d.throttle);
    }
    puts("PASS: frame bounds, all signed values, endian encoding, pair rejection, sequence rollover/replay, arming edges, kill and timeout rollover");
}
