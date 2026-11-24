#include "lora_protocol.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    // Golden CRTP bytes independently produced by Python struct.pack('<BfffH',...).
    const uint8_t expected[]={0x3c,0x00,0x00,0x04,0x41,0x00,0x00,0xc8,0xc0,
        0x00,0x00,0xf4,0x41,0x39,0x30}; // 8.25, -6.25, 30.5, 12345
    lr_full_control_t c={.seq=255,.flags=LR_ARM|LR_REPLY},d;
    memcpy(c.raw,expected,sizeof(expected));
    uint8_t frame[LR_FULL_CONTROL_SIZE],forwarded[LR_CRTP_COMMAND_SIZE];
    lr_encode_full_control(frame,0x1234,&c);
    assert(sizeof(frame)==19 && frame[0]==0x34 && frame[1]==0x12 && frame[2]==255 && frame[3]==0x55);
    assert(!memcmp(frame+4,expected,sizeof(expected)));
    assert(lr_decode_full_control(frame,19,0x1234,&d)==LR_ACCEPT);
    lr_full_control_forward(forwarded,&d,true);
    assert(!memcmp(forwarded,expected,sizeof(expected))); // Including cflib reserved header bits.
    float r,p,y;uint16_t t;
    assert(lr_crtp_read(forwarded,&r,&p,&y,&t) && r==8.25f && p==-6.25f && y==30.5f && t==12345);
    for(size_t len=0;len<=LR_MAX_FRAME;len++)if(len!=19)
        assert(lr_decode_full_control(frame,len,0x1234,&d)==LR_BAD_FRAME);
    assert(lr_decode_full_control(frame,19,0x1235,&d)==LR_WRONG_PAIR);
    frame[3]|=8;assert(lr_decode_full_control(frame,19,0x1234,&d)==LR_BAD_FRAME);frame[3]&=~8;
    frame[4]=0x31;assert(lr_decode_full_control(frame,19,0x1234,&d)==LR_BAD_FRAME);frame[4]=0x3c;
    frame[5]=0;frame[6]=0;frame[7]=0x80;frame[8]=0x7f; // +infinity
    assert(lr_decode_full_control(frame,19,0x1234,&d)==LR_BAD_FRAME);
    lr_crtp_pack(c.raw,NAN,0,0,0);lr_encode_full_control(frame,0x1234,&c);
    assert(lr_decode_full_control(frame,19,0x1234,&d)==LR_BAD_FRAME);

    // Full uint16 range and values not representable by the old 8-bit channels.
    for(unsigned thrust=0;thrust<=65535;thrust++){
        lr_crtp_pack(c.raw,8.123456f,-6.234567f,30.345678f,thrust);
        lr_encode_full_control(frame,0x1234,&c);
        assert(lr_decode_full_control(frame,19,0x1234,&d)==LR_ACCEPT);
        lr_full_control_forward(forwarded,&d,true);
        assert(!memcmp(forwarded,c.raw,sizeof(c.raw)));
        assert(lr_crtp_read(forwarded,&r,&p,&y,&t));
        assert(r==8.123456f && p==-6.234567f && y==30.345678f && t==thrust);
    }
    // Many finite float bit patterns, including negative zero and subnormals.
    uint32_t seed=1;
    for(unsigned n=0;n<10000;n++){
        c.raw[0]=n&1?0x30:0x3c;
        for(unsigned j=1;j<13;j++){seed=seed*1664525u+1013904223u;c.raw[j]=seed>>24;}
        if(!lr_crtp_read(c.raw,&r,&p,&y,&t))continue;
        lr_encode_full_control(frame,0x1234,&c);
        assert(lr_decode_full_control(frame,19,0x1234,&d)==LR_ACCEPT);
        lr_full_control_forward(forwarded,&d,true);assert(!memcmp(forwarded,c.raw,15));
    }
    lr_full_control_forward(forwarded,&c,false);
    assert(forwarded[0]==c.raw[0]);for(unsigned i=1;i<15;i++)assert(!forwarded[i]);
    c.flags=LR_ARM|LR_KILL;lr_full_control_forward(forwarded,&c,true);
    for(unsigned i=1;i<15;i++)assert(!forwarded[i]);

    lr_guard_t guard={0};c=(lr_full_control_t){0};lr_crtp_pack(c.raw,0,0,0,0);
    c.flags=LR_ARM;assert(lr_guard_accept_full(&guard,&c,0)==LR_STALE);
    c.flags=0;lr_crtp_pack(c.raw,0,0,0,1);assert(lr_guard_accept_full(&guard,&c,0)==LR_STALE);
    lr_crtp_pack(c.raw,0,0,0,0);assert(lr_guard_accept_full(&guard,&c,0)==LR_ACCEPT);
    c.seq=1;c.flags=LR_ARM;lr_crtp_pack(c.raw,0,0,0,1);
    assert(lr_guard_accept_full(&guard,&c,50)==LR_ACCEPT && !guard.armed); // Even thrust=1 blocks arming.
    c.seq=2;c.flags=0;lr_crtp_pack(c.raw,0,0,0,0);assert(lr_guard_accept_full(&guard,&c,100)==LR_ACCEPT);
    c.seq=3;c.flags=LR_ARM;assert(lr_guard_accept_full(&guard,&c,150)==LR_ACCEPT && guard.armed);
    lr_crtp_pack(c.raw,0,0,0,65535);c.seq=4;
    assert(lr_guard_accept_full(&guard,&c,200)==LR_ACCEPT && guard.armed);
    assert(lr_guard_accept_full(&guard,&c,220)==LR_STALE && guard.last_ms==200);
    assert(!lr_guard_expire(&guard,500));assert(lr_guard_expire(&guard,501) && !guard.armed);
    c.seq=5;assert(lr_guard_accept_full(&guard,&c,550)==LR_STALE); // No auto-rearm after loss.
    c.flags=0;lr_crtp_pack(c.raw,0,0,0,0);assert(lr_guard_accept_full(&guard,&c,600)==LR_ACCEPT);
    c.seq=6;c.flags=LR_ARM;assert(lr_guard_accept_full(&guard,&c,650)==LR_ACCEPT && guard.armed);
    c.seq=7;c.flags=LR_ARM|LR_KILL;assert(lr_guard_accept_full(&guard,&c,700)==LR_ACCEPT && !guard.armed);
    puts("PASS: original CRTP golden bytes, 65536 thrust values, float-bit preservation, bounds/NaN/headers, arm/kill/replay/expiry");
}
