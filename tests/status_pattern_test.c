#include "status_pattern.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    status_input_t s={.role=STATUS_DRONE,.radio_known=true,.radio_ok=true,
        .have_rx=true,.mpu_known=true,.mpu_ok=true,.gps_known=true,.gps_data=true,
        .gps_fix=true,.gps_stream_ok=true};
    // Every simultaneous axis combination must cycle only its actual channel codes.
    for(unsigned mask=1;mask<16;mask++)for(unsigned slot=0;slot<16;slot++){
        unsigned codes[4],count=0;
        for(unsigned bit=0;bit<4;bit++)if(mask&(1u<<bit))codes[count++]=bit+1;
        uint32_t now=800*slot+10;s.rx_ms=now;s.axes=mask;
        status_output_t o=status_pattern(&s,now);
        assert(o.axis==codes[slot%count] && (o.mask&1) && !o.fault);
    }
    s.axes=0;s.rx_ms=6000;assert(status_pattern(&s,6000).linked);
    assert((status_pattern(&s,6000).mask&1) && !(status_pattern(&s,6000).mask&4));
    s.mpu_ok=false;assert(status_pattern(&s,6000).fault==1);s.mpu_ok=true;
    s.radio_ok=false;assert(status_pattern(&s,6000).fault==2);s.radio_ok=true;
    s.rx_ms=5000;assert(status_pattern(&s,6000).fault==3);
    assert(!(status_pattern(&s,6000).mask&1));
    s.rx_ms=6000;s.gps_data=false;assert(status_pattern(&s,6000).fault==4);
    assert(!(status_pattern(&s,6000).mask&2));s.gps_data=true;
    s.have_gps=true;s.gps_ms=6000;
    assert(status_pattern(&s,6000).mask&2);
    assert(!(status_pattern(&s,6070).mask&2));
    assert(status_pattern(&s,6140).mask&2);
    assert(status_pattern(&s,6280).mask&2);
    s.role=STATUS_CONTROLLER;s.have_tx=true;s.tx_ms=6000;s.gps_stream_ok=false;
    assert(status_pattern(&s,6000).fault==5);
    s.gps_data=false;assert(status_pattern(&s,6000).fault==5);s.gps_data=true;
    for(uint32_t t=6000;t<10000;t+=25){s.rx_ms=t;s.tx_ms=t;assert(status_pattern(&s,t).mask&2);}
    // Link and event freshness must survive the 32-bit millisecond wrap.
    s.gps_stream_ok=true;s.rx_ms=0xfffffff0u;s.tx_ms=0xfffffff0u;s.gps_ms=0xfffffff0u;
    assert(status_pattern(&s,20).linked);
    // Dedicated link outputs cannot be overridden by axes, faults, arming or GPS bursts.
    for(unsigned role=0;role<2;role++)for(unsigned flags=0;flags<256;flags++)
    for(unsigned axes=0;axes<16;axes++)for(uint32_t phase=0;phase<3000;phase+=25){
        uint32_t now=6000+phase;
        s=(status_input_t){.role=role,.radio_known=true,.radio_ok=true,
            .have_rx=true,.have_tx=true,.have_gps=true,.mpu_known=true,
            .gps_known=true,.mpu_ok=(flags&1)!=0,.gps_data=(flags&2)!=0,
            .gps_fix=(flags&4)!=0,.gps_stream_ok=(flags&8)!=0,.armed=(flags&16)!=0,
            .axes=axes,.rx_ms=now,.tx_ms=now,.gps_ms=now-phase%600};
        if(flags&32)s.have_rx=false;
        if(flags&64)s.radio_ok=false;
        if(flags&128)s.rx_ms=now-2000;
        status_output_t o=status_pattern(&s,now);
        unsigned link_bit=role==STATUS_DRONE?1:2;
        bool expected=(flags&224)==0;
        assert(o.linked==expected && !!(o.mask&link_bit)==expected);
    }
    // Strict deadline boundaries, including a timestamp that crosses uint32 wrap.
    for(unsigned role=0;role<2;role++)for(unsigned wrap=0;wrap<2;wrap++){
        uint32_t deadline=role==STATUS_DRONE?STATUS_DRONE_LINK_MS:STATUS_CONTROLLER_LINK_MS;
        s=(status_input_t){.role=role,.radio_ok=true,.have_rx=true,
            .rx_ms=wrap?0xfffffff0u:6000u};
        assert(status_pattern(&s,s.rx_ms+deadline-1).linked);
        assert(!status_pattern(&s,s.rx_ms+deadline).linked);
    }
    puts("PASS: dedicated link invariants, startup/loss/radio faults, deadline boundaries, axes/GPS/fault patterns, timer wrap");
}
