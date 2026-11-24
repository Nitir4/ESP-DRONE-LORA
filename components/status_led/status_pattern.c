#include "status_pattern.h"

static bool pulses(uint32_t phase,unsigned count,unsigned step,unsigned on)
{return phase<count*step && phase%step<on;}

status_output_t status_pattern(const status_input_t *s,uint32_t now)
{
    status_output_t o={0};bool drone=s->role==STATUS_DRONE;
    o.linked=s->radio_ok && s->have_rx && (uint32_t)(now-s->rx_ms)<
        (drone?STATUS_DRONE_LINK_MS:STATUS_CONTROLLER_LINK_MS);
    uint8_t faults[5];unsigned n=0;
    if(s->mpu_known && !s->mpu_ok)faults[n++]=1;
    if(s->radio_known && !s->radio_ok)faults[n++]=2;
    if(s->radio_ok && now>=5000 && !o.linked)faults[n++]=3;
    // Missing telemetry means unknown remote sensor health, not a failed GPS.
    if(s->gps_known && s->gps_stream_ok && !s->gps_data)faults[n++]=4;
    if(!drone && s->gps_known && !s->gps_stream_ok)faults[n++]=5;
    if(n)o.fault=faults[(now/3000u)%n];

    bool activity=false;
    bool fresh=drone ? o.linked : (s->radio_ok && s->have_tx && (uint32_t)(now-s->tx_ms)<300u);
    if(fresh){
        uint8_t axes[4];unsigned count=0;
        for(unsigned i=0;i<4;i++)if(s->axes&(1u<<i))axes[count++]=i+1;
        if(count){o.axis=axes[(now/800u)%count];activity=pulses(now%800u,o.axis,140,60);}
        else activity=true; // Steady neutral stream cannot be mistaken for roll's one-pulse code.
    }
    bool gps=s->gps_data && (s->gps_fix || now%1000u<250);
    bool event=s->have_gps && (uint32_t)(now-s->gps_ms)<600;
    unsigned gps_pulses=!s->gps_data?1u:(s->gps_fix?3u:2u);
    if(event)gps=pulses((uint32_t)(now-s->gps_ms),gps_pulses,140,60);
    if(drone){
        bool fault=n ? pulses(now%3000u,o.fault,200,100) : s->armed;
        // Dedicated green link output must never pulse for axes, GPS or faults.
        o.mask=(o.linked?1:0)|(gps?2:0)|(fault?4:0); // green, blue, red
    }else{
        // GPIO21 is exclusively the return-link indicator, including while disarmed.
        bool other=n ? pulses(now%3000u,o.fault,200,100) : activity;
        o.mask=(other?1:0)|(o.linked?2:0);
    }
    return o;
}
