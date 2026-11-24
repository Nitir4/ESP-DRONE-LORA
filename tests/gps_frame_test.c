#include "lora_protocol.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    uint8_t p[LR_GPS_SIZE];lr_gps_t decoded;
    lr_gps_t g={.latitude_e7=-900000000,.longitude_e7=1800000000,.altitude_m=-123,
        .age_ms=65535,.satellites=12,.in_view=25,.flags=15,.seq=255};
    lr_encode_gps(p,0x1234,&g);
    assert(sizeof(p)==18 && p[0]==0x34 && p[1]==0x12 && p[2]==255 && p[3]==0x4f);
    assert(lr_decode_gps(p,18,0x1234,&decoded));
    assert(decoded.latitude_e7==g.latitude_e7 && decoded.longitude_e7==g.longitude_e7);
    assert(decoded.altitude_m== -123 && decoded.age_ms==65535 && decoded.satellites==12 && decoded.in_view==25);
    for(unsigned n=0;n<18;n++)assert(!lr_decode_gps(p,n,0x1234,&decoded));
    assert(!lr_decode_gps(p,18,0x1235,&decoded));
    g.flags=LR_GPS_FIX;lr_encode_gps(p,0x1234,&g);assert(!lr_decode_gps(p,18,0x1234,&decoded));
    g.flags=0;g.latitude_e7=900000001;lr_encode_gps(p,0x1234,&g);assert(!lr_decode_gps(p,18,0x1234,&decoded));
    g.latitude_e7=0;g.longitude_e7=-1800000001;lr_encode_gps(p,0x1234,&g);assert(!lr_decode_gps(p,18,0x1234,&decoded));
    g.longitude_e7=0;g.flags=LR_GPS_READY;g.age_ms=65535;lr_encode_gps(p,0x1234,&g);
    assert(lr_decode_gps(p,18,0x1234,&decoded) && !(decoded.flags&LR_GPS_DATA));
    puts("PASS: 18-byte GPS layout, signed coordinates/altitude, bounds, truncated frames, pair filtering, invalid fix flags");
}
