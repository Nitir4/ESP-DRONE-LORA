#include "gps_nmea.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static bool sentence(gps_nmea_t *s,const char *payload,uint32_t now)
{
    char line[180];unsigned checksum=0;
    for(const char *p=payload;*p;p++)checksum^=(unsigned char)*p;
    snprintf(line,sizeof(line),"$%s*%02X",payload,checksum);
    return gps_nmea_parse(s,line,now);
}

int main(void)
{
    gps_nmea_t s={0};
    assert(gps_nmea_parse(&s,"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47",100));
    assert(s.fix && s.satellites==8 && s.quality==1 && s.last_fix_ms==100);
    assert(fabs(s.latitude-48.1173)<1e-7 && fabs(s.longitude-11.5166666667)<1e-7);
    assert(fabs(s.altitude-545.4)<1e-7);
    assert(sentence(&s,"GPGGA,123520,,,,,0,00,99.99,,,,,,",1100));
    assert(!s.fix && s.satellites==0 && s.last_fix_ms==100);
    assert(sentence(&s,"GPRMC,123521,A,3350.000,S,15112.000,E,0.0,0.0,121126,,,A",2100));
    assert(s.fix && fabs(s.latitude+33.8333333333)<1e-7 && fabs(s.longitude-151.2)<1e-7);
    assert(sentence(&s,"GNRMC,123522,V,,,,,,,121126,,,N",3100));
    assert(!s.fix);
    assert(sentence(&s,"GPGSV,2,1,07,01,40,100,30",3100) && s.in_view==7);
    uint32_t count=s.sentences,age=s.last_nmea_ms;
    assert(!gps_nmea_parse(&s,"$GPRMC,123521,A,3350.000,S,15112.000,E,0.0,0.0,121126,,,A*00",3200));
    assert(s.sentences==count && s.last_nmea_ms==age && s.bad_checksum==1);
    assert(sentence(&s,"GPGGA,123523,4867.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,",4100));
    assert(!s.fix); // Invalid minutes cannot become a fix.
    assert(sentence(&s,"GPGGA,123524,4807.038,X,01131.000,E,1,08,0.9,545.4,M,46.9,M,,",5100));
    assert(!s.fix);
    assert(!gps_nmea_parse(&s,"$GPGGA*",5200));
    assert(!gps_nmea_parse(&s,"noise",5200));
    puts("PASS: checksum, empty no-fix fields, GGA/RMC/GSV, hemispheres, invalid coordinates");
}
