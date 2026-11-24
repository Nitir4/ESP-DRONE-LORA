#include "gps_nmea.h"
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int hex(char c)
{
    if(c>='0' && c<='9')return c-'0';
    c=toupper((unsigned char)c);
    return c>='A' && c<='F' ? c-'A'+10 : -1;
}

static bool coordinate(const char *value,const char *hemisphere,bool longitude,double *out)
{
    if(!*value || strlen(hemisphere)!=1)return false;
    char *end;double n=strtod(value,&end);
    if(*end || !isfinite(n) || n<0)return false;
    double deg=floor(n/100),minutes=n-deg*100,limit=longitude?180:90;
    if(minutes>=60 || deg+minutes/60>limit)return false;
    char h=*hemisphere;
    if(longitude ? (h!='E' && h!='W') : (h!='N' && h!='S'))return false;
    *out=(deg+minutes/60)*(h=='S' || h=='W' ? -1:1);
    return true;
}

bool gps_nmea_parse(gps_nmea_t *s,const char *line,uint32_t now)
{
    if(*line!='$')return false;
    const char *star=strchr(line,'*');
    if(!star || strlen(star)!=3)return false;
    int high=hex(star[1]),low=hex(star[2]);unsigned check=0;
    for(const char *p=line+1;p<star;p++)check^=(unsigned char)*p;
    if(high<0 || low<0 || check!=(unsigned)((high<<4)|low)){s->bad_checksum++;return false;}
    size_t len=(size_t)(star-line-1);
    char payload[160];if(len>=sizeof(payload))return false;
    memcpy(payload,line+1,len);payload[len]=0;
    char *f[24]={payload};int fields=1;
    for(char *p=payload;*p;p++)if(*p==','){
        *p=0;if(fields<24)f[fields++]=p+1;
    }
    if(strlen(f[0])!=5)return false;
    s->sentences++;s->last_nmea_ms=now;
    bool position=false;
    if(!strcmp(f[0]+2,"GGA") && fields>=11){
        s->quality=atoi(f[6]);s->satellites=atoi(f[7]);
        snprintf(s->utc,sizeof(s->utc),"%s",f[1]);
        s->fix=s->quality>0 && coordinate(f[2],f[3],false,&s->latitude) &&
               coordinate(f[4],f[5],true,&s->longitude);
        if(s->fix){s->altitude=atof(f[9]);position=true;}
    }else if(!strcmp(f[0]+2,"RMC") && fields>=10){
        snprintf(s->utc,sizeof(s->utc),"%s",f[1]);
        s->fix=!strcmp(f[2],"A") && coordinate(f[3],f[4],false,&s->latitude) &&
               coordinate(f[5],f[6],true,&s->longitude);
        position=s->fix;
    }else if(!strcmp(f[0]+2,"GSV") && fields>=4){
        s->in_view=atoi(f[3]);
    }
    if(position)s->last_fix_ms=now;
    return true;
}
