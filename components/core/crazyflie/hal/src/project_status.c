#include "project_status.h"
#include "gps_monitor.h"
#include "status_led.h"
#include "sensors_mpu6050_hm5883L_ms5611.h"
#include "loralink.h"
#include "system.h"
#include "log.h"
#include "led.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

static uint8_t gpsData,gpsFix,gpsSatellites,gpsInView,mpuOkay;
static int32_t gpsLatitude,gpsLongitude;
static float gpsAltitude;
static uint32_t gpsAge;
static bool gpsReturnState(void)
{
#ifdef CONFIG_TRANSPORT_LORA
    return loralinkGpsReturnEnabled();
#else
    return false;
#endif
}

static void status_task(void *arg)
{
    (void)arg;uint32_t report=0;
    char command[48];size_t used=0;
    for(;;){
        uint32_t now=esp_timer_get_time()/1000;
        gps_snapshot_t gps;gps_monitor_snapshot(&gps);
        bool mpu=sensorsMpu6050Health();
        status_led_health(now>=5000,mpu,gps.enabled && now>=5000,gps.data,gps.fix,true,systemIsArmed());
        status_led_tick(now);
#if defined(CONFIG_PROJECT_STATUS_LEDS) && defined(CONFIG_TRANSPORT_LORA)
        benchLedLevel[LED_GREEN]=(status_led_mask&1)!=0;
        benchLedLevel[LED_BLUE]=(status_led_mask&2)!=0;
        benchLedLevel[LED_RED]=(status_led_mask&4)!=0;
        benchLedRises[LED_GREEN]=status_led_edges[0];
        benchLedRises[LED_BLUE]=status_led_edges[1];
        benchLedRises[LED_RED]=status_led_edges[2];
#endif
        gpsData=gps.data;gpsFix=gps.fix;gpsSatellites=gps.nmea.satellites;
        gpsInView=gps.nmea.in_view;gpsLatitude=llround(gps.nmea.latitude*1e7);
        gpsLongitude=llround(gps.nmea.longitude*1e7);gpsAltitude=gps.nmea.altitude;
        gpsAge=gps.age_ms;mpuOkay=mpu;
        // This is a telemetry switch only; it cannot arm, move or bypass controls.
#ifdef CONFIG_TRANSPORT_LORA
        char data[48];int n=read(STDIN_FILENO,data,sizeof(data));
        for(int i=0;i<n;i++){
            if(data[i]=='\n' || data[i]=='\r'){
                command[used]=0;
                if(!strcmp(command,"GPS_RETURN 0")){loralinkSetGpsReturn(false);puts("GPS_RETURN 0");}
                else if(!strcmp(command,"GPS_RETURN 1")){loralinkSetGpsReturn(true);printf("GPS_RETURN %d\n",loralinkGpsReturnEnabled());}
                used=0;
            }else if(used<sizeof(command)-1)command[used++]=data[i];else used=0;
        }
#endif
        if(now-report>=1000){
            report=now;status_output_t leds=status_led_snapshot(now);
            printf("BOARD_STATUS {\"ms\":%lu,\"mpu_ok\":%d,\"gps_data\":%d,\"gps_fix\":%d,\"gps_return\":%d,\"led_mask\":%u,\"fault_code\":%u,\"axis_code\":%u}\n",
                (unsigned long)now,mpu,gps.data,gps.fix,gpsReturnState(),leds.mask,leds.fault,leds.axis);
        }
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

void projectStatusStart(void)
{
    bool active_low=false;
#ifdef CONFIG_BOARD_DRONE_LEDS_ACTIVE_LOW
    active_low=true;
#endif
#ifdef CONFIG_TRANSPORT_LORA
    status_led_init(STATUS_DRONE,CONFIG_LED_PIN_GREEN,CONFIG_LED_PIN_BLUE,CONFIG_LED_PIN_RED,active_low);
    fcntl(STDIN_FILENO,F_SETFL,O_NONBLOCK);
#endif
    if(xTaskCreate(status_task,"boardStatus",4096,NULL,1,NULL)!=pdPASS)puts("BOARD_STATUS_ERROR task allocation");
}

// Native ESP-Drone log variables, accessible over its existing CRTP log service.
// GPS is available to the project; no unvalidated GPS navigation/fusion is added.
LOG_GROUP_START(gps)
LOG_ADD(LOG_UINT8, data, &gpsData)
LOG_ADD(LOG_UINT8, fix, &gpsFix)
LOG_ADD(LOG_UINT8, sats, &gpsSatellites)
LOG_ADD(LOG_UINT8, inView, &gpsInView)
LOG_ADD(LOG_INT32, latE7, &gpsLatitude)
LOG_ADD(LOG_INT32, lonE7, &gpsLongitude)
LOG_ADD(LOG_FLOAT, altM, &gpsAltitude)
LOG_ADD(LOG_UINT32, ageMs, &gpsAge)
LOG_GROUP_STOP(gps)
LOG_GROUP_START(diagLed)
LOG_ADD(LOG_UINT8, mask, &status_led_mask)
LOG_ADD(LOG_UINT8, mpuOkay, &mpuOkay)
LOG_GROUP_STOP(diagLed)
