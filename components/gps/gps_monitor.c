#include "gps_monitor.h"
#include "gps_nmea.h"
#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static uint32_t millis(void){return esp_timer_get_time()/1000;}
static portMUX_TYPE gps_lock=portMUX_INITIALIZER_UNLOCKED;
static gps_snapshot_t published;
static uint32_t last_byte_ms;

void gps_monitor_snapshot(gps_snapshot_t *out)
{
    portENTER_CRITICAL(&gps_lock);*out=published;portEXIT_CRITICAL(&gps_lock);
    uint32_t now=millis();
    out->age_ms=out->nmea.sentences ? (uint32_t)(now-out->nmea.last_nmea_ms):UINT32_MAX;
    out->data=out->enabled && out->uart_ok && out->age_ms<2500;
    out->fix=out->data && out->nmea.fix && (uint32_t)(now-out->nmea.last_fix_ms)<2500;
}

static void gps_task(void *arg)
{
    (void)arg;
    gps_nmea_t state={0};
    char line[160];size_t used=0;
    uint32_t bytes=0,last_byte=0,report=0;
    for(;;){
        uint8_t data[128];
        int n=uart_read_bytes(CONFIG_GPS_UART_NUM,data,sizeof(data),pdMS_TO_TICKS(50));
        uint32_t now=millis();
        if(n>0){bytes+=n;last_byte=now;}
        for(int i=0;i<n;i++){
            char c=data[i];
            if(c=='$'){used=0;line[used++]=c;}
            else if(c=='\n'){
                line[used]=0;
                if(used && gps_nmea_parse(&state,line,now)){
#ifdef CONFIG_GPS_PRINT_NMEA
                    printf("GPS_NMEA %s\n",line);
#endif
                }
                used=0;
            }else if(c!='\r' && used){
                if(used<sizeof(line)-1 && c>=32 && c<=126)line[used++]=c;
                else used=0;
            }
        }
        portENTER_CRITICAL(&gps_lock);
        published.nmea=state;published.bytes=bytes;last_byte_ms=last_byte;
        portEXIT_CRITICAL(&gps_lock);
        if(now-report>=1000){
            report=now;
            bool data_ok=state.sentences && (uint32_t)(now-state.last_nmea_ms)<2500;
            bool fix=data_ok && state.fix && (uint32_t)(now-state.last_fix_ms)<2500;
            printf("GPS_STATUS {\"ms\":%lu,\"baud\":%d,\"rx\":%d,\"bytes\":%lu,\"sentences\":%lu,\"bad_checksum\":%lu,\"data\":%d,\"fix\":%d,\"satellites\":%d,\"in_view\":%d,\"quality\":%d,\"latitude\":%.7f,\"longitude\":%.7f,\"altitude\":%.2f,\"utc\":\"%s\",\"byte_age_ms\":%ld,\"nmea_age_ms\":%ld}\n",
                (unsigned long)now,CONFIG_GPS_BAUD,CONFIG_GPS_RX_GPIO,(unsigned long)bytes,(unsigned long)state.sentences,
                (unsigned long)state.bad_checksum,data_ok,fix,state.satellites,state.in_view,state.quality,
                state.latitude,state.longitude,state.altitude,state.utc,
                bytes?(long)(uint32_t)(now-last_byte):-1L,
                state.sentences?(long)(uint32_t)(now-state.last_nmea_ms):-1L);
        }
    }
}

void gps_monitor_start(void)
{
#ifndef CONFIG_GPS_ENABLED
    puts("GPS_DISABLED");return;
#endif
    published.enabled=true;
    uart_config_t config={.baud_rate=CONFIG_GPS_BAUD,.data_bits=UART_DATA_8_BITS,
        .parity=UART_PARITY_DISABLE,.stop_bits=UART_STOP_BITS_1,
        .flow_ctrl=UART_HW_FLOWCTRL_DISABLE,.source_clk=UART_SCLK_DEFAULT};
    esp_err_t err=uart_driver_install(CONFIG_GPS_UART_NUM,2048,0,0,NULL,0);
    if(err==ESP_OK)err=uart_param_config(CONFIG_GPS_UART_NUM,&config);
    // Leave GPS RX / ESP GPIO17 untouched; this monitor only listens to GPS TX.
    if(err==ESP_OK)err=uart_set_pin(CONFIG_GPS_UART_NUM,UART_PIN_NO_CHANGE,CONFIG_GPS_RX_GPIO,
                                  UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE);
    if(err!=ESP_OK){printf("GPS_ERROR %s\n",esp_err_to_name(err));return;}
    published.uart_ok=true;
    if(xTaskCreate(gps_task,"gpsMonitor",4096,NULL,1,NULL)!=pdPASS){published.uart_ok=false;puts("GPS_ERROR task allocation");}
    else printf("GPS_READY rx=%d baud=%d mode=receive-only\n",CONFIG_GPS_RX_GPIO,CONFIG_GPS_BAUD);
}
