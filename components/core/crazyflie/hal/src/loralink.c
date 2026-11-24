#include "loralink.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "lora_radio.h"
#include "lora_protocol.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "system.h"
#include "ledseq.h"
#include "log.h"
#include "sensors.h"
#include "gps_monitor.h"
#include "status_led.h"
#include "sensors_mpu6050_hm5883L_ms5611.h"
#include "param.h"

static bool initialized, ready, peer_seen;
static QueueHandle_t controls, incoming, outgoing;
static lr_guard_t guard;
static uint32_t peer_ms, accepted, wrong_pair, rejected, crc_bad, stale, timeouts, tx_count, tx_dropped;
static uint32_t tunnel_rx,tunnel_tx;
static int16_t last_rssi;
static int8_t last_snr;
static float last_roll,last_pitch,last_yaw;
static uint16_t last_thrust;
static uint8_t last_seq;
static logVarId_t roll_id,pitch_id,age_id;
static logVarId_t set_roll_id,set_pitch_id,set_yaw_id,set_thrust_id;
static float input_roll,input_pitch,input_yaw;
static uint16_t input_thrust;
static uint8_t input_flags,input_bytes;
static uint8_t last_forwarded[LR_CRTP_COMMAND_SIZE];
static uint32_t gps_tx_count,status_tx_count,last_gps_tx,last_status_tx;
#if defined(CONFIG_GPS_ENABLED) && defined(CONFIG_GPS_TELEMETRY_ENABLED)
static uint8_t gps_return_enabled=1;
#else
static uint8_t gps_return_enabled;
#endif
void loralinkSetGpsReturn(bool enabled)
{
#if defined(CONFIG_GPS_ENABLED) && defined(CONFIG_GPS_TELEMETRY_ENABLED)
    gps_return_enabled=enabled;
#else
    (void)enabled;gps_return_enabled=0;
#endif
}
bool loralinkGpsReturnEnabled(void){return gps_return_enabled!=0;}

static uint32_t millis(void) { return esp_timer_get_time()/1000; }

static void queue_control(const lr_control_t *c,bool armed)
{
    CRTPPacket packet={.size=14,.header=CRTP_HEADER(CRTP_PORT_SETPOINT,0)};
    float roll,pitch,yaw;uint16_t thrust;
    lr_expand_control(c,armed,&roll,&pitch,&yaw,&thrust);
    memcpy(packet.data,&roll,4);memcpy(packet.data+4,&pitch,4);
    memcpy(packet.data+8,&yaw,4);memcpy(packet.data+12,&thrust,2);
    last_roll=roll;last_pitch=pitch;last_yaw=yaw;last_thrust=thrust;
    memcpy(last_forwarded,packet.raw,LR_CRTP_COMMAND_SIZE);
    xQueueOverwrite(controls,&packet);
}


static void queue_full_control(const lr_full_control_t *c,bool armed)
{
    CRTPPacket packet={.size=LR_CRTP_COMMAND_SIZE-1};
    lr_full_control_forward(packet.raw,c,armed);
    memcpy(last_forwarded,packet.raw,LR_CRTP_COMMAND_SIZE);
    // Read only for diagnostics; packet.raw itself is queued byte-for-byte.
    lr_crtp_read(packet.raw,&last_roll,&last_pitch,&last_yaw,&last_thrust);
    xQueueOverwrite(controls,&packet);
}

static int8_t angle_byte(logVarId_t id)
{
    if(!LOG_VARID_IS_VALID(id))return 0;
    float x=logGetFloat(id);
    return (int8_t)lrintf(fmaxf(-127,fminf(127,x)));
}

static void reply(uint8_t sequence)
{
    uint8_t data[LR_MAX_FRAME];CRTPPacket p;
    // A request/reply exchange gets at most one downlink packet; CRTP data
    // consumes this slot when pending, otherwise compact link/MPU status does.
    size_t size;
    uint32_t now=millis();
    bool force_status=(uint32_t)(now-last_status_tx)>=400;
    bool gps_packet=false,status_packet=false;
    // The control-only build contains no GPS reply path.
#if defined(CONFIG_GPS_ENABLED) && defined(CONFIG_GPS_TELEMETRY_ENABLED)
    gps_snapshot_t gps;gps_monitor_snapshot(&gps);
    if(!force_status && gps_return_enabled && (uint32_t)(now-last_gps_tx)>=CONFIG_GPS_TELEMETRY_PERIOD_MS){
        lr_gps_t packet={.seq=sequence,
            .flags=LR_GPS_READY|(gps.data?LR_GPS_DATA:0)|(gps.fix?LR_GPS_FIX:0)|
                   (sensorsMpu6050Health()?LR_GPS_MPU_OK:0),
            .latitude_e7=(int32_t)llround(gps.nmea.latitude*1e7),
            .longitude_e7=(int32_t)llround(gps.nmea.longitude*1e7),
            .altitude_m=(int16_t)lrint(fmax(-32768,fmin(32767,gps.nmea.altitude))),
            .satellites=gps.nmea.satellites,.in_view=gps.nmea.in_view,
            .age_ms=(uint16_t)(gps.age_ms>65535?65535:gps.age_ms)};
        lr_encode_gps(data,LORA_PAIR_ID,&packet);size=LR_GPS_SIZE;gps_packet=true;
    }else
#endif
    if(!force_status && xQueueReceive(outgoing,&p,0)==pdTRUE) {
        lr_header(data,LORA_PAIR_ID,sequence,LR_CRTP);
        memcpy(data+4,p.raw,p.size+1);size=p.size+5;
        tunnel_tx++;
    } else {
        status_packet=true;
        uint8_t flags=LR_READY|(guard.armed ? LR_ARMED:0)|(!guard.have_control ? LR_FAILSAFE:0);
#ifdef CONFIG_WIFI_BENCH_NO_MOTORS
        flags|=8; // bench output-inhibit advertised to USB test tools
#endif
        lr_header(data,LORA_PAIR_ID,sequence,LR_STATUS|flags);
        data[4]=(uint8_t)angle_byte(roll_id);data[5]=(uint8_t)angle_byte(pitch_id);
        data[6]=(uint8_t)(int8_t)fmaxf(-127,fminf(0,last_rssi));
        data[7]=(uint8_t)last_snr;size=8;
    }
    // Give the controller time to finish TX-DONE handling and enter RX.
    vTaskDelay(pdMS_TO_TICKS(3));
    if(lr_radio_send(data,size)){
        tx_count++;
        if(status_packet){status_tx_count++;last_status_tx=millis();}
        if(gps_packet){
            gps_tx_count++;last_gps_tx=millis();status_led_gps_tx(last_gps_tx);
            char hex[LR_GPS_SIZE*2+1];
            for(size_t i=0;i<size;i++)snprintf(hex+i*2,3,"%02x",data[i]);
            printf("GPS_TX %lu %u %s\n",(unsigned long)last_gps_tx,sequence,hex);
        }
    }
}

static void radio_task(void *arg)
{
    (void)arg;systemWaitStart();
    while(!sensorsAreCalibrated())vTaskDelay(pdMS_TO_TICKS(10));
    roll_id=logGetVarId("stabilizer","roll");pitch_id=logGetVarId("stabilizer","pitch");
    age_id=logGetVarId("benchSet","ageMs");ready=true;
    set_roll_id=logGetVarId("benchSet","roll");set_pitch_id=logGetVarId("benchSet","pitch");
    set_yaw_id=logGetVarId("benchSet","yawRate");set_thrust_id=logGetVarId("benchSet","thrust");
    uint32_t report_ms=0;
    for(;;) {
        uint32_t now=millis();
        if(lr_guard_expire(&guard,now)) {
            timeouts++;systemSetArmed(false);
            lr_control_t zero={0};queue_control(&zero,false);
        }
        if(guard.armed && !sensorsMpu6050Health()){
            guard.armed=false;systemSetArmed(false);
            lr_control_t zero={0};queue_control(&zero,false);
        }
        uint8_t data[LR_MAX_FRAME];int16_t rssi;int8_t snr;
        int n=lr_radio_poll(data,&rssi,&snr);
        if(n<0)crc_bad++;
        if(n>0) {
            if(!lr_pair_matches(data,n,LORA_PAIR_ID)){wrong_pair++;}
            else if((data[3]&0xf0)==LR_CONTROL) {
                lr_control_t c;
                lr_result_t result=lr_decode_control(data,n,LORA_PAIR_ID,&c);
                if(result==LR_ACCEPT)result=lr_guard_accept(&guard,&c,now);
                if(result==LR_ACCEPT) {
                    if(!sensorsMpu6050Health())guard.armed=false;
                    input_roll=c.roll*.25f;input_pitch=c.pitch*.25f;input_yaw=c.yaw*2.f;
                    input_thrust=(uint16_t)(((uint32_t)c.throttle*60000u+127u)/255u);
                    input_flags=c.flags;input_bytes=LR_CONTROL_SIZE;
                    accepted++;last_seq=c.seq;last_rssi=rssi;last_snr=snr;
                    peer_seen=true;peer_ms=now;systemSetArmed(guard.armed);
                    queue_control(&c,guard.armed);
                    uint8_t axes=(c.roll?STATUS_ROLL:0)|(c.pitch?STATUS_PITCH:0)|
                                 (c.yaw?STATUS_YAW:0)|(c.throttle?STATUS_THROTTLE:0);
                    status_led_control_rx(now,axes);
                    if(c.flags&LR_REPLY)reply(c.seq);
                }else if(result==LR_STALE)stale++;else rejected++;
            }else if((data[3]&0xf0)==LR_FULL_CONTROL) {
                lr_full_control_t c;
                lr_result_t result=lr_decode_full_control(data,n,LORA_PAIR_ID,&c);
                if(result==LR_ACCEPT)result=lr_guard_accept_full(&guard,&c,now);
                if(result==LR_ACCEPT){
                    if(!sensorsMpu6050Health())guard.armed=false;
                    lr_crtp_read(c.raw,&input_roll,&input_pitch,&input_yaw,&input_thrust);
                    input_flags=c.flags;input_bytes=LR_FULL_CONTROL_SIZE;
                    accepted++;last_seq=c.seq;last_rssi=rssi;last_snr=snr;
                    peer_seen=true;peer_ms=now;systemSetArmed(guard.armed);
                    queue_full_control(&c,guard.armed);
                    uint8_t axes=(input_roll?STATUS_ROLL:0)|(input_pitch?STATUS_PITCH:0)|
                        (input_yaw?STATUS_YAW:0)|(input_thrust?STATUS_THROTTLE:0);
                    status_led_control_rx(now,axes);
                    if(c.flags&LR_REPLY)reply(c.seq);
                }else if(result==LR_STALE)stale++;else rejected++;
            }else if(data[3]==LR_CRTP && n>=5 && n<=35) {
                uint8_t port=data[4]>>4;
                // Control/arming always uses a guarded control channel, never the service tunnel.
                if(port==3 || port==7 || port==8){rejected++;}
                else {
                    tunnel_rx++;
                    CRTPPacket p={.size=n-5};memcpy(p.raw,data+4,n-4);
                    if(xQueueSend(incoming,&p,0)!=pdTRUE)rejected++;
                    peer_seen=true;peer_ms=now;last_rssi=rssi;last_snr=snr;
                }
            }else rejected++;
        }
        if(now-report_ms>=1000) {
            report_ms=now;
            printf("CRTP_CONTROL %lu %u %u ",(unsigned long)now,last_seq,input_bytes);
            for(unsigned i=0;i<LR_CRTP_COMMAND_SIZE;i++){printf("%02x",last_forwarded[i]);}
            puts("");
            printf("LINK_STATS {\"ms\":%lu,\"gps_sent\":%lu,\"status_sent\":%lu,\"gps_return\":%d,\"mpu_ok\":%d}\n",
                (unsigned long)now,(unsigned long)gps_tx_count,(unsigned long)status_tx_count,gps_return_enabled,sensorsMpu6050Health());
            printf("LORA_RX {\"ms\":%lu,\"ok\":%lu,\"wrong_pair\":%lu,\"bad\":%lu,\"crc\":%lu,\"stale\":%lu,\"timeouts\":%lu,\"tx\":%lu,\"drop\":%lu,\"tunnel_rx\":%lu,\"tunnel_tx\":%lu,\"armed\":%d,\"seq\":%u,\"age\":%lu,\"roll\":%.9g,\"pitch\":%.9g,\"yaw\":%.9g,\"thrust\":%u,\"mpu_roll\":%.9g,\"mpu_pitch\":%.9g,\"rssi\":%d,\"snr_q\":%d,\"input_roll\":%.9g,\"input_pitch\":%.9g,\"input_yaw\":%.9g,\"input_throttle\":%u,\"input_flags\":%u,\"control_bytes\":%u,\"set_roll\":%.9g,\"set_pitch\":%.9g,\"set_yaw\":%.9g,\"set_thrust\":%.9g}\n",
                (unsigned long)now,(unsigned long)accepted,(unsigned long)wrong_pair,
                (unsigned long)rejected,(unsigned long)crc_bad,(unsigned long)stale,
                (unsigned long)timeouts,(unsigned long)tx_count,(unsigned long)tx_dropped,
                (unsigned long)tunnel_rx,(unsigned long)tunnel_tx,
                guard.armed,last_seq,(unsigned long)(now-guard.last_ms),last_roll,last_pitch,last_yaw,last_thrust,
                LOG_VARID_IS_VALID(roll_id)?logGetFloat(roll_id):0,
                LOG_VARID_IS_VALID(pitch_id)?logGetFloat(pitch_id):0,last_rssi,last_snr,
                input_roll,input_pitch,input_yaw,input_thrust,input_flags,input_bytes,
                LOG_VARID_IS_VALID(set_roll_id)?logGetFloat(set_roll_id):0,
                LOG_VARID_IS_VALID(set_pitch_id)?logGetFloat(set_pitch_id):0,
                LOG_VARID_IS_VALID(set_yaw_id)?logGetFloat(set_yaw_id):0,
                LOG_VARID_IS_VALID(set_thrust_id)?logGetFloat(set_thrust_id):0);
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

static int enable(bool enabled) { (void)enabled;return 0; }
static int send_packet(CRTPPacket *p)
{
    if(!p || p->size>30)return 1;
    // Verbose console output must not starve parameter/log/link replies.
    if(p->port==CRTP_PORT_CONSOLE){tx_dropped++;return 1;}
    if(xQueueSend(outgoing,p,0)!=pdTRUE)tx_dropped++;
    return 1; // Matches crtpTxTask's boolean "accepted" convention.
}
static int receive_packet(CRTPPacket *p)
{
    // Control commands always take priority over optional CRTP traffic.
    for(unsigned i=0;i<10;i++) {
        if(xQueueReceive(controls,p,0)==pdTRUE)return 0;
        if(xQueueReceive(incoming,p,0)==pdTRUE)return 0;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return -1;
}
static bool connected(void) { return ready && peer_seen && (uint32_t)(millis()-peer_ms)<LORA_LINK_TIMEOUT_MS; }
static struct crtpLinkOperations ops={.setEnable=enable,.sendPacket=send_packet,
                                    .receivePacket=receive_packet,.isConnected=connected};

void loralinkInit(void)
{
    controls=xQueueCreate(1,sizeof(CRTPPacket));incoming=xQueueCreate(4,sizeof(CRTPPacket));
    outgoing=xQueueCreate(12,sizeof(CRTPPacket));
    if(!controls || !incoming || !outgoing){status_led_radio(false);return;}
    bool radio_ok=lr_radio_init()==ESP_OK;status_led_radio(radio_ok);
    if(!radio_ok)return;
    initialized=xTaskCreatePinnedToCore(radio_task,"loraRadio",6144,NULL,4,NULL,0)==pdPASS;
}
bool loralinkTest(void) { return initialized; }
struct crtpLinkOperations *loralinkGetLink(void) { return &ops; }

PARAM_GROUP_START(lora)
PARAM_ADD(PARAM_UINT8, gpsReturn, &gps_return_enabled)
PARAM_GROUP_STOP(lora)
