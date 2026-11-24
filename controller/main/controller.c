#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include "lora_protocol.h"
#include "lora_radio.h"
#include "board_wiring.h"
#include "status_led.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static adc_oneshot_unit_handle_t adc1,adc2;
// U2 top = roll/pitch. U1 bottom = yaw/throttle.
static adc_unit_t units[4];
static adc_channel_t channels[4];
static const int axis_pins[]={CONFIG_BOARD_CTRL_ROLL,CONFIG_BOARD_CTRL_PITCH,
    CONFIG_BOARD_CTRL_YAW,CONFIG_BOARD_CTRL_THROTTLE};
static int center[4],raw[4];
static bool armed,arm_consumed,was_arm_button,bench_peer,have_reply;
static bool peer_armed,peer_failsafe,have_status;
static uint32_t last_status,arm_request_ms;
static uint32_t hold_ms,last_reply,tx_count,rx_count,busy_count;
static int input_throttle;
static bool arm_button,stop_button;
static const char *arm_state="waiting for receiver";
static uint8_t sequence,reply_sequence;
static uint8_t pending[LR_MAX_FRAME];static size_t pending_size;
static int8_t measured_roll,measured_pitch,rssi,snr_q;
static bool pause_tx;
static uint32_t pause_until,override_until;
static lr_control_t override_control;
static uint16_t tx_pair=LORA_PAIR_ID;
static int sequence_mode; // 0 normal, 1 repeat, 2 out of order; bench diagnostics only.
static lr_gps_t remote_gps;
static bool have_remote_gps;
static uint32_t remote_gps_ms,gps_rx_count;
static int64_t reply_tx_us;

static uint32_t millis(void) { return esp_timer_get_time()/1000; }
static void led_task(void *arg)
{(void)arg;for(;;){status_led_tick(millis());vTaskDelay(pdMS_TO_TICKS(25));}}

static void read_axes(void)
{
    for(int i=0;i<4;i++) {
        int value=0;
        ESP_ERROR_CHECK(adc_oneshot_read(units[i]==ADC_UNIT_1?adc1:adc2,channels[i],&value));
        raw[i]=value;
    }
}
static float axis(int i)
{
    int delta=raw[i]-center[i];
    int span=delta>=0 ? 4095-center[i]:center[i];
    float x=span>0 ? (float)delta/span:0;
    float dead=.06f;
    if(fabsf(x)<dead)return 0;
    x=copysignf((fabsf(x)-dead)/(1-dead),x);
    return fmaxf(-1,fminf(1,x));
}
static void init_inputs(void)
{
    adc_oneshot_unit_init_cfg_t one={.unit_id=ADC_UNIT_1};
    adc_oneshot_unit_init_cfg_t two={.unit_id=ADC_UNIT_2};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&one,&adc1));
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&two,&adc2));
    adc_oneshot_chan_cfg_t cfg={.atten=ADC_ATTEN_DB_12,.bitwidth=ADC_BITWIDTH_12};
    for(int i=0;i<4;i++){
        ESP_ERROR_CHECK(adc_oneshot_io_to_channel(axis_pins[i],&units[i],&channels[i]));
        ESP_ERROR_CHECK(adc_oneshot_config_channel(units[i]==ADC_UNIT_1?adc1:adc2,channels[i],&cfg));
    }
    gpio_config_t buttons={.pin_bit_mask=(1ULL<<CONFIG_BOARD_CTRL_ARM_SW)|(1ULL<<CONFIG_BOARD_CTRL_STOP_SW),.mode=GPIO_MODE_INPUT,.pull_up_en=1};
    ESP_ERROR_CHECK(gpio_config(&buttons));
    printf("CENTER: leave both sticks centred for one second\n");
    for(int n=0;n<100;n++){read_axes();for(int i=0;i<4;i++)center[i]+=raw[i];vTaskDelay(pdMS_TO_TICKS(10));}
    for(int i=0;i<4;i++)center[i]/=100;
    printf("CENTER %d %d %d %d\n",center[0],center[1],center[2],center[3]);
}

static lr_control_t joystick_control(uint32_t now)
{
    read_axes();
    int throttle=(int)lrintf(fmaxf(0,axis(3))*255);
    input_throttle=throttle;
    bool button=!gpio_get_level(CONFIG_BOARD_CTRL_ARM_SW),kill=!gpio_get_level(CONFIG_BOARD_CTRL_STOP_SW);
    arm_button=button;stop_button=kill;
    bool linked=have_reply && now-last_reply<1000;
    if(!linked || kill){armed=false;arm_consumed=true;}
    // A request is not an acknowledgement. If fresh receiver status still
    // declines arming, return to disarmed instead of showing a false success.
    if(armed && have_status && now-last_status<1000 && !peer_armed &&
       (int32_t)(last_status-arm_request_ms)>=0 && now-arm_request_ms>=800){
        armed=false;arm_consumed=true;
    }
    if(!button){hold_ms=now;arm_consumed=false;}
    else if(!was_arm_button)hold_ms=now;
    if(button && !arm_consumed && now-hold_ms>=1000 && linked && throttle==0 && !kill) {
        armed=!armed;arm_consumed=true;if(armed)arm_request_ms=now;
    }
    was_arm_button=button;
    if(kill)arm_state="STOP button pressed";
    else if(!linked)arm_state="waiting for receiver";
    else if(armed)arm_state=peer_armed?"receiver confirmed armed":"waiting for receiver arm confirmation";
    else if(throttle)arm_state="throttle must be zero";
    else if(!button)arm_state="hold U1 button to arm";
    else if(arm_consumed)arm_state="release U1 then try again";
    else arm_state="holding U1 (need 1000 ms)";
    lr_control_t c={.roll=(int8_t)lrintf(axis(0)*120),
        .pitch=(int8_t)lrintf(-axis(1)*120),.yaw=(int8_t)lrintf(axis(2)*90),
        // Repeat zero-throttle ARM requests until an accepted status reply
        // confirms arming. A lost first ARM frame cannot block arming forever.
        .throttle=armed && peer_armed && have_status && now-last_status<1000?throttle:0,
        .flags=(armed?LR_ARM:0)|(kill?LR_KILL:0)};
    return c;
}

static void process_line(char *line)
{
    if(!strcmp(line,"LIVE")){override_until=0;pause_tx=false;sequence_mode=0;tx_pair=LORA_PAIR_ID;armed=false;puts("LIVE");return;}
    if(!strncmp(line,"CRTP ",5)) {
        size_t len=strlen(line+5)/2;
        if(len<1 || len>31 || strlen(line+5)!=len*2){puts("ERR CRTP length");return;}
        lr_header(pending,LORA_PAIR_ID,sequence,LR_CRTP);
        for(size_t i=0;i<len;i++) {
            char hex[3]={line[5+i*2],line[6+i*2],0};char *end;
            long value=strtol(hex,&end,16);if(*end){puts("ERR HEX");return;}pending[4+i]=value;
        }
        pending_size=len+4;puts("CRTP_QUEUED");return;
    }
    // Diagnostic injection is accepted only after a paired receiver advertises
    // that its physical motor outputs are disabled. It never persists to NVS.
    if(!bench_peer || !have_reply || millis()-last_reply>1000){puts("ERR bench receiver required");return;}
    if(!strncmp(line,"PAUSE ",6)) {
        unsigned n=strtoul(line+6,NULL,10);pause_tx=true;pause_until=millis()+(n>5000?5000:n);puts("PAUSED");return;
    }
    if(!strncmp(line,"PAIR ",5)){tx_pair=strtoul(line+5,NULL,16);puts("PAIR_SET");return;}
    if(!strncmp(line,"SEQ ",4)){sequence_mode=atoi(line+4);puts("SEQ_SET");return;}
    if(!strncmp(line,"SET ",4)) {
        int r,p,y,t,f,ms;
        if(sscanf(line+4,"%d %d %d %d %d %d",&r,&p,&y,&t,&f,&ms)!=6 ||
           r< -127 || r>127 || p< -127 || p>127 || y< -127 || y>127 ||
           t<0 || t>255 || f<0 || f>3 || ms<1 || ms>10000){puts("ERR SET range");return;}
        override_control=(lr_control_t){.roll=r,.pitch=p,.yaw=y,.throttle=t,.flags=f};
        override_until=millis()+ms;puts("SET_OK");return;
    }
    puts("ERR command");
}

static void serial_poll(void)
{
    static char line[160];static size_t used;
    char b[64];int n=read(STDIN_FILENO,b,sizeof(b));
    for(int i=0;i<n;i++) {
        if(b[i]=='\n' || b[i]=='\r'){if(used){line[used]=0;process_line(line);used=0;}}
        else if(used<sizeof(line)-1)line[used++]=b[i];
        else used=0;
    }
}

static void poll_reply(uint32_t now)
{
    uint8_t data[LR_MAX_FRAME];int16_t rx_rssi;int8_t rx_snr;
    int n=lr_radio_poll(data,&rx_rssi,&rx_snr);
    if(n<=0 || !lr_pair_matches(data,n,LORA_PAIR_ID))return;
    // Status and tunnel replies echo the control sequence opening their slot.
    if(data[2]!=reply_sequence)return;
    int64_t observed_us=esp_timer_get_time();now=observed_us/1000;
    if((data[3]&0xf0)==LR_STATUS && n==8) {
        bench_peer=(data[3]&8)!=0;
        measured_roll=(int8_t)data[4];measured_pitch=(int8_t)data[5];
        rssi=(int8_t)data[6];snr_q=(int8_t)data[7];
        if(!(data[3]&LR_READY))return;
        peer_armed=(data[3]&LR_ARMED)!=0;peer_failsafe=(data[3]&LR_FAILSAFE)!=0;
        last_status=now;have_status=true;
    }else if((data[3]&0xf0)==LR_GPS){
        if(!lr_decode_gps(data,n,LORA_PAIR_ID,&remote_gps))return;
        remote_gps_ms=now;have_remote_gps=true;gps_rx_count++;
        printf("GPS_REMOTE {\"ms\":%lu,\"seq\":%u,\"received\":%lu,\"bytes\":%d,\"data\":%d,\"fix\":%d,\"mpu_ok\":%d,\"latitude\":%.7f,\"longitude\":%.7f,\"altitude\":%d,\"satellites\":%u,\"in_view\":%u,\"age_ms\":%u}\n",
            (unsigned long)now,remote_gps.seq,(unsigned long)gps_rx_count,n,
            (remote_gps.flags&LR_GPS_DATA)!=0,(remote_gps.flags&LR_GPS_FIX)!=0,
            (remote_gps.flags&LR_GPS_MPU_OK)!=0,remote_gps.latitude_e7/1e7,
            remote_gps.longitude_e7/1e7,remote_gps.altitude_m,remote_gps.satellites,
            remote_gps.in_view,remote_gps.age_ms);
        if(!(remote_gps.flags&LR_GPS_READY))return;
    }else if(data[3]==LR_CRTP && n>=5) {
        printf("CRTP_RX ");for(int i=4;i<n;i++)printf("%02x",data[i]);puts("");
    }else return;
    rx_count++;have_reply=true;last_reply=now;
    status_led_reply_rx(now,(data[3]&0xf0)==LR_GPS);
    printf("LINK_ACK {\"ms\":%lu,\"seq\":%u,\"kind\":%u,\"bytes\":%d,\"rtt_us\":%lu}\n",
        (unsigned long)now,data[2],data[3]&0xf0,n,(unsigned long)(observed_us-reply_tx_us));
}

static bool clear_channel(void)
{
    // Bounded fresh retries avoid throwing away a whole 50 ms control slot
    // after one transient CAD detection. Never queue old control commands.
    for(int attempt=0;attempt<3;attempt++) {
        if(lr_radio_channel_free())return true;
        busy_count++;
        vTaskDelay(pdMS_TO_TICKS(2+esp_random()%5));
        poll_reply(millis());
    }
    return false;
}

void app_main(void)
{
    setvbuf(stdout,NULL,_IONBF,0);
    fcntl(STDIN_FILENO,F_SETFL,O_NONBLOCK);
    bool active_low=false;
#ifdef CONFIG_BOARD_CTRL_LEDS_ACTIVE_LOW
    active_low=true;
#endif
    status_led_init(STATUS_CONTROLLER,CONFIG_BOARD_CTRL_LED_ACTIVITY,CONFIG_BOARD_CTRL_LED_STATUS,-1,active_low);
    xTaskCreate(led_task,"statusLeds",2048,NULL,1,NULL);
    init_inputs();
    bool radio_ok=lr_radio_init()==ESP_OK;status_led_radio(radio_ok);
    if(!radio_ok){puts("CONTROLLER_ERROR radio unavailable");for(;;)vTaskDelay(pdMS_TO_TICKS(1000));}
    printf("CONTROLLER_READY pair=%04x frame=8 period_ms=%u\n",LORA_PAIR_ID,LORA_FRAME_MS);
    uint32_t next=millis(),report=0;uint8_t sent_seq=0;
    for(;;) {
        uint32_t now=millis();serial_poll();poll_reply(now);
        bool gps_fresh=have_remote_gps && (uint32_t)(now-remote_gps_ms)<3000;
#ifdef CONFIG_PROJECT_GPS_SUPPORT
        status_led_health(gps_fresh,(remote_gps.flags&LR_GPS_MPU_OK)!=0,
            now>=5000 && have_reply,gps_fresh && (remote_gps.flags&LR_GPS_DATA),
            gps_fresh && (remote_gps.flags&LR_GPS_FIX),gps_fresh,
            armed && have_status && now-last_status<1000 && peer_armed);
#else
        // Compact status has attitude/readiness, but no explicit MPU-health bit.
        // Missing GPS is intentional in this variant, never a GPS/sensor fault.
        (void)gps_fresh;
        status_led_health(false,false,false,false,false,true,
            armed && have_status && now-last_status<1000 && peer_armed);
#endif
        if(pause_tx && (int32_t)(now-pause_until)>=0){pause_tx=false;armed=false;override_until=0;}
        if((int32_t)(now-next)>=0) {
            next=now+LORA_FRAME_MS+(esp_random()%4); // slight jitter breaks persistent collisions
            lr_control_t c=joystick_control(now);
            bool injected=(int32_t)(override_until-now)>0;
            if(injected)c=override_control;
            if(!pause_tx) {
                bool want_reply=(tx_count%4)==0;
                c.flags|=want_reply?LR_REPLY:0;
                if(sequence_mode==1)c.seq=sent_seq;
                else if(sequence_mode==2)c.seq=(uint8_t)(sent_seq-1);
                else c.seq=sequence++;
                uint8_t data[8];lr_encode_control(data,tx_pair,&c);
                if(clear_channel()) {
                    int64_t tx_start_us=esp_timer_get_time();
                    if(lr_radio_send(data,8)) {
                        tx_count++;sent_seq=c.seq;
                        if(want_reply){reply_sequence=c.seq;reply_tx_us=tx_start_us;}
                        uint8_t axes=(c.roll?STATUS_ROLL:0)|(c.pitch?STATUS_PITCH:0)|
                            (c.yaw?STATUS_YAW:0)|(c.throttle?STATUS_THROTTLE:0);
                        status_led_control_tx(millis(),axes);
                        printf("TX8 %u ",(unsigned)now);for(int i=0;i<8;i++)printf("%02x",data[i]);puts("");
                    }
                    // Optional USB CRTP bridge uses alternate disarmed frames,
                    // preserving a bounded gap in the control stream.
                    if(pending_size && !armed && !injected) {
                        vTaskDelay(pdMS_TO_TICKS(25));poll_reply(millis());
                        if(clear_channel() && lr_radio_send(pending,pending_size)) {
                            printf("CRTP_SENT ");for(size_t i=4;i<pending_size;i++)printf("%02x",pending[i]);puts("");
                            pending_size=0;
                        }
                        next=millis()+LORA_FRAME_MS;
                    }
                }
            }
        }
        if(now-report>=200) {
            report=now;
            status_output_t leds=status_led_snapshot(now);
            printf("BOARD_STATUS {\"ms\":%lu,\"led_mask\":%u,\"fault_code\":%u,\"axis_code\":%u,\"gps_received\":%lu}\n",
                (unsigned long)now,leds.mask,leds.fault,leds.axis,(unsigned long)gps_rx_count);
            printf("LORA_TX {\"ms\":%lu,\"sent\":%lu,\"replies\":%lu,\"busy\":%lu,\"armed\":%d,\"bench\":%d,\"linked\":%d,\"mpu_roll\":%d,\"mpu_pitch\":%d,\"rssi\":%d,\"snr_q\":%d,\"arm_button\":%d,\"stop_button\":%d,\"input_throttle\":%d,\"hold_ms\":%lu,\"arm_state\":\"%s\",\"raw\":[%d,%d,%d,%d],\"center\":[%d,%d,%d,%d],\"receiver_armed\":%d,\"receiver_failsafe\":%d,\"receiver_status_fresh\":%d}\n",
                (unsigned long)now,(unsigned long)tx_count,(unsigned long)rx_count,(unsigned long)busy_count,
                armed,bench_peer,have_reply && now-last_reply<1000,measured_roll,measured_pitch,rssi,snr_q,
                arm_button,stop_button,input_throttle,(unsigned long)(arm_button?now-hold_ms:0),arm_state,
                raw[0],raw[1],raw[2],raw[3],center[0],center[1],center[2],center[3],
                peer_armed,peer_failsafe,have_status && now-last_status<1000);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
