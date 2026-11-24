#include "status_led.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
static status_input_t state;
static portMUX_TYPE lock=portMUX_INITIALIZER_UNLOCKED;
static int pins[3]={-1,-1,-1};static bool invert,initialized;
uint8_t status_led_mask;
uint32_t status_led_edges[3];

void status_led_init(int role,int activity,int status,int fault,bool active_low)
{
    state.role=role;state.gps_stream_ok=true;invert=active_low;
    pins[0]=activity;pins[1]=status;pins[2]=fault;
#ifdef CONFIG_PROJECT_STATUS_LEDS
    for(int i=0;i<3;i++)if(pins[i]>=0){
        gpio_config_t cfg={.pin_bit_mask=1ULL<<pins[i],.mode=GPIO_MODE_OUTPUT};
        ESP_ERROR_CHECK(gpio_config(&cfg));gpio_set_level(pins[i],invert);
    }
    initialized=true;
#endif
}
void status_led_radio(bool okay){portENTER_CRITICAL(&lock);state.radio_known=true;state.radio_ok=okay;portEXIT_CRITICAL(&lock);}
void status_led_health(bool mk,bool mo,bool gk,bool gd,bool gf,bool gs,bool armed)
{
    portENTER_CRITICAL(&lock);
    state.mpu_known=mk;state.mpu_ok=mo;state.gps_known=gk;state.gps_data=gd;
    state.gps_fix=gf;state.gps_stream_ok=gs;state.armed=armed;
    portEXIT_CRITICAL(&lock);
}
void status_led_control_rx(uint32_t now,uint8_t axes)
{portENTER_CRITICAL(&lock);state.have_rx=true;state.rx_ms=now;state.axes=axes;portEXIT_CRITICAL(&lock);}
void status_led_control_tx(uint32_t now,uint8_t axes)
{portENTER_CRITICAL(&lock);state.have_tx=true;state.tx_ms=now;state.axes=axes;portEXIT_CRITICAL(&lock);}
void status_led_reply_rx(uint32_t now,bool gps)
{portENTER_CRITICAL(&lock);state.have_rx=true;state.rx_ms=now;if(gps){state.have_gps=true;state.gps_ms=now;}portEXIT_CRITICAL(&lock);}
void status_led_gps_tx(uint32_t now)
{portENTER_CRITICAL(&lock);state.have_gps=true;state.gps_ms=now;portEXIT_CRITICAL(&lock);}
status_output_t status_led_snapshot(uint32_t now)
{status_input_t copy;portENTER_CRITICAL(&lock);copy=state;portEXIT_CRITICAL(&lock);return status_pattern(&copy,now);}
void status_led_tick(uint32_t now)
{
    if(!initialized)return;
    status_output_t o=status_led_snapshot(now);
    for(int i=0;i<3;i++)if(pins[i]>=0 && ((status_led_mask^o.mask)&(1u<<i))){
        bool on=(o.mask&(1u<<i))!=0;
        gpio_set_level(pins[i],on^invert);if(on)status_led_edges[i]++;
    }
    status_led_mask=o.mask;
}
