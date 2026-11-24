#include "lora_radio.h"
#include "board_wiring.h"
#include <string.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Both wired modules use NSS5/SCK18/MISO19/MOSI23/RST14/DIO0=2.
static spi_device_handle_t radio;
static const uint8_t MODE_LORA_LF=0x88;
static esp_err_t transfer(uint8_t reg, void *data, size_t size, bool write)
{
    spi_transaction_t t={.cmd=write ? reg|0x80 : reg&0x7f,.length=size*8};
    if(write)t.tx_buffer=data;else t.rx_buffer=data;
    return spi_device_polling_transmit(radio,&t);
}
static uint8_t rd(uint8_t reg) { uint8_t value=0; ESP_ERROR_CHECK(transfer(reg,&value,1,false));return value; }
static void wr(uint8_t reg,uint8_t value) { ESP_ERROR_CHECK(transfer(reg,&value,1,true)); }
static void mode(uint8_t value) { wr(0x01,MODE_LORA_LF|value); }

esp_err_t lr_radio_init(void)
{
    gpio_config_t outputs={.pin_bit_mask=1ULL<<BOARD_LORA_RST,.mode=GPIO_MODE_OUTPUT};
    ESP_RETURN_ON_ERROR(gpio_config(&outputs),"LORA","reset GPIO");
    gpio_set_level(BOARD_LORA_RST,0);vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(BOARD_LORA_RST,1);vTaskDelay(pdMS_TO_TICKS(10));
    gpio_config_t input={.pin_bit_mask=1ULL<<BOARD_LORA_DIO0,.mode=GPIO_MODE_INPUT};
    ESP_RETURN_ON_ERROR(gpio_config(&input),"LORA","DIO0 GPIO");
    spi_bus_config_t bus={.mosi_io_num=BOARD_LORA_MOSI,.miso_io_num=BOARD_LORA_MISO,.sclk_io_num=BOARD_LORA_SCK,
        .quadwp_io_num=-1,.quadhd_io_num=-1,.max_transfer_sz=LR_MAX_FRAME};
    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI3_HOST,&bus,SPI_DMA_DISABLED),"LORA","SPI bus");
    spi_device_interface_config_t dev={.command_bits=8,.clock_speed_hz=4000000,
        .mode=0,.spics_io_num=BOARD_LORA_NSS,.queue_size=1};
    ESP_RETURN_ON_ERROR(spi_bus_add_device(SPI3_HOST,&dev,&radio),"LORA","SPI device");
    uint8_t version=rd(0x42);
    ESP_LOGI("LORA","SX127x version=0x%02x",version);
    if(version!=0x12)return ESP_ERR_NOT_FOUND;
    mode(0);vTaskDelay(pdMS_TO_TICKS(2));
    uint32_t frf=((uint64_t)LORA_FREQUENCY_HZ<<19)/32000000u;
    wr(0x06,frf>>16);wr(0x07,frf>>8);wr(0x08,frf);
    wr(0x09,0x80); // PA_BOOST, 2 dBm for the attached bench radios.
    wr(0x0b,0x2b);wr(0x0c,0x23);
    wr(0x0e,0);wr(0x0f,0);
    wr(0x1d,0x92); // 500 kHz BW, coding 4/5, explicit header.
    wr(0x1e,0x74); // SF7, hardware payload CRC enabled.
    wr(0x26,0x04); // Automatic gain; low-data-rate optimization off.
    wr(0x20,0);wr(0x21,8);
    wr(0x31,0xc3);wr(0x37,0x0a);
    wr(0x33,0x27);wr(0x3b,0x1d); // Normal IQ.
    wr(0x39,LORA_SYNC_WORD);wr(0x40,0);
    wr(0x23,LR_MAX_FRAME);wr(0x12,0xff);
    lr_radio_receive();
    ESP_LOGI("LORA","%lu Hz SF7 BW500 CR4/5 sync=0x%02x pair=%04x",
        (unsigned long)LORA_FREQUENCY_HZ,LORA_SYNC_WORD,LORA_PAIR_ID);
    return ESP_OK;
}

void lr_radio_receive(void) { wr(0x40,0);mode(5); }

int lr_radio_poll(uint8_t out[LR_MAX_FRAME],int16_t *rssi,int8_t *snr_quarters)
{
    uint8_t flags=rd(0x12);
    if(!(flags&0x40))return 0;
    wr(0x12,flags);
    uint8_t size=rd(0x13);
    // Require the peer's payload CRC, in addition to checking CRC errors.
    if((flags&0x20) || !(rd(0x1c)&0x40) || size<4 || size>LR_MAX_FRAME)return -1;
    *snr_quarters=(int8_t)rd(0x19);
    *rssi=-164+(int16_t)rd(0x1a);
    if(*snr_quarters<0)*rssi+=*snr_quarters/4;
    wr(0x0d,rd(0x10));
    ESP_ERROR_CHECK(transfer(0,out,size,false));
    return size;
}

bool lr_radio_send(const uint8_t *data,size_t size)
{
    if(!size || size>LR_MAX_FRAME)return false;
    mode(1);wr(0x12,0xff);wr(0x0d,0);
    ESP_ERROR_CHECK(transfer(0,(void*)data,size,true));wr(0x22,size);wr(0x40,0x40);
    mode(3);
    int64_t until=esp_timer_get_time()+100000;
    while(!(rd(0x12)&8)) {
        if(esp_timer_get_time()>until){mode(1);lr_radio_receive();return false;}
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    wr(0x12,8);lr_radio_receive();return true;
}

bool lr_radio_channel_free(void)
{
    mode(1);wr(0x12,0xff);mode(7);
    int64_t until=esp_timer_get_time()+8000;
    while(!(rd(0x12)&4)) {
        if(esp_timer_get_time()>until){lr_radio_receive();return false;}
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    bool free=(rd(0x12)&1)==0;
    wr(0x12,0x05);lr_radio_receive();return free;
}
