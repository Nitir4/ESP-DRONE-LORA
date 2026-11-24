#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "lora_protocol.h"

// One task owns the SPI radio. No blocking SPI transactions occur in an ISR.
esp_err_t lr_radio_init(void);
void lr_radio_receive(void);
// Returns payload length, 0 for no packet, -1 for a bad CRC/length.
int lr_radio_poll(uint8_t out[LR_MAX_FRAME], int16_t *rssi, int8_t *snr_quarters);
bool lr_radio_send(const uint8_t *data, size_t size);
bool lr_radio_channel_free(void);
