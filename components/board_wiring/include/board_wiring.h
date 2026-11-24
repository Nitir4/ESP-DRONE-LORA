#pragma once
#include "sdkconfig.h"
// New peripherals use these project settings; stock MPU/motor/receiver LED
// assignments remain the existing ESP-Drone Kconfig values, not hidden literals.
#define BOARD_LORA_NSS CONFIG_BOARD_LORA_NSS
#define BOARD_LORA_SCK CONFIG_BOARD_LORA_SCK
#define BOARD_LORA_MISO CONFIG_BOARD_LORA_MISO
#define BOARD_LORA_MOSI CONFIG_BOARD_LORA_MOSI
#define BOARD_LORA_RST CONFIG_BOARD_LORA_RST
#define BOARD_LORA_DIO0 CONFIG_BOARD_LORA_DIO0
