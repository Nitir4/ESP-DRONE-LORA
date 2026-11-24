idf_component_register(SRCS "lora_radio.c" INCLUDE_DIRS "include"
                       REQUIRES esp_driver_spi esp_driver_gpio esp_timer lora_protocol board_wiring)
