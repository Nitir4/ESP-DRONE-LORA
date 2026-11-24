idf_component_register(SRCS "controller.c" INCLUDE_DIRS "."
    REQUIRES lora_protocol lora_radio esp_adc esp_driver_gpio esp_timer board_wiring status_led)
