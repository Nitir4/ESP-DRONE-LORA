cmake_minimum_required(VERSION 3.16)
if(NOT SDKCONFIG_DEFAULTS)
    set(SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults;${CMAKE_CURRENT_LIST_DIR}/../boards/controller-current.defaults")
endif()
set(EXTRA_COMPONENT_DIRS "../components/lora_protocol" "../components/lora_radio"
    "../components/board_wiring" "../components/status_led")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(LoRaController)
