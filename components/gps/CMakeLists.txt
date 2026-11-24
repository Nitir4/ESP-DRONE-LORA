# Requirements must be stable during ESP-IDF's early dependency scan, when
# project CONFIG values are not populated yet. Select only sources by Kconfig.
set(gps_sources "gps_disabled.c")
if(CONFIG_GPS_ENABLED)
    set(gps_sources "gps_monitor.c" "gps_nmea.c")
endif()
idf_component_register(SRCS ${gps_sources} INCLUDE_DIRS "include"
    REQUIRES esp_driver_uart esp_timer)
