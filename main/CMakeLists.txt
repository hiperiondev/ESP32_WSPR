file(
    GLOB_RECURSE
        SOURCES
            *.c
)

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS
        .
    REQUIRES
        nvs_flash
        esp_wifi
        esp_netif
        esp_event
        driver
        esp_http_server
        json
        esp_timer
        lwip
)
