idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp32-camera quirc
             esp_wifi esp_http_server nvs_flash esp_netif esp_event
)
