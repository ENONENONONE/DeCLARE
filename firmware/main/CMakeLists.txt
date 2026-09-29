set(SRCS
    "main.c"
    "csi_collector.c"
    "stream_sender.c"
    "nvs_config.c"
    "edge_processing.c"
    "crystal_touch.c"
    "field_observer.c"
    "declare_lcd.c"
    "splash_screen.c"
    "declared_screen.c"
    "ble_scanner.c"
    "optical_probe.c"
    "mesh_relay.c"
)

set(REQUIRES
    esp_wifi
    esp_netif
    esp_event
    nvs_flash
    esp_timer
    esp_driver_uart
    esp_driver_gpio
    esp_adc
    driver
    lwip
    mbedtls
    json
    spiffs
    esp_lcd
    bt
)

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "."
    REQUIRES ${REQUIRES}
)
