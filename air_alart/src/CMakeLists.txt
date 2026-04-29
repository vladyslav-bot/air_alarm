idf_component_register(
    SRCS "main.cpp"
    PRIV_REQUIRES
        nvs_flash
        esp_wifi
        esp_http_client
        esp_netif
        esp_event
        driver
        led_strip
)