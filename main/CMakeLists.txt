idf_component_register(SRCS "main.c"
                    PRIV_REQUIRES spi_flash driver esp_driver_rmt esp_wifi esp_netif esp_event nvs_flash lwip
                    INCLUDE_DIRS "")
