idf_component_register(SRCS "DHT11_Control.c"
                    PRIV_REQUIRES freertos driver esp_rom esp_timer
                    INCLUDE_DIRS "include")
