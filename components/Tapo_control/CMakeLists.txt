idf_component_register(SRCS "Tapo_control.c"
                    REQUIRES esp_netif
                    PRIV_REQUIRES freertos lwip
                    INCLUDE_DIRS "include")
