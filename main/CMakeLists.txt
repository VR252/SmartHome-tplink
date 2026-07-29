idf_component_register(SRCS "main.c"
                    PRIV_REQUIRES esp_wifi esp_netif esp_event nvs_flash Tapo_control IR_sensor DHT11_Control
                    INCLUDE_DIRS "")
