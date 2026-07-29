#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "Tapo_control.h"
#include "IR_sensor.h"
#include "DHT11_Control.h"

// ---- Fill these in before flashing ----
#define WIFI_SSID       "BigDawgHome"
#define WIFI_PASS       "reireirei"

#define IR_RX_GPIO      14
#define DHT11_GPIO      32
#define DHT11_POLL_MS   60000

// If set to a non-empty IP string, this is used directly and discovery is skipped.
// Leave as "" to auto-discover the bulb via UDP broadcast.
#define BULB_STATIC_IP  "192.168.2.224"

static const char *TAG = "smart_home";
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

// ---------------- WiFi ----------------

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected, retrying...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void) {
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to WiFi...");
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "WiFi connected");
}

// ---------------- DHT11 polling ----------------

static void dht11_task(void *arg) {
    dht11_init(DHT11_GPIO);
    while (1) {
        int humidity_pct, temperature_c;
        if (dht11_read(&humidity_pct, &temperature_c)) {
            ESP_LOGI(TAG, "DHT11: %dC, %d%% humidity", temperature_c, humidity_pct);
        } else {
            ESP_LOGW(TAG, "DHT11: read failed");
        }
        vTaskDelay(pdMS_TO_TICKS(DHT11_POLL_MS));
    }
}

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_sta();

    esp_ip4_addr_t bulb_ip;
    tapo_control_get_bulb_ip(BULB_STATIC_IP, &bulb_ip);

    ir_sensor_init(IR_RX_GPIO);
    xTaskCreate(dht11_task, "dht11_task", 4096, NULL, 5, NULL);

    while (1) {
        switch (ir_sensor_wait_event()) {
            case IR_EVENT_POWER:
                ESP_LOGI(TAG, "Power button -> toggling bulb");
                tapo_control_toggle_power(bulb_ip);
                break;
            case IR_EVENT_PRESET_NEXT:
                ESP_LOGI(TAG, "Forward -> next preset");
                tapo_control_preset_next(bulb_ip);
                break;
            case IR_EVENT_PRESET_PREV:
                ESP_LOGI(TAG, "Back -> previous preset");
                tapo_control_preset_prev(bulb_ip);
                break;
            default:
                break;
        }
    }
}
