#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "DHT11_Control.h"

#define START_LOW_MS          20   // host start pulse: >=18ms low
#define START_HIGH_US         30   // then ~20-40us high before releasing to the sensor
#define EDGE_TIMEOUT_US       100  // datasheet: no pulse should ever get near this
#define BIT_THRESHOLD_US      40   // high-pulse width: <40us -> bit 0, >=40us -> bit 1

static const char *TAG = "dht11_control";
static int dht_gpio;
static portMUX_TYPE dht_mux = portMUX_INITIALIZER_UNLOCKED;

void dht11_init(int gpio_num) {
    dht_gpio = gpio_num;
    gpio_set_direction(gpio_num, GPIO_MODE_INPUT);
    gpio_set_pull_mode(gpio_num, GPIO_PULLUP_ONLY);
    ESP_LOGI(TAG, "Ready on GPIO%d", gpio_num);
}

// Busy-waits (bounded by timeout_us, measured via a free-running hardware timer that keeps
// ticking with interrupts disabled) for the line to reach `level`.
static bool wait_for_level(int level, int timeout_us) {
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(dht_gpio) != level) {
        if (esp_timer_get_time() - start > timeout_us) {
            return false;
        }
    }
    return true;
}

bool dht11_read(int *humidity_pct, int *temperature_c) {
    // Host start condition.
    gpio_set_direction(dht_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(dht_gpio, 0);
    vTaskDelay(pdMS_TO_TICKS(START_LOW_MS));
    gpio_set_level(dht_gpio, 1);
    esp_rom_delay_us(START_HIGH_US);
    gpio_set_direction(dht_gpio, GPIO_MODE_INPUT);

    // The bit stream is timed in tens of microseconds, so interrupts (and thus FreeRTOS's
    // scheduler tick) must stay off for the duration of the read or a preempting ISR will
    // stretch a pulse measurement and corrupt the decoded bits.
    uint8_t bytes[5] = {0};
    bool ok = true;

    taskENTER_CRITICAL(&dht_mux);
    do {
        // Sensor ack: pulls low ~80us, then high ~80us, then starts the first bit's low phase.
        if (!wait_for_level(0, EDGE_TIMEOUT_US)) { ok = false; break; }
        if (!wait_for_level(1, EDGE_TIMEOUT_US)) { ok = false; break; }
        if (!wait_for_level(0, EDGE_TIMEOUT_US)) { ok = false; break; }

        for (int i = 0; i < 40; i++) {
            if (!wait_for_level(1, EDGE_TIMEOUT_US)) { ok = false; break; }  // 50us low start-of-bit
            int64_t high_start = esp_timer_get_time();
            if (!wait_for_level(0, EDGE_TIMEOUT_US)) { ok = false; break; }
            int width_us = (int)(esp_timer_get_time() - high_start);
            int bit = width_us >= BIT_THRESHOLD_US ? 1 : 0;
            bytes[i / 8] = (bytes[i / 8] << 1) | bit;  // MSB first per byte
        }
    } while (0);
    taskEXIT_CRITICAL(&dht_mux);

    if (!ok) {
        ESP_LOGW(TAG, "Timed out waiting for DHT11 signal");
        return false;
    }

    uint8_t checksum = (uint8_t)(bytes[0] + bytes[1] + bytes[2] + bytes[3]);
    if (checksum != bytes[4]) {
        ESP_LOGW(TAG, "Checksum mismatch: got 0x%02X, expected 0x%02X", bytes[4], checksum);
        return false;
    }

    *humidity_pct = bytes[0];
    *temperature_c = bytes[2];
    return true;
}
