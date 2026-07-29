#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/rmt_rx.h"
#include "esp_log.h"
#include "IR_sensor.h"

#define RMT_RESOLUTION  1000000   // 1 tick = 1 us
#define MAX_SYMBOLS     128

// This remote uses a single fixed address for every button; only the command byte changes.
#define REMOTE_ADDR         0x00
#define POWER_CMD           0x45
#define PRESET_NEXT_CMD     0x09
#define PRESET_PREV_CMD     0x07

static const char *TAG = "ir_sensor";
static QueueHandle_t rx_queue;
static rmt_channel_handle_t rx_channel;
static rmt_symbol_word_t raw_symbols[MAX_SYMBOLS];
static rmt_receive_config_t receive_cfg = {
    .signal_range_min_ns = 1000,
    .signal_range_max_ns = 20000000,
};

static bool IRAM_ATTR rx_done_cb(rmt_channel_handle_t ch,
                                  const rmt_rx_done_event_data_t *edata,
                                  void *user_ctx) {
    BaseType_t hp_task_wake = pdFALSE;
    xQueueSendFromISR(rx_queue, edata, &hp_task_wake);
    return hp_task_wake == pdTRUE;
}

// Decodes a captured NEC frame into address/command.
// Returns false if the frame doesn't look like valid NEC (bad header or failed checksum).
static bool nec_decode(const rmt_symbol_word_t *symbols, size_t num_symbols,
                        uint8_t *address, uint8_t *command) {
    if (num_symbols < 33) return false;

    // Header: ~9ms mark, ~4.5ms space
    if (symbols[0].duration0 < 8000 || symbols[0].duration0 > 10000) return false;
    if (symbols[0].duration1 < 3800 || symbols[0].duration1 > 5200) return false;

    uint8_t bytes[4] = {0};
    for (int i = 0; i < 32; i++) {
        int bit = symbols[1 + i].duration1 > 1000 ? 1 : 0;
        bytes[i / 8] |= (bit << (i % 8));  // LSB first per byte
    }

    uint8_t addr = bytes[0], addr_inv = bytes[1];
    uint8_t cmd = bytes[2], cmd_inv = bytes[3];
    if ((uint8_t)(~addr) != addr_inv || (uint8_t)(~cmd) != cmd_inv) {
        ESP_LOGW(TAG, "NEC checksum mismatch, discarding frame");
        return false;
    }

    *address = addr;
    *command = cmd;
    return true;
}

void ir_sensor_init(int gpio_num) {
    rmt_rx_channel_config_t rx_cfg = {
        .gpio_num = gpio_num,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RESOLUTION,
        .mem_block_symbols = MAX_SYMBOLS,
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_cfg, &rx_channel));

    rx_queue = xQueueCreate(4, sizeof(rmt_rx_done_event_data_t));
    rmt_rx_event_callbacks_t cbs = { .on_recv_done = rx_done_cb };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, NULL));
    ESP_ERROR_CHECK(rmt_enable(rx_channel));

    ESP_LOGI(TAG, "Ready — waiting for IR signals");
}

ir_event_t ir_sensor_wait_event(void) {
    ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_cfg));

    rmt_rx_done_event_data_t rx_data;
    if (xQueueReceive(rx_queue, &rx_data, portMAX_DELAY) != pdTRUE) {
        return IR_EVENT_NONE;
    }

    uint8_t address, command;
    if (!nec_decode(rx_data.received_symbols, rx_data.num_symbols, &address, &command)) {
        return IR_EVENT_NONE;
    }
    ESP_LOGI(TAG, "NEC addr=0x%02X cmd=0x%02X", address, command);

    if (address != REMOTE_ADDR) {
        return IR_EVENT_NONE;
    }
    switch (command) {
        case POWER_CMD:       return IR_EVENT_POWER;
        case PRESET_NEXT_CMD: return IR_EVENT_PRESET_NEXT;
        case PRESET_PREV_CMD: return IR_EVENT_PRESET_PREV;
        default:               return IR_EVENT_NONE;
    }
}
