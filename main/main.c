#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/rmt_rx.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"

// ---- Fill these in before flashing ----
#define WIFI_SSID       ""
#define WIFI_PASS       ""

#define IR_RX_GPIO      14
#define RMT_RESOLUTION  1000000   // 1 tick = 1 us
#define MAX_SYMBOLS     128

#define KASA_PORT       9999

// If set to a non-empty IP string, this is used directly and discovery is skipped.
// Leave as "" to auto-discover the bulb via UDP broadcast.
#define BULB_STATIC_IP  ""

// This remote uses a single fixed address for every button; only the command byte changes.
#define REMOTE_ADDR         0x00
#define POWER_CMD           0x45
#define PRESET_NEXT_CMD     0x09
#define PRESET_PREV_CMD     0x07

#define MAX_PRESETS         8

static const char *TAG = "smart_home";
static QueueHandle_t rx_queue;
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

// ---------------- Kasa protocol (legacy XOR) ----------------

static void kasa_xor_encrypt(uint8_t *data, size_t len) {
    uint8_t key = 171;
    for (size_t i = 0; i < len; i++) {
        data[i] ^= key;
        key = data[i];
    }
}

static void kasa_xor_decrypt(uint8_t *data, size_t len) {
    uint8_t key = 171;
    for (size_t i = 0; i < len; i++) {
        uint8_t next_key = data[i];
        data[i] ^= key;
        key = next_key;
    }
}

// Broadcasts a get_sysinfo query and returns the IP of the first bulb that answers.
static bool discover_bulb_ip(esp_ip4_addr_t *out_ip) {
    const char *query = "{\"system\":{\"get_sysinfo\":{}}}";
    uint8_t buf[512];
    size_t qlen = strlen(query);
    memcpy(buf, query, qlen);
    kasa_xor_encrypt(buf, qlen);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create UDP socket");
        return false;
    }
    int broadcast = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
    struct timeval tv = { .tv_sec = 3, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port = htons(KASA_PORT),
        .sin_addr.s_addr = htonl(INADDR_BROADCAST),
    };
    sendto(sock, buf, qlen, 0, (struct sockaddr *)&dest, sizeof(dest));

    struct sockaddr_in src;
    socklen_t src_len = sizeof(src);
    uint8_t rbuf[512];
    int n = recvfrom(sock, rbuf, sizeof(rbuf) - 1, 0, (struct sockaddr *)&src, &src_len);
    close(sock);

    if (n <= 0) {
        ESP_LOGW(TAG, "No bulb responded to discovery");
        return false;
    }
    out_ip->addr = src.sin_addr.s_addr;
    ESP_LOGI(TAG, "Found bulb at %s", inet_ntoa(src.sin_addr));
    return true;
}

// recv() until exactly `total` bytes are read, a socket error occurs, or the peer closes early.
static bool recv_all(int sock, void *buf, size_t total) {
    size_t got = 0;
    while (got < total) {
        int n = recv(sock, (uint8_t *)buf + got, total - got, 0);
        if (n <= 0) {
            ESP_LOGE(TAG, "recv_all: got %u/%u bytes, n=%d, errno=%d (%s)",
                     (unsigned)got, (unsigned)total, n, errno, strerror(errno));
            return false;
        }
        got += n;
    }
    return true;
}

// Sends a Kasa command over TCP (4-byte big-endian length header + XOR payload)
// and returns the decrypted JSON response in resp (null-terminated).
static bool kasa_send_command(esp_ip4_addr_t ip, const char *json_cmd, char *resp, size_t resp_size) {
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() failed: errno=%d (%s)", errno, strerror(errno));
        return false;
    }

    struct timeval tv = { .tv_sec = 3, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port = htons(KASA_PORT),
        .sin_addr.s_addr = ip.addr,
    };
    ESP_LOGI(TAG, "Connecting to %s:%d", inet_ntoa(dest.sin_addr), KASA_PORT);
    if (connect(sock, (struct sockaddr *)&dest, sizeof(dest)) != 0) {
        ESP_LOGE(TAG, "connect() failed: errno=%d (%s)", errno, strerror(errno));
        close(sock);
        return false;
    }

    size_t len = strlen(json_cmd);
    static uint8_t payload[512];
    memcpy(payload, json_cmd, len);
    kasa_xor_encrypt(payload, len);

    uint32_t len_hdr = htonl((uint32_t)len);
    if (send(sock, &len_hdr, sizeof(len_hdr), 0) < 0 || send(sock, payload, len, 0) < 0) {
        ESP_LOGE(TAG, "send() failed: errno=%d (%s)", errno, strerror(errno));
        close(sock);
        return false;
    }

    uint32_t resp_len_hdr;
    if (!recv_all(sock, &resp_len_hdr, sizeof(resp_len_hdr))) {
        close(sock);
        return false;
    }
    uint32_t resp_len = ntohl(resp_len_hdr);
    if (resp_len == 0 || resp_len >= resp_size) {
        ESP_LOGE(TAG, "Bad response length: %u (buffer size %u)", (unsigned)resp_len, (unsigned)resp_size);
        close(sock);
        return false;
    }
    if (!recv_all(sock, resp, resp_len)) {
        close(sock);
        return false;
    }
    close(sock);

    kasa_xor_decrypt((uint8_t *)resp, resp_len);
    resp[resp_len] = '\0';
    return true;
}

// Queries current bulb state, then sends the opposite state.
static void toggle_bulb_power(esp_ip4_addr_t ip) {
    static char resp[2048];
    if (!kasa_send_command(ip, "{\"system\":{\"get_sysinfo\":{}}}", resp, sizeof(resp))) {
        ESP_LOGE(TAG, "Failed to query bulb state");
        return;
    }

    char *on_off_ptr = strstr(resp, "\"on_off\"");
    if (!on_off_ptr) {
        ESP_LOGE(TAG, "Couldn't find on_off in response: %s", resp);
        return;
    }
    int current_state = atoi(strchr(on_off_ptr, ':') + 1);
    int new_state = current_state ? 0 : 1;

    char cmd[192];
    snprintf(cmd, sizeof(cmd),
             "{\"smartlife.iot.smartbulb.lightingservice\":{\"transition_light_state\":"
             "{\"on_off\":%d,\"ignore_default\":1}}}", new_state);

    if (kasa_send_command(ip, cmd, resp, sizeof(resp))) {
        ESP_LOGI(TAG, "Bulb toggled: %d -> %d", current_state, new_state);
    } else {
        ESP_LOGE(TAG, "Failed to send toggle command");
    }
}

// ---------------- Preset cycling ----------------
// Kasa app "presets" aren't IDs the bulb resolves itself -- the app just stores each
// preset's light state (hue/saturation/color_temp/brightness) and re-applies it. The
// bulb has no concept of "currently active preset," so we track that index locally.

typedef struct {
    int index;
    int hue;
    int saturation;
    int color_temp;
    int brightness;
} preset_t;

static preset_t presets[MAX_PRESETS];
static int preset_count = 0;
static int current_preset = 0;
static bool presets_loaded = false;

static int parse_int_field(const char *obj, const char *key) {
    char pattern[32];
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    const char *p = strstr(obj, pattern);
    if (!p) return 0;
    return atoi(p + strlen(pattern));
}

// Extracts the "preferred_state" array (the bulb's presets) out of a get_sysinfo response.
static int parse_presets(const char *json, preset_t *out, int max_presets) {
    const char *arr = strstr(json, "\"preferred_state\"");
    if (!arr) return 0;
    arr = strchr(arr, '[');
    if (!arr) return 0;
    const char *arr_close = strchr(arr, ']');
    if (!arr_close) return 0;

    int count = 0;
    const char *p = arr;
    while (count < max_presets) {
        const char *obj_start = strchr(p, '{');
        if (!obj_start || obj_start > arr_close) break;
        const char *obj_end = strchr(obj_start, '}');
        if (!obj_end || obj_end > arr_close) break;

        char obj[128];
        size_t obj_len = obj_end - obj_start + 1;
        if (obj_len >= sizeof(obj)) obj_len = sizeof(obj) - 1;
        memcpy(obj, obj_start, obj_len);
        obj[obj_len] = '\0';

        out[count].index = parse_int_field(obj, "index");
        out[count].hue = parse_int_field(obj, "hue");
        out[count].saturation = parse_int_field(obj, "saturation");
        out[count].color_temp = parse_int_field(obj, "color_temp");
        out[count].brightness = parse_int_field(obj, "brightness");
        count++;

        p = obj_end + 1;
    }
    return count;
}

static void ensure_presets_loaded(esp_ip4_addr_t ip) {
    if (presets_loaded) return;
    static char resp[2048];
    if (!kasa_send_command(ip, "{\"system\":{\"get_sysinfo\":{}}}", resp, sizeof(resp))) {
        ESP_LOGE(TAG, "Failed to query bulb for presets");
        return;
    }
    preset_count = parse_presets(resp, presets, MAX_PRESETS);
    if (preset_count == 0) {
        ESP_LOGW(TAG, "No presets found on bulb");
    } else {
        ESP_LOGI(TAG, "Loaded %d presets from bulb", preset_count);
        presets_loaded = true;
    }
}

static void apply_preset(esp_ip4_addr_t ip, int idx) {
    preset_t *p = &presets[idx];
    char cmd[256];
    snprintf(cmd, sizeof(cmd),
             "{\"smartlife.iot.smartbulb.lightingservice\":{\"transition_light_state\":"
             "{\"on_off\":1,\"hue\":%d,\"saturation\":%d,\"color_temp\":%d,\"brightness\":%d,"
             "\"ignore_default\":1}}}",
             p->hue, p->saturation, p->color_temp, p->brightness);

    static char resp[512];
    if (kasa_send_command(ip, cmd, resp, sizeof(resp))) {
        ESP_LOGI(TAG, "Applied preset %d (hue=%d sat=%d ct=%d bri=%d)",
                 p->index, p->hue, p->saturation, p->color_temp, p->brightness);
    } else {
        ESP_LOGE(TAG, "Failed to apply preset %d", idx);
    }
}

static void preset_next(esp_ip4_addr_t ip) {
    ensure_presets_loaded(ip);
    if (preset_count == 0) return;
    current_preset = (current_preset + 1) % preset_count;
    apply_preset(ip, current_preset);
}

static void preset_prev(esp_ip4_addr_t ip) {
    ensure_presets_loaded(ip);
    if (preset_count == 0) return;
    current_preset = (current_preset - 1 + preset_count) % preset_count;
    apply_preset(ip, current_preset);
}

// ---------------- IR receive + NEC decode ----------------

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

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_sta();

    esp_ip4_addr_t bulb_ip;
    if (strlen(BULB_STATIC_IP) > 0) {
        struct in_addr addr;
        if (inet_pton(AF_INET, BULB_STATIC_IP, &addr) != 1) {
            ESP_LOGE(TAG, "BULB_STATIC_IP \"%s\" is not a valid IP address", BULB_STATIC_IP);
            abort();
        }
        bulb_ip.addr = addr.s_addr;
        ESP_LOGI(TAG, "Using static bulb IP: %s", BULB_STATIC_IP);
    } else {
        while (!discover_bulb_ip(&bulb_ip)) {
            ESP_LOGW(TAG, "Retrying bulb discovery...");
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }

    rmt_channel_handle_t rx_channel = NULL;
    rmt_rx_channel_config_t rx_cfg = {
        .gpio_num = IR_RX_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RESOLUTION,
        .mem_block_symbols = MAX_SYMBOLS,
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_cfg, &rx_channel));

    rx_queue = xQueueCreate(4, sizeof(rmt_rx_done_event_data_t));
    rmt_rx_event_callbacks_t cbs = { .on_recv_done = rx_done_cb };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, NULL));
    ESP_ERROR_CHECK(rmt_enable(rx_channel));

    static rmt_symbol_word_t raw_symbols[MAX_SYMBOLS];
    rmt_receive_config_t receive_cfg = {
        .signal_range_min_ns = 1000,
        .signal_range_max_ns = 20000000,
    };

    ESP_LOGI(TAG, "Ready — waiting for IR signals");
    while (1) {
        ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_cfg));

        rmt_rx_done_event_data_t rx_data;
        if (xQueueReceive(rx_queue, &rx_data, portMAX_DELAY) == pdTRUE) {
            uint8_t address, command;
            if (nec_decode(rx_data.received_symbols, rx_data.num_symbols, &address, &command)) {
                ESP_LOGI(TAG, "NEC addr=0x%02X cmd=0x%02X", address, command);
                if (address != REMOTE_ADDR) {
                    // Not our remote -- ignore.
                } else if (command == POWER_CMD) {
                    ESP_LOGI(TAG, "Power button -> toggling bulb");
                    toggle_bulb_power(bulb_ip);
                } else if (command == PRESET_NEXT_CMD) {
                    ESP_LOGI(TAG, "Forward -> next preset");
                    preset_next(bulb_ip);
                } else if (command == PRESET_PREV_CMD) {
                    ESP_LOGI(TAG, "Back -> previous preset");
                    preset_prev(bulb_ip);
                }
            }
        }
    }
}
