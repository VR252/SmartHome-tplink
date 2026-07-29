#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "Tapo_control.h"

#define KASA_PORT 9999

static const char *TAG = "tapo_control";

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

void tapo_control_get_bulb_ip(const char *static_ip, esp_ip4_addr_t *out_ip) {
    if (static_ip && strlen(static_ip) > 0) {
        struct in_addr addr;
        if (inet_pton(AF_INET, static_ip, &addr) != 1) {
            ESP_LOGE(TAG, "static_ip \"%s\" is not a valid IP address", static_ip);
            abort();
        }
        out_ip->addr = addr.s_addr;
        ESP_LOGI(TAG, "Using static bulb IP: %s", static_ip);
        return;
    }

    while (!discover_bulb_ip(out_ip)) {
        ESP_LOGW(TAG, "Retrying bulb discovery...");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void tapo_control_toggle_power(esp_ip4_addr_t ip) {
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

static preset_t presets[TAPO_MAX_PRESETS];
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
    preset_count = parse_presets(resp, presets, TAPO_MAX_PRESETS);
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

void tapo_control_preset_next(esp_ip4_addr_t ip) {
    ensure_presets_loaded(ip);
    if (preset_count == 0) return;
    current_preset = (current_preset + 1) % preset_count;
    apply_preset(ip, current_preset);
}

void tapo_control_preset_prev(esp_ip4_addr_t ip) {
    ensure_presets_loaded(ip);
    if (preset_count == 0) return;
    current_preset = (current_preset - 1 + preset_count) % preset_count;
    apply_preset(ip, current_preset);
}
