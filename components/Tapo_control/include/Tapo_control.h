#pragma once

#include <stdbool.h>
#include "esp_netif_ip_addr.h"

#define TAPO_MAX_PRESETS 8

// If static_ip is non-NULL and non-empty, it's used directly (parsed as a dotted-quad).
// Otherwise the bulb is discovered via a UDP broadcast; this call blocks/retries until found.
void tapo_control_get_bulb_ip(const char *static_ip, esp_ip4_addr_t *out_ip);

// Queries current bulb state, then sends the opposite state.
void tapo_control_toggle_power(esp_ip4_addr_t ip);

// Cycles to the next/previous Kasa app preset (hue/saturation/color_temp/brightness),
// loading the preset list from the bulb on first use.
void tapo_control_preset_next(esp_ip4_addr_t ip);
void tapo_control_preset_prev(esp_ip4_addr_t ip);
