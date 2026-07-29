#pragma once

#include <stdbool.h>

// Configures gpio_num as the DHT11 single-wire data line. Call once before dht11_read().
void dht11_init(int gpio_num);

// Runs one full read cycle (start pulse -> ack -> 40 data bits) and, on success, fills
// *humidity_pct and *temperature_c with the sensor's integer readings.
// Returns false on timeout or checksum failure -- the DHT11 is unreliable enough that
// callers should tolerate occasional failed reads rather than treat them as fatal.
bool dht11_read(int *humidity_pct, int *temperature_c);
