#pragma once

typedef enum {
    IR_EVENT_NONE = 0,
    IR_EVENT_POWER,
    IR_EVENT_PRESET_NEXT,
    IR_EVENT_PRESET_PREV,
} ir_event_t;

// Sets up the RMT RX channel on gpio_num and starts listening for NEC IR frames.
void ir_sensor_init(int gpio_num);

// Blocks until an NEC frame is received. Frames from remote addresses other than
// this remote, or with unmapped command bytes, are decoded but reported as IR_EVENT_NONE.
ir_event_t ir_sensor_wait_event(void);
