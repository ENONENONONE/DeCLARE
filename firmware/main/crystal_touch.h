#pragma once

#include <stdint.h>
#include <stdbool.h>

#define CRYSTAL_TOUCH_NUM_PADS  6

typedef struct {
    uint8_t  gpio;
    uint32_t raw;
    uint32_t benchmark;
    int32_t  delta;
} crystal_touch_reading_t;

typedef struct {
    crystal_touch_reading_t pads[CRYSTAL_TOUCH_NUM_PADS];
    int64_t timestamp_us;
} crystal_touch_sweep_t;

void crystal_touch_init(void);
void crystal_touch_get_sweep(crystal_touch_sweep_t *out);
void crystal_touch_start_task(void);
void crystal_touch_start_usb_reader(void);
/* Load the stored declaration (NVS csi_cfg obs_*) into the display, if one exists. */
void crystal_touch_restore_declared(void);
