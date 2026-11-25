#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MOTION_STATE_CLEAR = 0,
    MOTION_STATE_PASS = 1,
} motion_state_t;

typedef struct {
    float baseline_diff;
    float frame_diff;
    float variance_diff;
    float motion_score;
    float trigger_threshold;
    float clear_threshold;
    uint16_t selected_count;
    bool in_warmup;
    bool state_changed;
    motion_state_t state;
} motion_result_t;

void motion_core_init(void);
void motion_core_force_clear(void);
bool motion_core_update(const int8_t *csi, int raw_len, motion_result_t *result);
