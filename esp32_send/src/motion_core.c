#include "motion_core.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define MAX_SUBCARRIERS          306
#define QUALITY_WINDOW           80
#define QUALITY_MIN_FRAMES       26
#define QUALITY_REFRESH_FRAMES   20
#define VARIANCE_WINDOW          6
#define SCORE_HISTORY_LEN        300
#define WARMUP_FRAMES            80

#define BASELINE_ALPHA           (0.08f / 3.0f)
#define PASS_BASELINE_ALPHA      0.015f
#define EDGE_RATIO               0.08f
#define SPIKE_Z_THRESHOLD        6.0f
#define CV_THRESHOLD             0.18f
#define MIN_TRIGGER              1.5f
#define MIN_CLEAR                0.8f
#define TRIGGER_K                3.5f
#define CLEAR_K                  1.8f

static float s_baseline[MAX_SUBCARRIERS];
static float s_last_amp[MAX_SUBCARRIERS];
static float s_static[QUALITY_WINDOW][MAX_SUBCARRIERS];
static float s_recent[VARIANCE_WINDOW][MAX_SUBCARRIERS];
static float s_scores[SCORE_HISTORY_LEN];
static float s_work[QUALITY_WINDOW > SCORE_HISTORY_LEN ? QUALITY_WINDOW : SCORE_HISTORY_LEN];
static float s_mean[MAX_SUBCARRIERS];
static float s_cv[MAX_SUBCARRIERS];
static float s_spike[MAX_SUBCARRIERS];
static float s_amp[MAX_SUBCARRIERS];
static bool s_selected[MAX_SUBCARRIERS];

static int s_n;
static uint32_t s_frames;
static int s_static_count;
static int s_static_pos;
static int s_recent_count;
static int s_recent_pos;
static int s_score_count;
static int s_score_pos;
static motion_state_t s_state;
static int s_trigger_counter;
static int s_clear_counter;

static int float_compare(const void *a, const void *b)
{
    float fa = *(const float *)a;
    float fb = *(const float *)b;
    return (fa > fb) - (fa < fb);
}

static float median(float *values, int count)
{
    qsort(values, count, sizeof(float), float_compare);
    if (count & 1) return values[count / 2];
    return 0.5f * (values[count / 2 - 1] + values[count / 2]);
}

static void reset_for_length(int n, const float *amp)
{
    s_n = n;
    s_frames = 0;
    s_static_count = 0;
    s_static_pos = 0;
    s_recent_count = 0;
    s_recent_pos = 0;
    s_score_count = 0;
    s_score_pos = 0;
    s_state = MOTION_STATE_CLEAR;
    s_trigger_counter = 0;
    s_clear_counter = 0;
    for (int i = 0; i < n; i++) {
        s_baseline[i] = amp[i];
        s_last_amp[i] = amp[i];
        s_selected[i] = true;
    }
}

void motion_core_init(void)
{
    s_n = 0;
    s_state = MOTION_STATE_CLEAR;
}

void motion_core_force_clear(void)
{
    s_state = MOTION_STATE_CLEAR;
    s_trigger_counter = 0;
    s_clear_counter = 0;
}

static void rebuild_quality_mask(void)
{
    int edge = (int)lroundf(s_n * EDGE_RATIO);
    if (edge < 1) edge = 1;
    int inner_start = edge;
    int inner_end = s_n - edge;
    int inner_count = inner_end - inner_start;
    if (inner_count <= 0) return;

    for (int sc = inner_start; sc < inner_end; sc++) {
        float sum = 0.0f;
        for (int k = 0; k < s_static_count; k++) sum += s_static[k][sc];
        float mean = sum / s_static_count;
        float var = 0.0f;
        for (int k = 0; k < s_static_count; k++) {
            float d = s_static[k][sc] - mean;
            var += d * d;
            s_work[k] = s_static[k][sc];
        }
        s_mean[sc] = mean;
        s_cv[sc] = sqrtf(var / s_static_count) / fmaxf(mean, 1e-6f);
        float med = median(s_work, s_static_count);
        for (int k = 0; k < s_static_count; k++) {
            s_work[k] = fabsf(s_static[k][sc] - med);
        }
        float sigma = fmaxf(1.4826f * median(s_work, s_static_count), 1e-3f);
        float max_z = 0.0f;
        for (int k = 0; k < s_static_count; k++) {
            float z = fabsf(s_static[k][sc] - med) / sigma;
            if (z > max_z) max_z = z;
        }
        s_spike[sc] = max_z;
    }

    for (int sc = inner_start; sc < inner_end; sc++) {
        s_work[sc - inner_start] = s_mean[sc];
    }
    qsort(s_work, inner_count, sizeof(float), float_compare);
    float mean_floor = s_work[(int)(0.2f * (inner_count - 1))];
    int selected = 0;
    for (int sc = 0; sc < s_n; sc++) {
        bool inner = sc >= inner_start && sc < inner_end;
        s_selected[sc] = inner && s_mean[sc] >= mean_floor &&
                         s_cv[sc] <= CV_THRESHOLD &&
                         s_spike[sc] <= SPIKE_Z_THRESHOLD;
        if (s_selected[sc]) selected++;
    }
    if (selected == 0) {
        for (int sc = 0; sc < s_n; sc++) {
            s_selected[sc] = sc >= inner_start && sc < inner_end;
        }
    }
}

static void get_thresholds(float *trigger, float *clear)
{
    if (s_score_count < 8) {
        *trigger = MIN_TRIGGER;
        *clear = MIN_CLEAR;
        return;
    }
    for (int i = 0; i < s_score_count; i++) s_work[i] = s_scores[i];
    float med = median(s_work, s_score_count);
    for (int i = 0; i < s_score_count; i++) s_work[i] = fabsf(s_scores[i] - med);
    float sigma = fmaxf(1.4826f * median(s_work, s_score_count), 1e-3f);
    *trigger = fmaxf(MIN_TRIGGER, med + TRIGGER_K * sigma);
    *clear = fmaxf(MIN_CLEAR, med + CLEAR_K * sigma);
    *clear = fminf(*clear, *trigger * 0.8f);
}

bool motion_core_update(const int8_t *csi, int raw_len, motion_result_t *out)
{
    if (!csi || !out || raw_len < 2) return false;
    int n = raw_len / 2;
    if (n > MAX_SUBCARRIERS) n = MAX_SUBCARRIERS;
    for (int i = 0; i < n; i++) {
        float a = csi[2 * i];
        float b = csi[2 * i + 1];
        s_amp[i] = sqrtf(a * a + b * b);
    }
    if (s_n != n) reset_for_length(n, s_amp);
    s_frames++;

    memcpy(s_static[s_static_pos], s_amp, n * sizeof(float));
    s_static_pos = (s_static_pos + 1) % QUALITY_WINDOW;
    if (s_static_count < QUALITY_WINDOW) s_static_count++;
    if (s_static_count >= QUALITY_MIN_FRAMES &&
        (s_frames == QUALITY_MIN_FRAMES || (s_frames % QUALITY_REFRESH_FRAMES) == 0)) {
        rebuild_quality_mask();
    }

    if (s_static_count >= QUALITY_MIN_FRAMES) {
        float alpha = s_state == MOTION_STATE_PASS ? PASS_BASELINE_ALPHA : BASELINE_ALPHA;
        for (int sc = 0; sc < n; sc++) {
            float sum = 0.0f;
            for (int k = 0; k < s_static_count; k++) sum += s_static[k][sc];
            float candidate = sum / s_static_count;
            s_baseline[sc] = (1.0f - alpha) * s_baseline[sc] + alpha * candidate;
        }
    }

    memcpy(s_recent[s_recent_pos], s_amp, n * sizeof(float));
    s_recent_pos = (s_recent_pos + 1) % VARIANCE_WINDOW;
    if (s_recent_count < VARIANCE_WINDOW) s_recent_count++;

    float baseline_sum = 0.0f, frame_sum = 0.0f, variance_sum = 0.0f;
    int selected_count = 0;
    for (int sc = 0; sc < n; sc++) {
        if (!s_selected[sc]) continue;
        selected_count++;
        baseline_sum += fabsf(s_amp[sc] - s_baseline[sc]);
        frame_sum += fabsf(s_amp[sc] - s_last_amp[sc]);
        if (s_recent_count >= 3) {
            float mean = 0.0f;
            for (int k = 0; k < s_recent_count; k++) mean += s_recent[k][sc];
            mean /= s_recent_count;
            float var = 0.0f;
            for (int k = 0; k < s_recent_count; k++) {
                float d = s_recent[k][sc] - mean;
                var += d * d;
            }
            variance_sum += var / s_recent_count;
        }
    }
    if (selected_count == 0) return false;

    float baseline_diff = baseline_sum / selected_count;
    float frame_diff = frame_sum / selected_count;
    float variance_diff = variance_sum / selected_count;
    float score = 0.70f * baseline_diff + 0.05f * frame_diff + 0.25f * variance_diff;
    float trigger, clear;
    get_thresholds(&trigger, &clear);
    bool warmup = s_frames <= WARMUP_FRAMES;
    motion_state_t previous = s_state;

    if (warmup) {
        s_state = MOTION_STATE_CLEAR;
        s_trigger_counter = s_clear_counter = 0;
        s_scores[s_score_pos] = score;
        s_score_pos = (s_score_pos + 1) % SCORE_HISTORY_LEN;
        if (s_score_count < SCORE_HISTORY_LEN) s_score_count++;
    } else if (s_state == MOTION_STATE_CLEAR) {
        s_scores[s_score_pos] = score;
        s_score_pos = (s_score_pos + 1) % SCORE_HISTORY_LEN;
        if (s_score_count < SCORE_HISTORY_LEN) s_score_count++;
        if (score >= trigger) {
            if (++s_trigger_counter >= 2) {
                s_state = MOTION_STATE_PASS;
                s_trigger_counter = s_clear_counter = 0;
            }
        } else {
            s_trigger_counter = 0;
        }
    } else {
        if (score <= clear) {
            if (++s_clear_counter >= 4) {
                s_state = MOTION_STATE_CLEAR;
                s_trigger_counter = s_clear_counter = 0;
                s_scores[s_score_pos] = score;
                s_score_pos = (s_score_pos + 1) % SCORE_HISTORY_LEN;
                if (s_score_count < SCORE_HISTORY_LEN) s_score_count++;
            }
        } else {
            s_clear_counter = 0;
        }
    }

    memcpy(s_last_amp, s_amp, n * sizeof(float));
    *out = (motion_result_t) {
        .baseline_diff = baseline_diff,
        .frame_diff = frame_diff,
        .variance_diff = variance_diff,
        .motion_score = score,
        .trigger_threshold = trigger,
        .clear_threshold = clear,
        .selected_count = selected_count,
        .in_warmup = warmup,
        .state_changed = previous != s_state,
        .state = s_state,
    };
    return true;
}
