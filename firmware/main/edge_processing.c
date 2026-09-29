/**
 * @file edge_processing.c
 * @brief Edge Intelligence — dual-core CSI processing pipeline.
 *
 * Core 0 (WiFi path): Pushes raw CSI frames into lock-free SPSC ring buffer.
 * Second core when present (DSP task): pops frames, runs signal processing.
 *   1. Phase extraction from I/Q pairs
 *   2. Phase unwrapping (continuous phase)
 *   3. Welford variance tracking per subcarrier
 *   4. Top-K subcarrier selection by variance
 *   5. Biquad IIR bandpass: breathing and heart rate (ray-derived bands)
 *   6. Zero-crossing BPM estimation
 *   7. Presence detection (adaptive or fixed threshold)
 *   8. Fall detection (phase acceleration)
 *   9. Multi-person vitals via subcarrier group clustering
 *  10. Delta compression (XOR + RLE) for bandwidth reduction
 *  11. Vitals packet broadcast (magic 0xC5110002)
 *
 * Declare trimmed version: no mmWave fusion, no WASM dispatch.
 */

#include "edge_processing.h"

/* Die temperature for thermal drift compensation */
#ifdef CONFIG_SOC_TEMP_SENSOR_SUPPORTED
#include "driver/temperature_sensor.h"
static temperature_sensor_handle_t s_temp_handle = NULL;
#endif
static float s_die_temp_c = 25.0f;

/* Amplitude EMA baseline for Fresnel profiling */
#define AMP_BASELINE_MAGIC  0xC5110008
#define AMP_BASELINE_ALPHA  0.02f   /* slow EMA: ~50s time constant at 1 Hz */
#define AMP_BASELINE_MAX_SC 256
static float s_amp_baseline[AMP_BASELINE_MAX_SC];
static bool  s_amp_baseline_valid = false;
static int64_t s_last_baseline_send_us = 0;
#define AMP_BASELINE_INTERVAL_US (30 * 1000000LL)  /* send every 30s */
static uint8_t s_latest_radio_meta = 0;
#include "nvs_config.h"
#include "csi_collector.h"

/* Runtime config */
extern nvs_config_t g_nvs_config;
#include "stream_sender.h"

#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

static const char *TAG = "edge_proc";

/* ======================================================================
 * SPSC Ring Buffer (lock-free, single-producer single-consumer)
 * ====================================================================== */

static edge_ring_buf_t s_ring;
static uint32_t s_ring_drops;

/* Scratch buffers — moved from stack to static to avoid stack overflow. */
static float s_scratch_br[EDGE_PHASE_HISTORY_LEN];
static float s_scratch_hr[EDGE_PHASE_HISTORY_LEN];
static float s_scratch_phases[EDGE_MAX_SUBCARRIERS];
static float s_scratch_amps[EDGE_MAX_SUBCARRIERS];
static float s_scratch_vars[EDGE_MAX_SUBCARRIERS];

static inline bool ring_push(const uint8_t *iq, uint16_t len,
                             int8_t rssi, uint8_t channel, uint8_t is_fold)
{
    uint32_t next = (s_ring.head + 1) % EDGE_RING_SLOTS;
    if (next == s_ring.tail) {
        s_ring_drops++;
        return false;
    }

    edge_ring_slot_t *slot = &s_ring.slots[s_ring.head];
    uint16_t copy_len = (len > EDGE_MAX_IQ_BYTES) ? EDGE_MAX_IQ_BYTES : len;
    memcpy(slot->iq_data, iq, copy_len);
    slot->iq_len = copy_len;
    slot->rssi = rssi;
    slot->channel = channel;
    slot->is_fold = is_fold;
    slot->timestamp_us = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);

    __sync_synchronize();
    s_ring.head = next;
    return true;
}

static inline bool ring_pop(edge_ring_slot_t *out)
{
    if (s_ring.tail == s_ring.head) {
        return false;
    }

    memcpy(out, &s_ring.slots[s_ring.tail], sizeof(edge_ring_slot_t));

    __sync_synchronize();
    s_ring.tail = (s_ring.tail + 1) % EDGE_RING_SLOTS;
    return true;
}

/* ======================================================================
 * Biquad IIR Filter
 * ====================================================================== */

static void biquad_bandpass_design(edge_biquad_t *bq, float fs,
                                   float f_lo, float f_hi)
{
    float w0 = 2.0f * M_PI * (f_lo + f_hi) / 2.0f / fs;
    float bw = 2.0f * M_PI * (f_hi - f_lo) / fs;
    float alpha = sinf(w0) * sinhf(logf(2.0f) / 2.0f * bw / sinf(w0));

    float a0_inv = 1.0f / (1.0f + alpha);
    bq->b0 =  alpha * a0_inv;
    bq->b1 =  0.0f;
    bq->b2 = -alpha * a0_inv;
    bq->a1 = -2.0f * cosf(w0) * a0_inv;
    bq->a2 =  (1.0f - alpha) * a0_inv;

    bq->x1 = bq->x2 = 0.0f;
    bq->y1 = bq->y2 = 0.0f;
}

static inline float biquad_process(edge_biquad_t *bq, float x)
{
    float y = bq->b0 * x + bq->b1 * bq->x1 + bq->b2 * bq->x2
            - bq->a1 * bq->y1 - bq->a2 * bq->y2;
    bq->x2 = bq->x1;
    bq->x1 = x;
    bq->y2 = bq->y1;
    bq->y1 = y;
    return y;
}

/* ======================================================================
 * Phase Extraction and Unwrapping
 * ====================================================================== */

static inline float extract_phase(const uint8_t *iq, uint16_t idx)
{
    int8_t i_val = (int8_t)iq[idx * 2];
    int8_t q_val = (int8_t)iq[idx * 2 + 1];
    return atan2f((float)q_val, (float)i_val);
}

static inline float unwrap_phase(float prev, float curr)
{
    float diff = curr - prev;
    if (diff > M_PI)       diff -= 2.0f * M_PI;
    else if (diff < -M_PI) diff += 2.0f * M_PI;
    return prev + diff;
}

/* ======================================================================
 * Welford Running Statistics
 * ====================================================================== */

static inline void welford_reset(edge_welford_t *w)
{
    w->mean = 0.0;
    w->m2   = 0.0;
    w->count = 0;
}

static inline void welford_update(edge_welford_t *w, double x)
{
    w->count++;
    double delta = x - w->mean;
    w->mean += delta / (double)w->count;
    double delta2 = x - w->mean;
    w->m2 += delta * delta2;
}

static inline double welford_variance(const edge_welford_t *w)
{
    return (w->count > 1) ? (w->m2 / (double)(w->count - 1)) : 0.0;
}

/* ======================================================================
 * Zero-Crossing BPM Estimation
 * ====================================================================== */

static float estimate_bpm_zero_crossing(const float *history, uint16_t len,
                                        float sample_rate)
{
    if (len < 4) return 0.0f;

    uint16_t crossings[128];
    uint16_t n_cross = 0;

    for (uint16_t i = 1; i < len && n_cross < 128; i++) {
        if (history[i - 1] <= 0.0f && history[i] > 0.0f) {
            crossings[n_cross++] = i;
        }
    }

    if (n_cross < 2) return 0.0f;

    float total_period = 0.0f;
    for (uint16_t i = 1; i < n_cross; i++) {
        total_period += (float)(crossings[i] - crossings[i - 1]);
    }
    float avg_period_samples = total_period / (float)(n_cross - 1);

    if (avg_period_samples < 1.0f) return 0.0f;

    float freq_hz = sample_rate / avg_period_samples;
    return freq_hz * 60.0f;
}

/* Field-derived constants from GF(257) */
#define FIELD_NRAYS       12
#define FIELD_OU_ALPHA    0.9048f    /* Ornstein-Uhlenbeck mean reversion */
#define FIELD_OU_BETA     0.02129f   /* O-U diffusion coefficient */
#define FIELD_EPSILON4    0.0486f    /* convergent error ε₄ */

/* HR harmonic rejection: return 1 if hr is within 12% of 2x or 3x br. */
static int hr_is_breath_harmonic(float hr, float br) {
    if (br <= 0.0f || hr <= 0.0f) return 0;
    for (int k = 2; k <= 3; k++) {
        if (fabsf(hr - (float)k * br) < FIELD_EPSILON4 * hr) return 1;
    }
    return 0;
}

/* ======================================================================
 * DSP Pipeline State
 * ====================================================================== */

static edge_config_t s_cfg;
static edge_welford_t s_subcarrier_var[EDGE_MAX_SUBCARRIERS];
static float s_prev_phase[EDGE_MAX_SUBCARRIERS];
static bool  s_phase_initialized;
static uint8_t s_top_k[EDGE_TOP_K];
static uint8_t s_top_k_count;
static float s_phase_history[EDGE_PHASE_HISTORY_LEN];
static uint16_t s_history_len;
static uint16_t s_history_idx;
static edge_biquad_t s_bq_breathing;
static edge_biquad_t s_bq_heartrate;

/* Brainwave band filters (within 10 Hz Nyquist at 20 Hz CSI rate). */
static edge_biquad_t s_bq_delta;   /* 0.5-4 Hz */
static edge_biquad_t s_bq_theta;   /* 4-8 Hz */
static edge_biquad_t s_bq_alpha;   /* 8-9.5 Hz */
static float s_delta_energy;
static float s_theta_energy;
static float s_alpha_energy;
#define BRAINWAVE_SMOOTH  FIELD_OU_ALPHA

static float s_breathing_filtered[EDGE_PHASE_HISTORY_LEN];
static float s_heartrate_filtered[EDGE_PHASE_HISTORY_LEN];

static float    s_breathing_bpm;
static float    s_heartrate_bpm;
static float    s_motion_energy;
static float    s_presence_score;
static bool     s_presence_detected;
static bool     s_fall_detected;
static int8_t   s_latest_rssi;
static uint32_t s_frame_count;
static float s_prev_phase_velocity;
static uint8_t  s_fall_consec_count;
static int64_t  s_fall_last_alert_us;
static bool     s_calibrated;
static float    s_calib_sum;
static float    s_calib_sum_sq;
static uint32_t s_calib_count;
static float    s_adaptive_threshold;
static int64_t s_last_vitals_send_us;
static uint8_t s_prev_iq[EDGE_MAX_IQ_BYTES];
static uint16_t s_prev_iq_len;
static bool s_has_prev_iq;
static uint16_t s_feature_seq;
static edge_person_vitals_t s_persons[EDGE_MAX_PERSONS];
static edge_biquad_t s_person_bq_br[EDGE_MAX_PERSONS];
static edge_biquad_t s_person_bq_hr[EDGE_MAX_PERSONS];
static float s_person_br_filt[EDGE_MAX_PERSONS][EDGE_PHASE_HISTORY_LEN];
static float s_person_hr_filt[EDGE_MAX_PERSONS][EDGE_PHASE_HISTORY_LEN];
static float s_breath_ref = 0.0f;
static float s_person_breath_ref[EDGE_MAX_PERSONS] = {0};
static uint32_t s_prev_frame_ts_us = 0;
static float s_sample_rate = 20.0f;
static volatile edge_vitals_pkt_t s_latest_pkt;
static volatile bool s_pkt_valid;

/* ======================================================================
 * Top-K Subcarrier Selection
 * ====================================================================== */

static void update_top_k(uint16_t n_subcarriers)
{
    uint8_t k = s_cfg.top_k_count;
    if (k > EDGE_TOP_K) k = EDGE_TOP_K;
    if (k > n_subcarriers) k = (uint8_t)n_subcarriers;

    bool used[EDGE_MAX_SUBCARRIERS];
    memset(used, 0, sizeof(used));

    for (uint8_t ki = 0; ki < k; ki++) {
        double best_var = -1.0;
        uint8_t best_idx = 0;

        for (uint16_t sc = 0; sc < n_subcarriers; sc++) {
            if (!used[sc]) {
                double v = welford_variance(&s_subcarrier_var[sc]);
                if (v > best_var) {
                    best_var = v;
                    best_idx = (uint8_t)sc;
                }
            }
        }

        s_top_k[ki] = best_idx;
        used[best_idx] = true;
    }

    s_top_k_count = k;
}

/* ======================================================================
 * Adaptive Presence Calibration
 * ====================================================================== */

static void calibration_update(float motion)
{
    if (s_calibrated) return;

    s_calib_sum += motion;
    s_calib_sum_sq += motion * motion;
    s_calib_count++;

    if (s_calib_count >= EDGE_CALIB_FRAMES) {
        float mean = s_calib_sum / (float)s_calib_count;
        float var = (s_calib_sum_sq / (float)s_calib_count) - (mean * mean);
        float sigma = (var > 0.0f) ? sqrtf(var) : 0.001f;

        s_adaptive_threshold = mean + EDGE_CALIB_SIGMA_MULT * sigma;
        if (s_adaptive_threshold < FIELD_OU_BETA) {
            s_adaptive_threshold = FIELD_OU_BETA;
        }

        s_calibrated = true;
        ESP_LOGI(TAG, "Adaptive calibration complete: mean=%.4f sigma=%.4f "
                 "threshold=%.4f (from %lu frames)",
                 mean, sigma, s_adaptive_threshold,
                 (unsigned long)s_calib_count);
    }
}

/* ======================================================================
 * Delta Compression (XOR + RLE)
 * ====================================================================== */

static uint16_t delta_compress(const uint8_t *curr, uint16_t len,
                               uint8_t *out, uint16_t out_max)
{
    if (!s_has_prev_iq || len != s_prev_iq_len || len == 0) {
        return 0;
    }

    static uint8_t xor_buf[EDGE_MAX_IQ_BYTES];
    for (uint16_t i = 0; i < len; i++) {
        xor_buf[i] = curr[i] ^ s_prev_iq[i];
    }

    uint16_t out_idx = 0;
    uint16_t i = 0;
    while (i < len) {
        uint8_t val = xor_buf[i];
        uint16_t run = 1;
        while (i + run < len && xor_buf[i + run] == val && run < 255) {
            run++;
        }

        if (out_idx + 2 > out_max) return 0;
        out[out_idx++] = val;
        out[out_idx++] = (uint8_t)run;
        i += run;
    }

    if (out_idx >= len) {
        return 0;
    }

    return out_idx;
}

static uint8_t s_comp_buf[EDGE_MAX_IQ_BYTES];
static uint8_t s_comp_pkt[10 + EDGE_MAX_IQ_BYTES];

static void send_compressed_frame(const uint8_t *iq_data, uint16_t iq_len,
                                  uint8_t channel)
{
    uint16_t comp_len = delta_compress(iq_data, iq_len,
                                       s_comp_buf, sizeof(s_comp_buf));
    if (comp_len == 0) {
        goto store_prev;
    }

    {
        uint16_t pkt_size = 10 + comp_len;
        uint8_t *pkt = s_comp_pkt;

        uint32_t magic = EDGE_COMPRESSED_MAGIC;
        memcpy(&pkt[0], &magic, 4);

        pkt[4] = csi_collector_get_node_id();
        pkt[5] = channel;
        memcpy(&pkt[6], &iq_len, 2);
        memcpy(&pkt[8], &comp_len, 2);
        memcpy(&pkt[10], s_comp_buf, comp_len);

        stream_sender_send(pkt, pkt_size);
    }

store_prev:
    memcpy(s_prev_iq, iq_data, iq_len);
    s_prev_iq_len = iq_len;
    s_has_prev_iq = true;
}

/* ======================================================================
 * Multi-Person Vitals
 * ====================================================================== */

static void update_multi_person_vitals(const uint8_t *iq_data, uint16_t n_sc,
                                       float sample_rate)
{
    if (s_top_k_count < 2) return;

    float ray_phase[FIELD_NRAYS];
    uint8_t ray_count[FIELD_NRAYS];
    uint8_t ray_rep[FIELD_NRAYS];
    memset(ray_count, 0, sizeof(ray_count));
    memset(ray_rep, 0, sizeof(ray_rep));

    for (uint8_t i = 0; i < s_top_k_count; i++) {
        uint8_t sc_idx = s_top_k[i];
        if (sc_idx >= n_sc) continue;
        uint8_t ray = sc_idx % FIELD_NRAYS;
        float ph = extract_phase(iq_data, sc_idx);
        if (ray_count[ray] == 0) {
            ray_phase[ray] = ph;
            ray_rep[ray] = sc_idx;
        } else {
            ray_phase[ray] += ph;
        }
        ray_count[ray]++;
    }

    uint8_t n_persons = 0;
    for (uint8_t r = 0; r < FIELD_NRAYS && n_persons < EDGE_MAX_PERSONS; r++) {
        if (ray_count[r] == 0) continue;
        float avg_phase = ray_phase[r] / (float)ray_count[r];

        edge_person_vitals_t *pv = &s_persons[n_persons];
        pv->active = true;
        pv->subcarrier_idx = ray_rep[r];

        if (pv->history_len > 0) {
            uint16_t prev_idx = (pv->history_idx + EDGE_PHASE_HISTORY_LEN - 1)
                                % EDGE_PHASE_HISTORY_LEN;
            avg_phase = unwrap_phase(pv->phase_history[prev_idx], avg_phase);
        }

        pv->phase_history[pv->history_idx] = avg_phase;
        pv->history_idx = (pv->history_idx + 1) % EDGE_PHASE_HISTORY_LEN;
        if (pv->history_len < EDGE_PHASE_HISTORY_LEN) pv->history_len++;

        float br_val = biquad_process(&s_person_bq_br[n_persons], avg_phase);
        float hr_val = biquad_process(&s_person_bq_hr[n_persons], avg_phase);

        uint16_t idx = (pv->history_idx + EDGE_PHASE_HISTORY_LEN - 1)
                       % EDGE_PHASE_HISTORY_LEN;
        s_person_br_filt[n_persons][idx] = br_val;
        s_person_hr_filt[n_persons][idx] = hr_val;

        if (pv->history_len >= 64) {
            uint16_t buf_len = pv->history_len;

            for (uint16_t i = 0; i < buf_len; i++) {
                uint16_t ri = (pv->history_idx + EDGE_PHASE_HISTORY_LEN
                               - buf_len + i) % EDGE_PHASE_HISTORY_LEN;
                s_scratch_br[i] = s_person_br_filt[n_persons][ri];
                s_scratch_hr[i] = s_person_hr_filt[n_persons][ri];
            }

            float br = estimate_bpm_zero_crossing(s_scratch_br, buf_len, sample_rate);
            float hr = estimate_bpm_zero_crossing(s_scratch_hr, buf_len, sample_rate);

            if (br > 0.0f) pv->breathing_bpm = br;
            if (br > 0.0f) s_person_breath_ref[n_persons] = (s_person_breath_ref[n_persons] > 0.0f) ? (FIELD_OU_ALPHA * s_person_breath_ref[n_persons] + (1.0f - FIELD_OU_ALPHA) * br) : br;
            if (hr > 0.0f && !hr_is_breath_harmonic(hr, s_person_breath_ref[n_persons])) pv->heartrate_bpm = hr;
            else if (hr_is_breath_harmonic(hr, s_person_breath_ref[n_persons])) pv->heartrate_bpm = 0.0f;
        }
        n_persons++;
    }

    for (uint8_t p = n_persons; p < EDGE_MAX_PERSONS; p++) {
        s_persons[p].active = false;
    }
}

/* ======================================================================
 * Vitals Packet Sending
 * ====================================================================== */

void edge_init_temperature(void)
{
#ifdef CONFIG_SOC_TEMP_SENSOR_SUPPORTED
    temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    if (temperature_sensor_install(&cfg, &s_temp_handle) == ESP_OK) {
        temperature_sensor_enable(s_temp_handle);
        ESP_LOGI("edge", "Die temperature sensor initialized");
    }
#endif
}

static void update_die_temperature(void)
{
#ifdef CONFIG_SOC_TEMP_SENSOR_SUPPORTED
    if (s_temp_handle) {
        temperature_sensor_get_celsius(s_temp_handle, &s_die_temp_c);
    }
#endif
}

/* Update amplitude EMA baseline from latest CSI frame */
static void update_amp_baseline(const uint8_t *iq_data, uint16_t n_sc,
                                uint16_t sc_offset)
{
    if (n_sc == 0 || (n_sc + sc_offset) > AMP_BASELINE_MAX_SC) return;
    for (uint16_t k = 0; k < n_sc; k++) {
        uint16_t idx = k + sc_offset;
        int8_t i_val = (int8_t)iq_data[2 * k];
        int8_t q_val = (int8_t)iq_data[2 * k + 1];
        float a = sqrtf((float)(i_val * i_val + q_val * q_val));
        if (!s_amp_baseline_valid) {
            s_amp_baseline[idx] = a;
        } else {
            s_amp_baseline[idx] = s_amp_baseline[idx] * (1.0f - AMP_BASELINE_ALPHA)
                                 + a * AMP_BASELINE_ALPHA;
        }
    }
    s_amp_baseline_valid = true;
}

/* Send amplitude baseline packet (0xC5110008) */
static void send_amp_baseline(uint16_t n_sc)
{
    if (!s_amp_baseline_valid || n_sc == 0) return;
    int64_t now = esp_timer_get_time();
    if ((now - s_last_baseline_send_us) < AMP_BASELINE_INTERVAL_US) return;
    s_last_baseline_send_us = now;

    uint16_t sc = (n_sc > AMP_BASELINE_MAX_SC) ? AMP_BASELINE_MAX_SC : n_sc;
    size_t pkt_len = 8 + sc * 2;
    static uint8_t pkt[8 + AMP_BASELINE_MAX_SC * 2];
    uint32_t magic = AMP_BASELINE_MAGIC;
    memcpy(&pkt[0], &magic, 4);
    pkt[4] = csi_collector_get_node_id();
    pkt[5] = (uint8_t)(int8_t)s_die_temp_c;
    memcpy(&pkt[6], &sc, 2);
    for (uint16_t k = 0; k < sc; k++) {
        uint16_t val = (uint16_t)(s_amp_baseline[k] * 100.0f);
        memcpy(&pkt[8 + k * 2], &val, 2);
    }
    stream_sender_send(pkt, pkt_len);
}

static void send_vitals_packet(void)
{
    edge_vitals_pkt_t pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.magic = EDGE_VITALS_MAGIC;
    pkt.node_id = csi_collector_get_node_id();

    pkt.flags = 0;
    if (s_presence_detected) pkt.flags |= 0x01;
    if (s_fall_detected)     pkt.flags |= 0x02;
    if (s_motion_energy > FIELD_OU_BETA) pkt.flags |= 0x04;

    pkt.breathing_rate = (uint16_t)(s_breathing_bpm * 100.0f);
    pkt.heartrate = (uint32_t)(s_heartrate_bpm * 10000.0f);
    pkt.rssi = s_latest_rssi;

    uint8_t n_active = 0;
    for (uint8_t p = 0; p < EDGE_MAX_PERSONS; p++) {
        if (s_persons[p].active) n_active++;
    }
    pkt.n_persons = n_active;

    pkt.motion_energy = s_motion_energy;
    pkt.presence_score = s_presence_score;
    pkt.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);

    update_die_temperature();
    pkt.die_temp_c = (int8_t)s_die_temp_c;
    pkt.radio_meta = s_latest_radio_meta;

    /* Update thread-safe copy. */
    s_latest_pkt = pkt;
    s_pkt_valid = true;

    /* Standard 32-byte vitals packet (no mmWave fusion in Declare). */
    stream_sender_send((const uint8_t *)&pkt, sizeof(pkt));
}

static void send_brainwave_packet(void)
{
    edge_bwave_pkt_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.magic = EDGE_BWAVE_MAGIC;
    pkt.node_id = csi_collector_get_node_id();
    pkt.timestamp_ms = (uint16_t)((esp_timer_get_time() / 1000) & 0xFFFF);
    pkt.delta = s_delta_energy;
    pkt.theta = s_theta_energy;
    pkt.alpha = s_alpha_energy;
    stream_sender_send((const uint8_t *)&pkt, sizeof(pkt));
}

static void send_person_vitals_packet(void)
{
    edge_person_pkt_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.magic = EDGE_PERSON_MAGIC;
    pkt.node_id = csi_collector_get_node_id();
    pkt.timestamp_ms = (uint16_t)((esp_timer_get_time() / 1000) & 0xFFFF);
    uint8_t n = 0;
    for (uint8_t p = 0; p < EDGE_MAX_PERSONS; p++) {
        pkt.persons[p].active = s_persons[p].active ? 1 : 0;
        if (s_persons[p].active) {
            pkt.persons[p].breathing_rate = (uint16_t)(s_persons[p].breathing_bpm * 100.0f);
            pkt.persons[p].heartrate = (uint16_t)(s_persons[p].heartrate_bpm * 100.0f);
            pkt.persons[p].subcarrier_idx = s_persons[p].subcarrier_idx;
            n++;
        }
    }
    pkt.n_persons = n;
    stream_sender_send((const uint8_t *)&pkt, sizeof(pkt));
}

static void send_feature_vector(void)
{
    edge_feature_pkt_t pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.magic = EDGE_FEATURE_MAGIC;
    pkt.node_id = csi_collector_get_node_id();
    pkt.reserved = 0;
    pkt.seq = s_feature_seq++;
    pkt.timestamp_us = esp_timer_get_time();

    pkt.features[0] = s_presence_score;
    pkt.features[1] = s_motion_energy;
    pkt.features[2] = s_breathing_bpm;
    pkt.features[3] = s_heartrate_bpm;

    float var_mean = 0.0f;
    if (s_top_k_count > 0) {
        float var_sum = 0.0f;
        uint8_t k = s_top_k_count < EDGE_TOP_K ? s_top_k_count : EDGE_TOP_K;
        for (uint8_t i = 0; i < k; i++) {
            var_sum += (float)welford_variance(&s_subcarrier_var[s_top_k[i]]);
        }
        var_mean = var_sum / (float)k;
    }
    pkt.features[4] = var_mean;

    uint8_t n_active = 0;
    for (uint8_t i = 0; i < EDGE_MAX_PERSONS; i++) {
        if (s_persons[i].active) n_active++;
    }
    pkt.features[5] = (float)n_active;
    pkt.features[6] = s_fall_detected ? 1.0f : 0.0f;
    pkt.features[7] = (float)s_latest_rssi;

    stream_sender_send((const uint8_t *)&pkt, sizeof(pkt));
}

/* ======================================================================
 * Main DSP Pipeline (runs on Core 1)
 * ====================================================================== */

static void process_frame(const edge_ring_slot_t *slot)
{
    uint16_t n_subcarriers = slot->iq_len / 2;
    if (n_subcarriers == 0 || n_subcarriers > EDGE_MAX_SUBCARRIERS) return;

    /* Fold offset: NDP/management frames populate the det=-1 side (128-255). */
    uint16_t sc_offset = slot->is_fold ? 128 : 0;
    if (n_subcarriers + sc_offset > EDGE_MAX_SUBCARRIERS) return;

    s_frame_count++;
    s_latest_rssi = slot->rssi;

    if (s_prev_frame_ts_us > 0 && slot->timestamp_us > s_prev_frame_ts_us) {
        uint32_t dt = slot->timestamp_us - s_prev_frame_ts_us;
        if (dt > 0 && dt < 1000000) {
            float r = 1000000.0f / (float)dt;
            s_sample_rate = FIELD_OU_ALPHA * s_sample_rate + (1.0f - FIELD_OU_ALPHA) * r;
        }
    }
    s_prev_frame_ts_us = slot->timestamp_us;
    const float sample_rate = s_sample_rate;

    /* Phase extraction + unwrapping per subcarrier (offset by 128 for fold frames). */
    float *phases = s_scratch_phases;
    for (uint16_t sc = 0; sc < n_subcarriers; sc++) {
        uint16_t idx = sc + sc_offset;
        float raw_phase = extract_phase(slot->iq_data, sc);

        if (s_phase_initialized) {
            phases[idx] = unwrap_phase(s_prev_phase[idx], raw_phase);
        } else {
            phases[idx] = raw_phase;
        }
        s_prev_phase[idx] = phases[idx];
    }
    s_phase_initialized = true;

    /* Update amplitude EMA baseline (offset for fold) */
    update_amp_baseline(slot->iq_data, n_subcarriers, sc_offset);

    /* Welford variance update per subcarrier (offset for fold) */
    for (uint16_t sc = 0; sc < n_subcarriers; sc++) {
        uint16_t idx = sc + sc_offset;
        welford_update(&s_subcarrier_var[idx], (double)phases[idx]);
    }

    if ((s_frame_count % 10) == 1 || s_top_k_count == 0) {
        update_top_k(EDGE_MAX_SUBCARRIERS);
    }

    if (s_top_k_count == 0) return;

    /* Phase of primary (highest-variance) subcarrier */
    float primary_phase = phases[s_top_k[0]];

    s_phase_history[s_history_idx] = primary_phase;
    s_history_idx = (s_history_idx + 1) % EDGE_PHASE_HISTORY_LEN;
    if (s_history_len < EDGE_PHASE_HISTORY_LEN) s_history_len++;

    /* Biquad bandpass filtering */
    float br_val = biquad_process(&s_bq_breathing, primary_phase);
    float hr_val = biquad_process(&s_bq_heartrate, primary_phase);

    /* Brainwave band energy */
    float dv = biquad_process(&s_bq_delta, primary_phase);
    float tv = biquad_process(&s_bq_theta, primary_phase);
    float av = biquad_process(&s_bq_alpha, primary_phase);
    s_delta_energy = BRAINWAVE_SMOOTH * s_delta_energy + (1.0f - BRAINWAVE_SMOOTH) * (dv * dv);
    s_theta_energy = BRAINWAVE_SMOOTH * s_theta_energy + (1.0f - BRAINWAVE_SMOOTH) * (tv * tv);
    s_alpha_energy = BRAINWAVE_SMOOTH * s_alpha_energy + (1.0f - BRAINWAVE_SMOOTH) * (av * av);

    uint16_t filt_idx = (s_history_idx + EDGE_PHASE_HISTORY_LEN - 1)
                        % EDGE_PHASE_HISTORY_LEN;
    s_breathing_filtered[filt_idx] = br_val;
    s_heartrate_filtered[filt_idx] = hr_val;

    /* BPM estimation (zero-crossing) */
    if (s_history_len >= 64) {
        uint16_t buf_len = s_history_len;

        for (uint16_t i = 0; i < buf_len; i++) {
            uint16_t ri = (s_history_idx + EDGE_PHASE_HISTORY_LEN
                           - buf_len + i) % EDGE_PHASE_HISTORY_LEN;
            s_scratch_br[i] = s_breathing_filtered[ri];
            s_scratch_hr[i] = s_heartrate_filtered[ri];
        }

        float br_bpm = estimate_bpm_zero_crossing(s_scratch_br, buf_len, sample_rate);
        float hr_bpm = estimate_bpm_zero_crossing(s_scratch_hr, buf_len, sample_rate);

        if (br_bpm >= 6.0f && br_bpm <= 40.0f) s_breathing_bpm = br_bpm;
        if (br_bpm >= 6.0f && br_bpm <= 40.0f) s_breath_ref = (s_breath_ref > 0.0f) ? (FIELD_OU_ALPHA * s_breath_ref + (1.0f - FIELD_OU_ALPHA) * br_bpm) : br_bpm;
        if (hr_bpm >= 40.0f && hr_bpm <= 180.0f && !hr_is_breath_harmonic(hr_bpm, s_breath_ref)) s_heartrate_bpm = hr_bpm;
        else if (hr_is_breath_harmonic(hr_bpm, s_breath_ref)) s_heartrate_bpm = 0.0f;
    }

    /* Motion energy */
    if (s_history_len >= FIELD_NRAYS) {
        float sum = 0.0f, sum2 = 0.0f;
        uint16_t window = (s_history_len < FIELD_NRAYS) ? s_history_len : FIELD_NRAYS;
        for (uint16_t i = 0; i < window; i++) {
            uint16_t ri = (s_history_idx + EDGE_PHASE_HISTORY_LEN
                           - window + i) % EDGE_PHASE_HISTORY_LEN;
            float v = s_phase_history[ri];
            sum += v;
            sum2 += v * v;
        }
        float mean = sum / (float)window;
        s_motion_energy = (sum2 / (float)window) - (mean * mean);
        if (s_motion_energy < 0.0f) s_motion_energy = 0.0f;
    }

    /* Presence detection */
    s_presence_score = s_motion_energy;

    if (!s_calibrated && s_cfg.presence_thresh == 0.0f) {
        calibration_update(s_motion_energy);
    }

    float threshold = s_cfg.presence_thresh;
    if (threshold == 0.0f && s_calibrated) {
        threshold = s_adaptive_threshold;
    } else if (threshold == 0.0f) {
        threshold = FIELD_EPSILON4;
    }
    s_presence_detected = (s_presence_score > threshold);

    /* Fall detection (phase acceleration + debounce) */
    if (s_history_len >= 3) {
        uint16_t i0 = (s_history_idx + EDGE_PHASE_HISTORY_LEN - 1) % EDGE_PHASE_HISTORY_LEN;
        uint16_t i1 = (s_history_idx + EDGE_PHASE_HISTORY_LEN - 2) % EDGE_PHASE_HISTORY_LEN;
        float velocity = s_phase_history[i0] - s_phase_history[i1];
        float accel = fabsf(velocity - s_prev_phase_velocity);
        s_prev_phase_velocity = velocity;

        if (accel > s_cfg.fall_thresh) {
            s_fall_consec_count++;
        } else {
            s_fall_consec_count = 0;
        }

        int64_t now_us = esp_timer_get_time();
        int64_t cooldown_us = (int64_t)EDGE_FALL_COOLDOWN_MS * 1000;
        if (s_fall_consec_count >= EDGE_FALL_CONSEC_MIN
            && (now_us - s_fall_last_alert_us) >= cooldown_us)
        {
            s_fall_detected = true;
            s_fall_last_alert_us = now_us;
            s_fall_consec_count = 0;
            ESP_LOGW(TAG, "Fall detected! accel=%.4f > thresh=%.4f",
                     accel, s_cfg.fall_thresh);
        } else if (s_fall_consec_count == 0) {
            s_fall_detected = false;
        }
    }

    /* Multi-person vitals */
    update_multi_person_vitals(slot->iq_data, n_subcarriers, sample_rate);
    if (s_cfg.tier >= 2) vTaskDelay(1);

    /* Delta compression */
    if (s_cfg.tier >= 2) {
        send_compressed_frame(slot->iq_data, slot->iq_len, slot->channel);
    }

    /* Send vitals packet at configured interval */
    int64_t now_us = esp_timer_get_time();
    int64_t interval_us = (int64_t)s_cfg.vital_interval_ms * 1000;
    if ((now_us - s_last_vitals_send_us) >= interval_us) {
        send_vitals_packet();
        send_feature_vector();
        send_brainwave_packet();
        send_person_vitals_packet();
        send_amp_baseline(EDGE_MAX_SUBCARRIERS);
        s_last_vitals_send_us = now_us;

        if ((s_frame_count % 200) == 0) {
            ESP_LOGI(TAG, "Vitals: br=%.1f hr=%.1f motion=%.4f pres=%s "
                     "fall=%s persons=%u frames=%lu drops=%lu",
                     s_breathing_bpm, s_heartrate_bpm, s_motion_energy,
                     s_presence_detected ? "YES" : "no",
                     s_fall_detected ? "YES" : "no",
                     (unsigned)s_latest_pkt.n_persons,
                     (unsigned long)s_frame_count,
                     (unsigned long)s_ring_drops);
        }
    }
}

/* ======================================================================
 * Edge Processing Task (pinned to Core 1)
 * ====================================================================== */

static void edge_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Edge DSP task started on core %d (tier=%u)",
             xPortGetCoreID(), s_cfg.tier);

    edge_ring_slot_t slot;

    while (1) {
        uint8_t processed = 0;

        while (processed < EDGE_BATCH_LIMIT && ring_pop(&slot)) {
            process_frame(&slot);
            processed++;
            vTaskDelay(1);
        }

        if (processed > 0) {
            { TickType_t d = pdMS_TO_TICKS(20); vTaskDelay(d > 0 ? d : 1); }
        } else {
            vTaskDelay(1);
        }
    }
}

/* ======================================================================
 * Public API
 * ====================================================================== */

bool edge_enqueue_csi(const uint8_t *iq_data, uint16_t iq_len,
                      int8_t rssi, uint8_t channel, uint8_t is_fold)
{
    return ring_push(iq_data, iq_len, rssi, channel, is_fold);
}

bool edge_get_vitals(edge_vitals_pkt_t *pkt)
{
    if (!s_pkt_valid || pkt == NULL) return false;
    memcpy(pkt, (const void *)&s_latest_pkt, sizeof(edge_vitals_pkt_t));
    return true;
}

void edge_get_multi_person(edge_person_vitals_t *persons, uint8_t *n_active)
{
    uint8_t active = 0;
    for (uint8_t p = 0; p < EDGE_MAX_PERSONS; p++) {
        if (persons) persons[p] = s_persons[p];
        if (s_persons[p].active) active++;
    }
    if (n_active) *n_active = active;
}

void edge_get_phase_history(const float **out_buf, uint16_t *out_len,
                            uint16_t *out_idx)
{
    if (out_buf) *out_buf = s_phase_history;
    if (out_len) *out_len = s_history_len;
    if (out_idx) *out_idx = s_history_idx;
}

void edge_get_variances(float *out_variances, uint16_t n_subcarriers)
{
    if (out_variances == NULL) return;
    uint16_t n = (n_subcarriers > EDGE_MAX_SUBCARRIERS) ? EDGE_MAX_SUBCARRIERS : n_subcarriers;
    for (uint16_t i = 0; i < n; i++) {
        out_variances[i] = (float)welford_variance(&s_subcarrier_var[i]);
    }
}

void edge_get_brainwave_bands(float *delta, float *theta, float *alpha)
{
    if (delta) *delta = s_delta_energy;
    if (theta) *theta = s_theta_energy;
    if (alpha) *alpha = s_alpha_energy;
}

esp_err_t edge_processing_init(const edge_config_t *cfg)
{
    if (cfg == NULL) {
        ESP_LOGE(TAG, "edge_processing_init: cfg is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    s_cfg = *cfg;

    ESP_LOGI(TAG, "Initializing edge processing (tier=%u, top_k=%u, "
             "vital_interval=%ums, presence_thresh=%.3f)",
             s_cfg.tier, s_cfg.top_k_count,
             s_cfg.vital_interval_ms, s_cfg.presence_thresh);

    /* Reset all state. */
    memset(&s_ring, 0, sizeof(s_ring));
    memset(s_subcarrier_var, 0, sizeof(s_subcarrier_var));
    memset(s_prev_phase, 0, sizeof(s_prev_phase));
    s_phase_initialized = false;
    s_top_k_count = 0;
    s_history_len = 0;
    s_history_idx = 0;
    s_breathing_bpm = 0.0f;
    s_heartrate_bpm = 0.0f;
    s_motion_energy = 0.0f;
    s_presence_score = 0.0f;
    s_presence_detected = false;
    s_fall_detected = false;
    s_latest_rssi = 0;
    s_frame_count = 0;
    s_prev_phase_velocity = 0.0f;
    s_fall_consec_count = 0;
    s_fall_last_alert_us = 0;
    s_last_vitals_send_us = 0;
    s_has_prev_iq = false;
    s_prev_iq_len = 0;
    s_pkt_valid = false;

    s_calibrated = false;
    s_calib_sum = 0.0f;
    s_calib_sum_sq = 0.0f;
    s_calib_count = 0;
    s_adaptive_threshold = FIELD_EPSILON4;

    memset(s_persons, 0, sizeof(s_persons));
    for (uint8_t p = 0; p < EDGE_MAX_PERSONS; p++) {
        s_persons[p].active = false;
    }

    const float fs = s_sample_rate > 0.0f ? s_sample_rate : 20.0f;
    const float ray_hz = fs / (2.0f * FIELD_NRAYS);
    biquad_bandpass_design(&s_bq_breathing, fs, ray_hz / FIELD_NRAYS, ray_hz);
    biquad_bandpass_design(&s_bq_heartrate, fs, ray_hz, 3.0f * ray_hz);

    const float nyquist = fs / 2.0f;
    biquad_bandpass_design(&s_bq_delta, fs, ray_hz / FIELD_NRAYS, 5.0f * ray_hz);
    biquad_bandpass_design(&s_bq_theta, fs, 5.0f * ray_hz, 10.0f * ray_hz);
    biquad_bandpass_design(&s_bq_alpha, fs, 10.0f * ray_hz, nyquist);
    s_delta_energy = 0.0f;
    s_theta_energy = 0.0f;
    s_alpha_energy = 0.0f;

    for (uint8_t p = 0; p < EDGE_MAX_PERSONS; p++) {
        biquad_bandpass_design(&s_person_bq_br[p], fs, ray_hz / FIELD_NRAYS, ray_hz);
        biquad_bandpass_design(&s_person_bq_hr[p], fs, ray_hz, 3.0f * ray_hz);
    }

    if (s_cfg.tier == 0) {
        ESP_LOGI(TAG, "Edge tier 0: raw passthrough (no DSP task)");
        return ESP_OK;
    }

    const BaseType_t dsp_core = (portNUM_PROCESSORS > 1) ? (BaseType_t)1 : (BaseType_t)0;

    BaseType_t ret = xTaskCreatePinnedToCore(
        edge_task,
        "edge_dsp",
        8192,
        NULL,
        5,
        NULL,
        dsp_core);

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create edge DSP task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Edge DSP task created on core %d (stack=8192, priority=5)",
             (int)dsp_core);
    return ESP_OK;
}
