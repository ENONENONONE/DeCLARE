/**
 * @file field_observer.c
 * @brief GF(257) full-field observation — direct + fold (det=-1).
 *
 * Direct: CSI amplitudes at positions 1-256.
 * Fold: p -> 257-p (multiply by 3^128 = -1, orientation reversal).
 * 15 orbit permutations via 83^n, all computed (not just crossing).
 * Direct + fold covers all 256 nonzero elements.
 */

#include "field_observer.h"
#include "field_stream.h"
#include "edge_processing.h"
#include "csi_collector.h"
#include "stream_sender.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "field_obs";

static uint16_t s_pow83[FIELD_ORBIT_COUNT];
static bool s_is_qr[257];
static field_orbit_entry_t s_orbits[FIELD_ORBIT_COUNT];
static uint16_t s_seq = 0;

/* Orbit permutation: orbit_map[orb][pos] = (pos * 83^orb) mod 257 */
static uint8_t s_orbit_map[FIELD_ORBIT_COUNT][256];

/* Fold table: fold[pos] = 257-pos (det=-1 reflection) */
static uint16_t s_fold[257];

void field_observer_init(void)
{
    /* pow83 table */
    s_pow83[0] = 1;
    for (int n = 1; n < FIELD_ORBIT_COUNT; n++) {
        s_pow83[n] = (s_pow83[n - 1] * 83) % FIELD_P;
    }

    /* QR classification via dlog parity */
    s_is_qr[0] = false;
    for (int pos = 1; pos <= 256; pos++) {
        uint8_t dl = FIELD_DLOG[pos < FIELD_P ? pos : 0];
        s_is_qr[pos] = (dl % 2 == 0);
    }

    /* Fold table: p -> 257-p = p * 3^128 = p * (-1) */
    s_fold[0] = 0;
    for (int pos = 1; pos <= 256; pos++) {
        s_fold[pos] = FIELD_P - pos;
    }

    /* Orbit tables + permutation maps — ALL 15 orbits */
    for (int orb = 0; orb < FIELD_ORBIT_COUNT; orb++) {
        s_orbits[orb].orbit_n = orb;
        s_orbits[orb].mult = s_pow83[orb];
        uint8_t mult_dlog = FIELD_DLOG[s_pow83[orb]];
        s_orbits[orb].crosses = (mult_dlog % 2 == 1);
        s_orbits[orb].mapped_energy = 0.0f;

        for (int p = 0; p < 256; p++) {
            if (p == 0) {
                s_orbit_map[orb][p] = 0;
            } else {
                uint32_t mapped = ((uint32_t)p * s_pow83[orb]) % FIELD_P;
                s_orbit_map[orb][p] = (mapped < 256) ? (uint8_t)mapped : 0;
            }
        }
    }

    ESP_LOGI(TAG, "Field observer init — full 256 (direct + fold)");
    ESP_LOGI(TAG, "  83=3^15, 15 orbits (8 cross), fold=3^128=-1");
    ESP_LOGI(TAG, "  QR: %d  QNR: %d  fold pairs: %d",
             FIELD_QR_COUNT, FIELD_QR_COUNT, FIELD_QR_COUNT);
}

bool field_observer_tick(field_orbit_pkt_t *pkt)
{
    float variances[EDGE_MAX_SUBCARRIERS];
    edge_get_variances(variances, FIELD_POSITIONS);

    float qr_total = 0.0f, qnr_total = 0.0f;
    float qr_peak = 0.0f, qnr_peak = 0.0f;
    uint8_t qr_peak_pos = 1, qnr_peak_pos = 1;
    float ray_accum[12] = {0};
    uint16_t illum = 0;
    float noise_floor = 0.001f;

    /* Fold space accumulators */
    float fold_total = 0.0f;
    float fold_peak = 0.0f;
    uint8_t fold_peak_pos = 1;
    float fold_ray_accum[12] = {0};
    uint16_t fold_illum = 0;

    for (int pos = 1; pos <= 256; pos++) {
        uint8_t idx = (pos < 256) ? pos : 0;
        float amp = (idx < FIELD_POSITIONS) ? variances[idx] : 0.0f;

        if (amp > noise_floor) illum++;

        uint8_t ray = FIELD_RAY[pos < FIELD_P ? pos : 0];
        if (ray < 12) ray_accum[ray] += amp;

        if (s_is_qr[pos]) {
            qr_total += amp;
            if (amp > qr_peak) { qr_peak = amp; qr_peak_pos = pos; }
        } else {
            qnr_total += amp;
            if (amp > qnr_peak) { qnr_peak = amp; qnr_peak_pos = pos; }
        }

        /* Fold: read amplitude at the reflected position */
        uint16_t fp = s_fold[pos];
        uint8_t fidx = (fp < 256) ? fp : 0;
        float famp = (fidx < FIELD_POSITIONS) ? variances[fidx] : 0.0f;

        if (famp > noise_floor) fold_illum++;

        uint8_t fray = FIELD_RAY[fp < FIELD_P ? fp : 0];
        if (fray < 12) fold_ray_accum[fray] += famp;

        fold_total += famp;
        if (famp > fold_peak) { fold_peak = famp; fold_peak_pos = fp; }
    }

    /* All 15 orbits — energy after permutation */
    uint8_t n_active = 0;
    for (int orb = 1; orb < FIELD_ORBIT_COUNT; orb++) {
        float orb_e = 0.0f;
        for (int p = 1; p < 256; p++) {
            uint8_t mapped = s_orbit_map[orb][p];
            if (mapped > 0 && mapped < 256) {
                orb_e += variances[mapped];
            }
        }
        s_orbits[orb].mapped_energy = orb_e;
        if (orb_e > noise_floor) n_active++;
    }

    /* Normalize ray energies to 0-255 */
    float ray_max = 0.0f, fray_max = 0.0f;
    for (int r = 0; r < 12; r++) {
        if (ray_accum[r] > ray_max) ray_max = ray_accum[r];
        if (fold_ray_accum[r] > fray_max) fray_max = fold_ray_accum[r];
    }
    uint8_t ray_norm[12], fray_norm[12];
    for (int r = 0; r < 12; r++) {
        ray_norm[r] = (ray_max > 0.0f) ? (uint8_t)(ray_accum[r] / ray_max * 255.0f) : 0;
        fray_norm[r] = (fray_max > 0.0f) ? (uint8_t)(fold_ray_accum[r] / fray_max * 255.0f) : 0;
    }

    /* Build packet */
    pkt->magic = FIELD_ORBIT_MAGIC;
    pkt->node_id = csi_collector_get_node_id();
    pkt->flags = 0;
    if (s_orbits[1].mapped_energy > noise_floor) pkt->flags |= 0x01;
    if (qnr_total > qr_total)                    pkt->flags |= 0x02;
    if (fold_total > noise_floor)                 pkt->flags |= 0x04;
    pkt->seq = s_seq++;
    pkt->timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    pkt->qr_energy = qr_total;
    pkt->qnr_energy = qnr_total;
    pkt->crossing_ratio = qnr_total / (qr_total + 1e-9f);
    pkt->peak_qr_pos = qr_peak_pos;
    pkt->peak_qnr_pos = qnr_peak_pos;

    pkt->fold_energy = fold_total;
    pkt->fold_peak_pos = (fold_peak_pos < 256) ? (uint8_t)fold_peak_pos : 0;
    pkt->n_orbits_active = n_active;

    pkt->full_energy = qr_total + qnr_total + fold_total;
    pkt->illum_count = illum;
    pkt->fold_illum_count = fold_illum;

    for (int r = 0; r < 12; r++) {
        pkt->ray_energy[r] = ray_norm[r];
        pkt->fold_ray[r] = fray_norm[r];
    }

    return true;
}

const field_orbit_entry_t *field_observer_get_orbits(void)
{
    return s_orbits;
}

bool field_observer_is_qr(uint8_t pos)
{
    return (pos > 0 && pos <= 256) ? s_is_qr[pos] : false;
}
