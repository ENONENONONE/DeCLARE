/**
 * @file field_observer.h
 * @brief GF(257) field observation — full 256-position space.
 *
 * Direct space: R=18 walk covers 128 QR positions.
 * Fold (det=-1): multiply by 3^128 = -1, maps p -> 257-p.
 * Perpendicular space: the folded 128 QNR positions.
 * 83 = 3^15 (PRAYER): 15 orbit permutations, 8 cross the partition.
 * Direct + fold = all 256 positions observed.
 */

#ifndef FIELD_OBSERVER_H
#define FIELD_OBSERVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define FIELD_ORBIT_MAGIC   0xC511F01D  /* FOLD magic */
#define FIELD_ORBIT_COUNT   16          /* orbit 0 (identity) + 15 permutations */
#define FIELD_QR_COUNT      128         /* |QR subgroup| = (p-1)/2 */
#define FIELD_POSITIONS     256         /* |GF(257)*| */

/**
 * Field observation packet (64 bytes).
 * Carries direct + fold (perpendicular) space energy.
 */
typedef struct __attribute__((packed)) {
    uint32_t magic;            /* FIELD_ORBIT_MAGIC */
    uint8_t  node_id;
    uint8_t  flags;            /* b0: crossing active, b1: qnr>qr, b2: fold active */
    uint16_t seq;
    uint32_t timestamp_ms;

    /* Direct space (QR walk) */
    float    qr_energy;
    float    qnr_energy;
    float    crossing_ratio;   /* qnr / (qr + 1e-9) */
    uint8_t  peak_qr_pos;
    uint8_t  peak_qnr_pos;

    /* Fold space (perpendicular, det=-1) */
    float    fold_energy;      /* total energy from folded positions */
    uint8_t  fold_peak_pos;    /* peak position in fold space */
    uint8_t  n_orbits_active;  /* orbits with above-noise energy (of 15) */

    /* Combined */
    float    full_energy;      /* direct + fold total */
    uint16_t illum_count;      /* positions above noise (direct) */
    uint16_t fold_illum_count; /* positions above noise (fold) */

    /* Per-ray energy: direct and fold */
    uint8_t  ray_energy[12];   /* direct space per-ray (0-255) */
    uint8_t  fold_ray[12];     /* fold space per-ray (0-255) */
} field_orbit_pkt_t;

_Static_assert(sizeof(field_orbit_pkt_t) == 64, "orbit packet must be 64 bytes");

typedef struct {
    uint8_t  orbit_n;
    bool     crosses;          /* true if odd orbit (QNR multiplier) */
    uint16_t mult;             /* 83^n mod 257 */
    float    mapped_energy;
} field_orbit_entry_t;

void field_observer_init(void);
bool field_observer_tick(field_orbit_pkt_t *pkt);
const field_orbit_entry_t *field_observer_get_orbits(void);
bool field_observer_is_qr(uint8_t pos);

#endif /* FIELD_OBSERVER_H */
