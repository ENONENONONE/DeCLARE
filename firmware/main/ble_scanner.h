/**
 * @file ble_scanner.h
 * @brief BLE passive scan — spatial shadow mapping via |H(f)|².
 *
 * Each BLE advertisement is one RSSI sample at one field position.
 * N devices × M nodes = N×M crossing rays; attenuation on a ray means
 * something occludes the straight line between node and device.
 * BLE hops 40 channels in the same 2.4 GHz band WiFi CSI measures —
 * same physical channel, amplitude-only (the GF(p) projection).
 */

#ifndef BLE_SCANNER_H
#define BLE_SCANNER_H

/**
 * Initialize BLE passive scanning.
 * Starts Bluedroid, registers a GAP callback, and begins continuous
 * passive scan. Advertisements are batched into 0x09 scan frames
 * and sent via stream_sender on a 200 ms tick.
 *
 * Call after WiFi + stream_sender are initialized.
 *
 * @return 0 on success, -1 on error.
 */
int ble_scanner_init(void);

#endif /* BLE_SCANNER_H */
