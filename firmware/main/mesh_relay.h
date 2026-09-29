/**
 * @file mesh_relay.h
 * @brief ESP-NOW mesh relay — swarm discovery and multi-hop data forwarding.
 *
 * Nodes broadcast heartbeats to discover neighbors, build a peer table,
 * and relay CSI data through the mesh when the aggregator is unreachable
 * via direct WiFi. Fragmentation handles payloads exceeding ESP-NOW's
 * 250-byte limit.
 */

#ifndef MESH_RELAY_H
#define MESH_RELAY_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define MESH_MAX_PEERS  16
#define MESH_MAX_HOPS   4
#define MESH_ZONE_LEN   16

typedef struct {
    uint8_t  mac[6];
    uint8_t  node_id;
    uint8_t  hops_to_agg;
    char     zone[MESH_ZONE_LEN];
    int64_t  last_seen_us;
    int8_t   rssi;
    uint8_t  flags;         /* bit0: wifi_connected, bit1: relay_capable */
} mesh_peer_t;

/**
 * Initialize ESP-NOW mesh relay.
 * WiFi must be started before calling this.
 */
int mesh_relay_init(uint8_t node_id, const char *zone);

/**
 * Start heartbeat broadcasts and relay processing.
 * @param heartbeat_sec  Interval between heartbeats (0 = disabled).
 */
void mesh_relay_start(uint16_t heartbeat_sec);

/**
 * Send data through the mesh (fragment + broadcast via ESP-NOW).
 * Used when direct UDP to the aggregator fails.
 * @return 0 on success, -1 if no relay path exists.
 */
int mesh_relay_send(const uint8_t *data, size_t len);

/**
 * Copy the peer table into caller's buffer.
 * @return Number of peers copied.
 */
int mesh_relay_get_peers(mesh_peer_t *out, int max);

/** This node's current hop count to aggregator (0 = direct, 255 = none). */
uint8_t mesh_relay_hops(void);

/** Number of active mesh peers. */
int mesh_relay_peer_count(void);

#endif /* MESH_RELAY_H */
