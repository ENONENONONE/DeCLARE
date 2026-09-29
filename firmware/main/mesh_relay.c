/**
 * @file mesh_relay.c
 * @brief ESP-NOW mesh relay — swarm heartbeats + fragmented data forwarding.
 *
 * Protocol (ESP-NOW payload, max 250 bytes):
 *
 *   HEARTBEAT (0xDE 0x48):
 *     [0]  0xDE magic  [1] 0x48 'H'  [2] node_id  [3] hops_to_agg
 *     [4..9] factory MAC  [10] flags  [11] peer_count
 *     [12..27] zone_name (16 bytes, null-padded)
 *
 *   DATA RELAY (0xDE 0x44):
 *     [0]  0xDE magic  [1] 0x44 'D'  [2] originator_node_id  [3] ttl
 *     [4]  seq  [5] frag_info: [7:4]=index [3:0]=total
 *     [6..] payload chunk (up to 244 bytes)
 */

#include "mesh_relay.h"
#include "stream_sender.h"

#include <string.h>
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "mesh_relay";

/* Protocol constants */
#define MESH_MAGIC       0xDE
#define MESH_TYPE_HEART  0x48
#define MESH_TYPE_DATA   0x44
#define MESH_HDR_LEN     6
#define MESH_FRAG_MAX    (ESP_NOW_MAX_DATA_LEN - MESH_HDR_LEN)  /* 244 */
#define MESH_DEFAULT_TTL 4

/* Peer aging */
#define PEER_TIMEOUT_US  (90LL * 1000000)

/* Reassembly */
#define REASM_SLOTS      4
#define REASM_MAX_LEN    600
#define REASM_TIMEOUT_US (500LL * 1000)

/* Dedup ring */
#define DEDUP_SIZE       32

static const uint8_t s_bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* ---- State ---- */
static uint8_t  s_node_id;
static uint8_t  s_mac[6];
static char     s_zone[MESH_ZONE_LEN];
static uint8_t  s_hops_to_agg = 255;
static uint8_t  s_seq;
static bool     s_initialized;

/* Peer table */
static mesh_peer_t s_peers[MESH_MAX_PEERS];
static int         s_peer_count;
static SemaphoreHandle_t s_peer_mtx;

/* Heartbeat timer */
static esp_timer_handle_t s_hb_timer;

/* Reassembly buffers */
typedef struct {
    uint8_t  originator;
    uint8_t  seq;
    uint8_t  total_frags;
    uint16_t frag_mask;
    int64_t  started_us;
    uint16_t frag_lens[16];
    uint8_t  buf[REASM_MAX_LEN];
} reasm_slot_t;

static reasm_slot_t s_reasm[REASM_SLOTS];

/* Dedup */
static struct { uint8_t node; uint8_t seq; } s_dedup[DEDUP_SIZE];
static int s_dedup_idx;

/* ---- Helpers ---- */

static bool check_wifi_connected(void)
{
    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!sta) return false;
    esp_netif_ip_info_t ip;
    return esp_netif_get_ip_info(sta, &ip) == ESP_OK && ip.ip.addr != 0;
}

static void recalc_hops(void)
{
    if (check_wifi_connected()) {
        s_hops_to_agg = 0;
        return;
    }
    uint8_t best = 255;
    for (int i = 0; i < s_peer_count; i++) {
        if (s_peers[i].hops_to_agg < best)
            best = s_peers[i].hops_to_agg;
    }
    s_hops_to_agg = (best < 254) ? best + 1 : 255;
}

static bool is_duplicate(uint8_t node, uint8_t seq)
{
    for (int i = 0; i < DEDUP_SIZE; i++) {
        if (s_dedup[i].node == node && s_dedup[i].seq == seq)
            return true;
    }
    return false;
}

static void mark_seen(uint8_t node, uint8_t seq)
{
    s_dedup[s_dedup_idx].node = node;
    s_dedup[s_dedup_idx].seq  = seq;
    s_dedup_idx = (s_dedup_idx + 1) % DEDUP_SIZE;
}

/* ---- Peer table ---- */

static void age_peers(void)
{
    int64_t now = esp_timer_get_time();
    int dst = 0;
    for (int i = 0; i < s_peer_count; i++) {
        if ((now - s_peers[i].last_seen_us) < PEER_TIMEOUT_US) {
            if (dst != i) s_peers[dst] = s_peers[i];
            dst++;
        }
    }
    s_peer_count = dst;
}

static void update_peer(uint8_t node_id, const uint8_t *mac,
                         uint8_t hops, int8_t rssi,
                         const char *zone, uint8_t flags)
{
    if (node_id == s_node_id) return;

    xSemaphoreTake(s_peer_mtx, portMAX_DELAY);
    age_peers();

    int idx = -1;
    for (int i = 0; i < s_peer_count; i++) {
        if (s_peers[i].node_id == node_id) { idx = i; break; }
    }

    if (idx < 0) {
        if (s_peer_count >= MESH_MAX_PEERS) {
            /* Evict oldest */
            int oldest = 0;
            for (int i = 1; i < s_peer_count; i++) {
                if (s_peers[i].last_seen_us < s_peers[oldest].last_seen_us)
                    oldest = i;
            }
            idx = oldest;
        } else {
            idx = s_peer_count++;
        }
        ESP_LOGI(TAG, "New peer: node=%u hops=%u rssi=%d", node_id, hops, rssi);
    }

    mesh_peer_t *p = &s_peers[idx];
    p->node_id      = node_id;
    memcpy(p->mac, mac, 6);
    p->hops_to_agg  = hops;
    p->rssi         = rssi;
    p->flags        = flags;
    p->last_seen_us = esp_timer_get_time();
    if (zone) {
        strncpy(p->zone, zone, MESH_ZONE_LEN - 1);
        p->zone[MESH_ZONE_LEN - 1] = '\0';
    }

    recalc_hops();
    xSemaphoreGive(s_peer_mtx);
}

/* ---- Fragmented send ---- */

static int send_fragments(const uint8_t *data, size_t len,
                           uint8_t originator, uint8_t ttl)
{
    uint8_t total = (uint8_t)((len + MESH_FRAG_MAX - 1) / MESH_FRAG_MAX);
    if (total > 15 || total == 0) return -1;

    uint8_t seq = s_seq++;
    uint8_t pkt[ESP_NOW_MAX_DATA_LEN];

    for (uint8_t i = 0; i < total; i++) {
        pkt[0] = MESH_MAGIC;
        pkt[1] = MESH_TYPE_DATA;
        pkt[2] = originator;
        pkt[3] = ttl;
        pkt[4] = seq;
        pkt[5] = (uint8_t)((i << 4) | total);

        size_t offset = (size_t)i * MESH_FRAG_MAX;
        size_t chunk  = len - offset;
        if (chunk > MESH_FRAG_MAX) chunk = MESH_FRAG_MAX;
        memcpy(&pkt[MESH_HDR_LEN], data + offset, chunk);

        esp_err_t err = esp_now_send(s_bcast, pkt, MESH_HDR_LEN + chunk);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "esp_now_send frag %u/%u failed: %s",
                     i, total, esp_err_to_name(err));
            return -1;
        }
    }
    return 0;
}

/* ---- Reassembly ---- */

static reasm_slot_t *find_reasm(uint8_t originator, uint8_t seq, uint8_t total)
{
    int64_t now = esp_timer_get_time();

    /* Look for existing slot */
    for (int i = 0; i < REASM_SLOTS; i++) {
        if (s_reasm[i].frag_mask != 0 &&
            s_reasm[i].originator == originator &&
            s_reasm[i].seq == seq)
            return &s_reasm[i];
    }

    /* Allocate new slot (prefer empty, then oldest) */
    int best = -1;
    int64_t oldest = INT64_MAX;
    for (int i = 0; i < REASM_SLOTS; i++) {
        if (s_reasm[i].frag_mask == 0) { best = i; break; }
        if ((now - s_reasm[i].started_us) > REASM_TIMEOUT_US) {
            s_reasm[i].frag_mask = 0;
            best = i;
            break;
        }
        if (s_reasm[i].started_us < oldest) {
            oldest = s_reasm[i].started_us;
            best = i;
        }
    }
    if (best < 0) best = 0;

    reasm_slot_t *s = &s_reasm[best];
    memset(s, 0, sizeof(*s));
    s->originator  = originator;
    s->seq         = seq;
    s->total_frags = total;
    s->started_us  = now;
    return s;
}

/* ---- ESP-NOW callbacks ---- */

static void handle_heartbeat(const uint8_t *src_mac, const uint8_t *data,
                              int len, int8_t rssi)
{
    if (len < 28) return;
    update_peer(data[2], data + 4, data[3], rssi,
                (const char *)(data + 12), data[10]);
}

static void handle_data_relay(const uint8_t *data, int len)
{
    if (len < 7) return;

    uint8_t originator = data[2];
    uint8_t ttl        = data[3];
    uint8_t seq        = data[4];
    uint8_t frag_idx   = data[5] >> 4;
    uint8_t frag_total = data[5] & 0x0F;

    if (originator == s_node_id) return;
    if (frag_idx >= frag_total || frag_total > 15) return;
    if (is_duplicate(originator, seq) && frag_idx == 0) return;

    reasm_slot_t *slot = find_reasm(originator, seq, frag_total);
    if (!slot) return;

    size_t offset    = (size_t)frag_idx * MESH_FRAG_MAX;
    size_t chunk_len = (size_t)(len - MESH_HDR_LEN);
    if (offset + chunk_len > REASM_MAX_LEN) return;

    memcpy(slot->buf + offset, data + MESH_HDR_LEN, chunk_len);
    slot->frag_mask |= (uint16_t)(1 << frag_idx);
    slot->frag_lens[frag_idx] = (uint16_t)chunk_len;

    uint16_t needed = (uint16_t)((1 << frag_total) - 1);
    if ((slot->frag_mask & needed) != needed) return;

    /* Complete — calculate total length */
    size_t total_len = 0;
    for (uint8_t i = 0; i < frag_total; i++) {
        if (i == frag_total - 1)
            total_len = (size_t)i * MESH_FRAG_MAX + slot->frag_lens[i];
    }

    mark_seen(originator, seq);
    slot->frag_mask = 0;

    ESP_LOGI(TAG, "Reassembled %u bytes from node %u (seq %u)",
             (unsigned)total_len, originator, seq);

    if (check_wifi_connected()) {
        stream_sender_send(slot->buf, total_len);
    } else if (ttl > 1) {
        send_fragments(slot->buf, total_len, originator, ttl - 1);
    }
}

static void recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (len < 2 || data[0] != MESH_MAGIC) return;
    int8_t rssi = info->rx_ctrl ? (int8_t)info->rx_ctrl->rssi : 0;
    switch (data[1]) {
    case MESH_TYPE_HEART:
        handle_heartbeat(info->src_addr, data, len, rssi);
        break;
    case MESH_TYPE_DATA:
        handle_data_relay(data, len);
        break;
    }
}

static void send_cb(const uint8_t *mac, esp_now_send_status_t status)
{
    (void)mac;
    (void)status;
}

/* ---- Heartbeat ---- */

static void heartbeat_cb(void *arg)
{
    (void)arg;

    recalc_hops();

    uint8_t pkt[28];
    memset(pkt, 0, sizeof(pkt));
    pkt[0] = MESH_MAGIC;
    pkt[1] = MESH_TYPE_HEART;
    pkt[2] = s_node_id;
    pkt[3] = s_hops_to_agg;
    memcpy(&pkt[4], s_mac, 6);
    pkt[10] = (check_wifi_connected() ? 0x01 : 0x00) | 0x02;
    pkt[11] = (uint8_t)s_peer_count;
    strncpy((char *)&pkt[12], s_zone, MESH_ZONE_LEN - 1);

    esp_now_send(s_bcast, pkt, sizeof(pkt));
}

/* ---- Public API ---- */

int mesh_relay_init(uint8_t node_id, const char *zone)
{
    if (s_initialized) return 0;

    s_node_id = node_id;
    esp_read_mac(s_mac, ESP_MAC_WIFI_STA);
    strncpy(s_zone, zone ? zone : "default", MESH_ZONE_LEN - 1);
    s_zone[MESH_ZONE_LEN - 1] = '\0';

    s_peer_mtx = xSemaphoreCreateMutex();
    if (!s_peer_mtx) return -1;

    esp_err_t err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_init failed: %s", esp_err_to_name(err));
        return -1;
    }

    esp_now_register_recv_cb(recv_cb);
    esp_now_register_send_cb(send_cb);

    /* Add broadcast peer */
    esp_now_peer_info_t bcast = {
        .channel = 0,
        .encrypt = false,
        .ifidx   = WIFI_IF_STA,
    };
    memcpy(bcast.peer_addr, s_bcast, 6);
    err = esp_now_add_peer(&bcast);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGE(TAG, "add broadcast peer failed: %s", esp_err_to_name(err));
        esp_now_deinit();
        return -1;
    }

    recalc_hops();
    s_initialized = true;

    ESP_LOGI(TAG, "Mesh relay initialized (node=%u zone=%s hops=%u)",
             s_node_id, s_zone, s_hops_to_agg);
    return 0;
}

void mesh_relay_start(uint16_t heartbeat_sec)
{
    if (!s_initialized || heartbeat_sec == 0) return;

    const esp_timer_create_args_t args = {
        .callback = heartbeat_cb,
        .name     = "mesh_hb",
    };
    if (esp_timer_create(&args, &s_hb_timer) == ESP_OK) {
        esp_timer_start_periodic(s_hb_timer,
                                 (uint64_t)heartbeat_sec * 1000000ULL);
        ESP_LOGI(TAG, "Heartbeat started (%u sec)", heartbeat_sec);
    }

    heartbeat_cb(NULL);
}

int mesh_relay_send(const uint8_t *data, size_t len)
{
    if (!s_initialized) return -1;
    if (s_hops_to_agg == 0) return -1;   /* we have direct WiFi */
    if (s_peer_count == 0) return -1;     /* no peers to relay through */

    return send_fragments(data, len, s_node_id, MESH_DEFAULT_TTL);
}

int mesh_relay_get_peers(mesh_peer_t *out, int max)
{
    if (!s_initialized || !out) return 0;
    xSemaphoreTake(s_peer_mtx, portMAX_DELAY);
    age_peers();
    int n = s_peer_count < max ? s_peer_count : max;
    memcpy(out, s_peers, n * sizeof(mesh_peer_t));
    xSemaphoreGive(s_peer_mtx);
    return n;
}

uint8_t mesh_relay_hops(void)
{
    return s_hops_to_agg;
}

int mesh_relay_peer_count(void)
{
    return s_peer_count;
}
