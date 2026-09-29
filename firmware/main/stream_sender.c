/**
 * @file stream_sender.c
 * @brief UDP stream sender with dual-target fallback and offline ring buffer.
 *
 * Primary target = WireGuard virtual IP (tunnel).
 * Fallback target = direct LAN IP.
 * On EHOSTUNREACH/ENETUNREACH on primary, switches to fallback.
 * On successful primary send, switches back.
 * Ring buffer holds packets when both are unreachable.
 */

#include "stream_sender.h"

#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "sdkconfig.h"

static const char *TAG = "stream_sender";

static int s_sock = -1;
static struct sockaddr_in s_primary;
static struct sockaddr_in s_fallback;
static bool s_has_fallback = false;
static bool s_using_fallback = false;

static int64_t s_backoff_until_us = 0;
#define ENOMEM_COOLDOWN_MS  100
#define ENOMEM_LOG_INTERVAL 50
static uint32_t s_enomem_suppressed = 0;

#define PRIMARY_RETRY_MS    5000
static int64_t s_primary_retry_at = 0;

/* --- Ring buffer for offline queueing --- */
#define RING_SLOTS     128
#define RING_PKT_MAX   512

typedef struct {
    uint16_t len;
    uint8_t  data[RING_PKT_MAX];
} ring_slot_t;

static ring_slot_t s_ring[RING_SLOTS];
static uint16_t s_ring_head = 0;
static uint16_t s_ring_tail = 0;
static uint16_t s_ring_count = 0;
static uint32_t s_ring_dropped = 0;
static bool     s_ring_active = false;

static void ring_enqueue(const uint8_t *data, size_t len)
{
    if (len > RING_PKT_MAX) len = RING_PKT_MAX;

    if (s_ring_count == RING_SLOTS) {
        s_ring_tail = (s_ring_tail + 1) % RING_SLOTS;
        s_ring_count--;
        s_ring_dropped++;
    }

    s_ring[s_ring_head].len = (uint16_t)len;
    memcpy(s_ring[s_ring_head].data, data, len);
    s_ring_head = (s_ring_head + 1) % RING_SLOTS;
    s_ring_count++;

    if (!s_ring_active) {
        s_ring_active = true;
        ESP_LOGI(TAG, "Buffering started (both targets unreachable)");
    }
}

static struct sockaddr_in *current_dest(void)
{
    return s_using_fallback ? &s_fallback : &s_primary;
}

static int try_send(const uint8_t *data, size_t len, struct sockaddr_in *dest)
{
    return sendto(s_sock, data, len, 0,
                  (struct sockaddr *)dest, sizeof(*dest));
}

static void ring_drain_all(void)
{
    int flushed = 0;
    while (s_ring_count > 0) {
        ring_slot_t *slot = &s_ring[s_ring_tail];
        int sent = try_send(slot->data, slot->len, current_dest());
        if (sent < 0) break;
        s_ring_tail = (s_ring_tail + 1) % RING_SLOTS;
        s_ring_count--;
        flushed++;
    }
    if (flushed > 0 && s_ring_count == 0) {
        ESP_LOGI(TAG, "Buffer drained: %d sent, %lu dropped",
                 flushed, (unsigned long)s_ring_dropped);
        s_ring_active = false;
        s_ring_dropped = 0;
    }
}

static int make_addr(struct sockaddr_in *addr, const char *ip, uint16_t port)
{
    memset(addr, 0, sizeof(*addr));
    addr->sin_family = AF_INET;
    addr->sin_port = htons(port);
    if (inet_pton(AF_INET, ip, &addr->sin_addr) <= 0) {
        ESP_LOGE(TAG, "Invalid IP: %s", ip);
        return -1;
    }
    return 0;
}

static int sender_init_internal(const char *ip, uint16_t port)
{
    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_sock < 0) {
        ESP_LOGE(TAG, "socket failed: errno %d", errno);
        return -1;
    }

    if (make_addr(&s_primary, ip, port) != 0) {
        close(s_sock);
        s_sock = -1;
        return -1;
    }

    s_using_fallback = false;
    ESP_LOGI(TAG, "Primary target: %s:%d", ip, port);
    return 0;
}

int stream_sender_init(void)
{
    return sender_init_internal(CONFIG_CSI_TARGET_IP, CONFIG_CSI_TARGET_PORT);
}

int stream_sender_init_with(const char *ip, uint16_t port)
{
    return sender_init_internal(ip, port);
}

void stream_sender_set_fallback(const char *ip, uint16_t port)
{
    if (ip == NULL || ip[0] == '\0') {
        s_has_fallback = false;
        return;
    }
    if (make_addr(&s_fallback, ip, port) == 0) {
        s_has_fallback = true;
        ESP_LOGI(TAG, "Fallback target: %s:%d", ip, port);
    }
}

int stream_sender_send(const uint8_t *data, size_t len)
{
    if (s_sock < 0) return -1;

    if (s_backoff_until_us > 0) {
        int64_t now = esp_timer_get_time();
        if (now < s_backoff_until_us) {
            s_enomem_suppressed++;
            if ((s_enomem_suppressed % ENOMEM_LOG_INTERVAL) == 1) {
                ESP_LOGW(TAG, "sendto suppressed (ENOMEM backoff, %lu dropped)",
                         (unsigned long)s_enomem_suppressed);
            }
            return -1;
        }
        ESP_LOGI(TAG, "ENOMEM backoff expired (%lu suppressed)",
                 (unsigned long)s_enomem_suppressed);
        s_backoff_until_us = 0;
        s_enomem_suppressed = 0;
    }

    /* If on fallback, periodically retry primary */
    if (s_using_fallback && s_primary_retry_at > 0) {
        int64_t now = esp_timer_get_time();
        if (now >= s_primary_retry_at) {
            int sent = try_send(data, len, &s_primary);
            if (sent >= 0) {
                ESP_LOGI(TAG, "Primary reachable — switching back");
                s_using_fallback = false;
                s_primary_retry_at = 0;
                if (s_ring_count > 0) ring_drain_all();
                return sent;
            }
            s_primary_retry_at = now + (int64_t)PRIMARY_RETRY_MS * 1000;
        }
    }

    int sent = try_send(data, len, current_dest());
    if (sent >= 0) {
        if (s_ring_count > 0) ring_drain_all();
        return sent;
    }

    if (errno == ENOMEM) {
        s_backoff_until_us = esp_timer_get_time() +
                             (int64_t)ENOMEM_COOLDOWN_MS * 1000;
        ESP_LOGW(TAG, "sendto ENOMEM — backing off %d ms", ENOMEM_COOLDOWN_MS);
        return -1;
    }

    if (errno == EHOSTUNREACH || errno == ENETUNREACH || errno == ENETDOWN) {
        if (!s_using_fallback && s_has_fallback) {
            /* Primary failed — try fallback */
            sent = try_send(data, len, &s_fallback);
            if (sent >= 0) {
                ESP_LOGW(TAG, "Primary unreachable — using fallback");
                s_using_fallback = true;
                s_primary_retry_at = esp_timer_get_time() +
                                     (int64_t)PRIMARY_RETRY_MS * 1000;
                return sent;
            }
        }
        ring_enqueue(data, len);
    } else {
        ESP_LOGW(TAG, "sendto failed: errno %d", errno);
    }

    return -1;
}

void stream_sender_deinit(void)
{
    if (s_sock >= 0) {
        close(s_sock);
        s_sock = -1;
    }
}
