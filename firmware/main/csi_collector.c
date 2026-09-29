/**
 * @file csi_collector.c
 * @brief CSI data collection and ADR-018 binary frame serialization.
 *
 * Registers the ESP-IDF WiFi CSI callback and serializes incoming CSI data
 * into the ADR-018 binary frame format for UDP transmission.
 *
 * Declare trimmed version: no device scan (0x09), no channel hopping.
 * Single-channel CSI capture with NDP injection.
 */

#include "csi_collector.h"
#include "nvs_config.h"
#include "stream_sender.h"
#include "edge_processing.h"
#include "mesh_relay.h"

#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <sys/time.h>
#include "esp_timer.h"
#include "sdkconfig.h"
#include <esp_heap_caps.h>

/* Access the global NVS config for MAC filter and channel override. */
extern nvs_config_t g_nvs_config;

/* Defensive capture of node_id before WiFi init can corrupt g_nvs_config. */
static uint8_t s_node_id = 1;
static bool s_node_id_early_set = false;

/* Defensive copy of MAC filter config. */
static uint8_t s_filter_mac[6] = {0};
static bool    s_filter_mac_set = false;
static uint8_t s_last_rx_mac[6] = {0};   /* src MAC from promiscuous cb */
static uint8_t s_frame_buf[CSI_MAX_FRAME_SIZE];

/* Build-time guard: fail if CSI is not enabled in sdkconfig. */
#ifndef CONFIG_ESP_WIFI_CSI_ENABLED
#error "CONFIG_ESP_WIFI_CSI_ENABLED must be set in sdkconfig."
#endif

static const char *TAG = "csi_collector";

static uint32_t s_sequence = 0;
static uint32_t s_cb_count = 0;
static uint32_t s_send_ok = 0;
static uint32_t s_send_fail = 0;
static uint32_t s_rate_skip = 0;

/* AGC jump detection for Fresnel gamma correction. */
static int8_t s_prev_rssi = -127;
#define AGC_JUMP_THRESHOLD 6

/**
 * Minimum interval between UDP sends in microseconds.
 * Default: 20 ms = 50 Hz max send rate.
 */
#define CSI_MIN_SEND_INTERVAL_US  (20 * 1000)
static int64_t s_last_send_us = 0;

/**
 * Minimum interval between processing ANY CSI callback in microseconds.
 * Early gate drops excess callbacks to ~50 Hz.
 */
#define CSI_MIN_PROCESS_INTERVAL_US  (20 * 1000)  /* 50 Hz */
static int64_t s_last_process_us = 0;
static uint32_t s_early_drop = 0;

/* ---- CSI snapshot for USB readout ---- */
static volatile int  s_snap_rssi = 0;
static volatile int  s_snap_channel = 0;
static int8_t        s_snap_iq[CSI_SNAPSHOT_IQ_MAX];
static volatile int  s_snap_iq_len = 0;

/* ---- HT40 sensing-frame injection ---- */
static esp_timer_handle_t s_inject_timer = NULL;

/* ENOMEM backoff for NDP injection. */
static int64_t s_inject_backoff_until_us = 0;
#define INJECT_ENOMEM_COOLDOWN_US  (200 * 1000)

static void inject_timer_cb(void *arg)
{
    (void)arg;
    if (s_inject_backoff_until_us > 0) {
        if (esp_timer_get_time() < s_inject_backoff_until_us) return;
        s_inject_backoff_until_us = 0;
    }
    esp_err_t err = csi_inject_ndp_frame();
    if (err == ESP_ERR_NO_MEM) {
        s_inject_backoff_until_us = esp_timer_get_time() + INJECT_ENOMEM_COOLDOWN_US;
    }
}

/**
 * Serialize CSI data into ADR-018 binary frame format.
 *
 * Layout:
 *   [0..3]   Magic: 0xC5110001 (LE)
 *   [4]      Node ID
 *   [5]      Number of antennas
 *   [6..7]   Number of subcarriers (LE u16)
 *   [8..11]  Frequency MHz (LE u32)
 *   [12..15] Sequence number (LE u32)
 *   [16]     RSSI (i8)
 *   [17]     Noise floor (i8)
 *   [18]     Frame version (2 = source MAC + radio metadata)
 *   [19]     Radio metadata
 *   [20..25] Source MAC (addr2) of captured frame
 *   [26..]   I/Q data
 */
size_t csi_serialize_frame(const wifi_csi_info_t *info, uint8_t *buf, size_t buf_len)
{
    if (info == NULL || buf == NULL || info->buf == NULL) {
        return 0;
    }

    uint8_t n_antennas = 1;
    uint16_t iq_len = (uint16_t)info->len;
    uint16_t n_subcarriers = iq_len / (2 * n_antennas);

    size_t frame_size = CSI_HEADER_SIZE + iq_len;
    if (frame_size > buf_len) {
        ESP_LOGW(TAG, "Buffer too small: need %u, have %u", (unsigned)frame_size, (unsigned)buf_len);
        return 0;
    }

    /* Derive frequency from channel number */
    uint8_t channel = info->rx_ctrl.channel;
    uint32_t freq_mhz;
    if (channel >= 1 && channel <= 13) {
        freq_mhz = 2412 + (channel - 1) * 5;
    } else if (channel == 14) {
        freq_mhz = 2484;
    } else if (channel >= 36 && channel <= 177) {
        freq_mhz = 5000 + channel * 5;
    } else {
        freq_mhz = 0;
    }

    /* Magic (LE) */
    uint32_t magic = CSI_MAGIC;
    memcpy(&buf[0], &magic, 4);

    /* Node ID */
    buf[4] = s_node_id;

    /* Number of antennas */
    buf[5] = n_antennas;

    /* Number of subcarriers (LE u16) */
    memcpy(&buf[6], &n_subcarriers, 2);

    /* Frequency MHz (LE u32) */
    memcpy(&buf[8], &freq_mhz, 4);

    /* Sequence number (LE u32) */
    uint32_t seq = s_sequence++;
    memcpy(&buf[12], &seq, 4);

    /* RSSI (i8) */
    buf[16] = (uint8_t)(int8_t)info->rx_ctrl.rssi;

    /* Noise floor (i8) */
    buf[17] = (uint8_t)(int8_t)info->rx_ctrl.noise_floor;

    /* Frame version 2 = source MAC + radio metadata */
    buf[18] = 2;

    /* Radio metadata byte[19]:
     *   bit0:   cwb       (0=HT20, 1=HT40)
     *   bit1-2: sig_mode  (0=non-HT, 1=HT, 3=VHT)
     *   bit3:   agc_jump  (|RSSI - prev| > 6 dB)
     *   bit4-7: reserved */
    {
#if CONFIG_SOC_WIFI_HE_SUPPORT
        uint8_t cwb = (info->rx_ctrl.second != 0) ? 1 : 0;
        uint8_t sm  = info->rx_ctrl.cur_bb_format & 0x03;
#else
        uint8_t cwb = info->rx_ctrl.cwb & 0x01;
        uint8_t sm  = info->rx_ctrl.sig_mode & 0x03;
#endif
        int8_t  cur_rssi = (int8_t)info->rx_ctrl.rssi;
        uint8_t agc = 0;
        if (s_prev_rssi != -127) {
            int diff = (int)cur_rssi - (int)s_prev_rssi;
            if (diff < 0) diff = -diff;
            if (diff > AGC_JUMP_THRESHOLD) agc = 1;
        }
        s_prev_rssi = cur_rssi;
        buf[19] = cwb | (sm << 1) | (agc << 3);
    }

    memcpy(&buf[20], s_last_rx_mac, 6);

    /* I/Q data */
    memcpy(&buf[CSI_HEADER_SIZE], info->buf, iq_len);

    return frame_size;
}

/**
 * WiFi CSI callback — invoked by ESP-IDF when CSI data is available.
 */
static void wifi_csi_callback(void *ctx, wifi_csi_info_t *info)
{
    (void)ctx;

    /* Early rate gate: drop excess callbacks to ~50 Hz. */
    int64_t now_us = esp_timer_get_time();
    if ((now_us - s_last_process_us) < CSI_MIN_PROCESS_INTERVAL_US) {
        s_early_drop++;
        return;
    }
    s_last_process_us = now_us;

    s_cb_count++;

    if (s_cb_count <= 3 || (s_cb_count % 100) == 0) {
#if CONFIG_SOC_WIFI_HE_SUPPORT
        ESP_LOGI(TAG, "CSI cb #%lu: len=%d rssi=%d ch=%d fmt=%d",
                 (unsigned long)s_cb_count, info->len,
                 info->rx_ctrl.rssi, info->rx_ctrl.channel,
                 info->rx_ctrl.cur_bb_format);
#else
        ESP_LOGI(TAG, "CSI cb #%lu: len=%d rssi=%d ch=%d cwb=%d sig=%d",
                 (unsigned long)s_cb_count, info->len,
                 info->rx_ctrl.rssi, info->rx_ctrl.channel,
                 info->rx_ctrl.cwb, info->rx_ctrl.sig_mode);
#endif
    }

    s_snap_rssi = info->rx_ctrl.rssi;
    s_snap_channel = info->rx_ctrl.channel;
    if (info->buf && info->len > 0) {
        int copy = info->len < CSI_SNAPSHOT_IQ_MAX ? info->len : CSI_SNAPSHOT_IQ_MAX;
        memcpy(s_snap_iq, info->buf, copy);
        s_snap_iq_len = copy;
    }

    size_t frame_len = csi_serialize_frame(info, s_frame_buf, sizeof(s_frame_buf));

    if (frame_len > 0) {
        /* Rate-limit UDP sends to avoid ENOMEM from lwIP pbuf exhaustion. */
        int64_t now = esp_timer_get_time();
        if ((now - s_last_send_us) >= CSI_MIN_SEND_INTERVAL_US) {
            int ret = stream_sender_send(s_frame_buf, frame_len);
            if (ret > 0) {
                s_send_ok++;
                s_last_send_us = now;
            } else {
                s_send_fail++;
                if (mesh_relay_send(s_frame_buf, frame_len) == 0) {
                    s_send_ok++;
                    s_last_send_us = now;
                } else if (s_send_fail <= 5) {
                    ESP_LOGW(TAG, "sendto failed (fail #%lu)", (unsigned long)s_send_fail);
                }
            }
        } else {
            s_rate_skip++;
        }
    }

    /* Enqueue raw I/Q into edge processing ring buffer.
     * Detect NDP source (02:57:42:xx) → fold side (det=-1, carriers 128-255). */
    if (info->buf && info->len > 0) {
        uint8_t is_fold = (s_last_rx_mac[0] == 0x02 &&
                           s_last_rx_mac[1] == 0x57 &&
                           s_last_rx_mac[2] == 0x42) ? 1 : 0;
        edge_enqueue_csi((const uint8_t *)info->buf, (uint16_t)info->len,
                         (int8_t)info->rx_ctrl.rssi, info->rx_ctrl.channel,
                         is_fold);
    }
}

/**
 * Promiscuous mode callback — fires on every received frame.
 * Harvests the source MAC for CSI frame correlation.
 */
static void wifi_promiscuous_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    if (!buf || (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA)) return;
    const wifi_promiscuous_pkt_t *p = (const wifi_promiscuous_pkt_t *)buf;
    if (p->rx_ctrl.sig_len < 16) return;
    memcpy(s_last_rx_mac, p->payload + 10, 6);   /* addr2 = transmitter */
}

void csi_collector_set_node_id(uint8_t node_id)
{
    s_node_id = node_id;
    s_node_id_early_set = true;
    ESP_LOGI(TAG, "Early capture node_id=%u (before WiFi init)",
             (unsigned)node_id);

    /* Also capture MAC filter config now. */
    s_filter_mac_set = (g_nvs_config.filter_mac_set != 0);
    if (s_filter_mac_set) {
        memcpy(s_filter_mac, g_nvs_config.filter_mac, 6);
        ESP_LOGI(TAG, "Early capture filter_mac=%02x:%02x:%02x:%02x:%02x:%02x",
                 s_filter_mac[0], s_filter_mac[1], s_filter_mac[2],
                 s_filter_mac[3], s_filter_mac[4], s_filter_mac[5]);
    }
}

void csi_collector_init(void)
{
    if (!s_node_id_early_set) {
        s_node_id = g_nvs_config.node_id;
        ESP_LOGW(TAG, "Late capture node_id=%u (no early set_node_id call)",
                 (unsigned)s_node_id);
    } else if (g_nvs_config.node_id != s_node_id) {
        ESP_LOGW(TAG, "node_id clobber CONFIRMED: early=%u g_nvs_config=%u "
                 "(using early value)",
                 (unsigned)s_node_id, (unsigned)g_nvs_config.node_id);
    } else {
        ESP_LOGI(TAG, "node_id=%u verified (early capture matches g_nvs_config)",
                 (unsigned)s_node_id);
    }

    /* Determine the CSI channel. */
    uint8_t csi_channel = (uint8_t)CONFIG_CSI_WIFI_CHANNEL;

    if (g_nvs_config.csi_channel > 0) {
        csi_channel = g_nvs_config.csi_channel;
        ESP_LOGI(TAG, "Using NVS channel override: %u", (unsigned)csi_channel);
    } else {
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK && ap_info.primary > 0) {
            csi_channel = ap_info.primary;
            ESP_LOGI(TAG, "Auto-detected AP channel: %u", (unsigned)csi_channel);
        } else {
            ESP_LOGW(TAG, "Could not detect AP channel, using Kconfig default: %u",
                     (unsigned)csi_channel);
        }
    }

    /* Disable WiFi modem sleep for reliable CSI capture. */
    esp_err_t ps_err = esp_wifi_set_ps(WIFI_PS_NONE);
    if (ps_err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_set_ps(WIFI_PS_NONE) failed: %s",
                 esp_err_to_name(ps_err));
    }

    if (g_nvs_config.ftm_mode) {
        /* FTM mode: skip promiscuous, keep STA-mode CSI. */
        ESP_LOGI(TAG, "FTM mode: promiscuous DISABLED (STA-mode CSI)");

#if CONFIG_SOC_WIFI_HE_SUPPORT
        wifi_csi_config_t csi_cfg = {
            .enable = 1,
            .acquire_csi_legacy = 1,
            .acquire_csi_ht20 = 1,
            .acquire_csi_ht40 = 1,
            .acquire_csi_su = 1,
            .acquire_csi_mu = 1,
            .acquire_csi_dcm = 1,
            .acquire_csi_beamformed = 1,
            .val_scale_cfg = 0,
            .dump_ack_en = 0,
        };
#else
        wifi_csi_config_t csi_cfg = {
            .lltf_en = true,
            .htltf_en = true,
            .stbc_htltf2_en = true,
            .ltf_merge_en = true,
            .channel_filter_en = false,
            .manu_scale = false,
            .shift = false,
        };
#endif
        ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_cfg));
        ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_callback, NULL));
        ESP_ERROR_CHECK(esp_wifi_set_csi(true));
    } else {
        /* Promiscuous mode — captures CSI from ALL frames. */
        ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
        ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(wifi_promiscuous_cb));

        wifi_promiscuous_filter_t filt = {
            .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA,
        };
        ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filt));
        ESP_LOGI(TAG, "Promiscuous mode enabled (MGMT+DATA)");
    }

    /* HT40 capture — 128 subcarriers, double angular resolution. */
    {
        esp_err_t bw_err = esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40);
        if (bw_err != ESP_OK) {
            ESP_LOGW(TAG, "esp_wifi_set_bandwidth(HT40) failed: %s",
                     esp_err_to_name(bw_err));
        }
        wifi_second_chan_t second = (csi_channel <= 7) ? WIFI_SECOND_CHAN_ABOVE : WIFI_SECOND_CHAN_BELOW;
        esp_err_t ch_err = esp_wifi_set_channel(csi_channel, second);
        if (ch_err != ESP_OK) {
            ESP_LOGW(TAG, "esp_wifi_set_channel(%u, %s) failed: %s",
                     (unsigned)csi_channel,
                     (second == WIFI_SECOND_CHAN_ABOVE) ? "ABOVE" : "BELOW",
                     esp_err_to_name(ch_err));
        } else {
            ESP_LOGI(TAG, "HT40 channel %u+%s", (unsigned)csi_channel,
                     (second == WIFI_SECOND_CHAN_ABOVE) ? "above" : "below");
        }
    }

    /* MCS0 TX rate for range. */
    {
        esp_err_t rate_err = esp_wifi_config_80211_tx_rate(WIFI_IF_STA, WIFI_PHY_RATE_MCS0_LGI);
        if (rate_err != ESP_OK) {
            ESP_LOGW(TAG, "config_80211_tx_rate(MCS0) failed: %s", esp_err_to_name(rate_err));
        }
    }

#if CONFIG_SOC_WIFI_HE_SUPPORT
    wifi_csi_config_t csi_config;
    memset(&csi_config, 0, sizeof(csi_config));
    csi_config.enable = 1U;
    csi_config.acquire_csi_legacy = 1U;
    csi_config.acquire_csi_ht20 = 1U;
    csi_config.acquire_csi_ht40 = 1U;
    csi_config.acquire_csi_su = 1U;
    csi_config.acquire_csi_mu = 1U;
    csi_config.acquire_csi_dcm = 1U;
    csi_config.acquire_csi_beamformed = 1U;
#if CONFIG_SOC_WIFI_MAC_VERSION_NUM >= 3
    csi_config.acquire_csi_force_lltf = 1U;
    csi_config.acquire_csi_vht = 1U;
    csi_config.acquire_csi_he_stbc_mode = ESP_CSI_ACQUIRE_STBC_SAMPLE_HELTFS;
    csi_config.val_scale_cfg = 0U;
#else
    csi_config.acquire_csi_he_stbc = ESP_CSI_ACQUIRE_STBC_SAMPLE_HELTFS;
    csi_config.val_scale_cfg = 0U;
#endif
    csi_config.dump_ack_en = 0U;
#else
    wifi_csi_config_t csi_config = {
        .lltf_en = true,
        .htltf_en = true,
        .stbc_htltf2_en = true,
        .ltf_merge_en = true,
        .channel_filter_en = false,
        .manu_scale = false,
        .shift = false,
    };
#endif

    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_callback, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));

    /* Start NDP injection stream (20 Hz). Skip in FTM mode. */
    if (!g_nvs_config.ftm_mode) {
        const esp_timer_create_args_t inj_args = {
            .callback = &inject_timer_cb,
            .name = "csi_inject",
        };
        if (s_inject_timer == NULL && esp_timer_create(&inj_args, &s_inject_timer) == ESP_OK) {
            esp_timer_start_periodic(s_inject_timer, 50000);  /* 50 ms = 20 Hz */
            ESP_LOGI(TAG, "NDP injection timer started (20 Hz)");
        } else if (s_inject_timer == NULL) {
            ESP_LOGW(TAG, "failed to create NDP injection timer");
        }
    }

    ESP_LOGI(TAG, "CSI collection initialized (node_id=%u, channel=%u)",
             (unsigned)s_node_id, (unsigned)csi_channel);
}

uint8_t csi_collector_get_node_id(void)
{
    return s_node_id;
}

uint16_t csi_collector_get_pkt_yield_per_sec(void)
{
    static int64_t  s_yield_window_start_us = 0;
    static uint32_t s_yield_window_start_cb = 0;
    static uint16_t s_last_yield            = 0;

    int64_t now = esp_timer_get_time();
    if (s_yield_window_start_us == 0) {
        s_yield_window_start_us = now;
        s_yield_window_start_cb = s_cb_count;
        return 0;
    }
    int64_t elapsed = now - s_yield_window_start_us;
    if (elapsed < 1000000LL) {
        return s_last_yield;
    }
    uint32_t delta = s_cb_count - s_yield_window_start_cb;
    uint64_t per_sec = ((uint64_t)delta * 1000000ULL) / (uint64_t)elapsed;
    if (per_sec > 0xFFFFu) per_sec = 0xFFFFu;
    s_last_yield            = (uint16_t)per_sec;
    s_yield_window_start_us = now;
    s_yield_window_start_cb = s_cb_count;
    return s_last_yield;
}

uint16_t csi_collector_get_send_fail_count(void)
{
    uint32_t f = s_send_fail;
    return (f > 0xFFFFu) ? 0xFFFFu : (uint16_t)f;
}

/* ---- Channel hopping stubs (API declared in header, no-op in Declare) ---- */

void csi_collector_set_hop_table(const uint8_t *channels, uint8_t hop_count, uint32_t dwell_ms)
{
    (void)channels; (void)hop_count; (void)dwell_ms;
    ESP_LOGI(TAG, "Channel hopping not available in Declare firmware");
}

void csi_collector_set_heartbeat(int period, int window)
{
    (void)period; (void)window;
}

void csi_hop_next_channel(void)
{
    /* No-op: single-channel mode. */
}

void csi_collector_start_hop_timer(void)
{
    ESP_LOGI(TAG, "Single-channel mode: hop timer not available");
}

/* ---- NDP frame injection ---- */

esp_err_t csi_inject_ndp_frame(void)
{
    uint8_t ndp_frame[24];
    memset(ndp_frame, 0, sizeof(ndp_frame));

    /* FC: Type=Data(10), Subtype=Null(0100) — data frame uses HT rate */
    ndp_frame[0] = 0x48;
    ndp_frame[1] = 0x00;

    /* Addr1 (DA): broadcast */
    memset(&ndp_frame[4], 0xFF, 6);

    /* Addr2 (TA): LAA MAC 02:57:42:00:00:<node_id> */
    ndp_frame[10] = 0x02; ndp_frame[11] = 0x57; ndp_frame[12] = 0x42;
    ndp_frame[13] = 0x00; ndp_frame[14] = 0x00; ndp_frame[15] = s_node_id;

    /* Addr3 (BSSID): broadcast */
    memset(&ndp_frame[16], 0xFF, 6);

    esp_err_t err = esp_wifi_80211_tx(WIFI_IF_STA, ndp_frame, sizeof(ndp_frame), false);
    if (err != ESP_OK && err != ESP_ERR_NO_MEM) {
        ESP_LOGW(TAG, "NDP inject failed: %s", esp_err_to_name(err));
    }

    return err;
}

void csi_collector_get_snapshot(csi_snapshot_t *out)
{
    out->rssi      = s_snap_rssi;
    out->channel   = s_snap_channel;
    out->cb_count  = s_cb_count;
    out->send_ok   = s_send_ok;
    out->send_fail = s_send_fail;
    out->iq_len    = s_snap_iq_len;
    if (s_snap_iq_len > 0)
        memcpy(out->iq, s_snap_iq, s_snap_iq_len);
}
