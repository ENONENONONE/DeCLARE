/**
 * @file ble_scanner.c
 * @brief BLE passive scan → 0x09 scan frames over UDP.
 *
 * Spatial shadow mapping: each BLE device at a known field position is a
 * point source. RSSI attenuation on a node→device link = something is
 * occluding that ray. The backend (devices module) tracks baseline vs
 * current RSSI per link to compute shadow_db.
 *
 * BLE's 40 channels interleave with WiFi in the 2.4 GHz band — same
 * physical channel. Each hop samples |H(f)|² at one frequency.
 * WiFi CSI gives H(f) dense; BLE gives |H(f)|² sparse across hops.
 */

#include "ble_scanner.h"
#include "csi_collector.h"
#include "stream_sender.h"

#include <string.h>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "ble_scan";

/*
 * 0x09 scan frame wire format (matches devices.Packet in Go):
 *   [0..3]  magic: 0x09, 0x00, 0x11, 0xC5
 *   [4]     node_id
 *   [5]     count (number of entries)
 *   [6..11] reserved (pad to 12-byte header)
 *   then count × 24-byte entries:
 *     [0..5]  MAC address (6 bytes)
 *     [6]     RSSI (int8)
 *     [7..23] reserved
 */
#define SCAN_HDR     12
#define SCAN_ENTRY   24
#define BATCH_MAX    10
#define FLUSH_MS     200

static SemaphoreHandle_t s_lock;
static uint8_t  s_buf[SCAN_HDR + BATCH_MAX * SCAN_ENTRY];
static int      s_count = 0;
static esp_timer_handle_t s_flush_timer;

static void flush_batch(void)
{
    if (s_count == 0) return;
    s_buf[0] = 0x09;
    s_buf[1] = 0x00;
    s_buf[2] = 0x11;
    s_buf[3] = 0xC5;
    s_buf[4] = csi_collector_get_node_id();
    s_buf[5] = (uint8_t)s_count;
    memset(&s_buf[6], 0, 6);
    size_t len = SCAN_HDR + s_count * SCAN_ENTRY;
    stream_sender_send(s_buf, len);
    s_count = 0;
}

static void flush_timer_cb(void *arg)
{
    (void)arg;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    flush_batch();
    xSemaphoreGive(s_lock);
}

static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    if (event != ESP_GAP_BLE_SCAN_RESULT_EVT) return;
    if (param->scan_rst.search_evt != ESP_GAP_SEARCH_INQ_RES_EVT) return;

    uint8_t *bda = param->scan_rst.bda;
    if (bda[0] & 0x01) return;

    int8_t rssi = param->scan_rst.rssi;
    if (rssi <= -100 || rssi >= -5) return;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    int off = SCAN_HDR + s_count * SCAN_ENTRY;
    memcpy(&s_buf[off], bda, 6);
    s_buf[off + 6] = (uint8_t)rssi;
    memset(&s_buf[off + 7], 0, SCAN_ENTRY - 7);
    s_count++;
    if (s_count >= BATCH_MAX) {
        flush_batch();
    }
    xSemaphoreGive(s_lock);
}

int ble_scanner_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) {
        ESP_LOGE(TAG, "mutex create failed");
        return -1;
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "bt controller init: %s", esp_err_to_name(ret));
        return -1;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "bt controller enable: %s", esp_err_to_name(ret));
        return -1;
    }
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "bluedroid init: %s", esp_err_to_name(ret));
        return -1;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "bluedroid enable: %s", esp_err_to_name(ret));
        return -1;
    }

    ret = esp_ble_gap_register_callback(gap_cb);
    if (ret) {
        ESP_LOGE(TAG, "gap register: %s", esp_err_to_name(ret));
        return -1;
    }

    esp_ble_scan_params_t scan_params = {
        .scan_type          = BLE_SCAN_TYPE_PASSIVE,
        .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval      = 160,   /* 100 ms in 0.625 ms units */
        .scan_window        = 80,    /* 50 ms — 50% duty, passive to limit coex */
        .scan_duplicate     = BLE_SCAN_DUPLICATE_DISABLE,
    };
    ret = esp_ble_gap_set_scan_params(&scan_params);
    if (ret) {
        ESP_LOGE(TAG, "scan params: %s", esp_err_to_name(ret));
        return -1;
    }

    ret = esp_ble_gap_start_scanning(0);
    if (ret) {
        ESP_LOGE(TAG, "start scan: %s", esp_err_to_name(ret));
        return -1;
    }

    const esp_timer_create_args_t timer_args = {
        .callback = flush_timer_cb,
        .name = "ble_flush",
    };
    ret = esp_timer_create(&timer_args, &s_flush_timer);
    if (ret) {
        ESP_LOGE(TAG, "timer create: %s", esp_err_to_name(ret));
        return -1;
    }
    esp_timer_start_periodic(s_flush_timer, FLUSH_MS * 1000);

    ESP_LOGI(TAG, "BLE passive scan started — batch %d, flush %d ms",
             BATCH_MAX, FLUSH_MS);
    return 0;
}
