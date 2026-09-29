/**
 * @file main.c
 * @brief Declare — ESP32-C6 biometric CSI sensor.
 *
 * Trimmed single-device firmware: WiFi CSI capture, vitals extraction
 * (HR, BR, presence, motion, brainwave bands, amplitude baseline),
 * and UDP streaming. ESP-NOW mesh relay for multi-hop swarm sensing.
 *
 * All credentials come from NVS (provisioned via serial).
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_netif_sntp.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "driver/gpio.h"

#include "csi_collector.h"
#include "stream_sender.h"
#include "nvs_config.h"
#include "edge_processing.h"
#include "crystal_touch.h"
#include "field_observer.h"
#include "declare_lcd.h"
#include "ble_scanner.h"
#include "optical_probe.h"
#include "mesh_relay.h"
#include "esp_spiffs.h"

static const char *TAG = "declare";

/* Runtime configuration (loaded from NVS or Kconfig defaults). */
nvs_config_t g_nvs_config;

/* Event group bits */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
#define MAX_RETRY 10

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "WiFi disconnected: reason=%u",
                 disc ? (unsigned)disc->reason : 0);
        s_retry_num++;
        if (s_retry_num % 10 == 0) {
            ESP_LOGW(TAG, "WiFi retry %d", s_retry_num);
        }
        vTaskDelay(pdMS_TO_TICKS(s_retry_num > 30 ? 5000 : 1000));
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    strncpy((char *)wifi_config.sta.ssid, g_nvs_config.wifi_ssid,
            sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, g_nvs_config.wifi_password,
            sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.bssid_set = false;

    /* If password is empty, use open auth */
    if (strlen((char *)wifi_config.sta.password) == 0) {
        wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    /* Per-node distinct MAC from node_id */
    {
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        mac[5] = csi_collector_get_node_id();
        esp_err_t mret = esp_wifi_set_mac(WIFI_IF_STA, mac);
        ESP_LOGI(TAG, "STA MAC -> %02x:%02x:%02x:%02x:%02x:%02x (%s)",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                 mret == ESP_OK ? "set" : esp_err_to_name(mret));
    }

    esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40);
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);  /* disable PM for reliable CSI */

    ESP_LOGI(TAG, "WiFi STA connecting to SSID: %s", g_nvs_config.wifi_ssid);

    /* Wait for connection with 15s timeout */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE, pdMS_TO_TICKS(15000));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to WiFi");
    } else {
        ESP_LOGW(TAG, "WiFi connection timed out after 15s — continuing without WiFi");
    }
}

void app_main(void)
{
    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Load runtime config (NVS overrides Kconfig/env defaults) */
    nvs_config_load(&g_nvs_config);

    /* Capture node_id BEFORE wifi_init_sta() can corrupt g_nvs_config. */
    csi_collector_set_node_id(g_nvs_config.node_id);

    ESP_LOGI(TAG, "Declare CSI Sensor — Node ID: %d", g_nvs_config.node_id);
    ESP_LOGI(TAG, "  WiFi SSID: %s", g_nvs_config.wifi_ssid);
    ESP_LOGI(TAG, "  Target:    %s:%d", g_nvs_config.target_ip, g_nvs_config.target_port);

    if (strlen(g_nvs_config.wifi_ssid) == 0) {
        ESP_LOGW(TAG, "========================================");
        ESP_LOGW(TAG, "  WiFi not configured.");
        ESP_LOGW(TAG, "  Open declare.html, connect USB,");
        ESP_LOGW(TAG, "  and use NODE SETUP to configure.");
        ESP_LOGW(TAG, "========================================");
    }

    declare_lcd_init();
    declare_lcd_show_splash();
    crystal_touch_restore_declared();

    crystal_touch_init();
    crystal_touch_start_usb_reader();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Initialize WiFi STA (skipped if no SSID configured) */
    if (strlen(g_nvs_config.wifi_ssid) > 0) {
        wifi_init_sta();
    }

    /* ESP-NOW mesh relay — swarm discovery + multi-hop data forwarding */
    if (strlen(g_nvs_config.wifi_ssid) > 0) {
        if (mesh_relay_init(g_nvs_config.node_id, g_nvs_config.zone_name) == 0) {
            mesh_relay_start(g_nvs_config.swarm_heartbeat_sec);
        }
    }

    /* Initialize UDP sender (primary = WG tunnel, fallback = direct LAN) */
    if (stream_sender_init_with(g_nvs_config.target_ip, g_nvs_config.target_port) != 0) {
        ESP_LOGE(TAG, "Failed to initialize UDP sender");
        return;
    }
    if (g_nvs_config.fallback_ip[0] != '\0') {
        stream_sender_set_fallback(g_nvs_config.fallback_ip, g_nvs_config.fallback_port);
    }

    /* Initialize CSI collection (requires WiFi) */
    if (strlen(g_nvs_config.wifi_ssid) > 0) {
        csi_collector_init();
    }

    /* Die temperature sensor */
    edge_init_temperature();

    /* Edge processing pipeline */
    {
        edge_config_t edge_cfg = {
            .tier = g_nvs_config.edge_tier > 0 ? g_nvs_config.edge_tier : 1,
            .presence_thresh = g_nvs_config.presence_thresh,
            .fall_thresh = g_nvs_config.fall_thresh > 0.0f ? g_nvs_config.fall_thresh : 2.0f,
            .vital_window = g_nvs_config.vital_window > 0 ? g_nvs_config.vital_window : EDGE_PHASE_HISTORY_LEN,
            .vital_interval_ms = g_nvs_config.vital_interval_ms > 0 ? g_nvs_config.vital_interval_ms : 1000,
            .top_k_count = g_nvs_config.top_k_count > 0 ? g_nvs_config.top_k_count : EDGE_TOP_K,
            .power_duty = g_nvs_config.power_duty > 0 ? g_nvs_config.power_duty : 100,
        };
        esp_err_t edge_ret = edge_processing_init(&edge_cfg);
        if (edge_ret != ESP_OK) {
            ESP_LOGW(TAG, "Edge processing init failed: %s", esp_err_to_name(edge_ret));
        } else {
            ESP_LOGI(TAG, "Edge processing active (tier=%u)", edge_cfg.tier);
        }
    }

    /* Mount SPIFFS — field.txt self-referencing document */
    {
        esp_vfs_spiffs_conf_t spiffs_cfg = {
            .base_path = "/field",
            .partition_label = "storage",
            .max_files = 2,
            .format_if_mount_failed = false,
        };
        esp_err_t sp_ret = esp_vfs_spiffs_register(&spiffs_cfg);
        if (sp_ret == ESP_OK) {
            size_t total = 0, used = 0;
            esp_spiffs_info("storage", &total, &used);
            ESP_LOGI(TAG, "SPIFFS mounted: %zu/%zu bytes", used, total);
        } else {
            ESP_LOGW(TAG, "SPIFFS mount failed: %s", esp_err_to_name(sp_ret));
        }
    }

    /* Field observer — GF(257) orbit crossing via PRAYER (83 = 3^15, QNR) */
    field_observer_init();

    /* Optical probe — reflective IR + transmitter on GPIO9/GPIO8 */
    optical_probe_start();

    /* BLE passive scan — spatial shadow mapping via |H(f)|² */
    if (g_nvs_config.ble_scan) {
        if (ble_scanner_init() == 0) {
            ESP_LOGI(TAG, "BLE shadow scan active");
        } else {
            ESP_LOGW(TAG, "BLE scan init failed — continuing without");
        }
    }

    ESP_LOGI(TAG, "CSI streaming active -> %s:%d (edge_tier=%u, mesh_hops=%u)",
             g_nvs_config.target_ip, g_nvs_config.target_port,
             g_nvs_config.edge_tier, mesh_relay_hops());

    /* Hold splash screen until first CSI data arrives */
    ESP_LOGI(TAG, "Splash screen displayed — waiting for CSI data");

    /* Main loop — field observation tick every 1s, LCD update every 200ms */
    int lcd_div = 0;
    bool splash_active = true;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(200));

        lcd_div++;
        if (lcd_div >= 5) {
            lcd_div = 0;
            field_orbit_pkt_t orbit_pkt;
            if (field_observer_tick(&orbit_pkt)) {
                stream_sender_send((const uint8_t *)&orbit_pkt, sizeof(orbit_pkt));
            }
        }

        edge_vitals_pkt_t vit;
        memset(&vit, 0, sizeof(vit));
        float delta = 0, theta = 0, alpha = 0;
        float hr = 0, br = 0, motion = 0;
        int rssi = 0, channel = 0;
        if (edge_get_vitals(&vit)) {
            hr = (float)vit.heartrate / 10000.0f;
            br = (float)vit.breathing_rate / 100.0f;
            motion = vit.motion_energy;
            rssi = vit.rssi;
        }
        edge_get_brainwave_bands(&delta, &theta, &alpha);

        /* Stay on splash until we have actual CSI data */
        if (splash_active) {
            if (rssi != 0 || hr > 0.5f) {
                splash_active = false;
                declare_lcd_force_redraw();
                ESP_LOGI(TAG, "CSI data received — leaving splash");
            }
            continue;
        }

        declare_lcd_update(hr, br, delta, theta, alpha,
                           rssi, channel, g_nvs_config.node_id,
                           motion, 0);
    }
}
