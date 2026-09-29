#include "crystal_touch.h"
#include "csi_collector.h"
#include "edge_processing.h"
#include "stream_sender.h"
#include "nvs_config.h"
#include "declare_lcd.h"
#include "soc/soc_caps.h"
#if SOC_TOUCH_SENSOR_SUPPORTED
#include "driver/touch_sensor.h"
#endif
#include "driver/usb_serial_jtag.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

static const char *TAG = "crystal_touch";

#if SOC_TOUCH_SENSOR_SUPPORTED
static const touch_pad_t s_pads[CRYSTAL_TOUCH_NUM_PADS] = {
    TOUCH_PAD_NUM1, TOUCH_PAD_NUM2, TOUCH_PAD_NUM4,
    TOUCH_PAD_NUM5, TOUCH_PAD_NUM6, TOUCH_PAD_NUM7,
};
static const uint8_t s_gpios[CRYSTAL_TOUCH_NUM_PADS] = {1, 2, 4, 5, 6, 7};
#endif

static bool s_inited = false;
static TaskHandle_t s_sweep_handle = NULL;
static bool s_usb_driver_installed = false;
static bool s_uart0_installed = false;

typedef void (*reply_fn_t)(const char *str);

static void usb_send(const char *str)
{
    size_t len = strlen(str);
    usb_serial_jtag_write_bytes(str, len, pdMS_TO_TICKS(100));
}

static void uart0_send(const char *str)
{
    size_t len = strlen(str);
    uart_write_bytes(UART_NUM_0, str, len);
}

static void crystal_touch_stream_cb(void *arg)
{
    (void)arg;
    if (!s_inited) return;
    crystal_touch_sweep_t sweep;
    crystal_touch_get_sweep(&sweep);

    /* 8-byte header + 6 pads * 4 bytes (int32 delta) = 32 bytes */
    uint8_t pkt[32];
    uint32_t magic = EDGE_TOUCH_MAGIC;
    memcpy(&pkt[0], &magic, 4);
    pkt[4] = csi_collector_get_node_id();
    pkt[5] = CRYSTAL_TOUCH_NUM_PADS;
    uint16_t ts = (uint16_t)((sweep.timestamp_us / 1000) & 0xFFFF);
    memcpy(&pkt[6], &ts, 2);
    for (int i = 0; i < CRYSTAL_TOUCH_NUM_PADS && i < 6; i++) {
        int32_t d = sweep.pads[i].delta;
        memcpy(&pkt[8 + i * 4], &d, 4);
    }
    stream_sender_send(pkt, 32);
}

void crystal_touch_init(void)
{
    if (s_inited) return;

#if SOC_TOUCH_SENSOR_SUPPORTED
    touch_pad_init();
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);

    for (int i = 0; i < CRYSTAL_TOUCH_NUM_PADS; i++) {
        touch_pad_config(s_pads[i]);
    }

    touch_pad_set_charge_discharge_times(500);
    touch_pad_set_measurement_interval(0x1000);
    touch_pad_fsm_start();
    vTaskDelay(pdMS_TO_TICKS(200));

    s_inited = true;
    ESP_LOGI(TAG, "touch sensor init: %d pads on GPIO 1-7", CRYSTAL_TOUCH_NUM_PADS);

    static esp_timer_handle_t s_touch_timer = NULL;
    if (!s_touch_timer) {
        const esp_timer_create_args_t args = {
            .callback = &crystal_touch_stream_cb,
            .name = "touch_stream",
        };
        if (esp_timer_create(&args, &s_touch_timer) == ESP_OK) {
            esp_timer_start_periodic(s_touch_timer, 1000000);
            ESP_LOGI(TAG, "Touch UDP stream started (1 Hz, magic 0x%08X)", EDGE_TOUCH_MAGIC);
        }
    }
#else
    s_inited = true;
    ESP_LOGI(TAG, "no touch sensor on this SoC, skipping");
#endif
}

void crystal_touch_get_sweep(crystal_touch_sweep_t *out)
{
    out->timestamp_us = esp_timer_get_time();
    for (int i = 0; i < CRYSTAL_TOUCH_NUM_PADS; i++) {
#if SOC_TOUCH_SENSOR_SUPPORTED
        out->pads[i].gpio = s_gpios[i];
        touch_pad_read_raw_data(s_pads[i], &out->pads[i].raw);
        touch_pad_read_benchmark(s_pads[i], &out->pads[i].benchmark);
        out->pads[i].delta = (int32_t)out->pads[i].raw - (int32_t)out->pads[i].benchmark;
#else
        out->pads[i].gpio = 0;
        out->pads[i].raw = 0;
        out->pads[i].benchmark = 0;
        out->pads[i].delta = 0;
#endif
    }
}

static void crystal_touch_sweep_task(void *arg)
{
    crystal_touch_sweep_t sweep;
    while (1) {
        crystal_touch_get_sweep(&sweep);
        ESP_LOGI(TAG, "t=%lld", sweep.timestamp_us);
        for (int i = 0; i < CRYSTAL_TOUCH_NUM_PADS; i++) {
            ESP_LOGI(TAG, "  GPIO%2d: raw=%6lu  bench=%6lu  delta=%+ld",
                     sweep.pads[i].gpio,
                     (unsigned long)sweep.pads[i].raw,
                     (unsigned long)sweep.pads[i].benchmark,
                     (long)sweep.pads[i].delta);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void crystal_touch_start_task(void)
{
    crystal_touch_init();
    if (!s_sweep_handle) {
        xTaskCreate(crystal_touch_sweep_task, "crystal_touch", 4096, NULL, 5, &s_sweep_handle);
    }
}

/* ── declaration: declare.html's {"cmd":"declare",...} → NVS csi_cfg obs_* + display ── */

static bool json_get_str(const char *json, const char *key, char *out, size_t n)
{
    char pat[24];
    snprintf(pat, sizeof(pat), "\"%s\":\"", key);
    const char *p = strstr(json, pat);
    if (!p || n == 0) return false;
    p += strlen(pat);
    size_t i = 0;
    while (*p && *p != '"' && i < n - 1) out[i++] = *p++;
    out[i] = '\0';
    return true;
}

static bool json_get_int(const char *json, const char *key, int *out)
{
    char pat[24];
    snprintf(pat, sizeof(pat), "\"%s\":", key);
    const char *p = strstr(json, pat);
    if (!p) return false;
    *out = atoi(p + strlen(pat));
    return true;
}

/* The 5x7 font is ASCII, so the display shows each genome glyph as its 0-F code: the hash nibbles. */
static void declared_codes(lcd_declared_t *d)
{
    size_t i;
    for (i = 0; d->hash[i] && i < sizeof(d->genome) - 1; i++)
        d->genome[i] = (char)toupper((unsigned char)d->hash[i]);
    d->genome[i] = '\0';
}

/* obs_hash is written last: restore only trusts a declaration whose hash made it to flash. */
static esp_err_t declared_save(const lcd_declared_t *d, const char *genome)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open("csi_cfg", NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_set_str(h, "obs_name", d->name);
    if (err == ESP_OK) err = nvs_set_u16(h, "obs_pos", d->position);
    if (err == ESP_OK) err = nvs_set_u16(h, "obs_elem", d->element);
    if (err == ESP_OK) err = nvs_set_u16(h, "obs_inv", d->inverse);
    if (err == ESP_OK) err = nvs_set_u8(h, "obs_ray", d->ray);
    if (err == ESP_OK) err = nvs_set_u8(h, "obs_dlog", d->dlog);
    if (err == ESP_OK) err = nvs_set_u8(h, "obs_layers", d->layers_measured);
    if (err == ESP_OK) err = nvs_set_str(h, "obs_genome", genome);
    if (err == ESP_OK) err = nvs_set_str(h, "obs_hash", d->hash);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

void crystal_touch_restore_declared(void)
{
    nvs_handle_t h;
    if (nvs_open("csi_cfg", NVS_READONLY, &h) != ESP_OK) return;
    lcd_declared_t *d = calloc(1, sizeof(*d));
    size_t len = d ? sizeof(d->hash) : 0;
    if (d && nvs_get_str(h, "obs_hash", d->hash, &len) == ESP_OK && strlen(d->hash) == 64) {
        len = sizeof(d->name);
        nvs_get_str(h, "obs_name", d->name, &len);
        nvs_get_u16(h, "obs_pos", &d->position);
        nvs_get_u16(h, "obs_elem", &d->element);
        nvs_get_u16(h, "obs_inv", &d->inverse);
        nvs_get_u8(h, "obs_ray", &d->ray);
        nvs_get_u8(h, "obs_dlog", &d->dlog);
        nvs_get_u8(h, "obs_layers", &d->layers_measured);
        declared_codes(d);
        declare_lcd_set_declared(d);
        ESP_LOGI(TAG, "Declared observer restored: %s pos %u", d->name, (unsigned)d->position);
    }
    free(d);
    nvs_close(h);
}

static void handle_touch_cmd_reply(const char *json, reply_fn_t reply)
{
    char cmd[32] = {0};
    const char *p = strstr(json, "\"cmd\"");
    if (p) {
        p = strchr(p + 5, '"');
        if (p) { p++; int i = 0; while (*p && *p != '"' && i < 31) cmd[i++] = *p++; cmd[i] = 0; }
    }

    char buf[512];

    if (strcmp(cmd, "touch_read") == 0) {
        crystal_touch_init();
        crystal_touch_sweep_t sweep;
        crystal_touch_get_sweep(&sweep);

        int pos = snprintf(buf, sizeof(buf),
            "{\"ok\":true,\"cmd\":\"touch_read\",\"t\":%lld,\"pads\":[",
            sweep.timestamp_us);
        for (int i = 0; i < CRYSTAL_TOUCH_NUM_PADS; i++) {
            if (i) buf[pos++] = ',';
            pos += snprintf(buf + pos, sizeof(buf) - pos,
                "{\"gpio\":%d,\"raw\":%lu,\"bench\":%lu,\"delta\":%ld}",
                sweep.pads[i].gpio,
                (unsigned long)sweep.pads[i].raw,
                (unsigned long)sweep.pads[i].benchmark,
                (long)sweep.pads[i].delta);
        }
        pos += snprintf(buf + pos, sizeof(buf) - pos, "]}\n");
        reply(buf);

    } else if (strcmp(cmd, "touch_start") == 0) {
        crystal_touch_start_task();
        reply("{\"ok\":true,\"cmd\":\"touch_start\"}\n");

    } else if (strcmp(cmd, "touch_stop") == 0) {
        if (s_sweep_handle) { vTaskDelete(s_sweep_handle); s_sweep_handle = NULL; }
        reply("{\"ok\":true,\"cmd\":\"touch_stop\"}\n");

    } else if (strcmp(cmd, "csi_read") == 0) {
        csi_snapshot_t snap;
        csi_collector_get_snapshot(&snap);

        int pos = snprintf(buf, sizeof(buf),
            "{\"ok\":true,\"cmd\":\"csi_read\",\"rssi\":%d,\"ch\":%d,"
            "\"cb\":%lu,\"ok_tx\":%lu,\"fail\":%lu,\"iq_len\":%d,\"amp\":[",
            snap.rssi, snap.channel,
            (unsigned long)snap.cb_count, (unsigned long)snap.send_ok,
            (unsigned long)snap.send_fail, snap.iq_len);

        int n_sc = snap.iq_len / 2;
        if (n_sc > 64) n_sc = 64;
        for (int i = 0; i < n_sc; i++) {
            float im = snap.iq[i * 2];
            float re = snap.iq[i * 2 + 1];
            int amp = (int)sqrtf(im * im + re * re);
            if (i) buf[pos++] = ',';
            pos += snprintf(buf + pos, sizeof(buf) - pos, "%d", amp);
        }
        pos += snprintf(buf + pos, sizeof(buf) - pos, "]}\n");
        reply(buf);

    } else if (strcmp(cmd, "vitals") == 0) {
        edge_vitals_pkt_t pkt;
        bool ok = edge_get_vitals(&pkt);
        if (ok) {
            snprintf(buf, sizeof(buf),
                "{\"ok\":true,\"cmd\":\"vitals\",\"node\":%u,\"flags\":%u,"
                "\"br\":%.2f,\"hr\":%.2f,\"rssi\":%d,\"n_persons\":%u,"
                "\"motion\":%.4f,\"presence\":%.4f,\"temp\":%d,\"ts\":%lu}\n",
                pkt.node_id, pkt.flags,
                pkt.breathing_rate / 100.0f, pkt.heartrate / 10000.0f,
                pkt.rssi, pkt.n_persons,
                pkt.motion_energy, pkt.presence_score,
                pkt.die_temp_c, (unsigned long)pkt.timestamp_ms);
        } else {
            snprintf(buf, sizeof(buf),
                "{\"ok\":false,\"cmd\":\"vitals\",\"err\":\"no data yet\"}\n");
        }
        reply(buf);

    } else if (strcmp(cmd, "bio") == 0) {
        csi_snapshot_t snap;
        csi_collector_get_snapshot(&snap);
        edge_vitals_pkt_t pkt;
        bool has_vitals = edge_get_vitals(&pkt);

        char big[2048];
        int pos = snprintf(big, sizeof(big),
            "{\"ok\":true,\"cmd\":\"bio\",\"rssi\":%d,\"ch\":%d,"
            "\"cb\":%lu,\"iq_len\":%d",
            snap.rssi, snap.channel,
            (unsigned long)snap.cb_count, snap.iq_len);

        if (has_vitals) {
            pos += snprintf(big + pos, sizeof(big) - pos,
                ",\"br\":%.2f,\"hr\":%.2f,\"presence\":%.4f,"
                "\"motion\":%.4f,\"temp\":%d,\"n_persons\":%u",
                pkt.breathing_rate / 100.0f, pkt.heartrate / 10000.0f,
                pkt.presence_score, pkt.motion_energy,
                pkt.die_temp_c, pkt.n_persons);
        }

        pos += snprintf(big + pos, sizeof(big) - pos, ",\"iq\":[");
        int n_sc = snap.iq_len / 2;
        if (n_sc > 64) n_sc = 64;
        for (int i = 0; i < n_sc; i++) {
            if (i) big[pos++] = ',';
            pos += snprintf(big + pos, sizeof(big) - pos,
                "[%d,%d]", snap.iq[i * 2], snap.iq[i * 2 + 1]);
        }
        pos += snprintf(big + pos, sizeof(big) - pos, "]}\n");
        esp_log_level_set("*", ESP_LOG_NONE);
        reply(big);
        esp_log_level_set("*", ESP_LOG_INFO);

    } else if (strcmp(cmd, "brainwave") == 0) {
        float delta, theta, alpha;
        edge_get_brainwave_bands(&delta, &theta, &alpha);

        csi_snapshot_t snap;
        csi_collector_get_snapshot(&snap);
        int n_sc = snap.iq_len / 2;
        if (n_sc > 64) n_sc = 64;

        char big[2048];
        int pos = snprintf(big, sizeof(big),
            "{\"ok\":true,\"cmd\":\"brainwave\","
            "\"delta\":%.6f,\"theta\":%.6f,\"alpha\":%.6f,"
            "\"rssi\":%d,\"iq\":[",
            delta, theta, alpha, snap.rssi);

        for (int i = 0; i < n_sc; i++) {
            if (i) big[pos++] = ',';
            pos += snprintf(big + pos, sizeof(big) - pos,
                "[%d,%d]", snap.iq[i * 2], snap.iq[i * 2 + 1]);
        }
        pos += snprintf(big + pos, sizeof(big) - pos, "]}\n");
        reply(big);

    } else if (strcmp(cmd, "set_config") == 0) {
        nvs_handle_t h;
        esp_err_t err = nvs_open("csi_cfg", NVS_READWRITE, &h);
        if (err != ESP_OK) {
            snprintf(buf, sizeof(buf), "{\"ok\":false,\"cmd\":\"set_config\",\"err\":\"nvs_open: %s\"}\n",
                     esp_err_to_name(err));
            reply(buf);
        } else {
            int wrote = 0;
            const char *key;
            char val[128];

            key = "\"ssid\"";
            p = strstr(json, key);
            if (p) {
                p = strchr(p + strlen(key), '"');
                if (p) { p++; int i = 0; while (*p && *p != '"' && i < 127) val[i++] = *p++; val[i] = 0;
                    nvs_set_str(h, "ssid", val); wrote++; }
            }

            key = "\"password\"";
            p = strstr(json, key);
            if (p) {
                p = strchr(p + strlen(key), '"');
                if (p) { p++; int i = 0; while (*p && *p != '"' && i < 127) val[i++] = *p++; val[i] = 0;
                    nvs_set_str(h, "password", val); wrote++; }
            }

            key = "\"target_ip\"";
            p = strstr(json, key);
            if (p) {
                p = strchr(p + strlen(key), '"');
                if (p) { p++; int i = 0; while (*p && *p != '"' && i < 127) val[i++] = *p++; val[i] = 0;
                    nvs_set_str(h, "target_ip", val); wrote++; }
            }

            key = "\"target_port\"";
            p = strstr(json, key);
            if (p) {
                p += strlen(key);
                while (*p && (*p == ':' || *p == ' ')) p++;
                uint16_t pv = (uint16_t)atoi(p);
                if (pv > 0) { nvs_set_u16(h, "target_port", pv); wrote++; }
            }

            key = "\"node_id\"";
            p = strstr(json, key);
            if (p) {
                p += strlen(key);
                while (*p && (*p == ':' || *p == ' ')) p++;
                uint8_t nv = (uint8_t)atoi(p);
                nvs_set_u8(h, "node_id", nv); wrote++;
            }

            nvs_commit(h);
            nvs_close(h);

            snprintf(buf, sizeof(buf), "{\"ok\":true,\"cmd\":\"set_config\",\"wrote\":%d}\n", wrote);
            reply(buf);

            if (wrote > 0) {
                ESP_LOGI(TAG, "Config written (%d keys), rebooting in 1s...", wrote);
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_restart();
            }
        }

    } else if (strcmp(cmd, "declare") == 0) {
        lcd_declared_t *d = calloc(1, sizeof(*d));
        char *genome = calloc(1, 260);
        int pos = 0, v = 0;
        if (!d || !genome) {
            reply("{\"ok\":false,\"cmd\":\"declare\",\"err\":\"no memory\"}\n");
        } else if (!json_get_int(json, "pos", &pos) || pos < 1 || pos > 256 ||
                   !json_get_str(json, "hash", d->hash, sizeof(d->hash)) || strlen(d->hash) != 64) {
            reply("{\"ok\":false,\"cmd\":\"declare\",\"err\":\"need pos 1..256 and a 64-hex hash\"}\n");
        } else {
            json_get_str(json, "name", d->name, sizeof(d->name));
            json_get_str(json, "genome", genome, 260);
            d->position = (uint16_t)pos;
            if (json_get_int(json, "element", &v)) d->element = (uint16_t)v;
            if (json_get_int(json, "inv", &v))     d->inverse = (uint16_t)v;
            if (json_get_int(json, "ray", &v))     d->ray = (uint8_t)v;
            if (json_get_int(json, "dlog", &v))    d->dlog = (uint8_t)v;
            if (json_get_int(json, "layers", &v))  d->layers_measured = (uint8_t)v;
            declared_codes(d);
            esp_err_t err = declared_save(d, genome);
            if (err != ESP_OK) {
                snprintf(buf, sizeof(buf), "{\"ok\":false,\"cmd\":\"declare\",\"err\":\"nvs: %s\"}\n",
                         esp_err_to_name(err));
            } else {
                declare_lcd_set_declared(d);
                declare_lcd_show_declared();
                ESP_LOGI(TAG, "Declared observer stored: %s pos %u", d->name, (unsigned)d->position);
                snprintf(buf, sizeof(buf),
                         "{\"ok\":true,\"cmd\":\"declare\",\"name\":\"%s\",\"pos\":%u,\"hash\":\"%s\"}\n",
                         d->name, (unsigned)d->position, d->hash);
            }
            reply(buf);
        }
        free(genome);
        free(d);

    } else if (strcmp(cmd, "get_config") == 0) {
        extern nvs_config_t g_nvs_config;
        int n = snprintf(buf, sizeof(buf),
            "{\"ok\":true,\"cmd\":\"get_config\",\"ssid\":\"%s\","
            "\"target_ip\":\"%s\",\"target_port\":%u,\"node_id\":%u",
            g_nvs_config.wifi_ssid, g_nvs_config.target_ip,
            g_nvs_config.target_port, g_nvs_config.node_id);
        nvs_handle_t h;
        if (n > 0 && n < (int)sizeof(buf) && nvs_open("csi_cfg", NVS_READONLY, &h) == ESP_OK) {
            char oname[32] = {0}, ohash[65] = {0};
            size_t ln = sizeof(oname), lh = sizeof(ohash);
            uint16_t opos = 0;
            if (nvs_get_str(h, "obs_hash", ohash, &lh) == ESP_OK) {
                nvs_get_str(h, "obs_name", oname, &ln);
                nvs_get_u16(h, "obs_pos", &opos);
                n += snprintf(buf + n, sizeof(buf) - n,
                              ",\"obs_name\":\"%s\",\"obs_pos\":%u,\"obs_hash\":\"%s\"",
                              oname, (unsigned)opos, ohash);
            }
            nvs_close(h);
        }
        if (n > 0 && n < (int)sizeof(buf))
            snprintf(buf + n, sizeof(buf) - n, "}\n");
        reply(buf);

    } else {
        snprintf(buf, sizeof(buf), "{\"ok\":false,\"err\":\"unknown: %s\"}\n", cmd);
        reply(buf);
    }
}

static void usb_serial_task(void *arg)
{
    char line[512];
    int pos = 0;
    uint8_t c;

    ESP_LOGI(TAG, "USB serial reader started");

    while (1) {
        int n = usb_serial_jtag_read_bytes(&c, 1, pdMS_TO_TICKS(50));
        if (n <= 0) continue;

        if (c == '\n' || c == '\r') {
            if (pos > 0) {
                line[pos] = '\0';
                if (strstr(line, "\"cmd\""))
                    handle_touch_cmd_reply(line, usb_send);
                pos = 0;
            }
            continue;
        }
        if (pos < (int)sizeof(line) - 1)
            line[pos++] = (char)c;
    }
}

static void uart0_serial_task(void *arg)
{
    char line[512];
    int pos = 0;
    uint8_t c;

    ESP_LOGI(TAG, "UART0 serial reader started");

    while (1) {
        int n = uart_read_bytes(UART_NUM_0, &c, 1, pdMS_TO_TICKS(50));
        if (n <= 0) continue;

        if (c == '\n' || c == '\r') {
            if (pos > 0) {
                line[pos] = '\0';
                if (strstr(line, "\"cmd\""))
                    handle_touch_cmd_reply(line, uart0_send);
                pos = 0;
            }
            continue;
        }
        if (pos < (int)sizeof(line) - 1)
            line[pos++] = (char)c;
    }
}

void crystal_touch_start_usb_reader(void)
{
    if (!s_usb_driver_installed) {
        usb_serial_jtag_driver_config_t cfg = {
            .rx_buffer_size = 1024,
            .tx_buffer_size = 1024,
        };
        esp_err_t err = usb_serial_jtag_driver_install(&cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "USB JTAG driver install failed: %s", esp_err_to_name(err));
        } else {
            s_usb_driver_installed = true;
            xTaskCreate(usb_serial_task, "usb_touch", 4096, NULL, 5, NULL);
        }
    } else {
        xTaskCreate(usb_serial_task, "usb_touch", 4096, NULL, 5, NULL);
    }

    if (!s_uart0_installed) {
        uart_config_t uart_cfg = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
        esp_err_t err = uart_driver_install(UART_NUM_0, 2048, 0, 0, NULL, 0);
        if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
            if (err == ESP_OK)
                uart_param_config(UART_NUM_0, &uart_cfg);
            s_uart0_installed = true;
            xTaskCreate(uart0_serial_task, "uart0_touch", 8192, NULL, 5, NULL);
            ESP_LOGI(TAG, "UART0 command reader started");
        } else {
            ESP_LOGE(TAG, "UART0 driver install failed: %s", esp_err_to_name(err));
        }
    }
}

void crystal_touch_dispatch(const char *json)
{
    handle_touch_cmd_reply(json, usb_send);
}
