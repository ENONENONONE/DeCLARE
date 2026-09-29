/* optical_probe.c -- three-sensor optical cluster:
 *   GPIO8  (ADC1_CH7) = IR transmitter feedback
 *   GPIO9  (ADC1_CH8) = reflective optical proximity sensor
 *   GPIO14 (ADC2_CH3) = IR receiver
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "optical_probe.h"
#include "stream_sender.h"
#include "csi_collector.h"
#include "edge_processing.h"
#include <string.h>

static const char *TAG = "OPTICAL";
#define OPT_THRESH 3500

volatile int g_optical_raw = -1;
volatile int g_optical_present = 0;
volatile int g_ir_tx_raw = -1;
volatile int g_ir_rx_raw = -1;

static int64_t s_last_send_us = 0;

static void optical_send(void)
{
    int64_t now = esp_timer_get_time();
    if ((now - s_last_send_us) < 1000000LL) return;
    s_last_send_us = now;

    uint8_t pkt[20];
    uint32_t magic = EDGE_OPTICAL_MAGIC;
    memcpy(&pkt[0], &magic, 4);
    pkt[4] = csi_collector_get_node_id();
    pkt[5] = (uint8_t)g_optical_present;
    int16_t raw = (int16_t)g_optical_raw;
    memcpy(&pkt[6], &raw, 2);
    int16_t ir_tx = (int16_t)g_ir_tx_raw;
    memcpy(&pkt[8], &ir_tx, 2);
    int16_t ir_rx = (int16_t)g_ir_rx_raw;
    memcpy(&pkt[10], &ir_rx, 2);
    uint16_t thresh = OPT_THRESH;
    memcpy(&pkt[12], &thresh, 2);
    memset(&pkt[14], 0, 2);
    uint32_t ts = (uint32_t)(now / 1000);
    memcpy(&pkt[16], &ts, 4);
    stream_sender_send(pkt, 20);
}

static void optical_probe_task(void *arg) {
    adc_oneshot_unit_handle_t adc1 = NULL;
    adc_oneshot_unit_init_cfg_t uc1 = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&uc1, &adc1);

    adc_oneshot_unit_handle_t adc2 = NULL;
    adc_oneshot_unit_init_cfg_t uc2 = { .unit_id = ADC_UNIT_2 };
    adc_oneshot_new_unit(&uc2, &adc2);

    adc_oneshot_chan_cfg_t cc = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
    adc_oneshot_config_channel(adc1, ADC_CHANNEL_7, &cc);   /* GPIO8 = IR TX */
    adc_oneshot_config_channel(adc1, ADC_CHANNEL_8, &cc);   /* GPIO9 = reflective */
    adc_oneshot_config_channel(adc2, ADC_CHANNEL_3, &cc);   /* GPIO14 = IR RX */

    int raw = 0, ema = 4095;
    int ir_tx_raw = 0, ir_tx_ema = 4095;
    int ir_rx_raw = 0, ir_rx_ema = 4095;

    ESP_LOGI(TAG, "optical cluster: GPIO9=reflective GPIO8=IR_TX GPIO14=IR_RX");
    while (1) {
        if (adc_oneshot_read(adc1, ADC_CHANNEL_8, &raw) == ESP_OK) {
            ema = (ema * 3 + raw) / 4;
            g_optical_raw = ema;
            g_optical_present = (ema < OPT_THRESH) ? 1 : 0;
        }
        if (adc_oneshot_read(adc1, ADC_CHANNEL_7, &ir_tx_raw) == ESP_OK) {
            ir_tx_ema = (ir_tx_ema * 3 + ir_tx_raw) / 4;
            g_ir_tx_raw = ir_tx_ema;
        }
        if (adc_oneshot_read(adc2, ADC_CHANNEL_3, &ir_rx_raw) == ESP_OK) {
            ir_rx_ema = (ir_rx_ema * 3 + ir_rx_raw) / 4;
            g_ir_rx_raw = ir_rx_ema;
        }
        optical_send();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void optical_probe_start(void) { xTaskCreate(optical_probe_task, "optical_probe", 4096, NULL, 4, NULL); }
