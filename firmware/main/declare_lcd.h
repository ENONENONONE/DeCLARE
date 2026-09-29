#ifndef DECLARE_LCD_H
#define DECLARE_LCD_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    char     name[32];
    uint16_t position;   /* 1..256 */
    uint16_t element;    /* 1..256 */
    uint8_t  ray;
    uint16_t inverse;    /* 1..256 */
    uint8_t  dlog;
    uint8_t  layers_measured;
    char     hash[65];
    char     genome[65];
    bool     active;
} lcd_declared_t;

#if CONFIG_IDF_TARGET_ESP32C6

esp_err_t declare_lcd_init(void);
void declare_lcd_show_splash(void);
void declare_lcd_show_declared(void);
void declare_lcd_set_declared(const lcd_declared_t *d);
const lcd_declared_t *declare_lcd_get_declared(void);
void declare_lcd_update(float hr, float br,
                        float delta, float theta, float alpha,
                        int rssi, int channel, int node_id,
                        float motion, int csi_rate);
void declare_lcd_update_body(const int8_t *iq, int iq_len);
void declare_lcd_force_redraw(void);

#else

static inline esp_err_t declare_lcd_init(void) { return ESP_OK; }
static inline void declare_lcd_show_splash(void) {}
static inline void declare_lcd_show_declared(void) {}
static inline void declare_lcd_set_declared(const lcd_declared_t *d) { (void)d; }
static inline const lcd_declared_t *declare_lcd_get_declared(void) { return NULL; }
static inline void declare_lcd_update(float hr, float br,
                        float delta, float theta, float alpha,
                        int rssi, int channel, int node_id,
                        float motion, int csi_rate) {
    (void)hr;(void)br;(void)delta;(void)theta;(void)alpha;
    (void)rssi;(void)channel;(void)node_id;(void)motion;(void)csi_rate;
}
static inline void declare_lcd_update_body(const int8_t *iq, int iq_len) { (void)iq;(void)iq_len; }
static inline void declare_lcd_force_redraw(void) {}

#endif

#endif
