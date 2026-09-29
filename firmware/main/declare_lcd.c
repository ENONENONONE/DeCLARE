#include "declare_lcd.h"
#if !CONFIG_IDF_TARGET_ESP32C6
/* LCD hardware is C6-only; stubs live in declare_lcd.h */
#else
#include "splash_screen.h"
#include "declared_screen.h"
#include "field_stream.h"
#include "field_observer.h"
#include "csi_collector.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_timer.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

static const char *TAG = "declare_lcd";

/* Waveshare ESP32-C6 1.47" Display — ST7789 172x320 */
#define LCD_W       172
#define LCD_H       320
#define LCD_MOSI    GPIO_NUM_6
#define LCD_CLK     GPIO_NUM_7
#define LCD_CS      GPIO_NUM_14
#define LCD_DC      GPIO_NUM_15
#define LCD_RST     GPIO_NUM_21
#define LCD_BL      GPIO_NUM_22

#define BTN_BOOT    GPIO_NUM_9
#define NUM_PAGES   8
#define Y_OFF       0
#define BTN_DEBOUNCE_US 200000
#define GF_P        257

static esp_lcd_panel_handle_t s_panel = NULL;
static uint16_t s_framebuf[LCD_W * 40];
static int  s_page = 0;
static volatile bool s_btn_pressed = false;
static int64_t s_btn_last_us = 0;

static void IRAM_ATTR btn_isr(void *arg)
{
    (void)arg;
    int64_t now = esp_timer_get_time();
    if (now - s_btn_last_us > BTN_DEBOUNCE_US) {
        s_btn_pressed = true;
        s_btn_last_us = now;
    }
}

static lcd_declared_t s_declared = { .active = false };

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return __builtin_bswap16(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

/* ── 5x7 bitmap font, ASCII 32-95 ── */
static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // 32 space
    {0x00,0x00,0x5F,0x00,0x00}, // 33 !
    {0x00,0x07,0x00,0x07,0x00}, // 34 "
    {0x14,0x7F,0x14,0x7F,0x14}, // 35 #
    {0x24,0x2A,0x7F,0x2A,0x12}, // 36 $
    {0x23,0x13,0x08,0x64,0x62}, // 37 %
    {0x36,0x49,0x55,0x22,0x50}, // 38 &
    {0x00,0x05,0x03,0x00,0x00}, // 39 '
    {0x00,0x1C,0x22,0x41,0x00}, // 40 (
    {0x00,0x41,0x22,0x1C,0x00}, // 41 )
    {0x14,0x08,0x3E,0x08,0x14}, // 42 *
    {0x08,0x08,0x3E,0x08,0x08}, // 43 +
    {0x00,0x50,0x30,0x00,0x00}, // 44 ,
    {0x08,0x08,0x08,0x08,0x08}, // 45 -
    {0x00,0x60,0x60,0x00,0x00}, // 46 .
    {0x20,0x10,0x08,0x04,0x02}, // 47 /
    {0x3E,0x51,0x49,0x45,0x3E}, // 48 0
    {0x00,0x42,0x7F,0x40,0x00}, // 49 1
    {0x42,0x61,0x51,0x49,0x46}, // 50 2
    {0x21,0x41,0x45,0x4B,0x31}, // 51 3
    {0x18,0x14,0x12,0x7F,0x10}, // 52 4
    {0x27,0x45,0x45,0x45,0x39}, // 53 5
    {0x3C,0x4A,0x49,0x49,0x30}, // 54 6
    {0x01,0x71,0x09,0x05,0x03}, // 55 7
    {0x36,0x49,0x49,0x49,0x36}, // 56 8
    {0x06,0x49,0x49,0x29,0x1E}, // 57 9
    {0x00,0x36,0x36,0x00,0x00}, // 58 :
    {0x00,0x56,0x36,0x00,0x00}, // 59 ;
    {0x08,0x14,0x22,0x41,0x00}, // 60 <
    {0x14,0x14,0x14,0x14,0x14}, // 61 =
    {0x00,0x41,0x22,0x14,0x08}, // 62 >
    {0x02,0x01,0x51,0x09,0x06}, // 63 ?
    {0x32,0x49,0x79,0x41,0x3E}, // 64 @
    {0x7E,0x11,0x11,0x11,0x7E}, // 65 A
    {0x7F,0x49,0x49,0x49,0x36}, // 66 B
    {0x3E,0x41,0x41,0x41,0x22}, // 67 C
    {0x7F,0x41,0x41,0x22,0x1C}, // 68 D
    {0x7F,0x49,0x49,0x49,0x41}, // 69 E
    {0x7F,0x09,0x09,0x09,0x01}, // 70 F
    {0x3E,0x41,0x49,0x49,0x7A}, // 71 G
    {0x7F,0x08,0x08,0x08,0x7F}, // 72 H
    {0x00,0x41,0x7F,0x41,0x00}, // 73 I
    {0x20,0x40,0x41,0x3F,0x01}, // 74 J
    {0x7F,0x08,0x14,0x22,0x41}, // 75 K
    {0x7F,0x40,0x40,0x40,0x40}, // 76 L
    {0x7F,0x02,0x0C,0x02,0x7F}, // 77 M
    {0x7F,0x04,0x08,0x10,0x7F}, // 78 N
    {0x3E,0x41,0x41,0x41,0x3E}, // 79 O
    {0x7F,0x09,0x09,0x09,0x06}, // 80 P
    {0x3E,0x41,0x51,0x21,0x5E}, // 81 Q
    {0x7F,0x09,0x19,0x29,0x46}, // 82 R
    {0x46,0x49,0x49,0x49,0x31}, // 83 S
    {0x01,0x01,0x7F,0x01,0x01}, // 84 T
    {0x3F,0x40,0x40,0x40,0x3F}, // 85 U
    {0x1F,0x20,0x40,0x20,0x1F}, // 86 V
    {0x3F,0x40,0x38,0x40,0x3F}, // 87 W
    {0x63,0x14,0x08,0x14,0x63}, // 88 X
    {0x07,0x08,0x70,0x08,0x07}, // 89 Y
    {0x61,0x51,0x49,0x45,0x43}, // 90 Z
    {0x00,0x7F,0x41,0x41,0x00}, // 91 [
    {0x02,0x04,0x08,0x10,0x20}, // 92 backslash
    {0x00,0x41,0x41,0x7F,0x00}, // 93 ]
    {0x04,0x02,0x01,0x02,0x04}, // 94 ^
    {0x40,0x40,0x40,0x40,0x40}, // 95 _
};
#define FONT_CHARS 64

static void fill_rect(int x, int y, int w, int h, uint16_t color)
{
    if (!s_panel || w <= 0 || h <= 0) return;
    int buf_cap = (int)(sizeof(s_framebuf) / sizeof(s_framebuf[0]));
    int rows_per = buf_cap / w;
    if (rows_per < 1) rows_per = 1;

    int drawn = 0;
    while (drawn < h) {
        int chunk = h - drawn;
        if (chunk > rows_per) chunk = rows_per;
        int px = w * chunk;
        for (int i = 0; i < px; i++)
            s_framebuf[i] = color;
        esp_lcd_panel_draw_bitmap(s_panel, x, y + drawn + Y_OFF,
                                  x + w, y + drawn + chunk + Y_OFF, s_framebuf);
        drawn += chunk;
    }
}

/* ── PALETTE ───────────────────────────────────────────── */
#define PAL_BG      rgb565(  0,   0,   0)
#define PAL_SURF    rgb565( 32,  32,  32)
#define PAL_LINE    rgb565( 80,  80,  80)
#define PAL_TEXT    rgb565(255, 255, 255)
#define PAL_MUTED   rgb565(180, 180, 180)
#define PAL_DIM     rgb565(120, 120, 120)
#define PAL_ACC     rgb565(255, 255, 255)

/* ── region renderer ─────────────────────────────────── */
static int s_rgn_x, s_rgn_y, s_rgn_w, s_rgn_h;

static void rgn_begin(int x, int y, int w, int h, uint16_t bg)
{
    s_rgn_x = x; s_rgn_y = y; s_rgn_w = w; s_rgn_h = h;
    int px = w * h;
    for (int i = 0; i < px; i++) s_framebuf[i] = bg;
}

static void rgn_text(int x, int y, const char *str, uint16_t fg, int scale)
{
    int ox = x - s_rgn_x;
    int oy = y - s_rgn_y;
    int cw = 5 * scale;
    int step = cw + scale;
    int cx = ox;
    for (int ci = 0; str[ci]; ci++) {
        int ch_idx = str[ci];
        if (ch_idx >= 'a' && ch_idx <= 'z') ch_idx -= 32;
        ch_idx -= 32;
        if (ch_idx < 0 || ch_idx >= FONT_CHARS) { cx += step; continue; }
        const uint8_t *glyph = font5x7[ch_idx];
        for (int col = 0; col < 5; col++) {
            uint8_t bits = glyph[col];
            for (int row = 0; row < 7; row++) {
                if (!(bits & (1 << row))) continue;
                for (int sy = 0; sy < scale; sy++)
                    for (int sx = 0; sx < scale; sx++) {
                        int px = cx + col * scale + sx;
                        int py = oy + row * scale + sy;
                        if (px >= 0 && px < s_rgn_w && py >= 0 && py < s_rgn_h)
                            s_framebuf[py * s_rgn_w + px] = fg;
                    }
            }
        }
        cx += step;
    }
}

static void rgn_flush(void)
{
    esp_lcd_panel_draw_bitmap(s_panel, s_rgn_x, s_rgn_y + Y_OFF,
                              s_rgn_x + s_rgn_w, s_rgn_y + s_rgn_h + Y_OFF, s_framebuf);
}

/* ── trend history ────────────────────────────────────── */
#define HIST_N 172
static float s_hr_hist[HIST_N];
static int   s_hr_len = 0;
static float s_delta_hist[HIST_N];
static int   s_delta_len = 0;
static float s_theta_hist[HIST_N];
static int   s_theta_len = 0;
static float s_alpha_hist[HIST_N];
static int   s_alpha_len = 0;

static void hist_push(float *buf, int *len, float v)
{
    if (!isfinite(v)) return;
    if (*len < HIST_N) {
        buf[(*len)++] = v;
    } else {
        for (int i = 1; i < HIST_N; i++) buf[i - 1] = buf[i];
        buf[HIST_N - 1] = v;
    }
}

static void draw_sparkline(int x, int y, int w, int h,
                           const float *data, int data_len,
                           float margin,
                           uint16_t fg, uint16_t axis, uint16_t bg)
{
    if (!s_panel || w <= 0 || h <= 0) return;
    int buf_cap = (int)(sizeof(s_framebuf) / sizeof(s_framebuf[0]));
    int rows_per = buf_cap / w;
    if (rows_per < 1) rows_per = 1;
    if (rows_per > h) rows_per = h;

    int n = 0, off = 0;
    int pt_row[HIST_N];
    if (data_len >= 2) {
        float lo = data[0], hi = data[0];
        for (int i = 1; i < data_len; i++) {
            if (data[i] < lo) lo = data[i];
            if (data[i] > hi) hi = data[i];
        }
        lo -= margin; hi += margin;
        float span = hi - lo;
        if (span < margin) span = margin;
        n = data_len < w ? data_len : w;
        off = data_len - n;
        for (int i = 0; i < n; i++)
            pt_row[i] = h - 1 - (int)((data[off + i] - lo) / span * (float)(h - 2));
    }

    int drawn = 0;
    while (drawn < h) {
        int chunk = h - drawn;
        if (chunk > rows_per) chunk = rows_per;
        int px = w * chunk;
        for (int i = 0; i < px; i++) s_framebuf[i] = bg;

        if (h - 1 >= drawn && h - 1 < drawn + chunk) {
            int row = h - 1 - drawn;
            for (int c = 0; c < w; c++)
                s_framebuf[row * w + c] = axis;
        }

        for (int i = 0; i < n; i++) {
            int col = w - n + i;
            if (col < 0 || col >= w) continue;
            int r0 = pt_row[i];
            int r1 = r0;
            if (i + 1 < n) {
                r1 = pt_row[i + 1];
                if (r1 < r0) { int t = r0; r0 = pt_row[i + 1]; r1 = t; }
                else         { r0 = pt_row[i]; r1 = pt_row[i + 1]; }
            }
            if (r1 < r0) r1 = r0;
            for (int r = r0; r <= r1; r++) {
                if (r >= drawn && r < drawn + chunk)
                    s_framebuf[(r - drawn) * w + col] = fg;
            }
        }

        esp_lcd_panel_draw_bitmap(s_panel, x, y + drawn + Y_OFF,
                                  x + w, y + drawn + chunk + Y_OFF, s_framebuf);
        drawn += chunk;
    }
}

/* ── CSI body shape ──────────────────────────────────── */
#define BODY_SLICES  32
static float s_body_width[BODY_SLICES];
static int   s_body_frames = 0;

void declare_lcd_update_body(const int8_t *iq, int iq_len)
{
    if (!iq || iq_len < 4) return;
    int n_sc = iq_len / 2;
    if (n_sc > 256) n_sc = 256;

    float amps[256];
    float max_amp = 1.0f;
    for (int i = 0; i < n_sc; i++) {
        float re = (float)iq[i * 2];
        float im = (float)iq[i * 2 + 1];
        amps[i] = sqrtf(re * re + im * im);
        if (amps[i] > max_amp) max_amp = amps[i];
    }

    for (int s = 0; s < BODY_SLICES; s++) {
        int idx = (s * n_sc) / BODY_SLICES;
        if (idx >= n_sc) idx = n_sc - 1;
        float norm = amps[idx] / max_amp;
        if (s_body_frames == 0)
            s_body_width[s] = norm;
        else
            s_body_width[s] = s_body_width[s] * 0.85f + norm * 0.15f;
    }
    s_body_frames++;
}

static void fb_line(uint16_t *fb, int fw, int fh,
                    int x0, int y0, int x1, int y1, uint16_t col)
{
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        if (x0 >= 0 && x0 < fw && y0 >= 0 && y0 < fh)
            fb[y0 * fw + x0] = col;
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static void draw_body_page(float hr, float br, int node_id)
{
    char tmp[32];
    char pgstr[8];
    snprintf(pgstr, sizeof(pgstr), "%d/%d", s_page + 1, NUM_PAGES);

    rgn_begin(0, 0, LCD_W, 30, PAL_BG);
    rgn_text(4, 4, "BODY", PAL_TEXT, 2);
    rgn_text(LCD_W - 30, 4, pgstr, PAL_DIM, 1);
    rgn_flush();

    rgn_begin(0, 30, LCD_W, 36, PAL_BG);
    if (hr > 0.5f)
        snprintf(tmp, sizeof(tmp), "HR  %-4.0f BPM", hr);
    else
        snprintf(tmp, sizeof(tmp), "HR  --");
    rgn_text(4, 32, tmp, PAL_TEXT, 2);
    if (br > 0.5f)
        snprintf(tmp, sizeof(tmp), "BR  %-5.1f RPM", br);
    else
        snprintf(tmp, sizeof(tmp), "BR  --");
    rgn_text(4, 50, tmp, PAL_TEXT, 2);
    rgn_flush();

    if (s_body_frames > 0) {
        rgn_begin(0, 70, LCD_W, 20, PAL_BG);
        rgn_text(4, 72, "CSI BODY PROFILE", PAL_DIM, 1);
        snprintf(tmp, sizeof(tmp), "%d FRAMES", s_body_frames);
        rgn_text(4, 82, tmp, PAL_DIM, 1);
        rgn_flush();

        draw_sparkline(0, 94, LCD_W, 80, s_body_width, BODY_SLICES, 0.05f,
                       PAL_TEXT, PAL_DIM, PAL_BG);
    } else {
        rgn_begin(0, 70, LCD_W, 30, PAL_BG);
        rgn_text(4, 72, "AWAITING CSI", PAL_DIM, 2);
        rgn_flush();
    }

    rgn_begin(0, 180, LCD_W, 30, PAL_BG);
    snprintf(tmp, sizeof(tmp), "N%d", node_id);
    rgn_text(4, 184, tmp, PAL_DIM, 1);
    rgn_flush();
}

/* ── static chrome ───────────────────────────────────── */
static bool s_chrome_drawn = false;

void declare_lcd_force_redraw(void) { s_chrome_drawn = false; }

void declare_lcd_set_declared(const lcd_declared_t *d)
{
    if (d) {
        memcpy(&s_declared, d, sizeof(s_declared));
        s_declared.active = true;
        s_chrome_drawn = false;
    }
}

const lcd_declared_t *declare_lcd_get_declared(void)
{
    return s_declared.active ? &s_declared : NULL;
}

void declare_lcd_show_declared(void)
{
    if (!s_panel) return;
    int rows_per = (int)(sizeof(s_framebuf) / sizeof(s_framebuf[0])) / LCD_W;
    int drawn = 0;
    while (drawn < LCD_H) {
        int chunk = LCD_H - drawn;
        if (chunk > rows_per) chunk = rows_per;
        int px = LCD_W * chunk;
        for (int i = 0; i < px; i++)
            s_framebuf[i] = __builtin_bswap16(declared_screen[drawn * LCD_W + i]);
        esp_lcd_panel_draw_bitmap(s_panel, 0, drawn + Y_OFF, LCD_W, drawn + chunk + Y_OFF, s_framebuf);
        drawn += chunk;
    }
}

static void draw_static(void)
{
    fill_rect(0, 0, LCD_W, LCD_H, PAL_BG);
    s_chrome_drawn = true;
}

esp_err_t declare_lcd_init(void)
{
    gpio_config_t bl_cfg = {
        .pin_bit_mask = 1ULL << LCD_BL,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&bl_cfg);
    gpio_set_level(LCD_BL, 1);

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = LCD_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = LCD_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_W * 40 * 2,
    };
    spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = LCD_DC,
        .cs_gpio_num = LCD_CS,
        .pclk_hz = 40 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 1,
    };
    esp_err_t ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_cfg, &io);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Panel IO failed: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    ret = esp_lcd_new_panel_st7789(io, &panel_cfg, &s_panel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Panel init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_lcd_panel_reset(s_panel);
    esp_lcd_panel_init(s_panel);
    esp_lcd_panel_invert_color(s_panel, true);
    esp_lcd_panel_set_gap(s_panel, 34, 0);
    esp_lcd_panel_disp_on_off(s_panel, true);

    fill_rect(0, 0, LCD_W, LCD_H, rgb565(0, 0, 0));

    gpio_config_t btn_cfg = {
        .pin_bit_mask = 1ULL << BTN_BOOT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&btn_cfg);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_BOOT, btn_isr, NULL);

    ESP_LOGI(TAG, "LCD initialized (%dx%d ST7789), BOOT button on GPIO%d", LCD_W, LCD_H, BTN_BOOT);
    return ESP_OK;
}

void declare_lcd_show_splash(void)
{
    if (!s_panel) return;
    int rows_per = (int)(sizeof(s_framebuf) / sizeof(s_framebuf[0])) / LCD_W;
    int drawn = 0;
    while (drawn < LCD_H) {
        int chunk = LCD_H - drawn;
        if (chunk > rows_per) chunk = rows_per;
        int px = LCD_W * chunk;
        for (int i = 0; i < px; i++)
            s_framebuf[i] = __builtin_bswap16(splash_screen[drawn * LCD_W + i]);
        esp_lcd_panel_draw_bitmap(s_panel, 0, drawn + Y_OFF, LCD_W, drawn + chunk + Y_OFF, s_framebuf);
        drawn += chunk;
    }
}

void declare_lcd_update(float hr, float br,
                        float delta, float theta, float alpha,
                        int rssi, int channel, int node_id,
                        float motion, int csi_rate)
{
    if (!s_panel) return;

    if (s_btn_pressed) {
        s_btn_pressed = false;
        s_page = (s_page + 1) % NUM_PAGES;
        s_chrome_drawn = false;
    }

    if (!s_chrome_drawn) draw_static();

    char tmp[32];
    char pgstr[16];
    snprintf(pgstr, sizeof(pgstr), "%d/%d", s_page + 1, NUM_PAGES);

    if (s_page == 0) {
        draw_body_page(hr, br, node_id);
        return;
    }

    if (s_page == 1) {
        char id_label[48];
        if (s_declared.active)
            snprintf(id_label, sizeof(id_label), "%s %u", s_declared.name, (unsigned)s_declared.position);
        else
            snprintf(id_label, sizeof(id_label), "NODE %d", node_id);

        rgn_begin(0, 0, LCD_W, 30, PAL_BG);
        rgn_text(4, 8, id_label, PAL_TEXT, 2);
        rgn_text(LCD_W - 30, 8, pgstr, PAL_DIM, 1);
        rgn_flush();

        rgn_begin(0, 30, LCD_W, 40, PAL_BG);
        rgn_text(4, 32, "HR",  PAL_DIM, 1);
        if (hr > 0.5f) snprintf(tmp, sizeof(tmp), "%-4.0f", hr);
        else           snprintf(tmp, sizeof(tmp), "--  ");
        rgn_text(4, 44, tmp, PAL_TEXT, 2);
        rgn_text(4, 62, "BPM", PAL_DIM, 1);
        rgn_text(100, 32, "D", PAL_DIM, 1);
        float band_max = delta;
        if (theta > band_max) band_max = theta;
        if (alpha > band_max) band_max = alpha;
        if (band_max < 0.001f) band_max = 0.001f;
        band_max *= 1.25f;
        snprintf(tmp, sizeof(tmp), "%-4d", (int)(delta / band_max * 100.0f + 0.5f));
        rgn_text(100, 44, tmp, PAL_TEXT, 2);
        rgn_text(100, 62, "T", PAL_DIM, 1);
        rgn_flush();

        rgn_begin(0, 70, LCD_W, 40, PAL_BG);
        rgn_text(4, 78, "BR",  PAL_DIM, 1);
        if (br > 0.5f) snprintf(tmp, sizeof(tmp), "%-5.1f", br);
        else           snprintf(tmp, sizeof(tmp), "--   ");
        rgn_text(4, 90, tmp, PAL_TEXT, 2);
        rgn_text(4, 108, "RPM", PAL_DIM, 1);
        snprintf(tmp, sizeof(tmp), "%-4d", (int)(theta / band_max * 100.0f + 0.5f));
        rgn_text(100, 74, tmp, PAL_TEXT, 2);
        rgn_text(100, 92, "A", PAL_DIM, 1);
        snprintf(tmp, sizeof(tmp), "%-4d", (int)(alpha / band_max * 100.0f + 0.5f));
        rgn_text(100, 104, tmp, PAL_TEXT, 2);
        rgn_flush();

        rgn_begin(0, 110, LCD_W, 40, PAL_BG);
        rgn_text(100, 124, "RSSI", PAL_DIM, 1);
        snprintf(tmp, sizeof(tmp), "%-5d", rssi);
        rgn_text(100, 134, tmp, PAL_TEXT, 2);
        rgn_text(4, 140, "HR TREND", PAL_DIM, 1);
        rgn_flush();

        if (hr > 0.5f) hist_push(s_hr_hist, &s_hr_len, hr);
        draw_sparkline(0, 194, LCD_W, 50, s_hr_hist, s_hr_len, 1.0f,
                       PAL_TEXT, PAL_DIM, PAL_BG);

    } else if (s_page == 2) {
        uint16_t pos = (uint16_t)(node_id % GF_P);
        if (pos == 0) pos = 1;
        uint16_t dl = FIELD_DLOG[pos];
        uint16_t ray = dl % 12;
        int fold = 256 - pos;
        int mod210 = pos % 210;
        bool qr = field_observer_is_qr(pos);

        rgn_begin(0, 0, LCD_W, 40, PAL_BG);
        rgn_text(4, 8, "CASCADE", PAL_TEXT, 2);
        rgn_text(LCD_W - 30, 8, pgstr, PAL_DIM, 1);
        rgn_text(4, 32, "GF(257) G=3", PAL_DIM, 1);
        rgn_flush();

        rgn_begin(0, 40, LCD_W, 40, PAL_BG);
        rgn_text(4, 48, "POS",  PAL_DIM, 1);
        snprintf(tmp, sizeof(tmp), "%-5u", (unsigned)pos);
        rgn_text(4, 60, tmp, PAL_TEXT, 2);
        rgn_text(100, 48, "FOLD", PAL_DIM, 1);
        snprintf(tmp, sizeof(tmp), "%-5d", fold);
        rgn_text(100, 60, tmp, PAL_TEXT, 2);
        rgn_flush();

        rgn_begin(0, 80, LCD_W, 40, PAL_BG);
        rgn_text(4, 84, "DLOG", PAL_DIM, 1);
        snprintf(tmp, sizeof(tmp), "%-5u", (unsigned)dl);
        rgn_text(4, 96, tmp, PAL_TEXT, 2);
        rgn_text(100, 84, "M210", PAL_DIM, 1);
        snprintf(tmp, sizeof(tmp), "%-5d", mod210);
        rgn_text(100, 96, tmp, PAL_TEXT, 2);
        rgn_flush();

        rgn_begin(0, 120, LCD_W, 40, PAL_BG);
        rgn_text(4, 120, "RAY",  PAL_DIM, 1);
        snprintf(tmp, sizeof(tmp), "%-3u", (unsigned)ray);
        rgn_text(4, 132, tmp, PAL_TEXT, 2);
        rgn_text(100, 120, "QR",   PAL_DIM, 1);
        rgn_text(100, 132, qr ? "YES " : "NO  ", PAL_TEXT, 2);
        rgn_flush();

    } else if (s_page == 3) {
        /* FIELD — direct + fold space from field_observer */
        uint16_t npos = (uint16_t)(node_id % GF_P);
        if (npos == 0) npos = 1;
        uint16_t nfold = (uint16_t)(GF_P - npos);

        const field_orbit_entry_t *orbits = field_observer_get_orbits();
        field_orbit_pkt_t fpkt;
        bool have_field = field_observer_tick(&fpkt);

        float e_pos  = have_field ? fpkt.qr_energy : 0.0f;
        float e_fold = have_field ? fpkt.fold_energy : 0.0f;
        float xray   = have_field ? fpkt.crossing_ratio : 0.0f;
        int n_active  = have_field ? fpkt.n_orbits_active : 0;

        rgn_begin(0, 0, LCD_W, 30, PAL_BG);
        rgn_text(4, 8, "FIELD", PAL_TEXT, 2);
        rgn_text(LCD_W - 30, 8, pgstr, PAL_DIM, 1);
        rgn_flush();

        rgn_begin(0, 30, LCD_W, 40, PAL_BG);
        rgn_text(4, 32, "POS", PAL_DIM, 1);
        snprintf(tmp, sizeof(tmp), "%-3u  E%.2f", (unsigned)npos, e_pos);
        rgn_text(4, 44, tmp, PAL_TEXT, 2);
        rgn_text(4, 62, "FOLD", PAL_DIM, 1);
        rgn_flush();

        rgn_begin(0, 70, LCD_W, 40, PAL_BG);
        snprintf(tmp, sizeof(tmp), "%-3u  E%.2f", (unsigned)nfold, e_fold);
        rgn_text(4, 74, tmp, PAL_TEXT, 2);
        rgn_text(4, 92, "XRAY", PAL_DIM, 1);
        snprintf(tmp, sizeof(tmp), "%.2f", xray);
        rgn_text(4, 104, tmp, PAL_TEXT, 2);
        rgn_flush();

        rgn_begin(0, 120, LCD_W, 40, PAL_BG);
        rgn_text(4, 128, "ORBITS", PAL_DIM, 1);
        snprintf(tmp, sizeof(tmp), "%d/15 ACTIVE", n_active);
        rgn_text(4, 140, tmp, PAL_TEXT, 2);
        rgn_flush();

        rgn_begin(0, 160, LCD_W, 30, PAL_BG);
        snprintf(tmp, sizeof(tmp), "R%u  %s",
                 (unsigned)(FIELD_DLOG[npos] % 12),
                 field_observer_is_qr(npos) ? "QR" : "QNR");
        rgn_text(4, 168, tmp, PAL_TEXT, 2);
        rgn_flush();

    } else if (s_page == 4) {
        hist_push(s_delta_hist, &s_delta_len, delta);
        hist_push(s_theta_hist, &s_theta_len, theta);
        hist_push(s_alpha_hist, &s_alpha_len, alpha);

        rgn_begin(0, 0, LCD_W, 30, PAL_BG);
        rgn_text(4, 8, "WAVES", PAL_TEXT, 2);
        rgn_text(LCD_W - 30, 8, pgstr, PAL_DIM, 1);
        rgn_flush();

        snprintf(tmp, sizeof(tmp), "D %-7.4f", delta);
        rgn_begin(0, 32, LCD_W, 10, PAL_BG);
        rgn_text(4, 32, tmp, PAL_DIM, 1);
        rgn_flush();
        draw_sparkline(0, 44, LCD_W, 50, s_delta_hist, s_delta_len, 0.001f,
                       PAL_TEXT, PAL_DIM, PAL_BG);

        snprintf(tmp, sizeof(tmp), "T %-7.4f", theta);
        rgn_begin(0, 96, LCD_W, 10, PAL_BG);
        rgn_text(4, 96, tmp, PAL_DIM, 1);
        rgn_flush();
        draw_sparkline(0, 108, LCD_W, 50, s_theta_hist, s_theta_len, 0.001f,
                       PAL_TEXT, PAL_DIM, PAL_BG);

        snprintf(tmp, sizeof(tmp), "A %-7.4f", alpha);
        rgn_begin(0, 160, LCD_W, 10, PAL_BG);
        rgn_text(4, 160, tmp, PAL_DIM, 1);
        rgn_flush();
        draw_sparkline(0, 172, LCD_W, 50, s_alpha_hist, s_alpha_len, 0.001f,
                       PAL_TEXT, PAL_DIM, PAL_BG);

    } else if (s_page == 5) {
        rgn_begin(0, 0, LCD_W, 30, PAL_BG);
        rgn_text(4, 8, "LINK", PAL_TEXT, 2);
        rgn_text(LCD_W - 30, 8, pgstr, PAL_DIM, 1);
        rgn_flush();

        rgn_begin(0, 30, LCD_W, 40, PAL_BG);
        rgn_text(4, 32, "NODE",    PAL_DIM, 1);
        snprintf(tmp, sizeof(tmp), "%-5d", node_id);
        rgn_text(4, 44, tmp, PAL_TEXT, 2);
        rgn_text(4, 68, "RSSI",    PAL_DIM, 1);
        rgn_flush();

        rgn_begin(0, 70, LCD_W, 40, PAL_BG);
        snprintf(tmp, sizeof(tmp), "%-5d DBM", rssi);
        rgn_text(4, 80, tmp, PAL_TEXT, 2);
        rgn_text(4, 104, "CHANNEL", PAL_DIM, 1);
        rgn_flush();

        rgn_begin(0, 110, LCD_W, 40, PAL_BG);
        snprintf(tmp, sizeof(tmp), "%-5d", channel);
        rgn_text(4, 116, tmp, PAL_TEXT, 2);
        rgn_text(4, 140, "MOTION",  PAL_DIM, 1);
        rgn_flush();

        rgn_begin(0, 150, LCD_W, 40, PAL_BG);
        snprintf(tmp, sizeof(tmp), "%-6.2f", motion);
        rgn_text(4, 152, tmp, PAL_TEXT, 2);
        rgn_text(4, 176, "UPTIME",  PAL_DIM, 1);
        uint32_t sec = (uint32_t)(esp_timer_get_time() / 1000000ULL);
        uint32_t m = sec / 60;
        uint32_t s = sec % 60;
        snprintf(tmp, sizeof(tmp), "%lu:%02lu ", (unsigned long)m, (unsigned long)s);
        rgn_text(4, 188, tmp, PAL_TEXT, 2);
        rgn_flush();

    } else if (s_page == 6) {
        rgn_begin(0, 0, LCD_W, 30, PAL_BG);
        rgn_text(4, 8, "KEYS", PAL_TEXT, 2);
        rgn_text(LCD_W - 30, 8, pgstr, PAL_DIM, 1);
        rgn_flush();

        if (s_declared.active) {
            rgn_begin(0, 30, LCD_W, 40, PAL_BG);
            snprintf(tmp, sizeof(tmp), "1 DECLARED");
            rgn_text(4, 30, tmp, PAL_DIM, 1);
            rgn_text(4, 42, s_declared.name, PAL_TEXT, 2);
            snprintf(tmp, sizeof(tmp), "%uP R%u %s",
                     (unsigned)s_declared.position,
                     (unsigned)s_declared.ray,
                     field_observer_is_qr(s_declared.position) ? "QR" : "QNR");
            rgn_text(4, 57, tmp, PAL_DIM, 1);
            rgn_flush();

            rgn_begin(0, 70, LCD_W, 30, PAL_BG);
            char hash_trunc[17];
            snprintf(hash_trunc, sizeof(hash_trunc), "%.16s", s_declared.hash);
            rgn_text(4, 76, hash_trunc, PAL_TEXT, 1);
            snprintf(hash_trunc, sizeof(hash_trunc), "%.16s", s_declared.hash + 16);
            rgn_text(4, 86, hash_trunc, PAL_TEXT, 1);
            rgn_flush();
        } else {
            rgn_begin(0, 30, LCD_W, 40, PAL_BG);
            rgn_text(4, 30, "0 DECLARED", PAL_DIM, 1);
            rgn_text(4, 50, "CONNECT USB", PAL_TEXT, 2);
            rgn_flush();

            rgn_begin(0, 70, LCD_W, 20, PAL_BG);
            rgn_text(4, 72, "OPEN DECLARE.HTML", PAL_DIM, 1);
            rgn_flush();
        }

    } else if (s_page == 7) {
        if (!s_declared.active) {
            rgn_begin(0, 0, LCD_W, 60, PAL_BG);
            rgn_text(4, 8, "DECLARED", PAL_TEXT, 2);
            rgn_text(LCD_W - 30, 8, pgstr, PAL_DIM, 1);
            rgn_text(4, 36, "NOT YET", PAL_DIM, 2);
            rgn_flush();
            return;
        }

        rgn_begin(0, 0, LCD_W, 30, PAL_BG);
        rgn_text(4, 8, "DECLARED", PAL_TEXT, 2);
        rgn_text(LCD_W - 30, 8, pgstr, PAL_DIM, 1);
        rgn_flush();

        rgn_begin(0, 30, LCD_W, 40, PAL_BG);
        rgn_text(4, 32, s_declared.name, PAL_TEXT, 2);
        rgn_text(4, 56, "POS",  PAL_DIM, 1);
        snprintf(tmp, sizeof(tmp), "%-5u", (unsigned)s_declared.position);
        rgn_text(4, 68, tmp, PAL_TEXT, 2);
        rgn_flush();

        rgn_begin(0, 70, LCD_W, 40, PAL_BG);
        rgn_text(100, 76, "INV", PAL_DIM, 1);
        snprintf(tmp, sizeof(tmp), "%-5u", (unsigned)s_declared.inverse);
        rgn_text(100, 88, tmp, PAL_TEXT, 2);
        rgn_text(4, 76, "RAY",  PAL_DIM, 1);
        snprintf(tmp, sizeof(tmp), "%-3u", (unsigned)s_declared.ray);
        rgn_text(4, 88, tmp, PAL_TEXT, 2);
        rgn_flush();

        rgn_begin(0, 110, LCD_W, 40, PAL_BG);
        rgn_text(100, 110, "LYRS", PAL_DIM, 1);
        snprintf(tmp, sizeof(tmp), "%u/11", (unsigned)s_declared.layers_measured);
        rgn_text(100, 122, tmp, PAL_TEXT, 2);
        rgn_text(4, 112, "HASH", PAL_DIM, 1);
        char hash_short[25];
        snprintf(hash_short, sizeof(hash_short), "%.12s...", s_declared.hash);
        rgn_text(4, 124, hash_short, PAL_TEXT, 1);
        rgn_text(4, 138, "GENOME", PAL_DIM, 1);
        rgn_flush();

        rgn_begin(0, 150, LCD_W, 40, PAL_BG);
        char genome_short[21];
        snprintf(genome_short, sizeof(genome_short), "%.20s", s_declared.genome);
        rgn_text(4, 154, genome_short, PAL_TEXT, 1);
        snprintf(tmp, sizeof(tmp), "3^%u=%u R%u",
                 (unsigned)s_declared.dlog,
                 (unsigned)s_declared.position,
                 (unsigned)s_declared.ray);
        rgn_text(4, 172, tmp, PAL_DIM, 1);
        rgn_flush();
    }
}
#endif /* CONFIG_IDF_TARGET_ESP32C6 */
