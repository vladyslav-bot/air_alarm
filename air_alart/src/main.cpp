#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "led_strip.h"

// =========================== WiFi ===========================
#define WIFI_SSID "test"
#define WIFI_PASS "12345678"
#define WIFI_MAX_RETRY 5

// =========================== LED Strip ===========================
#define LED_GPIO 16
#define NUM_LEDS 12

// =========================== I2C + SSD1306 ===========================
#define I2C_SDA GPIO_NUM_8
#define I2C_SCL GPIO_NUM_9
#define I2C_FREQ 400000
#define SSD1306_ADDR 0x3C
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_PAGES (OLED_HEIGHT / 8)

// =========================== HTTP Fetch Interval ===========================
#define FETCH_INTERVAL_MS 30000
#define SPLASH_DURATION_MS 2000
#define CONNECTING_DOT_MS   500

static const char *TAG = "air_alarm";

// FreeRTOS event group for WiFi state
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int wifi_retry_num = 0;

// Handles
static led_strip_handle_t led_strip = NULL;
static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t ssd1306_dev = NULL;

// App state
static char current_status = 'X';
static bool was_connected = false;
static bool show_splash = false;
static TickType_t splash_start = 0;
static TickType_t last_fetch = 0;

// OLED framebuffer: 128 columns × 8 pages (64 pixels high)
static uint8_t oled_buffer[OLED_WIDTH * OLED_PAGES];

// ======================= 5×7 Font (ASCII 32–127) =======================
// Each character is 5 bytes (vertical columns, bit0 = top pixel)
static const uint8_t font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // 32 space
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // 33 !    — simplified
    {0x00, 0x07, 0x00, 0x07, 0x00}, // 34 "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // 35 #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // 36 $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // 37 %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // 38 &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // 39 '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // 40 (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // 41 )
    {0x08, 0x2A, 0x1C, 0x2A, 0x08}, // 42 *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // 43 +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // 44 ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // 45 -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // 46 .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // 47 /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 48 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 49 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 50 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 51 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 52 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 53 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 54 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 55 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 56 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 57 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // 58 :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // 59 ;
    {0x00, 0x08, 0x14, 0x22, 0x41}, // 60 <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // 61 =
    {0x41, 0x22, 0x14, 0x08, 0x00}, // 62 >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // 63 ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // 64 @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // 65 A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // 66 B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // 67 C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // 68 D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // 69 E
    {0x7F, 0x09, 0x09, 0x01, 0x01}, // 70 F
    {0x3E, 0x41, 0x41, 0x51, 0x32}, // 71 G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // 72 H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // 73 I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // 74 J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // 75 K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // 76 L
    {0x7F, 0x02, 0x04, 0x02, 0x7F}, // 77 M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // 78 N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // 79 O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // 80 P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // 81 Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // 82 R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // 83 S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // 84 T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // 85 U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // 86 V
    {0x7F, 0x20, 0x18, 0x20, 0x7F}, // 87 W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // 88 X
    {0x03, 0x04, 0x78, 0x04, 0x03}, // 89 Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // 90 Z
    {0x00, 0x00, 0x7F, 0x41, 0x41}, // 91 [
    {0x02, 0x04, 0x08, 0x10, 0x20}, // 92 backslash
    {0x41, 0x41, 0x7F, 0x00, 0x00}, // 93 ]
    {0x04, 0x02, 0x01, 0x02, 0x04}, // 94 ^
    {0x40, 0x40, 0x40, 0x40, 0x40}, // 95 _
    {0x00, 0x01, 0x02, 0x04, 0x00}, // 96 `
    {0x20, 0x54, 0x54, 0x54, 0x78}, // 97 a
    {0x7F, 0x48, 0x44, 0x44, 0x38}, // 98 b
    {0x38, 0x44, 0x44, 0x44, 0x20}, // 99 c
    {0x38, 0x44, 0x44, 0x48, 0x7F}, // 100 d
    {0x38, 0x54, 0x54, 0x54, 0x18}, // 101 e
    {0x08, 0x7E, 0x09, 0x01, 0x02}, // 102 f
    {0x08, 0x14, 0x54, 0x54, 0x3C}, // 103 g
    {0x7F, 0x08, 0x04, 0x04, 0x78}, // 104 h
    {0x00, 0x44, 0x7D, 0x40, 0x00}, // 105 i
    {0x20, 0x40, 0x44, 0x3D, 0x00}, // 106 j
    {0x00, 0x7F, 0x10, 0x28, 0x44}, // 107 k
    {0x00, 0x41, 0x7F, 0x40, 0x00}, // 108 l
    {0x7C, 0x04, 0x18, 0x04, 0x78}, // 109 m
    {0x7C, 0x08, 0x04, 0x04, 0x78}, // 110 n
    {0x38, 0x44, 0x44, 0x44, 0x38}, // 111 o
    {0x7C, 0x14, 0x14, 0x14, 0x08}, // 112 p
    {0x08, 0x14, 0x14, 0x18, 0x7C}, // 113 q
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // 114 r
    {0x48, 0x54, 0x54, 0x54, 0x20}, // 115 s
    {0x04, 0x3F, 0x44, 0x40, 0x20}, // 116 t
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, // 117 u
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, // 118 v
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, // 119 w
    {0x44, 0x28, 0x10, 0x28, 0x44}, // 120 x
    {0x0C, 0x50, 0x50, 0x50, 0x3C}, // 121 y
    {0x44, 0x64, 0x54, 0x4C, 0x44}, // 122 z
    {0x00, 0x08, 0x36, 0x41, 0x00}, // 123 {
    {0x00, 0x00, 0x7F, 0x00, 0x00}, // 124 |
    {0x00, 0x41, 0x36, 0x08, 0x00}, // 125 }
    {0x08, 0x08, 0x2A, 0x1C, 0x08}, // 126 ~
    {0x08, 0x1C, 0x2A, 0x08, 0x08}, // 127 del
};

// ======================= OLED Helpers =======================

static void ssd1306_write_cmd(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd}; // 0x00 = command mode
    i2c_master_transmit(ssd1306_dev, buf, 2, -1);
}

static void ssd1306_write_data(const uint8_t *data, size_t len)
{
    // SSD1306 data prefix is 0x40, send it separately then data
    uint8_t prefix = 0x40;
    // Build a buffer: prefix + data. But we can't stack-allocate dynamic size safely for large len.
    // Instead, send prefix + first chunk, then rest without prefix (SSD1306 stays in data mode)
    // Actually simplest: send in one shot with a small stack buffer
    // For OLED update, len is 1024 bytes.
    i2c_master_transmit(ssd1306_dev, &prefix, 1, -1);
    // Now SSD1306 is in data mode; subsequent writes are treated as data
    // But we can't guarantee that with i2c_master_transmit (it sends STOP).
    // So we send prefix+data in one shot, but buffer on stack is risky for 1024 bytes.
    // Alternative: allocate on heap, or send in chunks with prefix each time.
    // For simplicity: send in chunks of 128 bytes with prefix.
    size_t offset = 0;
    while (offset < len) {
        size_t chunk = (len - offset > 128) ? 128 : (len - offset);
        uint8_t tx_buf[129];
        tx_buf[0] = 0x40;
        memcpy(tx_buf + 1, data + offset, chunk);
        i2c_master_transmit(ssd1306_dev, tx_buf, chunk + 1, -1);
        offset += chunk;
    }
}

static void ssd1306_init(void)
{
    // SSD1306 initialization sequence
    ssd1306_write_cmd(0xAE); // Display OFF
    ssd1306_write_cmd(0xD5); // Set display clock divide ratio
    ssd1306_write_cmd(0x80);
    ssd1306_write_cmd(0xA8); // Set multiplex ratio
    ssd1306_write_cmd(0x3F); // 64
    ssd1306_write_cmd(0xD3); // Set display offset
    ssd1306_write_cmd(0x00);
    ssd1306_write_cmd(0x40); // Set start line
    ssd1306_write_cmd(0x8D); // Charge pump
    ssd1306_write_cmd(0x14); // Enable
    ssd1306_write_cmd(0x20); // Memory mode
    ssd1306_write_cmd(0x00); // Horizontal
    ssd1306_write_cmd(0xA1); // Segment remap
    ssd1306_write_cmd(0xC8); // COM scan direction
    ssd1306_write_cmd(0xDA); // COM pins
    ssd1306_write_cmd(0x12);
    ssd1306_write_cmd(0x81); // Contrast
    ssd1306_write_cmd(0xCF);
    ssd1306_write_cmd(0xD9); // Pre-charge
    ssd1306_write_cmd(0xF1);
    ssd1306_write_cmd(0xDB); // VCOM detect
    ssd1306_write_cmd(0x40);
    ssd1306_write_cmd(0xA4); // Display all on resume
    ssd1306_write_cmd(0xA6); // Normal display
    ssd1306_write_cmd(0x2E); // Deactivate scroll
    ssd1306_write_cmd(0xAF); // Display ON
}

static void ssd1306_clear_buffer(void)
{
    memset(oled_buffer, 0, sizeof(oled_buffer));
}

static void ssd1306_flush(void)
{
    ssd1306_write_cmd(0x21); // Column address
    ssd1306_write_cmd(0x00);
    ssd1306_write_cmd(0x7F); // 127
    ssd1306_write_cmd(0x22); // Page address
    ssd1306_write_cmd(0x00);
    ssd1306_write_cmd(0x07); // 7
    ssd1306_write_data(oled_buffer, sizeof(oled_buffer));
}

static void ssd1306_draw_pixel(int x, int y, bool on)
{
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) return;
    uint8_t page = y / 8;
    uint8_t bit = y % 8;
    if (on)
        oled_buffer[x + page * OLED_WIDTH] |= (1 << bit);
    else
        oled_buffer[x + page * OLED_WIDTH] &= ~(1 << bit);
}

static void ssd1306_draw_char(int x, int y, char c, int size)
{
    if (c < 32 || c > 127) c = '?';
    const uint8_t *glyph = font5x7[c - 32];
    for (int col = 0; col < 5; col++) {
        uint8_t line = glyph[col];
        for (int row = 0; row < 8; row++) {
            if (line & (1 << row)) {
                for (int sx = 0; sx < size; sx++) {
                    for (int sy = 0; sy < size; sy++) {
                        ssd1306_draw_pixel(x + col * size + sx, y + row * size + sy, true);
                    }
                }
            }
        }
    }
}

static void ssd1306_draw_string(int x, int y, const char *str, int size)
{
    while (*str) {
        if (x + 5 * size > OLED_WIDTH) { x = 0; y += 8 * size; }
        if (y + 8 * size > OLED_HEIGHT) break;
        ssd1306_draw_char(x, y, *str, size);
        x += 6 * size; // 5 pixels + 1 spacing
        str++;
    }
}

// ======================= LED Strip =======================

static void set_all_leds(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < NUM_LEDS; i++) {
        led_strip_set_pixel(led_strip, i, r, g, b);
    }
    led_strip_refresh(led_strip);
}

static void update_led_by_status(char status)
{
    switch (status) {
        case 'A': set_all_leds(255, 0, 0);   break;   // Alarm — Red
        case 'P': set_all_leds(255, 255, 0); break;   // Warning — Yellow
        case 'N': set_all_leds(0, 150, 255); break;   // Normal — Blue
        default:  set_all_leds(255, 255, 255); break; // Unknown — White
    }
}

// ======================= WiFi Event Handler =======================

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            if (wifi_retry_num < WIFI_MAX_RETRY) {
                esp_wifi_connect();
                wifi_retry_num++;
                ESP_LOGW(TAG, "WiFi disconnected, retry %d/%d", wifi_retry_num, WIFI_MAX_RETRY);
            } else {
                xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
            }
            break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        wifi_retry_num = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
        &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
        &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {};
    strcpy((char *)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char *)wifi_config.sta.password, WIFI_PASS);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi STA init finished, connecting to %s...", WIFI_SSID);
}

// ======================= HTTP Fetch =======================

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    return ESP_OK;
}

static void fetch_air_alarm_status(void)
{
    esp_http_client_config_t config = {};
    config.url = "http://alarmair.com.ua/";
    config.event_handler = http_event_handler;
    config.timeout_ms = 10000;

    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        int status_code = esp_http_client_get_status_code(client);
        if (status_code == 200) {
            int content_len = esp_http_client_get_content_length(client);
            if (content_len > 0) {
                char *buf = (char *)malloc(content_len + 1);
                if (buf) {
                    int read_len = esp_http_client_read_response(client, buf, content_len);
                    buf[read_len] = '\0';

                    // Remove quotes, brackets — same logic as Arduino version
                    char *clean = buf;
                    char *dst = buf;
                    while (*clean) {
                        if (*clean != '\"' && *clean != '[' && *clean != ']')
                            *dst++ = *clean;
                        clean++;
                    }
                    *dst = '\0';

                    // Trim leading/trailing whitespace
                    char *start = buf;
                    while (*start == ' ' || *start == '\t' || *start == '\n' || *start == '\r') start++;

                    if (strlen(start) >= 10) {
                        current_status = start[9];
                        const char *status_text = "UNKNOWN";
                        if (current_status == 'A') status_text = "ALARM";
                        else if (current_status == 'N') status_text = "GOOD";
                        else if (current_status == 'P') status_text = "WARNING";

                        update_led_by_status(current_status);
                        ESP_LOGI(TAG, "Status: %c -> %s", current_status, status_text);

                        ssd1306_clear_buffer();
                        ssd1306_draw_string(0, 0, "Kiev", 2);
                        int sy = 30;
                        int sz = 3;
                        if (current_status == 'P' || current_status == 'A') { sz = 3; sy = 30; }
                        else if (strcmp(status_text, "WARNING") == 0) { sz = 2; sy = 35; }
                        else { sz = 3; sy = 30; }
                        ssd1306_draw_string(0, sy, status_text, sz);
                        ssd1306_flush();
                    } else {
                        ESP_LOGW(TAG, "Bad data length: %d", strlen(start));
                        ssd1306_clear_buffer();
                        ssd1306_draw_string(0, 0, "Error", 2);
                        ssd1306_draw_string(0, 30, "Bad Data", 2);
                        ssd1306_flush();
                    }
                    free(buf);
                }
            }
        }
    } else {
        ESP_LOGE(TAG, "HTTP request failed: %s", esp_err_to_name(err));
        ssd1306_clear_buffer();
        ssd1306_draw_string(0, 0, "Error", 2);
        ssd1306_draw_string(0, 30, "HTTP Fail", 2);
        ssd1306_flush();
    }

    esp_http_client_cleanup(client);
}

// ======================= Main Loop Task =======================

static void main_loop_task(void *pvParameters)
{
    TickType_t last_dot_tick = 0;
    int dot_count = 0;

    while (1) {
        EventBits_t bits = xEventGroupGetBits(wifi_event_group);
        bool wifi_connected = (bits & WIFI_CONNECTED_BIT) != 0;
        // bool wifi_failed = (bits & WIFI_FAIL_BIT) != 0;
        TickType_t now = xTaskGetTickCount();

        if (!wifi_connected) {
            was_connected = false;
            current_status = 'X';
            set_all_leds(255, 255, 255); // White — no WiFi

            if (pdTICKS_TO_MS(now - last_dot_tick) >= CONNECTING_DOT_MS) {
                last_dot_tick = now;
                ssd1306_clear_buffer();
                ssd1306_draw_string(0, 10, "Connecting WiFi", 1);
                dot_count = (dot_count + 1) % 4;
                char dots[4] = {};
                for (int i = 0; i < dot_count; i++) dots[i] = '.';
                ssd1306_draw_string(0, 20, dots, 1);
                ssd1306_flush();
            }
        } else {
            if (!was_connected) {
                was_connected = true;
                splash_start = now;
                show_splash = true;
                ssd1306_clear_buffer();
                ssd1306_draw_string(0, 10, "WiFi Connected!", 1);
                ssd1306_flush();
            }

            if (show_splash) {
                if (pdTICKS_TO_MS(now - splash_start) >= SPLASH_DURATION_MS) {
                    show_splash = false;
                    // Trigger immediate fetch by setting last_fetch far in the past
                    last_fetch = now - pdMS_TO_TICKS(FETCH_INTERVAL_MS);
                }
            } else {
                if (pdTICKS_TO_MS(now - last_fetch) >= FETCH_INTERVAL_MS) {
                    last_fetch = now;
                    fetch_air_alarm_status();
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ======================= App Main =======================

extern "C" void app_main(void)
{
    // Init NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Init LED strip
    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = LED_GPIO;
    strip_config.max_leds = NUM_LEDS;
    strip_config.led_pixel_format = LED_PIXEL_FORMAT_GRB;
    strip_config.led_model = LED_MODEL_WS2812;
    strip_config.flags.invert_out = false;

    led_strip_rmt_config_t rmt_config = {};
    rmt_config.resolution_hz = 10 * 1000 * 1000; // 10 MHz
    rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rmt_config.flags.with_dma = false;

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    set_all_leds(0, 0, 0); // Off at start

    // Init I2C
    i2c_master_bus_config_t i2c_cfg = {};
    i2c_cfg.i2c_port = I2C_NUM_0;
    i2c_cfg.sda_io_num = I2C_SDA;
    i2c_cfg.scl_io_num = I2C_SCL;
    i2c_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_cfg.glitch_ignore_cnt = 7;
    i2c_cfg.flags.enable_internal_pullup = true;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_cfg, &i2c_bus));

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = SSD1306_ADDR;
    dev_cfg.scl_speed_hz = I2C_FREQ;

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_cfg, &ssd1306_dev));

    // Init SSD1306
    ESP_LOGI(TAG, "Probing SSD1306 at 0x%02X...", SSD1306_ADDR);
    ret = i2c_master_probe(i2c_bus, SSD1306_ADDR, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SSD1306 not found! I2C error: %s", esp_err_to_name(ret));
        // Keep going — display won't work but LEDs will
    } else {
        ESP_LOGI(TAG, "SSD1306 found");
        ssd1306_init();
    }

    // Init WiFi
    wifi_init_sta();

    // Create main loop task
    xTaskCreate(main_loop_task, "main_loop", 8192, NULL, 5, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}