#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "led_strip.h"

static const char *TAG = "SIMPLE_DISPLAY_TEST";

// Pin definitions for ESP32-C6-LCD-1.47
// Pin definitions - CORRECTED from Display_ST7789.h
static int PIN_MISO = 5;   // CORRECTED: Real MISO pin
static int PIN_MOSI = 6;   // CORRECTED: Real MOSI pin  
static int PIN_SCLK = 7;   // CORRECTED: Real SCLK pin
static int PIN_CS = 14;    // CORRECTED: Real CS pin
static int PIN_DC = 15;    // CORRECTED: Real DC pin
static int PIN_RST = 21;   // CORRECTED: Real RST pin (was our backlight!)
static int PIN_BCKL = 22;  // CORRECTED: Real backlight pin

// RGB LED configuration (WS2812B/NeoPixel)
#define RGB_LED_PIN     8    // GPIO8 for RGB LED strip
#define RGB_LED_COUNT   1    // Number of RGB LEDs

// Display dimensions - CORRECTED based on LBS147TC-IF15 datasheet
#define LCD_WIDTH   172   // 172(H)RGB per datasheet
#define LCD_HEIGHT  320   // 320(V) per datasheet

// Animation states
typedef enum {
    ANIM_LOOK_LEFT_RIGHT = 0,
    ANIM_BLINK_NATURAL,
    ANIM_SMILE,
    ANIM_SUSPICIOUS,
    ANIM_ANGER,
    ANIM_SLEEP,
    ANIM_COUNT
} animation_state_t;

// Animation timing
#define ANIM_MIN_DURATION_MS 3000  // 3 seconds
#define ANIM_MAX_DURATION_MS 7000  // 7 seconds
#define LOOK_SPEED_MS 16           // ~60fps (16ms per frame)
#define BLINK_SPEED_MS 16          // Speed of blink animation

// Sleep animation timing
#define SLEEP_MIN_INTERVAL_MS 30000  // 30 seconds minimum between sleep animations
#define SLEEP_DURATION_MS 12000      // 12 seconds sleep duration
#define SLEEP_CLOSING_MS 3000        // 3 seconds to close eyes
#define SLEEP_ZZZ_MS 6000            // 6 seconds of zZz animation
#define SLEEP_OPENING_MS 3000        // 3 seconds to open eyes

// Framebuffer for fast rendering
#define FRAMEBUFFER_SIZE (LCD_WIDTH * LCD_HEIGHT * 2) // 2 bytes per pixel (RGB565)
static uint16_t* framebuffer = NULL;

// CRITICAL: Display offset values from Display_ST7789.h
#define OFFSET_X    34    // X offset for this display variant
#define OFFSET_Y    0     // Y offset for this display variant

// ST7789 Commands
#define ST7789_SWRESET    0x01
#define ST7789_SLPOUT     0x11
#define ST7789_DISPON     0x29
#define ST7789_CASET      0x2A
#define ST7789_RASET      0x2B
#define ST7789_RAMWR      0x2C
#define ST7789_MADCTL     0x36
#define ST7789_COLMOD     0x3A

static spi_device_handle_t spi;
static led_strip_handle_t led_strip = NULL;

// Function declarations
esp_err_t init_spi_safe(void);
void test_pin_config(int mosi, int sclk, int cs, int dc, int rst, int bckl);
void test_multiple_pin_configs(void);
void test_display_orientations(void);
void test_different_windows(void);
void fill_screen_color(uint16_t color);
void init_rgb_led(void);
void set_rgb_color(uint8_t r, uint8_t g, uint8_t b);
void rgb_color_cycle(void);
void draw_anime_eyes(void);
void draw_anime_eyes_looking(int pupil_offset_x, int pupil_offset_y);
void draw_anime_eyes_natural_blink(float blink_progress);
void draw_anime_eyes_laughing(uint32_t time_offset);
void draw_anime_eyes_suspicious(void);
void draw_anime_eyes_suspicious_animated(float eyelid_progress, float pupil_progress);
void draw_anime_eyes_angry(uint32_t time_offset);
void draw_anime_eyes_sleeping(float sleep_progress);
void draw_zzz_letters(uint32_t time_offset);
void animate_eyes(void);
void draw_circle(int center_x, int center_y, int radius, uint16_t color);
void draw_filled_circle(int center_x, int center_y, int radius, uint16_t color);
void draw_ellipse_filled(int center_x, int center_y, int radius_x, int radius_y, uint16_t color);
void set_pixel(int x, int y, uint16_t color);
void set_pixel_fast(int x, int y, uint16_t color);
void clear_framebuffer(uint16_t color);
void display_framebuffer(void);
bool init_framebuffer(void);
uint32_t get_random_duration(void);

void spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc = (int)t->user;
    gpio_set_level(PIN_DC, dc);
}

void st7789_write_cmd(uint8_t cmd)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &cmd;
    t.user = (void*)0; // DC low for command
    spi_device_polling_transmit(spi, &t);
}

void st7789_write_data(const uint8_t *data, int len)
{
    if (len == 0) return;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = len * 8;
    t.tx_buffer = data;
    t.user = (void*)1; // DC high for data
    spi_device_polling_transmit(spi, &t);
}

void st7789_write_data_byte(uint8_t data)
{
    st7789_write_data(&data, 1);
}

esp_err_t init_spi_safe(void)
{
    spi_bus_config_t buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = PIN_MOSI,
        .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 8192  // 8KB buffer for framebuffer chunks
    };
    
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 20*1000*1000,  // Higher speed for fast framebuffer transfer
        .mode = 0,
        .spics_io_num = PIN_CS,
        .queue_size = 1,  // Single queue for sequential transfers
        .pre_cb = spi_pre_transfer_callback,
    };
    
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        spi_bus_free(SPI2_HOST);
        return ret;
    }
    
    return ESP_OK;
}

void init_spi(void)
{
    ESP_LOGI(TAG, "Initializing SPI...");
    
    spi_bus_config_t buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = PIN_MOSI,
        .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_WIDTH * LCD_HEIGHT * 2 + 8
    };
    
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10*1000*1000,  // 10 MHz
        .mode = 0,
        .spics_io_num = PIN_CS,
        .queue_size = 7,
        .pre_cb = spi_pre_transfer_callback,
    };
    
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));
}

void init_rgb_led(void)
{
    ESP_LOGI(TAG, "Initializing RGB LED on pin %d", RGB_LED_PIN);
    
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = RGB_LED_PIN,   // The GPIO that connected to the LED strip's data line
        .max_leds = RGB_LED_COUNT,       // The number of LEDs in the strip,
    };
    
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
        .flags.with_dma = false,            // whether to enable the DMA feature
    };
    
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    
    /* Set all LED off to clear any previous state */
    ESP_ERROR_CHECK(led_strip_clear(led_strip));
    
    ESP_LOGI(TAG, "RGB LED initialized successfully");
}

void set_rgb_color(uint8_t r, uint8_t g, uint8_t b)
{
    if (led_strip != NULL) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, r, g, b));
        /* Refresh the strip to send data */
        ESP_ERROR_CHECK(led_strip_refresh(led_strip));
    }
}

void rgb_color_cycle(void)
{
    static uint8_t hue = 0;
    static uint32_t last_time = 0;
    
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Update color every 50ms for smooth transition
    if (current_time - last_time >= 50) {
        last_time = current_time;
        
        // Convert HSV to RGB (simplified)
        uint8_t r, g, b;
        uint8_t region = hue / 43;
        uint8_t remainder = (hue - (region * 43)) * 6;
        
        uint8_t p = 0;
        uint8_t q = (255 * (255255 - remainder)) >> 8;
        uint8_t t = (255 * remainder) >> 8;
        
        switch (region) {
            case 0: r = 255; g = t; b = p; break;
            case 1: r = p; g = 255; b = t; break;
            case 2: r = q; g = 255; b = p; break;
            case 3: r = p; g = q; b = 255; break;
            case 4: r = 255; g = p; b = q; break;
            default: r = t; g = p; b = 255; break;
        }
        
        // Convert RGB to GRB for WS2812B LED strip
        set_rgb_color(g, r, b); // Swap R and G channels
        hue = (hue + 2) % 255; // Increment hue for next cycle
    }
}

void init_pins(void)
{
    ESP_LOGI(TAG, "Initializing pins...");
    
    // Configure DC pin
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << PIN_DC),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);
    
    // Configure RST pin
    io_conf.pin_bit_mask = (1ULL << PIN_RST);
    gpio_config(&io_conf);
    
    // Configure backlight pin
    io_conf.pin_bit_mask = (1ULL << PIN_BCKL);
    gpio_config(&io_conf);
    
    ESP_LOGI(TAG, "Pins configured: DC=%d, RST=%d, BCKL=%d", PIN_DC, PIN_RST, PIN_BCKL);
}

void init_backlight(void)
{
    ESP_LOGI(TAG, "Initializing backlight...");
    
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 5000,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);
    
    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_CHANNEL_0,
        .duty       = 50, // Reduced brightness
        .gpio_num   = PIN_BCKL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel  = LEDC_TIMER_0
    };
    ledc_channel_config(&ledc_channel);
    
    ESP_LOGI(TAG, "Backlight set to reduced brightness (80/255) on pin %d", PIN_BCKL);
}

void st7789_reset(void)
{
    ESP_LOGI(TAG, "Resetting display...");
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
}

void st7789_init_display(void)
{
    // Reset display
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Software reset
    st7789_write_cmd(ST7789_SWRESET);
    vTaskDelay(pdMS_TO_TICKS(150));
    
    // Sleep out
    st7789_write_cmd(ST7789_SLPOUT);
    vTaskDelay(pdMS_TO_TICKS(120));
    
    // Color mode: 16-bit/pixel
    st7789_write_cmd(0x3A);
    st7789_write_data_byte(0x05);
    
    // Memory access control 
    st7789_write_cmd(ST7789_MADCTL);
    st7789_write_data_byte(0x00);
    
    // Porch control
    st7789_write_cmd(0xB2);
    st7789_write_data_byte(0x0C);
    st7789_write_data_byte(0x0C);
    st7789_write_data_byte(0x00);
    st7789_write_data_byte(0x33);
    st7789_write_data_byte(0x33);
    
    // Gate control
    st7789_write_cmd(0xB7);
    st7789_write_data_byte(0x35);
    
    // VCOM setting
    st7789_write_cmd(0xBB);
    st7789_write_data_byte(0x19);
    
    // LCM control
    st7789_write_cmd(0xC0);
    st7789_write_data_byte(0x2C);
    
    // VDV and VRH command enable
    st7789_write_cmd(0xC2);
    st7789_write_data_byte(0x01);
    
    // VRH set
    st7789_write_cmd(0xC3);
    st7789_write_data_byte(0x12);
    
    // VDV set
    st7789_write_cmd(0xC4);
    st7789_write_data_byte(0x20);
    
    // Frame rate control
    st7789_write_cmd(0xC6);
    st7789_write_data_byte(0x0F);
    
    // Power control 1
    st7789_write_cmd(0xD0);
    st7789_write_data_byte(0xA4);
    st7789_write_data_byte(0xA1);
    
    // Positive voltage gamma control
    st7789_write_cmd(0xE0);
    st7789_write_data_byte(0xD0);
    st7789_write_data_byte(0x04);
    st7789_write_data_byte(0x0D);
    st7789_write_data_byte(0x11);
    st7789_write_data_byte(0x13);
    st7789_write_data_byte(0x2B);
    st7789_write_data_byte(0x3F);
    st7789_write_data_byte(0x54);
    st7789_write_data_byte(0x4C);
    st7789_write_data_byte(0x18);
    st7789_write_data_byte(0x0D);
    st7789_write_data_byte(0x0B);
    st7789_write_data_byte(0x1F);
    st7789_write_data_byte(0x23);
    
    // Negative voltage gamma control
    st7789_write_cmd(0xE1);
    st7789_write_data_byte(0xD0);
    st7789_write_data_byte(0x04);
    st7789_write_data_byte(0x0C);
    st7789_write_data_byte(0x11);
    st7789_write_data_byte(0x13);
    st7789_write_data_byte(0x2C);
    st7789_write_data_byte(0x3F);
    st7789_write_data_byte(0x44);
    st7789_write_data_byte(0x51);
    st7789_write_data_byte(0x2F);
    st7789_write_data_byte(0x1F);
    st7789_write_data_byte(0x1F);
    st7789_write_data_byte(0x20);
    st7789_write_data_byte(0x23);
    
    // Inversion on
    st7789_write_cmd(0x21);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Normal display mode on
    st7789_write_cmd(0x13);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Display on
    st7789_write_cmd(ST7789_DISPON);
    vTaskDelay(pdMS_TO_TICKS(120));
}

void test_display_orientations(void)
{
    ESP_LOGI(TAG, "Testing different display orientations and addressing...");
    
    uint8_t orientations[] = {0x00, 0x60, 0xC0, 0xA0, 0x08, 0x68, 0xC8, 0xA8};
    char* orientation_names[] = {
        "Normal", "90° CW", "180°", "270° CW", 
        "Normal+MV", "90° CW+MV", "180°+MV", "270° CW+MV"
    };
    
    for (int i = 0; i < 8; i++) {
        ESP_LOGI(TAG, "--- Testing orientation %d: %s (MADCTL=0x%02X) ---", 
                 i+1, orientation_names[i], orientations[i]);
        
        // Set memory access control
        st7789_write_cmd(ST7789_MADCTL);
        st7789_write_data_byte(orientations[i]);
        vTaskDelay(pdMS_TO_TICKS(10));
        
        // Try different window sizes to test addressing
        test_different_windows();
        
        vTaskDelay(pdMS_TO_TICKS(3000)); // 3 seconds per orientation
    }
}

void test_different_windows(void)
{
    // Test 1: Small centered window - CORRECTED for 172x320 display
    ESP_LOGI(TAG, "Testing small centered window...");
    
    // Set column address (centered 80x80 window)
    st7789_write_cmd(ST7789_CASET);
    st7789_write_data_byte(0x00);
    st7789_write_data_byte(46);  // Start X: (172-80)/2 = 46
    st7789_write_data_byte(0x00);
    st7789_write_data_byte(126); // End X: 46+80-1 = 125
    
    // Set row address
    st7789_write_cmd(ST7789_RASET);
    st7789_write_data_byte(0x00);
    st7789_write_data_byte(120); // Start Y: (320-80)/2 = 120
    st7789_write_data_byte(0x00);
    st7789_write_data_byte(200); // End Y: 120+80-1 = 199
    
    // Memory write with RED
    st7789_write_cmd(ST7789_RAMWR);
    uint8_t red_bytes[2] = {0xF8, 0x00}; // Red in RGB565
    for (int i = 0; i < 80 * 80; i++) {
        st7789_write_data(red_bytes, 2);
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Test 2: Full screen with CORRECT addressing for 172x320
    ESP_LOGI(TAG, "Testing full screen with correct 172x320 addressing...");
    
    // Column address: 0 to 171 (172 pixels wide)
    st7789_write_cmd(ST7789_CASET);
    st7789_write_data_byte(0x00);
    st7789_write_data_byte(0x00);
    st7789_write_data_byte(0x00);
    st7789_write_data_byte(171); // 172-1 = 171
    
    // Row address: 0 to 319 (320 pixels tall)
    st7789_write_cmd(ST7789_RASET);
    st7789_write_data_byte(0x00);
    st7789_write_data_byte(0x00);
    st7789_write_data_byte(0x01);
    st7789_write_data_byte(0x3F); // 320-1 = 319 = 0x013F
    
    // Memory write with WHITE
    st7789_write_cmd(ST7789_RAMWR);
    uint8_t white_bytes[2] = {0xFF, 0xFF};
    for (int i = 0; i < 172 * 320; i++) {
        st7789_write_data(white_bytes, 2);
    }
}

void fill_screen_color(uint16_t color)
{
    // Set column address with OFFSET_X
    st7789_write_cmd(ST7789_CASET);
    st7789_write_data_byte((OFFSET_X) >> 8);
    st7789_write_data_byte((OFFSET_X) & 0xFF);
    st7789_write_data_byte((OFFSET_X + LCD_WIDTH - 1) >> 8);
    st7789_write_data_byte((OFFSET_X + LCD_WIDTH - 1) & 0xFF);
    
    // Set row address with OFFSET_Y
    st7789_write_cmd(ST7789_RASET);
    st7789_write_data_byte((OFFSET_Y) >> 8);
    st7789_write_data_byte((OFFSET_Y) & 0xFF);
    st7789_write_data_byte((OFFSET_Y + LCD_HEIGHT - 1) >> 8);
    st7789_write_data_byte((OFFSET_Y + LCD_HEIGHT - 1) & 0xFF);
    
    // Memory write
    st7789_write_cmd(ST7789_RAMWR);
    
    // Send color data
    uint8_t color_bytes[2] = {color >> 8, color & 0xFF};
    for (int i = 0; i < LCD_WIDTH * LCD_HEIGHT; i++) {
        st7789_write_data(color_bytes, 2);
    }
}

void test_multiple_pin_configs(void)
{
    ESP_LOGI(TAG, "=== TESTING MULTIPLE PIN CONFIGURATIONS ===");
    
    // Configuration 1: Current standard config
    ESP_LOGI(TAG, "--- Config 1: Standard (MOSI:7 SCLK:6 CS:5 DC:4 RST:8) ---");
    // Already tested above
    
    vTaskDelay(pdMS_TO_TICKS(5000)); // Wait 5 seconds
    
    // Configuration 2: Alternative SPI pins
    ESP_LOGI(TAG, "--- Config 2: Alt SPI (MOSI:19 SCLK:18 CS:17 DC:16 RST:15) ---");
    test_pin_config(19, 18, 17, 16, 15, 21);
    
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    // Configuration 3: Another alternative
    ESP_LOGI(TAG, "--- Config 3: Alt SPI2 (MOSI:3 SCLK:2 CS:1 DC:0 RST:10) ---");
    test_pin_config(3, 2, 1, 0, 10, 21);
    
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    // Configuration 4: ESP32-C6 specific pins
    ESP_LOGI(TAG, "--- Config 4: C6 specific (MOSI:11 SCLK:12 CS:13 DC:14 RST:22) ---");
    test_pin_config(11, 12, 13, 14, 22, 21);
    
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    ESP_LOGI(TAG, "=== PIN CONFIGURATION TEST COMPLETE ===");
}

void test_pin_config(int mosi, int sclk, int cs, int dc, int rst, int bckl)
{
    ESP_LOGI(TAG, "Testing: MOSI:%d SCLK:%d CS:%d DC:%d RST:%d BCKL:%d", 
             mosi, sclk, cs, dc, rst, bckl);
    
    // Deinitialize current SPI
    if (spi != NULL) {
        spi_bus_remove_device(spi);
        spi_bus_free(SPI2_HOST);
        spi = NULL;
    }
    
    // Configure new pins
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << dc) | (1ULL << rst),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);
    
    // Initialize SPI with new pins
    spi_bus_config_t buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = mosi,
        .sclk_io_num = sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096
    };
    
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10*1000*1000,
        .mode = 0,
        .spics_io_num = cs,
        .queue_size = 7,
        .pre_cb = spi_pre_transfer_callback,
    };
    
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed for this config");
        return;
    }
    
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed for this config");
        spi_bus_free(SPI2_HOST);
        return;
    }
    
    // Update global pin variables for the callback
    PIN_DC = dc;
    PIN_RST = rst;
    
    // Reset and initialize display
    gpio_set_level(rst, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(rst, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Basic display initialization
    st7789_write_cmd(ST7789_SWRESET);
    vTaskDelay(pdMS_TO_TICKS(150));
    st7789_write_cmd(ST7789_SLPOUT);
    vTaskDelay(pdMS_TO_TICKS(10));
    st7789_write_cmd(ST7789_DISPON);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Test with bright white screen
    ESP_LOGI(TAG, "Sending WHITE screen test...");
    fill_screen_color(0xFFFF);
    
    ESP_LOGI(TAG, "Config test complete - CHECK DISPLAY NOW!");
}

void set_pixel(int x, int y, uint16_t color)
{
    if (x < 0 || x >= LCD_WIDTH || y < 0 || y >= LCD_HEIGHT) return;
    
    // Set window to single pixel
    st7789_write_cmd(ST7789_CASET);
    st7789_write_data_byte((OFFSET_X + x) >> 8);
    st7789_write_data_byte((OFFSET_X + x) & 0xFF);
    st7789_write_data_byte((OFFSET_X + x) >> 8);
    st7789_write_data_byte((OFFSET_X + x) & 0xFF);
    
    st7789_write_cmd(ST7789_RASET);
    st7789_write_data_byte((OFFSET_Y + y) >> 8);
    st7789_write_data_byte((OFFSET_Y + y) & 0xFF);
    st7789_write_data_byte((OFFSET_Y + y) >> 8);
    st7789_write_data_byte((OFFSET_Y + y) & 0xFF);
    
    st7789_write_cmd(ST7789_RAMWR);
    uint8_t color_bytes[2] = {color >> 8, color & 0xFF};
    st7789_write_data(color_bytes, 2);
}

void draw_filled_circle(int center_x, int center_y, int radius, uint16_t color)
{
    for (int y = -radius; y <= radius; y++) {
        for (int x = -radius; x <= radius; x++) {
            if (x * x + y * y <= radius * radius) {
                set_pixel_fast(center_x + x, center_y + y, color);
            }
        }
    }
}

void draw_circle(int center_x, int center_y, int radius, uint16_t color)
{
    int x = 0;
    int y = radius;
    int d = 3 - 2 * radius;
    
    while (y >= x) {
        set_pixel_fast(center_x + x, center_y + y, color);
        set_pixel_fast(center_x - x, center_y + y, color);
        set_pixel_fast(center_x + x, center_y - y, color);
        set_pixel_fast(center_x - x, center_y - y, color);
        set_pixel_fast(center_x + y, center_y + x, color);
        set_pixel_fast(center_x - y, center_y + x, color);
        set_pixel_fast(center_x + y, center_y - x, color);
        set_pixel_fast(center_x - y, center_y - x, color);
        
        if (d > 0) {
            y--;
            d = d + 4 * (x - y) + 10;
        } else {
            d = d + 4 * x + 6;
        }
        x++;
    }
}

void draw_ellipse_filled(int center_x, int center_y, int radius_x, int radius_y, uint16_t color)
{
    for (int y = -radius_y; y <= radius_y; y++) {
        for (int x = -radius_x; x <= radius_x; x++) {
            if ((x * x * radius_y * radius_y + y * y * radius_x * radius_x) <= (radius_x * radius_x * radius_y * radius_y)) {
                set_pixel_fast(center_x + x, center_y + y, color);
            }
        }
    }
}

uint32_t get_random_duration(void)
{
    static uint32_t seed = 1;
    seed = (seed * 1103515245 + 12345) & 0x7fffffff; // Simple LCG
    return ANIM_MIN_DURATION_MS + (seed % (ANIM_MAX_DURATION_MS - ANIM_MIN_DURATION_MS));
}

bool init_framebuffer(void)
{
    framebuffer = (uint16_t*)heap_caps_malloc(FRAMEBUFFER_SIZE, MALLOC_CAP_DMA);
    if (framebuffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate framebuffer memory");
        return false;
    }
    ESP_LOGI(TAG, "Framebuffer allocated: %d bytes", FRAMEBUFFER_SIZE);
    clear_framebuffer(0x0000); // Initialize to black
    return true;
}

void clear_framebuffer(uint16_t color)
{
    if (framebuffer == NULL) return;
    
    for (int i = 0; i < LCD_WIDTH * LCD_HEIGHT; i++) {
        framebuffer[i] = color;
    }
}

void set_pixel_fast(int x, int y, uint16_t color)
{
    if (framebuffer == NULL || x < 0 || x >= LCD_WIDTH || y < 0 || y >= LCD_HEIGHT) return;
    framebuffer[y * LCD_WIDTH + x] = color;
}

void display_framebuffer(void)
{
    if (framebuffer == NULL) return;
    
    // Set full screen window
    st7789_write_cmd(ST7789_CASET);
    st7789_write_data_byte((OFFSET_X) >> 8);
    st7789_write_data_byte((OFFSET_X) & 0xFF);
    st7789_write_data_byte((OFFSET_X + LCD_WIDTH - 1) >> 8);
    st7789_write_data_byte((OFFSET_X + LCD_WIDTH - 1) & 0xFF);
    
    st7789_write_cmd(ST7789_RASET);
    st7789_write_data_byte((OFFSET_Y) >> 8);
    st7789_write_data_byte((OFFSET_Y) & 0xFF);
    st7789_write_data_byte((OFFSET_Y + LCD_HEIGHT - 1) >> 8);
    st7789_write_data_byte((OFFSET_Y + LCD_HEIGHT - 1) & 0xFF);
    
    st7789_write_cmd(ST7789_RAMWR);
    
    // Send framebuffer in chunks to avoid SPI transfer size limits
    const int chunk_size = 4000; // ~4KB chunks (smaller than buffer limit)
    uint8_t* buffer_ptr = (uint8_t*)framebuffer;
    int remaining = FRAMEBUFFER_SIZE;
    
    while (remaining > 0) {
        int transfer_size = (remaining > chunk_size) ? chunk_size : remaining;
        st7789_write_data(buffer_ptr, transfer_size);
        buffer_ptr += transfer_size;
        remaining -= transfer_size;
    }
}

void draw_anime_eyes(void)
{
    draw_anime_eyes_looking(0, 0);
}

void draw_anime_eyes_looking(int pupil_offset_x, int pupil_offset_y)
{
    // Clear framebuffer with black background
    clear_framebuffer(0x0000);
    
    // Define eye positions (for 172x320 display)
    int left_eye_x = 43;   // Quarter of width
    int right_eye_x = 129; // Three quarters of width
    int eye_y = 160;       // Center height
    
    // Draw outer eye shape (white oval/circle)
    int outer_radius = 35;
    draw_filled_circle(left_eye_x, eye_y, outer_radius, 0xFFFF); // White
    draw_filled_circle(right_eye_x, eye_y, outer_radius, 0xFFFF); // White
    
    // Draw iris (blue) with offset for looking direction
    int iris_radius = 25;
    int max_offset = 8; // Limit pupil movement
    int safe_offset_x = (pupil_offset_x > max_offset) ? max_offset : (pupil_offset_x < -max_offset) ? -max_offset : pupil_offset_x;
    int safe_offset_y = (pupil_offset_y > max_offset) ? max_offset : (pupil_offset_y < -max_offset) ? -max_offset : pupil_offset_y;
    
    draw_filled_circle(left_eye_x + safe_offset_x, eye_y + safe_offset_y, iris_radius, 0x0016); // Pure blue
    draw_filled_circle(right_eye_x + safe_offset_x, eye_y + safe_offset_y, iris_radius, 0x0016); // Pure blue
    
    // Draw pupil (black) with same offset
    int pupil_radius = 12;
    draw_filled_circle(left_eye_x + safe_offset_x, eye_y + safe_offset_y, pupil_radius, 0x0000); // Black
    draw_filled_circle(right_eye_x + safe_offset_x, eye_y + safe_offset_y, pupil_radius, 0x0000); // Black
    
    // Draw light reflection (white highlight) with offset
    int highlight_radius = 4;
    draw_filled_circle(left_eye_x + safe_offset_x - 8, eye_y + safe_offset_y - 8, highlight_radius, 0xFFFF); // White
    draw_filled_circle(right_eye_x + safe_offset_x - 8, eye_y + safe_offset_y - 8, highlight_radius, 0xFFFF); // White
    
    // Draw smaller highlight with offset
    draw_filled_circle(left_eye_x + safe_offset_x + 6, eye_y + safe_offset_y - 12, 2, 0xFFFF); // White
    draw_filled_circle(right_eye_x + safe_offset_x + 6, eye_y + safe_offset_y - 12, 2, 0xFFFF); // White
    
    // Display the framebuffer
    display_framebuffer();
}

void draw_anime_eyes_blinking(bool left_closed, bool right_closed, float blink_amount)
{
    // Clear framebuffer with black background
    clear_framebuffer(0x0000);
    
    // Define eye positions
    int left_eye_x = 43;
    int right_eye_x = 129;
    int eye_y = 160;
    
    // Left eye
    if (left_closed) {
        // Draw closed eye as horizontal line
        for (int x = -30; x <= 30; x++) {
            set_pixel_fast(left_eye_x + x, eye_y, 0xFFFF);
            set_pixel_fast(left_eye_x + x, eye_y + 1, 0xFFFF);
            set_pixel_fast(left_eye_x + x, eye_y - 1, 0xFFFF);
        }
    } else {
        // Draw normal round eye with eyelid coverage based on blink_amount
        int outer_radius = 35;
        int iris_radius = 25;
        int pupil_radius = 12;
        
        // Draw full round eye first
        draw_filled_circle(left_eye_x, eye_y, outer_radius, 0xFFFF);
        draw_filled_circle(left_eye_x, eye_y, iris_radius, 0x0016);
        draw_filled_circle(left_eye_x, eye_y, pupil_radius, 0x0000);
        
        // Draw eyelids covering part of the eye based on blink_amount
        if (blink_amount > 0) {
            int eyelid_coverage = (int)(outer_radius * blink_amount);
            // Top eyelid
            for (int y = eye_y - outer_radius; y < eye_y - outer_radius + eyelid_coverage; y++) {
                for (int x = left_eye_x - outer_radius; x <= left_eye_x + outer_radius; x++) {
                    int dx = x - left_eye_x;
                    int dy = y - eye_y;
                    if (dx * dx + dy * dy <= outer_radius * outer_radius) {
                        set_pixel_fast(x, y, 0x0000); // Black eyelid
                    }
                }
            }
            // Bottom eyelid
            for (int y = eye_y + outer_radius - eyelid_coverage; y <= eye_y + outer_radius; y++) {
                for (int x = left_eye_x - outer_radius; x <= left_eye_x + outer_radius; x++) {
                    int dx = x - left_eye_x;
                    int dy = y - eye_y;
                    if (dx * dx + dy * dy <= outer_radius * outer_radius) {
                        set_pixel_fast(x, y, 0x0000); // Black eyelid
                    }
                }
            }
        }
        
        // Highlights (only if eye is mostly open)
        if (blink_amount < 0.5) {
            draw_filled_circle(left_eye_x - 8, eye_y - 8, 4, 0xFFFF);
            draw_filled_circle(left_eye_x + 6, eye_y - 12, 2, 0xFFFF);
        }
    }
    
    // Right eye
    if (right_closed) {
        // Draw closed eye as horizontal line
        for (int x = -30; x <= 30; x++) {
            set_pixel_fast(right_eye_x + x, eye_y, 0xFFFF);
            set_pixel_fast(right_eye_x + x, eye_y + 1, 0xFFFF);
            set_pixel_fast(right_eye_x + x, eye_y - 1, 0xFFFF);
        }
    } else {
        // Draw normal round eye with eyelid coverage based on blink_amount
        int outer_radius = 35;
        int iris_radius = 25;
        int pupil_radius = 12;
        
        // Draw full round eye first
        draw_filled_circle(right_eye_x, eye_y, outer_radius, 0xFFFF);
        draw_filled_circle(right_eye_x, eye_y, iris_radius, 0x0016);
        draw_filled_circle(right_eye_x, eye_y, pupil_radius, 0x0000);
        
        // Draw eyelids covering part of the eye based on blink_amount
        if (blink_amount > 0) {
            int eyelid_coverage = (int)(outer_radius * blink_amount);
            // Top eyelid
            for (int y = eye_y - outer_radius; y < eye_y - outer_radius + eyelid_coverage; y++) {
                for (int x = right_eye_x - outer_radius; x <= right_eye_x + outer_radius; x++) {
                    int dx = x - right_eye_x;
                    int dy = y - eye_y;
                    if (dx * dx + dy * dy <= outer_radius * outer_radius) {
                        set_pixel_fast(x, y, 0x0000); // Black eyelid
                    }
                }
            }
            // Bottom eyelid
            for (int y = eye_y + outer_radius - eyelid_coverage; y <= eye_y + outer_radius; y++) {
                for (int x = right_eye_x - outer_radius; x <= right_eye_x + outer_radius; x++) {
                    int dx = x - right_eye_x;
                    int dy = y - eye_y;
                    if (dx * dx + dy * dy <= outer_radius * outer_radius) {
                        set_pixel_fast(x, y, 0x0000); // Black eyelid
                    }
                }
            }
        }
        
        // Highlights (only if eye is mostly open)
        if (blink_amount < 0.5) {
            draw_filled_circle(right_eye_x - 8, eye_y - 8, 4, 0xFFFF);
            draw_filled_circle(right_eye_x + 6, eye_y - 12, 2, 0xFFFF);
        }
    }
    
    // Display the framebuffer
    display_framebuffer();
}

void draw_anime_eyes_natural_blink(float blink_progress)
{
    // Clear framebuffer with black background
    clear_framebuffer(0x0000);
    
    // Define eye positions
    int left_eye_x = 43;
    int right_eye_x = 129;
    int eye_y = 160;
    int outer_radius = 35;
    int iris_radius = 25;
    int pupil_radius = 12;
    
    // Draw full round eyes first (both eyes)
    draw_filled_circle(left_eye_x, eye_y, outer_radius, 0xFFFF);
    draw_filled_circle(right_eye_x, eye_y, outer_radius, 0xFFFF);
    
    // Draw iris and pupils (both eyes)
    draw_filled_circle(left_eye_x, eye_y, iris_radius, 0x0016);
    draw_filled_circle(right_eye_x, eye_y, iris_radius, 0x0016);
    
    draw_filled_circle(left_eye_x, eye_y, pupil_radius, 0x0000);
    draw_filled_circle(right_eye_x, eye_y, pupil_radius, 0x0000);
    
    // Natural eyelid closing with circular motion from top
    if (blink_progress > 0.0) {
        // Calculate eyelid position (circular eyelid moving down)
        int max_eyelid_drop = outer_radius + 15; // Eyelid travels beyond eye center
        int eyelid_drop = (int)(max_eyelid_drop * blink_progress);
        
        // Eyelid center position moves down from above the eye
        int eyelid_center_y = eye_y - outer_radius - 10 + eyelid_drop;
        int eyelid_radius = outer_radius + 8; // Slightly larger than eye to cover completely
        
        // Draw eyelid as large black circle covering the eye from top
        // Left eye eyelid
        draw_filled_circle(left_eye_x, eyelid_center_y, eyelid_radius, 0x0000);
        
        // Right eye eyelid  
        draw_filled_circle(right_eye_x, eyelid_center_y, eyelid_radius, 0x0000);
    }
    
    // Highlights (only show when eyes are mostly open)
    if (blink_progress < 0.3) {
        draw_filled_circle(left_eye_x - 8, eye_y - 8, 4, 0xFFFF);
        draw_filled_circle(left_eye_x + 6, eye_y - 12, 2, 0xFFFF);
        
        draw_filled_circle(right_eye_x - 8, eye_y - 8, 4, 0xFFFF);
        draw_filled_circle(right_eye_x + 6, eye_y - 12, 2, 0xFFFF);
    }
    
    // Display the framebuffer
    display_framebuffer();
}

void draw_anime_eyes_laughing(uint32_t time_offset)
{
    // Clear framebuffer with black background
    clear_framebuffer(0x0000);
    
    // Base eye positions
    int base_left_x = 43;
    int base_right_x = 129;
    int base_eye_y = 160;
    
    // Natural laughter bouncing - not sine wave, but natural rhythm
    // Create a bouncing pattern: quick up, slower down, pause, repeat
    int laugh_cycle = 800; // 800ms cycle for natural laughter rhythm
    int cycle_pos = time_offset % laugh_cycle;
    
    int bounce_y = 0;
    if (cycle_pos < 150) {
        // Quick bounce up (0-150ms)
        float bounce_progress = (float)cycle_pos / 150.0;
        bounce_y = -(int)(6 * bounce_progress); // Up to 6 pixels up
    } else if (cycle_pos < 400) {
        // Slower fall down (150-400ms) 
        float fall_progress = (float)(cycle_pos - 150) / 250.0;
        bounce_y = -6 + (int)(8 * fall_progress); // From -6 to +2 pixels
    } else if (cycle_pos < 550) {
        // Small settle bounce (400-550ms)
        float settle_progress = (float)(cycle_pos - 400) / 150.0;
        bounce_y = 2 - (int)(2 * settle_progress); // From +2 to 0 pixels
    }
    // Rest of cycle (550-800ms): eyes at normal position
    
    // Current eye positions with bounce
    int left_eye_x = base_left_x;
    int right_eye_x = base_right_x;
    int eye_y = base_eye_y + bounce_y;
    
    // Draw base eye structure
    int outer_radius = 35;
    int iris_radius = 25;
    int pupil_radius = 12;
    
    // Draw full round eyes
    draw_filled_circle(left_eye_x, eye_y, outer_radius, 0xFFFF);
    draw_filled_circle(right_eye_x, eye_y, outer_radius, 0xFFFF);
    
    // Draw iris and pupils
    draw_filled_circle(left_eye_x, eye_y, iris_radius, 0x0016);
    draw_filled_circle(right_eye_x, eye_y, iris_radius, 0x0016);
    
    draw_filled_circle(left_eye_x, eye_y, pupil_radius, 0x0000);
    draw_filled_circle(right_eye_x, eye_y, pupil_radius, 0x0000);
    
    // Add lower eyelids coming from below (happy squinting)
    // These don't close fully - they create the happy squint look
    int lower_eyelid_height = 12; // How much the lower eyelid rises
    int lower_eyelid_radius = outer_radius + 5; // Slightly larger than eye
    
    // Calculate lower eyelid position (coming up from below)
    int lower_eyelid_center_y = eye_y + outer_radius + 5 - lower_eyelid_height;
    
    // Draw lower eyelids (black circles from below)
    draw_filled_circle(left_eye_x, lower_eyelid_center_y, lower_eyelid_radius, 0x0000);
    draw_filled_circle(right_eye_x, lower_eyelid_center_y, lower_eyelid_radius, 0x0000);
    
    // Happy highlights (slightly moved due to squinting)
    draw_filled_circle(left_eye_x - 6, eye_y - 6, 4, 0xFFFF);
    draw_filled_circle(left_eye_x + 8, eye_y - 10, 2, 0xFFFF);
    
    draw_filled_circle(right_eye_x - 6, eye_y - 6, 4, 0xFFFF);
    draw_filled_circle(right_eye_x + 8, eye_y - 10, 2, 0xFFFF);
    
    // Display the framebuffer
    display_framebuffer();
}

void draw_anime_eyes_suspicious(void)
{
    // Clear framebuffer with black background
    clear_framebuffer(0x0000);
    
    // Define eye positions
    int left_eye_x = 43;
    int right_eye_x = 129;
    int eye_y = 160;
    int outer_radius = 35;
    int iris_radius = 25;
    int pupil_radius = 12;
    int side_offset = 8; // Looking to the side (suspicious look)
    
    // Draw full round eyes first
    draw_filled_circle(left_eye_x, eye_y, outer_radius, 0xFFFF);
    draw_filled_circle(right_eye_x, eye_y, outer_radius, 0xFFFF);
    
    // Draw iris and pupils looking to the side
    draw_filled_circle(left_eye_x + side_offset, eye_y, iris_radius, 0x0016);
    draw_filled_circle(right_eye_x + side_offset, eye_y, iris_radius, 0x0016);
    
    draw_filled_circle(left_eye_x + side_offset, eye_y, pupil_radius, 0x0000);
    draw_filled_circle(right_eye_x + side_offset, eye_y, pupil_radius, 0x0000);
    
    // Add suspicious sliding eyelids (black curtains from top and bottom)
    int eyelid_closure = 20; // How much the eyelids close in
    
    // Left eye eyelids
    // Top eyelid (black rectangle sliding down)
    for (int y = eye_y - outer_radius; y < eye_y - outer_radius + eyelid_closure; y++) {
        for (int x = left_eye_x - outer_radius; x <= left_eye_x + outer_radius; x++) {
            int dx = x - left_eye_x;
            int dy = y - eye_y;
            if (dx * dx + dy * dy <= outer_radius * outer_radius) {
                set_pixel_fast(x, y, 0x0000); // Black eyelid
            }
        }
    }
    // Bottom eyelid (black rectangle sliding up)
    for (int y = eye_y + outer_radius - eyelid_closure; y <= eye_y + outer_radius; y++) {
        for (int x = left_eye_x - outer_radius; x <= left_eye_x + outer_radius; x++) {
            int dx = x - left_eye_x;
            int dy = y - eye_y;
            if (dx * dx + dy * dy <= outer_radius * outer_radius) {
                set_pixel_fast(x, y, 0x0000); // Black eyelid
            }
        }
    }
    
    // Right eye eyelids
    // Top eyelid
    for (int y = eye_y - outer_radius; y < eye_y - outer_radius + eyelid_closure; y++) {
        for (int x = right_eye_x - outer_radius; x <= right_eye_x + outer_radius; x++) {
            int dx = x - right_eye_x;
            int dy = y - eye_y;
            if (dx * dx + dy * dy <= outer_radius * outer_radius) {
                set_pixel_fast(x, y, 0x0000); // Black eyelid
            }
        }
    }
    // Bottom eyelid
    for (int y = eye_y + outer_radius - eyelid_closure; y <= eye_y + outer_radius; y++) {
        for (int x = right_eye_x - outer_radius; x <= right_eye_x + outer_radius; x++) {
            int dx = x - right_eye_x;
            int dy = y - eye_y;
            if (dx * dx + dy * dy <= outer_radius * outer_radius) {
                set_pixel_fast(x, y, 0x0000); // Black eyelid
            }
        }
    }
    
    // Small highlights in the visible slit
    draw_filled_circle(left_eye_x + side_offset - 4, eye_y - 2, 2, 0xFFFF);
    draw_filled_circle(right_eye_x + side_offset - 4, eye_y - 2, 2, 0xFFFF);
    
    // Display the framebuffer
    display_framebuffer();
}

void draw_anime_eyes_suspicious_animated(float eyelid_progress, float pupil_progress)
{
    // Clear framebuffer with black background
    clear_framebuffer(0x0000);
    
    // Define eye positions
    int left_eye_x = 43;
    int right_eye_x = 129;
    int eye_y = 160;
    int outer_radius = 35;
    int iris_radius = 25;
    int pupil_radius = 12;
    
    // Calculate pupil movement (gradually moving to the side)
    int max_side_offset = 8;
    int side_offset = (int)(max_side_offset * pupil_progress);
    
    // Draw full round eyes first
    draw_filled_circle(left_eye_x, eye_y, outer_radius, 0xFFFF);
    draw_filled_circle(right_eye_x, eye_y, outer_radius, 0xFFFF);
    
    // Draw iris and pupils looking to the side (gradually)
    draw_filled_circle(left_eye_x + side_offset, eye_y, iris_radius, 0x0016);
    draw_filled_circle(right_eye_x + side_offset, eye_y, iris_radius, 0x0016);
    
    draw_filled_circle(left_eye_x + side_offset, eye_y, pupil_radius, 0x0000);
    draw_filled_circle(right_eye_x + side_offset, eye_y, pupil_radius, 0x0000);
    
    // Gradually closing eyelids (black curtains from top and bottom)
    int max_eyelid_closure = 20;
    int eyelid_closure = (int)(max_eyelid_closure * eyelid_progress);
    
    if (eyelid_closure > 0) {
        // Left eye eyelids
        // Top eyelid (black rectangle sliding down gradually)
        for (int y = eye_y - outer_radius; y < eye_y - outer_radius + eyelid_closure; y++) {
            for (int x = left_eye_x - outer_radius; x <= left_eye_x + outer_radius; x++) {
                int dx = x - left_eye_x;
                int dy = y - eye_y;
                if (dx * dx + dy * dy <= outer_radius * outer_radius) {
                    set_pixel_fast(x, y, 0x0000); // Black eyelid
                }
            }
        }
        // Bottom eyelid (black rectangle sliding up gradually)
        for (int y = eye_y + outer_radius - eyelid_closure; y <= eye_y + outer_radius; y++) {
            for (int x = left_eye_x - outer_radius; x <= left_eye_x + outer_radius; x++) {
                int dx = x - left_eye_x;
                int dy = y - eye_y;
                if (dx * dx + dy * dy <= outer_radius * outer_radius) {
                    set_pixel_fast(x, y, 0x0000); // Black eyelid
                }
            }
        }
        
        // Right eye eyelids
        // Top eyelid
        for (int y = eye_y - outer_radius; y < eye_y - outer_radius + eyelid_closure; y++) {
            for (int x = right_eye_x - outer_radius; x <= right_eye_x + outer_radius; x++) {
                int dx = x - right_eye_x;
                int dy = y - eye_y;
                if (dx * dx + dy * dy <= outer_radius * outer_radius) {
                    set_pixel_fast(x, y, 0x0000); // Black eyelid
                }
            }
        }
        // Bottom eyelid
        for (int y = eye_y + outer_radius - eyelid_closure; y <= eye_y + outer_radius; y++) {
            for (int x = right_eye_x - outer_radius; x <= right_eye_x + outer_radius; x++) {
                int dx = x - right_eye_x;
                int dy = y - eye_y;
                if (dx * dx + dy * dy <= outer_radius * outer_radius) {
                    set_pixel_fast(x, y, 0x0000); // Black eyelid
                }
            }
        }
    }
    
    // Highlights (dimmer as eyelids close, return as they open)
    if (eyelid_progress < 0.8) { // Show highlights when eyes are sufficiently open
        // Adjust highlight position based on pupil position and eyelid state
        int highlight_y_offset = (eyelid_progress < 0.5) ? -8 : -4; // Move highlight down as eyelids close
        int highlight_x_offset = (int)(side_offset * 0.5); // Follow pupils partially
        
        // Main highlights
        draw_filled_circle(left_eye_x + highlight_x_offset - 4, eye_y + highlight_y_offset, 2, 0xFFFF);
        draw_filled_circle(right_eye_x + highlight_x_offset - 4, eye_y + highlight_y_offset, 2, 0xFFFF);
        
        // Secondary highlights (only when eyes are more open)
        if (eyelid_progress < 0.4) {
            draw_filled_circle(left_eye_x + highlight_x_offset - 8, eye_y - 8, 3, 0xFFFF);
            draw_filled_circle(right_eye_x + highlight_x_offset - 8, eye_y - 8, 3, 0xFFFF);
        }
    }
    
    // Display the framebuffer
    display_framebuffer();
}

void draw_zzz_letters(uint32_t time_offset)
{
    // Floating zZz letters above the eyes
    int base_x = 86; // Center of screen
    int base_y = 80;  // Above the eyes
    
    // Three Z letters at different positions and sizes
    char z_chars[] = {'z', 'Z', 'z'};
    int z_positions[] = {-20, 0, 20}; // X offsets
    int z_heights[] = {0, -15, -30};   // Y offsets
    int z_sizes[] = {8, 12, 10};      // Different sizes
    
    for (int i = 0; i < 3; i++) {
        // Floating motion with sine waves
        float time_factor = (float)(time_offset + i * 1000) / 1000.0;
        int float_x = (int)(5 * sin(time_factor * 0.8));
        int float_y = (int)(3 * sin(time_factor * 1.2));
        
        // Size variation
        float size_factor = 0.8 + 0.4 * sin(time_factor * 0.6);
        int letter_size = (int)(z_sizes[i] * size_factor);
        
        int letter_x = base_x + z_positions[i] + float_x;
        int letter_y = base_y + z_heights[i] + float_y;
        
        // Draw simple Z letter as lines (simplified for LCD)
        if (letter_size > 5) {
            // Draw Z shape with white pixels
            for (int x = -letter_size/2; x <= letter_size/2; x++) {
                set_pixel_fast(letter_x + x, letter_y - letter_size/2, 0xFFFF); // Top line
                set_pixel_fast(letter_x + x, letter_y + letter_size/2, 0xFFFF); // Bottom line
            }
            // Diagonal line
            for (int d = -letter_size/2; d <= letter_size/2; d++) {
                set_pixel_fast(letter_x - d, letter_y + d, 0xFFFF);
            }
        }
    }
}

void draw_anime_eyes_sleeping(float sleep_progress)
{
    // Clear framebuffer with black background
    clear_framebuffer(0x0000);
    
    // Define eye positions
    int left_eye_x = 43;
    int right_eye_x = 129;
    int eye_y = 160;
    int outer_radius = 35;
    int iris_radius = 25;
    int pupil_radius = 12;
    
    if (sleep_progress < 1.0 && sleep_progress >= 0.0) {
        // Phase 1 & 3: Eyes closing/opening gradually with pupils moving down/up
        
        // Draw full round eyes first
        draw_filled_circle(left_eye_x, eye_y, outer_radius, 0xFFFF);
        draw_filled_circle(right_eye_x, eye_y, outer_radius, 0xFFFF);
        
        // Pupils gradually move down (closing) or up (opening)
        int pupil_drop = (int)(8 * sleep_progress); // Max 8 pixels down
        
        // Draw iris and pupils with movement
        draw_filled_circle(left_eye_x, eye_y + pupil_drop, iris_radius, 0x0016);
        draw_filled_circle(right_eye_x, eye_y + pupil_drop, iris_radius, 0x0016);
        
        draw_filled_circle(left_eye_x, eye_y + pupil_drop, pupil_radius, 0x0000);
        draw_filled_circle(right_eye_x, eye_y + pupil_drop, pupil_radius, 0x0000);
        
        // Gradual eyelid closing/opening
        if (sleep_progress > 0.0) {
            int max_eyelid_drop = outer_radius + 15;
            int eyelid_drop = (int)(max_eyelid_drop * sleep_progress);
            int eyelid_center_y = eye_y - outer_radius - 10 + eyelid_drop;
            int eyelid_radius = outer_radius + 8;
            
            // Draw eyelids covering both eyes
            draw_filled_circle(left_eye_x, eyelid_center_y, eyelid_radius, 0x0000);
            draw_filled_circle(right_eye_x, eyelid_center_y, eyelid_radius, 0x0000);
        }
        
        // Highlights fade as eyes close, return as eyes open
        if (sleep_progress < 0.3) {
            draw_filled_circle(left_eye_x - 8, eye_y - 8, 4, 0xFFFF);
            draw_filled_circle(left_eye_x + 6, eye_y - 12, 2, 0xFFFF);
            draw_filled_circle(right_eye_x - 8, eye_y - 8, 4, 0xFFFF);
            draw_filled_circle(right_eye_x + 6, eye_y - 12, 2, 0xFFFF);
        }
    } else {
        // Phase 2: Fully asleep - draw sleepy eye lines (semicircles)
        
        // Draw sleepy eye lines as curved arcs with ends pointing up
        for (int x = -25; x <= 25; x++) {
            // Create curved line with upward-pointing ends
            float curve_factor = (float)x / 25.0;
            int curve_y = eye_y + (int)(5 * (1.0 - curve_factor * curve_factor)); // Upward curve
            
            // Draw thicker lines for better visibility
            set_pixel_fast(left_eye_x + x, curve_y, 0xFFFF);
            set_pixel_fast(left_eye_x + x, curve_y + 1, 0xFFFF);
            set_pixel_fast(left_eye_x + x, curve_y - 1, 0xFFFF);
            
            set_pixel_fast(right_eye_x + x, curve_y, 0xFFFF);
            set_pixel_fast(right_eye_x + x, curve_y + 1, 0xFFFF);
            set_pixel_fast(right_eye_x + x, curve_y - 1, 0xFFFF);
        }
        
        // Draw floating zZz letters
        draw_zzz_letters((uint32_t)(sleep_progress * 1000));
    }
    
    // Display the framebuffer
    display_framebuffer();
}

void draw_anime_eyes_angry(uint32_t time_offset)
{
    // Clear framebuffer with black background
    clear_framebuffer(0x0000);
    
    // Define eye positions
    int left_eye_x = 43;
    int right_eye_x = 129;
    int eye_y = 160;
    
    // Natural anger animation timing (3 second cycle: 1s building up, 1s hold, 1s return)
    int anger_cycle = 3000; // 3000ms cycle
    int cycle_pos = time_offset % anger_cycle;
    float anger_progress;
    
    if (cycle_pos < 1000) {
        // Phase 1: Building anger (0-1s)
        anger_progress = (float)cycle_pos / 1000.0; // 0.0 to 1.0
    } else if (cycle_pos < 2000) {
        // Phase 2: Hold angry expression (1-2s) 
        anger_progress = 1.0; // Hold at maximum
    } else {
        // Phase 3: Return to normal (2-3s)
        float return_progress = (float)(cycle_pos - 2000) / 1000.0;
        anger_progress = 1.0 - return_progress; // 1.0 to 0.0
    }
    
    // Base eye measurements
    int outer_radius = 35;
    int normal_iris_radius = 25;
    int normal_pupil_radius = 12;
    
    // Slightly bigger and darker pupils when angry (but not completely black)
    int angry_pupil_radius = normal_pupil_radius + (int)(3 * anger_progress); // 12 to 15
    int angry_iris_radius = normal_iris_radius - (int)(3 * anger_progress);   // 25 to 22
    
    // Inward tilt for squinting effect (pupils move slightly toward nose)
    int squint_offset_x = (int)(4 * anger_progress); // 0 to 4 pixels inward
    
    // Draw base eyes
    draw_filled_circle(left_eye_x, eye_y, outer_radius, 0xFFFF);
    draw_filled_circle(right_eye_x, eye_y, outer_radius, 0xFFFF);
    
    // Draw iris with squint (pure blue, slightly narrower when angry)
    draw_filled_circle(left_eye_x + squint_offset_x, eye_y, angry_iris_radius, 0x0016);
    draw_filled_circle(right_eye_x - squint_offset_x, eye_y, angry_iris_radius, 0x0016);
    
    // Draw pupils (darker but not completely black - dark gray when angry)
    uint16_t pupil_color = anger_progress > 0.5 ? 0x2104 : 0x0000; // Dark gray or black
    draw_filled_circle(left_eye_x + squint_offset_x, eye_y, angry_pupil_radius, pupil_color);
    draw_filled_circle(right_eye_x - squint_offset_x, eye_y, angry_pupil_radius, pupil_color);
    
    // Natural frowning brows - angled upper eyelids coming down
    if (anger_progress > 0.0) {
        int max_brow_drop = 15; // How far down the "brow" eyelid comes
        int brow_drop = (int)(max_brow_drop * anger_progress);
        
        // Left eye frowning brow - angled eyelid from outer corner down and inward
        for (int x = -outer_radius; x <= outer_radius; x++) {
            // Create angled frowning shape: higher on outer edge, lower on inner edge
            int brow_height = brow_drop + (int)((float)x / outer_radius * 8); // 8 pixel angle difference
            if (brow_height < 0) brow_height = 0;
            
            for (int y = 0; y < brow_height; y++) {
                int pixel_x = left_eye_x + x;
                int pixel_y = eye_y - outer_radius + y;
                
                // Only draw if within eye circle and screen bounds
                int dx = pixel_x - left_eye_x;
                int dy = pixel_y - eye_y;
                if (dx * dx + dy * dy <= outer_radius * outer_radius && 
                    pixel_x >= 0 && pixel_x < LCD_WIDTH && pixel_y >= 0 && pixel_y < LCD_HEIGHT) {
                    set_pixel_fast(pixel_x, pixel_y, 0x0000); // Black frowning eyelid
                }
            }
        }
        
        // Right eye frowning brow - angled eyelid from outer corner down and inward  
        for (int x = -outer_radius; x <= outer_radius; x++) {
            // Create angled frowning shape: higher on outer edge, lower on inner edge
            int brow_height = brow_drop + (int)((float)(-x) / outer_radius * 8); // Mirrored angle
            if (brow_height < 0) brow_height = 0;
            
            for (int y = 0; y < brow_height; y++) {
                int pixel_x = right_eye_x + x;
                int pixel_y = eye_y - outer_radius + y;
                
                // Only draw if within eye circle and screen bounds
                int dx = pixel_x - right_eye_x;
                int dy = pixel_y - eye_y;
                if (dx * dx + dy * dy <= outer_radius * outer_radius && 
                    pixel_x >= 0 && pixel_x < LCD_WIDTH && pixel_y >= 0 && pixel_y < LCD_HEIGHT) {
                    set_pixel_fast(pixel_x, pixel_y, 0x0000); // Black frowning eyelid
                }
            }
        }
    }
    
    // Subdued highlights (angry eyes have less sparkle)
    if (anger_progress < 0.7) { // Highlights fade as anger increases
        int highlight_intensity = (int)((1.0 - anger_progress) * 255);
        uint16_t highlight_color = (highlight_intensity > 128) ? 0xFFFF : 0x8410; // Dim or bright
        
        draw_filled_circle(left_eye_x + squint_offset_x - 6, eye_y - 6, 3, highlight_color);
        draw_filled_circle(right_eye_x - squint_offset_x - 6, eye_y - 6, 3, highlight_color);
    }
    
    // Display the framebuffer
    display_framebuffer();
}

void animate_eyes(void)
{
    static animation_state_t current_state = ANIM_LOOK_LEFT_RIGHT;
    static uint32_t last_state_change = 0;
    static uint32_t state_duration = 4000;
    static uint32_t last_sleep_time = 0;
    static int animation_step = 0;
    
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Check for sleep animation (special timing - every 30+ seconds)
    if (current_time - last_sleep_time >= SLEEP_MIN_INTERVAL_MS && current_state != ANIM_SLEEP) {
        // Random chance for sleep (20% chance when interval is met)
        static uint32_t sleep_seed = 789;
        sleep_seed = (sleep_seed * 1103515245 + 12345) & 0x7fffffff;
        if ((sleep_seed % 5) == 0) { // 20% chance
            current_state = ANIM_SLEEP;
            last_state_change = current_time;
            last_sleep_time = current_time;
            state_duration = SLEEP_DURATION_MS;
            ESP_LOGI(TAG, "💤 Sleep animation started");
        }
    }
    
    // Check if it's time to change animation state
    if (current_time - last_state_change >= state_duration) {
        last_state_change = current_time;
        
        if (current_state == ANIM_SLEEP) {
            // After sleep completes, always go to default looking animation
            current_state = ANIM_LOOK_LEFT_RIGHT;
            state_duration = get_random_duration();
            ESP_LOGI(TAG, "💤 Sleep animation completed, returning to normal looking");
        } else if (current_state == ANIM_LOOK_LEFT_RIGHT) {
            // From default animation, can go to any other animation (except sleep)
            state_duration = get_random_duration();
            
            // Random next state (excluding sleep and current default state)
            static uint32_t rnd_seed = 123;
            rnd_seed = (rnd_seed * 1103515245 + 12345) & 0x7fffffff;
            animation_state_t next_state = (rnd_seed % (ANIM_COUNT - 2)); // Exclude ANIM_SLEEP
            if (next_state >= ANIM_LOOK_LEFT_RIGHT) next_state++; // Skip the default state
            current_state = next_state;
            
            ESP_LOGI(TAG, "From default to animation state: %d", current_state);
        } else {
            // After any special animation (blink, smile, suspicious), return to default
            current_state = ANIM_LOOK_LEFT_RIGHT;
            state_duration = get_random_duration();
            ESP_LOGI(TAG, "Special animation completed, returning to default looking");
        }
        
        animation_step = 0;
    }
    
    // Execute current animation
    switch (current_state) {
        case ANIM_LOOK_LEFT_RIGHT: {
            // Slower eye scan with more frames
            int cycle_time = 4000; // 4 seconds for full cycle (slower)
            int cycle_pos = (current_time - last_state_change) % cycle_time;
            float progress = (float)cycle_pos / cycle_time;
            
            int pupil_x;
            if (progress < 0.25) {
                // Moving slowly from center to left
                pupil_x = (int)(-8 * (progress * 4));
            } else if (progress < 0.5) {
                // Pause at left position
                pupil_x = -8;
            } else if (progress < 0.75) {
                // Moving slowly from left to right
                float move_progress = (progress - 0.5) * 4;
                pupil_x = (int)(-8 + 16 * move_progress);
            } else {
                // Pause at right position, then return to center
                float return_progress = (progress - 0.75) * 4;
                pupil_x = (int)(8 - 8 * return_progress);
            }
            
            draw_anime_eyes_looking(pupil_x, 0);
            break;
        }
        
        case ANIM_BLINK_NATURAL: {
            // Natural synchronized blinking with random timing within animation duration
            static uint32_t last_blink_time = 0;
            static uint32_t next_blink_delay = 2000; // Initial delay
            
            uint32_t time_since_blink = current_time - last_blink_time;
            
            if (time_since_blink >= next_blink_delay) {
                // Time for a blink - reset timing for next blink
                last_blink_time = current_time;
                
                // Random delay between 1-5 seconds for next blink
                static uint32_t blink_seed = 456;
                blink_seed = (blink_seed * 1103515245 + 12345) & 0x7fffffff;
                next_blink_delay = 1000 + (blink_seed % 4000); // 1000-5000ms
            }
            
            // Check if currently in blink animation (200ms duration)
            uint32_t blink_progress_time = time_since_blink;
            if (blink_progress_time < 200) {
                // Currently blinking
                float blink_progress = (float)blink_progress_time / 200.0;
                
                // Smooth blink curve (slow-fast-slow)
                if (blink_progress < 0.5) {
                    // Closing (accelerate)
                    blink_progress = blink_progress * blink_progress * 2;
                } else {
                    // Opening (decelerate)
                    float open_progress = (blink_progress - 0.5) * 2;
                    blink_progress = 1.0 - (1.0 - open_progress) * (1.0 - open_progress);
                }
                
                draw_anime_eyes_natural_blink(blink_progress);
            } else {
                // Eyes open, normal looking
                draw_anime_eyes_looking(0, 0);
            }
            break;
        }
        
        case ANIM_SMILE: {
            // Animated laughing eyes with natural bouncing
            uint32_t laugh_time = current_time - last_state_change;
            draw_anime_eyes_laughing(laugh_time);
            break;
        }
        
        case ANIM_SUSPICIOUS: {
            // Animated suspicious look with gradually closing eyelids and moving pupils, then opening
            int animation_time = 4500; // 4.5 seconds for complete suspicious cycle
            int anim_pos = (current_time - last_state_change) % animation_time;
            float anim_progress = (float)anim_pos / animation_time;
            
            float eyelid_progress, pupil_progress;
            
            if (anim_progress < 0.3) {
                // First 30%: Pupils start moving sideways, eyelids begin closing
                pupil_progress = anim_progress / 0.3; // 0.0 to 1.0
                eyelid_progress = (anim_progress / 0.3) * 0.5; // 0.0 to 0.5
            } else if (anim_progress < 0.5) {
                // Next 20%: Continue closing eyelids, pupils fully to side
                pupil_progress = 1.0; // Fully to the side
                float close_progress = (anim_progress - 0.3) / 0.2;
                eyelid_progress = 0.5 + (close_progress * 0.5); // 0.5 to 1.0
            } else if (anim_progress < 0.75) {
                // Middle 25%: Hold the suspicious look
                pupil_progress = 1.0; // Keep pupils to the side
                eyelid_progress = 1.0; // Keep eyelids mostly closed
            } else {
                // Last 25%: Opening back up - pupils return to center, eyelids open
                float open_progress = (anim_progress - 0.75) / 0.25; // 0.0 to 1.0
                pupil_progress = 1.0 - open_progress; // 1.0 to 0.0 (pupils move back to center)
                eyelid_progress = 1.0 - open_progress; // 1.0 to 0.0 (eyelids open)
            }
            
            draw_anime_eyes_suspicious_animated(eyelid_progress, pupil_progress);
            break;
        }
        
        case ANIM_SLEEP: {
            // Sleep animation with 3 phases: closing (3s), sleeping with zZz (6s), opening (3s)
            int anim_pos = current_time - last_state_change;
            float total_progress = (float)anim_pos / SLEEP_DURATION_MS;
            
            float sleep_progress;
            
            if (anim_pos < SLEEP_CLOSING_MS) {
                // Phase 1: Closing eyes (0-3 seconds)
                sleep_progress = (float)anim_pos / SLEEP_CLOSING_MS; // 0.0 to 1.0
            } else if (anim_pos < SLEEP_CLOSING_MS + SLEEP_ZZZ_MS) {
                // Phase 2: Fully asleep with zZz (3-9 seconds)
                sleep_progress = 1.0 + (float)(anim_pos - SLEEP_CLOSING_MS) / SLEEP_ZZZ_MS; // 1.0 to 2.0
            } else {
                // Phase 3: Opening eyes (9-12 seconds)
                float open_progress = (float)(anim_pos - SLEEP_CLOSING_MS - SLEEP_ZZZ_MS) / SLEEP_OPENING_MS;
                // Clamp open_progress to ensure smooth completion
                if (open_progress > 1.0) open_progress = 1.0;
                sleep_progress = 1.0 - open_progress; // 1.0 to 0.0
            }
            
            draw_anime_eyes_sleeping(sleep_progress);
            break;
        }
        
        case ANIM_ANGER: {
            // Natural anger animation with frowning brows, squinting, and smooth return
            uint32_t anger_time = current_time - last_state_change;
            
            // Check if we're in the final return phase (2-3 seconds) to add a blink
            int cycle_pos = anger_time % 3000;
            if (cycle_pos >= 2800 && cycle_pos < 3000) {
                // Final 200ms: add a natural blink as part of returning to normal
                float blink_progress = (float)(cycle_pos - 2800) / 200.0;
                if (blink_progress < 0.5) {
                    blink_progress = blink_progress * 2; // Closing
                } else {
                    blink_progress = 1.0 - ((blink_progress - 0.5) * 2); // Opening
                }
                draw_anime_eyes_natural_blink(blink_progress);
            } else {
                draw_anime_eyes_angry(anger_time);
            }
            break;
        }
        
        default:
            draw_anime_eyes_looking(0, 0);
            break;
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== ESP32-C6-LCD Anime Eyes Starting ===");
    
    // Initialize backlight with PWM control
    init_backlight();
    
    // Configure GPIO pins
    gpio_config_t io_conf = {};
    uint64_t pin_mask = (1ULL << PIN_DC) | (1ULL << PIN_RST) | (1ULL << PIN_CS);
    io_conf.pin_bit_mask = pin_mask;
    io_conf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io_conf);
    
    // Initialize RGB LED
    init_rgb_led();
    
    // Initialize SPI
    esp_err_t ret = init_spi_safe();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SPI initialized successfully");
        
        // Initialize display
        st7789_init_display();
        ESP_LOGI(TAG, "Display initialized");
        
        // Initialize framebuffer for fast rendering
        if (!init_framebuffer()) {
            ESP_LOGE(TAG, "Failed to initialize framebuffer!");
            return;
        }
        
        // Start animated anime eyes
        ESP_LOGI(TAG, "🎉 Starting animated anime eyes...");
        set_rgb_color(0, 0, 255); // Blue LED to match eye color
        
        while (1) {
            animate_eyes();
            vTaskDelay(pdMS_TO_TICKS(LOOK_SPEED_MS)); // 100ms update rate
        }
    } else {
        ESP_LOGE(TAG, "❌ SPI initialization failed: %s", esp_err_to_name(ret));
        
        while (1) {
            // Flash backlight and RGB LED to indicate error
            gpio_set_level(PIN_BCKL, 1);
            set_rgb_color(0, 255, 0); // GRB: Green channel for Red error indication
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_set_level(PIN_BCKL, 0);
            set_rgb_color(0, 0, 0); // LED off
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}