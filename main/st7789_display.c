#include "st7789_display.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"

static const char *TAG = "ST7789";
static st7789_handle_t lcd_handle;

static void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &cmd;
    gpio_set_level(lcd_handle.dc_io, 0);
    ret = spi_device_polling_transmit(spi, &t);
    assert(ret == ESP_OK);
}

static void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len == 0) return;
    memset(&t, 0, sizeof(t));
    t.length = len * 8;
    t.tx_buffer = data;
    gpio_set_level(lcd_handle.dc_io, 1);
    ret = spi_device_polling_transmit(spi, &t);
    assert(ret == ESP_OK);
}

static void st7789_set_addr_window(spi_device_handle_t spi, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t data[4];
    
    lcd_cmd(spi, 0x2A);
    data[0] = (x0 >> 8) & 0xFF;
    data[1] = x0 & 0xFF;
    data[2] = (x1 >> 8) & 0xFF;
    data[3] = x1 & 0xFF;
    lcd_data(spi, data, 4);

    lcd_cmd(spi, 0x2B);
    data[0] = (y0 >> 8) & 0xFF;
    data[1] = y0 & 0xFF;
    data[2] = (y1 >> 8) & 0xFF;
    data[3] = y1 & 0xFF;
    lcd_data(spi, data, 4);

    lcd_cmd(spi, 0x2C);
}

void st7789_init(void)
{
    esp_err_t ret;
    
    // Configure GPIO pins for SPI and control
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_NUM_DC) | (1ULL << PIN_NUM_RST) | (1ULL << PIN_NUM_BCKL);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    
    ESP_LOGI(TAG, "GPIO configured - DC: %d, RST: %d, BCKL: %d", PIN_NUM_DC, PIN_NUM_RST, PIN_NUM_BCKL);
    
    lcd_handle.dc_io = PIN_NUM_DC;
    lcd_handle.reset_io = PIN_NUM_RST;
    lcd_handle.backlight_io = PIN_NUM_BCKL;
    
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * LCD_V_RES * sizeof(uint16_t),
    };
    
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 26 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
        .pre_cb = NULL,
    };
    
    ret = spi_bus_initialize(LCD_HOST, &buscfg, DMA_CHAN);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(LCD_HOST, &devcfg, &lcd_handle.spi);
    ESP_ERROR_CHECK(ret);
    
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    lcd_cmd(lcd_handle.spi, 0x01);
    vTaskDelay(pdMS_TO_TICKS(150));
    
    lcd_cmd(lcd_handle.spi, 0x11);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    lcd_cmd(lcd_handle.spi, 0x3A);
    uint8_t data = 0x05;
    lcd_data(lcd_handle.spi, &data, 1);
    
    lcd_cmd(lcd_handle.spi, 0x36);
    data = 0x60;  // Try 0x60 orientation for ESP32-C6-LCD-1.47
    lcd_data(lcd_handle.spi, &data, 1);
    
    lcd_cmd(lcd_handle.spi, 0x2A);
    uint8_t col_data[] = {0x00, 0x00, (LCD_H_RES - 1) >> 8, (LCD_H_RES - 1) & 0xFF};
    lcd_data(lcd_handle.spi, col_data, 4);
    
    lcd_cmd(lcd_handle.spi, 0x2B);
    uint8_t row_data[] = {0x00, 0x00, (LCD_V_RES - 1) >> 8, (LCD_V_RES - 1) & 0xFF};
    lcd_data(lcd_handle.spi, row_data, 4);
    
    lcd_cmd(lcd_handle.spi, 0x21);
    
    lcd_cmd(lcd_handle.spi, 0x13);
    
    lcd_cmd(lcd_handle.spi, 0xB2);
    uint8_t porch_data[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
    lcd_data(lcd_handle.spi, porch_data, 5);
    
    lcd_cmd(lcd_handle.spi, 0xB7);
    data = 0x35;
    lcd_data(lcd_handle.spi, &data, 1);
    
    lcd_cmd(lcd_handle.spi, 0xBB);
    data = 0x1F;
    lcd_data(lcd_handle.spi, &data, 1);
    
    lcd_cmd(lcd_handle.spi, 0xC0);
    data = 0x2C;
    lcd_data(lcd_handle.spi, &data, 1);
    
    lcd_cmd(lcd_handle.spi, 0xC2);
    data = 0x01;
    lcd_data(lcd_handle.spi, &data, 1);
    
    lcd_cmd(lcd_handle.spi, 0xC3);
    data = 0x12;
    lcd_data(lcd_handle.spi, &data, 1);
    
    lcd_cmd(lcd_handle.spi, 0xC4);
    data = 0x20;
    lcd_data(lcd_handle.spi, &data, 1);
    
    lcd_cmd(lcd_handle.spi, 0xC6);
    data = 0x0F;
    lcd_data(lcd_handle.spi, &data, 1);
    
    lcd_cmd(lcd_handle.spi, 0xD0);
    uint8_t power_data[] = {0xA4, 0xA1};
    lcd_data(lcd_handle.spi, power_data, 2);
    
    lcd_cmd(lcd_handle.spi, 0x29);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // First try setting backlight pin directly to high
    ESP_LOGI(TAG, "Setting primary backlight pin %d to HIGH", PIN_NUM_BCKL);
    gpio_set_level(PIN_NUM_BCKL, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Try common ESP32-C6 LCD backlight pins systematically
    int backlight_pins[] = {9, 15, 21, 10, 3, 2, 1, 0, 18, 19, 20, 22, 23};
    int num_pins = sizeof(backlight_pins) / sizeof(backlight_pins[0]);
    
    ESP_LOGI(TAG, "Testing %d potential backlight pins...", num_pins);
    
    for (int i = 0; i < num_pins; i++) {
        gpio_config_t alt_io_conf = {};
        alt_io_conf.intr_type = GPIO_INTR_DISABLE;
        alt_io_conf.mode = GPIO_MODE_OUTPUT;
        alt_io_conf.pin_bit_mask = (1ULL << backlight_pins[i]);
        alt_io_conf.pull_down_en = 0;
        alt_io_conf.pull_up_en = 0;
        
        esp_err_t alt_ret = gpio_config(&alt_io_conf);
        if (alt_ret == ESP_OK) {
            gpio_set_level(backlight_pins[i], 1);
            ESP_LOGI(TAG, "Backlight pin %d set to HIGH", backlight_pins[i]);
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            ESP_LOGW(TAG, "Failed to configure pin %d as output", backlight_pins[i]);
        }
    }
    
    // Then configure LEDC for PWM control
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 5000,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC timer config failed: %s", esp_err_to_name(ret));
    }
    
    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL_0,
        .duty = 2048,  // Reduced from 5000 to ~25% brightness (0-8191 range)
        .gpio_num = PIN_NUM_BCKL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0,
    };
    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC channel config failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "LEDC backlight configured successfully on pin %d", PIN_NUM_BCKL);
    }
    
    ESP_LOGI(TAG, "ST7789 display initialized");
    
    // Set a lower brightness level (0-255, where 255 is max brightness)
    display_set_brightness(128); // 50% brightness
    ESP_LOGI(TAG, "Display brightness set to 50%");
    
    // Temporarily re-enable display test to debug black screen
    ESP_LOGI(TAG, "Running basic display test to debug black screen...");
    esp_err_t test_ret = st7789_test_display();
    if (test_ret != ESP_OK) {
        ESP_LOGE(TAG, "Display test failed");
    }
}

esp_err_t st7789_test_display(void)
{
    ESP_LOGI(TAG, "=== SAFE PIN MAPPING TEST FOR ESP32-C6-LCD-1.47 ===");
    ESP_LOGI(TAG, "Current configuration:");
    ESP_LOGI(TAG, "MOSI: GPIO %d, SCLK: GPIO %d, CS: GPIO %d", PIN_NUM_MOSI, PIN_NUM_CLK, PIN_NUM_CS);
    ESP_LOGI(TAG, "DC: GPIO %d, RST: GPIO %d, BCKL: GPIO %d", PIN_NUM_DC, PIN_NUM_RST, PIN_NUM_BCKL);
    
    // Known working pin configurations for ESP32-C6 LCD boards
    struct pin_config {
        int mosi, sclk, cs, dc, rst;
        const char* desc;
        bool safe; // Mark configs as safe to prevent crashes
    } pin_configs[] = {
        {7, 6, 5, 4, 8, "Standard config (current)", true},
        {19, 18, 17, 16, 15, "Alt SPI2 config", true},
        {3, 2, 1, 0, 10, "Alt SPI1 config", true},
        {11, 12, 13, 14, 22, "Custom config 1", true},
        {1, 2, 3, 4, 5, "Custom config 2", true}
    };
    
    int num_configs = sizeof(pin_configs) / sizeof(pin_configs[0]);
    
    ESP_LOGI(TAG, "Testing %d safe pin configurations...", num_configs);
    
    for (int config = 0; config < num_configs; config++) {
        if (!pin_configs[config].safe) {
            ESP_LOGI(TAG, "Skipping unsafe config %d", config + 1);
            continue;
        }
        
        ESP_LOGI(TAG, "=== Testing config %d: %s ===", config + 1, pin_configs[config].desc);
        ESP_LOGI(TAG, "MOSI:%d SCLK:%d CS:%d DC:%d RST:%d", 
                pin_configs[config].mosi, pin_configs[config].sclk, 
                pin_configs[config].cs, pin_configs[config].dc, pin_configs[config].rst);
        
        // Check if pins are valid for ESP32-C6
        if (pin_configs[config].mosi > 23 || pin_configs[config].sclk > 23 || 
            pin_configs[config].cs > 23 || pin_configs[config].dc > 23 || 
            pin_configs[config].rst > 23) {
            ESP_LOGE(TAG, "Invalid pin numbers for ESP32-C6, skipping config %d", config + 1);
            continue;
        }
        
        // Safely deinitialize current SPI
        esp_err_t deinit_ret = ESP_OK;
        if (lcd_handle.spi) {
            deinit_ret = spi_bus_remove_device(lcd_handle.spi);
            if (deinit_ret != ESP_OK) {
                ESP_LOGW(TAG, "Warning: Failed to remove SPI device: %s", esp_err_to_name(deinit_ret));
            }
            lcd_handle.spi = NULL;
        }
        
        deinit_ret = spi_bus_free(LCD_HOST);
        if (deinit_ret != ESP_OK && deinit_ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "Warning: Failed to free SPI bus: %s", esp_err_to_name(deinit_ret));
        }
        
        // Small delay for safe transition
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Configure new pins carefully
        spi_bus_config_t buscfg = {
            .miso_io_num = -1,
            .mosi_io_num = pin_configs[config].mosi,
            .sclk_io_num = pin_configs[config].sclk,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 64 * 8, // Small transfer for testing
        };
        
        spi_device_interface_config_t devcfg = {
            .clock_speed_hz = 1 * 1000 * 1000,  // Very slow for safety
            .mode = 0,
            .spics_io_num = pin_configs[config].cs,
            .queue_size = 1,  // Small queue
            .pre_cb = NULL,
        };
        
        esp_err_t ret = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_DISABLED);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to init SPI bus for config %d: %s", config + 1, esp_err_to_name(ret));
            continue;
        }
        
        ret = spi_bus_add_device(LCD_HOST, &devcfg, &lcd_handle.spi);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add SPI device for config %d: %s", config + 1, esp_err_to_name(ret));
            spi_bus_free(LCD_HOST);
            continue;
        }
        
        // Configure DC and RST pins safely
        gpio_reset_pin(pin_configs[config].dc);
        gpio_reset_pin(pin_configs[config].rst);
        
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL << pin_configs[config].dc) | (1ULL << pin_configs[config].rst);
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 1; // Enable pull-up for stability
        esp_err_t gpio_ret = gpio_config(&io_conf);
        if (gpio_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure GPIO for config %d: %s", config + 1, esp_err_to_name(gpio_ret));
            spi_bus_remove_device(lcd_handle.spi);
            spi_bus_free(LCD_HOST);
            continue;
        }
        
        lcd_handle.dc_io = pin_configs[config].dc;
        lcd_handle.reset_io = pin_configs[config].rst;
        
        // Safe display reset sequence
        gpio_set_level(pin_configs[config].rst, 0);
        vTaskDelay(pdMS_TO_TICKS(20));
        gpio_set_level(pin_configs[config].rst, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
        
        ESP_LOGI(TAG, "Testing basic SPI communication...");
        
        // Try a very simple command - just wake up
        lcd_cmd(lcd_handle.spi, 0x11); // Sleep out
        ESP_LOGI(TAG, "Sleep out command sent");
        
        vTaskDelay(pdMS_TO_TICKS(300));
        
        // Try to turn display on
        lcd_cmd(lcd_handle.spi, 0x29); // Display on
        ESP_LOGI(TAG, "Display on command sent");
        
        vTaskDelay(pdMS_TO_TICKS(100));
        
        ESP_LOGI(TAG, "Config %d: Basic commands sent successfully", config + 1);
        ESP_LOGI(TAG, "Config %d: CHECK DISPLAY - Did it turn on or show any change?", config + 1);
        
        // Wait for visual inspection
        vTaskDelay(pdMS_TO_TICKS(5000)); // 5 seconds to check
        
        ESP_LOGI(TAG, "Config %d test completed", config + 1);
        ESP_LOGI(TAG, "If you saw any change, this might be the correct pinout!");
        
        // Clean up for next test
        spi_bus_remove_device(lcd_handle.spi);
        spi_bus_free(LCD_HOST);
        lcd_handle.spi = NULL;
        
        vTaskDelay(pdMS_TO_TICKS(500)); // Wait between tests
    }
    
    ESP_LOGI(TAG, "=== SAFE PIN MAPPING TEST COMPLETED ===");
    ESP_LOGI(TAG, "Did any configuration cause a visible change in the display?");
    ESP_LOGI(TAG, "If YES, note which config number and we'll use those pins!");
    
    return ESP_OK;
}

void st7789_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    uint32_t size = (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1);
    
    st7789_set_addr_window(lcd_handle.spi, area->x1, area->y1, area->x2, area->y2);
    
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = size * 16;
    t.tx_buffer = color_map;
    gpio_set_level(lcd_handle.dc_io, 1);
    ret = spi_device_polling_transmit(lcd_handle.spi, &t);
    assert(ret == ESP_OK);
    
    lv_disp_flush_ready(drv);
}

void display_set_brightness(uint8_t brightness)
{
    uint32_t duty = (brightness * 8191) / 255;
    ESP_LOGI(TAG, "Setting brightness to %d (duty: %d)", brightness, duty);
    
    esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LEDC duty: %s", esp_err_to_name(ret));
        // Fallback to direct GPIO control
        ESP_LOGI(TAG, "Falling back to direct GPIO control");
        gpio_set_level(PIN_NUM_BCKL, brightness > 0 ? 1 : 0);
        return;
    }
    
    ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update LEDC duty: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Backlight duty updated successfully");
    }
}