#ifndef ST7789_DISPLAY_H
#define ST7789_DISPLAY_H

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "lvgl.h"

#define LCD_HOST    SPI2_HOST
#define DMA_CHAN    SPI_DMA_CH_AUTO

#define PIN_NUM_MISO -1
#define PIN_NUM_MOSI 7
#define PIN_NUM_CLK  6
#define PIN_NUM_CS   5
#define PIN_NUM_DC   4
#define PIN_NUM_RST  8
#define PIN_NUM_BCKL 21  // Try GPIO 21 for backlight

#define LCD_H_RES 320
#define LCD_V_RES 172

typedef struct {
    spi_device_handle_t spi;
    int dc_io;
    int reset_io;
    int backlight_io;
} st7789_handle_t;

void st7789_init(void);
void st7789_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);
void display_set_brightness(uint8_t brightness);
esp_err_t st7789_test_display(void);

#endif