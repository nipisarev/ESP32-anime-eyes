#ifndef TOUCH_INPUT_H
#define TOUCH_INPUT_H

#include "driver/i2c.h"
#include "lvgl.h"

#define I2C_MASTER_SCL_IO           10
#define I2C_MASTER_SDA_IO           11
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

#define AXS5106L_ADDR               0x3C
#define TOUCH_MAX_X                 320
#define TOUCH_MAX_Y                 172

typedef struct {
    uint16_t x;
    uint16_t y;
    bool pressed;
} touch_data_t;

void touch_init(void);
bool touch_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data);
void touch_calibrate(void);

#endif