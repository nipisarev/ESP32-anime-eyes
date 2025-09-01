#include "touch_input.h"
#include "esp_log.h"
#include "anime_eyes.h"

static const char *TAG = "TOUCH";
static touch_data_t last_touch;

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        return err;
    }
    
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t axs5106l_read_touch(touch_data_t *touch_data)
{
    uint8_t data[6];
    esp_err_t ret;
    
    ret = i2c_master_write_read_device(I2C_MASTER_NUM, AXS5106L_ADDR, 
                                       NULL, 0, data, sizeof(data), 
                                       pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    uint8_t touch_count = data[0] & 0x0F;
    
    if (touch_count > 0) {
        touch_data->x = ((data[1] & 0x0F) << 8) | data[2];
        touch_data->y = ((data[3] & 0x0F) << 8) | data[4];
        touch_data->pressed = true;
        
        touch_data->x = (touch_data->x * TOUCH_MAX_X) / 4095;
        touch_data->y = (touch_data->y * TOUCH_MAX_Y) / 4095;
    } else {
        touch_data->pressed = false;
    }
    
    return ESP_OK;
}

void touch_init(void)
{
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C master initialization failed: %s", esp_err_to_name(ret));
        return;
    }
    
    memset(&last_touch, 0, sizeof(touch_data_t));
    ESP_LOGI(TAG, "Touch controller initialized");
}

bool touch_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    touch_data_t touch_data;
    
    if (axs5106l_read_touch(&touch_data) == ESP_OK) {
        if (touch_data.pressed) {
            data->point.x = touch_data.x;
            data->point.y = touch_data.y;
            data->state = LV_INDEV_STATE_PRESSED;
            
            if (last_touch.pressed) {
                float look_x = ((float)touch_data.x - (TOUCH_MAX_X / 2)) / (TOUCH_MAX_X / 2);
                float look_y = ((float)touch_data.y - (TOUCH_MAX_Y / 2)) / (TOUCH_MAX_Y / 2);
                anime_eyes_look_at(look_x, look_y);
                
                if (touch_data.y < TOUCH_MAX_Y / 4) {
                    anime_eyes_set_emotion(EYE_EMOTION_SURPRISED);
                } else if (touch_data.y > (3 * TOUCH_MAX_Y) / 4) {
                    anime_eyes_set_emotion(EYE_EMOTION_SLEEPY);
                } else if (touch_data.x < TOUCH_MAX_X / 3) {
                    anime_eyes_set_emotion(EYE_EMOTION_HAPPY);
                } else if (touch_data.x > (2 * TOUCH_MAX_X) / 3) {
                    anime_eyes_set_emotion(EYE_EMOTION_ANGRY);
                } else {
                    anime_eyes_set_emotion(EYE_EMOTION_NORMAL);
                }
            }
            
            if (!last_touch.pressed) {
                anime_eyes_start_blink();
            }
            
            last_touch = touch_data;
        } else {
            data->state = LV_INDEV_STATE_RELEASED;
            if (last_touch.pressed) {
                anime_eyes_set_emotion(EYE_EMOTION_NORMAL);
            }
            last_touch.pressed = false;
        }
        return false;
    }
    
    data->state = LV_INDEV_STATE_RELEASED;
    return false;
}

void touch_calibrate(void)
{
    ESP_LOGI(TAG, "Touch calibration complete");
}