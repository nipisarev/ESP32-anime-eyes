#include "anime_eyes.h"
#include "st7789_display.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include <math.h>

static const char *TAG = "ANIME_EYES";
static eye_state_t eye_state;
static lv_obj_t *eye_canvas;

#define BLINK_DURATION_MS 150
#define BLINK_INTERVAL_MS 3000
#define MOVEMENT_SMOOTH_FACTOR 0.1f
#define LOOK_RANGE 0.3f

static void draw_eye_base(lv_draw_ctx_t *draw_ctx, const lv_area_t *coords)
{
    lv_draw_rect_dsc_t bg_dsc;
    lv_draw_rect_dsc_init(&bg_dsc);
    bg_dsc.bg_color = lv_color_black();
    bg_dsc.bg_opa = LV_OPA_COVER;
    
    lv_area_t bg_area = *coords;
    lv_draw_rect(draw_ctx, &bg_dsc, &bg_area);
    
    // Add a simple test pattern - white border
    lv_draw_rect_dsc_t border_dsc;
    lv_draw_rect_dsc_init(&border_dsc);
    border_dsc.bg_color = lv_color_white();
    border_dsc.bg_opa = LV_OPA_COVER;
    
    // Draw a white border around the screen
    lv_area_t border_area;
    border_area.x1 = 0;
    border_area.y1 = 0;
    border_area.x2 = LCD_H_RES - 1;
    border_area.y2 = 5;  // Top border
    lv_draw_rect(draw_ctx, &border_dsc, &border_area);
    
    border_area.y1 = LCD_V_RES - 6;
    border_area.y2 = LCD_V_RES - 1;  // Bottom border
    lv_draw_rect(draw_ctx, &border_dsc, &border_area);
    
    border_area.x1 = 0;
    border_area.y1 = 0;
    border_area.x2 = 5;
    border_area.y2 = LCD_V_RES - 1;  // Left border
    lv_draw_rect(draw_ctx, &border_dsc, &border_area);
    
    border_area.x1 = LCD_H_RES - 6;
    border_area.x2 = LCD_H_RES - 1;  // Right border
    lv_draw_rect(draw_ctx, &border_dsc, &border_area);
}

static void draw_eye_white(lv_draw_ctx_t *draw_ctx, int16_t center_x, int16_t center_y)
{
    lv_draw_rect_dsc_t white_dsc;
    lv_draw_rect_dsc_init(&white_dsc);
    white_dsc.bg_color = lv_color_white();
    white_dsc.bg_opa = LV_OPA_COVER;
    white_dsc.radius = EYE_HEIGHT / 2;
    
    lv_area_t eye_area;
    eye_area.x1 = center_x - EYE_WIDTH / 2;
    eye_area.y1 = center_y - EYE_HEIGHT / 2;
    eye_area.x2 = center_x + EYE_WIDTH / 2;
    eye_area.y2 = center_y + EYE_HEIGHT / 2;
    
    if (eye_state.emotion == EYE_EMOTION_HAPPY) {
        eye_area.y2 = center_y + (EYE_HEIGHT / 4);
        white_dsc.radius = EYE_HEIGHT / 4;
    } else if (eye_state.emotion == EYE_EMOTION_SLEEPY) {
        eye_area.y2 = center_y + (EYE_HEIGHT / 6);
        white_dsc.radius = EYE_HEIGHT / 6;
    }
    
    if (eye_state.is_blinking) {
        float blink_progress = (float)(esp_timer_get_time() / 1000 - eye_state.blink_start_time) / BLINK_DURATION_MS;
        if (blink_progress < 0.5f) {
            eye_state.eyelid_height = 1.0f - (blink_progress * 2.0f);
        } else if (blink_progress < 1.0f) {
            eye_state.eyelid_height = (blink_progress - 0.5f) * 2.0f;
        } else {
            eye_state.is_blinking = false;
            eye_state.eyelid_height = 1.0f;
            eye_state.last_blink_time = esp_timer_get_time() / 1000;
        }
        
        int16_t blink_height = (int16_t)(EYE_HEIGHT * eye_state.eyelid_height);
        eye_area.y1 = center_y - blink_height / 2;
        eye_area.y2 = center_y + blink_height / 2;
    }
    
    lv_draw_rect(draw_ctx, &white_dsc, &eye_area);
}

static void draw_iris_and_pupil(lv_draw_ctx_t *draw_ctx, int16_t center_x, int16_t center_y)
{
    if (eye_state.is_blinking && eye_state.eyelid_height < 0.3f) {
        return;
    }
    
    int16_t look_offset_x = (int16_t)(eye_state.current_position.x * EYE_WIDTH * LOOK_RANGE);
    int16_t look_offset_y = (int16_t)(eye_state.current_position.y * EYE_HEIGHT * LOOK_RANGE);
    
    int16_t iris_x = center_x + look_offset_x;
    int16_t iris_y = center_y + look_offset_y;
    
    lv_draw_rect_dsc_t iris_dsc;
    lv_draw_rect_dsc_init(&iris_dsc);
    
    switch (eye_state.emotion) {
        case EYE_EMOTION_HAPPY:
            iris_dsc.bg_color = lv_color_make(0x4A, 0x90, 0xE2);
            break;
        case EYE_EMOTION_ANGRY:
            iris_dsc.bg_color = lv_color_make(0xE2, 0x4A, 0x4A);
            break;
        case EYE_EMOTION_SURPRISED:
            iris_dsc.bg_color = lv_color_make(0x90, 0xE2, 0x4A);
            break;
        case EYE_EMOTION_SAD:
            iris_dsc.bg_color = lv_color_make(0x6A, 0x6A, 0xE2);
            break;
        default:
            iris_dsc.bg_color = lv_color_make(0x4A, 0x90, 0xE2);
            break;
    }
    
    iris_dsc.bg_opa = LV_OPA_COVER;
    iris_dsc.radius = IRIS_RADIUS;
    
    lv_area_t iris_area;
    iris_area.x1 = iris_x - IRIS_RADIUS;
    iris_area.y1 = iris_y - IRIS_RADIUS;
    iris_area.x2 = iris_x + IRIS_RADIUS;
    iris_area.y2 = iris_y + IRIS_RADIUS;
    
    lv_draw_rect(draw_ctx, &iris_dsc, &iris_area);
    
    lv_draw_rect_dsc_t pupil_dsc;
    lv_draw_rect_dsc_init(&pupil_dsc);
    pupil_dsc.bg_color = lv_color_black();
    pupil_dsc.bg_opa = LV_OPA_COVER;
    pupil_dsc.radius = PUPIL_RADIUS;
    
    lv_area_t pupil_area;
    pupil_area.x1 = iris_x - PUPIL_RADIUS;
    pupil_area.y1 = iris_y - PUPIL_RADIUS;
    pupil_area.x2 = iris_x + PUPIL_RADIUS;
    pupil_area.y2 = iris_y + PUPIL_RADIUS;
    
    lv_draw_rect(draw_ctx, &pupil_dsc, &pupil_area);
    
    lv_draw_rect_dsc_t highlight_dsc;
    lv_draw_rect_dsc_init(&highlight_dsc);
    highlight_dsc.bg_color = lv_color_white();
    highlight_dsc.bg_opa = LV_OPA_70;
    highlight_dsc.radius = 8;
    
    lv_area_t highlight_area;
    highlight_area.x1 = iris_x - 8;
    highlight_area.y1 = iris_y - 12;
    highlight_area.x2 = iris_x + 8;
    highlight_area.y2 = iris_y + 4;
    
    lv_draw_rect(draw_ctx, &highlight_dsc, &highlight_area);
}

static void eye_canvas_draw_cb(lv_event_t *e)
{
    lv_obj_t *canvas = lv_event_get_target(e);
    lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(e);
    
    if (draw_ctx == NULL) {
        return;
    }
    
    lv_area_t canvas_area;
    lv_obj_get_coords(canvas, &canvas_area);
    
    draw_eye_base(draw_ctx, &canvas_area);
    draw_eye_white(draw_ctx, EYE_CENTER_X, EYE_CENTER_Y);
    draw_iris_and_pupil(draw_ctx, EYE_CENTER_X, EYE_CENTER_Y);
}

void anime_eyes_init(void)
{
    memset(&eye_state, 0, sizeof(eye_state_t));
    eye_state.emotion = EYE_EMOTION_NORMAL;
    eye_state.eyelid_height = 1.0f;
    eye_state.last_blink_time = esp_timer_get_time() / 1000;
    
    // Create the eye canvas object
    eye_canvas = lv_obj_create(lv_scr_act());
    lv_obj_set_size(eye_canvas, LCD_H_RES, LCD_V_RES);
    lv_obj_set_pos(eye_canvas, 0, 0);
    lv_obj_clear_flag(eye_canvas, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(eye_canvas, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(eye_canvas, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(eye_canvas, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(eye_canvas, 0, LV_PART_MAIN);
    
    // Add drawing event callback
    lv_obj_add_event_cb(eye_canvas, eye_canvas_draw_cb, LV_EVENT_DRAW_MAIN, NULL);
    
    // Force initial redraw
    lv_obj_invalidate(eye_canvas);
    
    ESP_LOGI(TAG, "Anime eyes initialized");
}

void anime_eyes_update(void)
{
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    if (!eye_state.is_blinking && 
        (current_time - eye_state.last_blink_time) > BLINK_INTERVAL_MS) {
        anime_eyes_start_blink();
    }
    
    eye_state.current_position.x += (eye_state.target_position.x - eye_state.current_position.x) * MOVEMENT_SMOOTH_FACTOR;
    eye_state.current_position.y += (eye_state.target_position.y - eye_state.current_position.y) * MOVEMENT_SMOOTH_FACTOR;
    
    if ((current_time - eye_state.last_movement_time) > 2000) {
        float random_x = ((float)(esp_random() % 200) - 100) / 100.0f;
        float random_y = ((float)(esp_random() % 200) - 100) / 100.0f;
        anime_eyes_look_at(random_x * 0.5f, random_y * 0.5f);
        eye_state.last_movement_time = current_time;
    }
    
    lv_obj_invalidate(eye_canvas);
}

void anime_eyes_set_emotion(eye_emotion_t emotion)
{
    eye_state.emotion = emotion;
    ESP_LOGI(TAG, "Eye emotion set to: %d", emotion);
}

void anime_eyes_look_at(float x, float y)
{
    eye_state.target_position.x = (x < -1.0f) ? -1.0f : (x > 1.0f) ? 1.0f : x;
    eye_state.target_position.y = (y < -1.0f) ? -1.0f : (y > 1.0f) ? 1.0f : y;
}

void anime_eyes_start_blink(void)
{
    eye_state.is_blinking = true;
    eye_state.blink_start_time = esp_timer_get_time() / 1000;
}

void anime_eyes_set_sleepy_mode(bool enable)
{
    if (enable) {
        eye_state.emotion = EYE_EMOTION_SLEEPY;
    } else if (eye_state.emotion == EYE_EMOTION_SLEEPY) {
        eye_state.emotion = EYE_EMOTION_NORMAL;
    }
}