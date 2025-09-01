#ifndef ANIME_EYES_H
#define ANIME_EYES_H

#include "lvgl.h"
#include "st7789_display.h"
#include <stdbool.h>

#define EYE_WIDTH 140
#define EYE_HEIGHT 80
#define PUPIL_RADIUS 25
#define IRIS_RADIUS 35
#define EYE_CENTER_X (LCD_H_RES / 2)
#define EYE_CENTER_Y (LCD_V_RES / 2)

typedef enum {
    EYE_EMOTION_NORMAL,
    EYE_EMOTION_HAPPY,
    EYE_EMOTION_SURPRISED,
    EYE_EMOTION_SLEEPY,
    EYE_EMOTION_ANGRY,
    EYE_EMOTION_SAD
} eye_emotion_t;

typedef struct {
    float x;
    float y;
} eye_position_t;

typedef struct {
    eye_position_t look_direction;
    eye_emotion_t emotion;
    bool is_blinking;
    uint32_t blink_start_time;
    uint32_t last_blink_time;
    uint32_t last_movement_time;
    eye_position_t target_position;
    eye_position_t current_position;
    float eyelid_height;
} eye_state_t;

void anime_eyes_init(void);
void anime_eyes_update(void);
void anime_eyes_set_emotion(eye_emotion_t emotion);
void anime_eyes_look_at(float x, float y);
void anime_eyes_start_blink(void);
void anime_eyes_set_sleepy_mode(bool enable);

#endif