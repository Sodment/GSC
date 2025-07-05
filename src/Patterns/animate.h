#ifndef ANIMATE_H
#define ANIMATE_H

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "../common.h"
#include "../Detector/HLK-LD2450.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//-----------------------------------------------------------------------

extern const int LEDS[];
extern const char *ANIMATION_DEBUG_TAG;
extern const uint32_t MEM_BUFFER_SIZE;

extern const int64_t ANIMATION_DURATION_MS;
extern const int64_t DURATION_PER_STEP;

enum BOX
{
    B_BOT,
    B_MID,
    B_TOP,
    B_MAX
};

enum STATE
{
    S_OFF,
    S_ANIMATE,
    S_HOLD,
    S_REVERSE_ANIMATE,
    S_MAX
};

enum DIRECTION
{
    D_BOT_TO_TOP,
    D_TOP_TO_BOT,
    D_MAX,
};

typedef struct ANIM_DEBUG_STATE
{
    uint8_t state;
    uint8_t presence;
    uint8_t direction;
} AnimDebugState;

typedef struct range
{
    int32_t min;
    int32_t max;
} Range;

typedef struct BoundingBox2D
{
    Range X;
    Range Y;
} BoundingBox2D;

struct Kalman1D
{
    float estimate;
    float error_cov;
    float previous_estimate; // Store previous estimate for smoothing
    bool initialized;        // Track if filter has been initialized
};

extern const BoundingBox2D boundingBox[B_MAX];

//-----------------------------------------------------------------------
extern ld2450_t *ld2450_bottom;

bool in_range(Range r, int32_t a);
bool in_bounding_box2D(BoundingBox2D bb, int32_t x, int32_t y);
int clamp(int32_t a, int32_t min, int32_t max);
AnimDebugState led_animation(int32_t x, int32_t y);
void thread_detect_person();

#endif