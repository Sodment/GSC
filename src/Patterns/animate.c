#include "animate.h"

const char *ANIMATION_DEBUG_TAG = "ANIM_DEBUG";
const uint32_t MEM_BUFFER_SIZE = 50; // Size of buffer that "remembers" last detected position

const int64_t ANIMATION_DURATION_MS = 3 * 1000;
const int64_t DURATION_PER_STEP = ANIMATION_DURATION_MS / STAIRS_COUNT;

// ==== KALMAN FILTER CONFIGURATION ====
// Adjust these values to tune the Kalman filter performance
const float KALMAN_PROCESS_NOISE = 1.0f;        // Lower = smoother but slower response
const float KALMAN_MEASUREMENT_NOISE = 40.0f;   // Lower = more trust in measurements
const float KALMAN_INITIAL_COVARIANCE = 100.0f; // Initial uncertainty
const float KALMAN_OUTLIER_THRESHOLD = 100.0f;  // Threshold for outlier detection
const float KALMAN_SMOOTHING_FACTOR = 0.9f;     // Smoothing factor (0.0-1.0, higher = more smoothing)

const BoundingBox2D boundingBox[B_MAX] = {
    {{-100, 50}, {10, 170}}, // B_BOT
    {{-60, 60}, {130, 530}}, // B_MID
    {{-110, 10}, {490, 580}} // B_TOP
};

bool in_range(Range r, int32_t a)
{
    return r.min < a && a < r.max;
}

bool in_bounding_box2D(BoundingBox2D bb, int32_t x, int32_t y)
{
    return in_range(bb.X, x) && in_range(bb.Y, y);
}

int clamp(int32_t a, int32_t min, int32_t max)
{
    if (a < min)
        return min;
    if (a > max)
        return max;
    return a;
}

AnimDebugState led_animation(int32_t x, int32_t y)
{
    static uint8_t state = S_OFF;
    static uint8_t direction = D_MAX;
    static int64_t animation_time = 0;
    static int64_t previous_time = 0;

    uint8_t presence = B_MAX;
    for (uint8_t i = 0; i < B_MAX; i++)
    {
        if (in_bounding_box2D(boundingBox[i], x, y))
        {
            presence = i;
            break;
        }
    }

    int64_t current_time = millis();
    int64_t delta_time = current_time - previous_time;
    previous_time = current_time;

    switch (state)
    {
    case S_OFF:
    {
        animation_time = 0;
        for (uint32_t i = 1; i < STAIRS_COUNT - 1; i++)
        {
            set_brightness(i, 0);
        }
        // First and last step should have small light always on
        set_brightness(0, S_OFF_BRIGHTNESS);
        set_brightness(STAIRS_COUNT - 1, S_OFF_BRIGHTNESS);
        switch (presence)
        {
        case B_MAX:
            state = S_OFF; // Do nothing when outside of bounding boxes
            break;
        case B_MID:
            state = S_HOLD; // Do not animate if person is in the middle
            break;
        case B_BOT:
            state = S_ANIMATE;
            direction = D_BOT_TO_TOP;
            break;
        case B_TOP:
            state = S_ANIMATE;
            direction = D_TOP_TO_BOT;
            break;
        }
        break;
    }
    case S_ANIMATE:
    case S_REVERSE_ANIMATE:
    {
        // Progress animation

        animation_time += delta_time;
        animation_time = clamp(animation_time, 0, ANIMATION_DURATION_MS);

        int i;
        int64_t animation = animation_time;

        for (i = direction == D_BOT_TO_TOP ? 0 : STAIRS_COUNT - 1;
             i >= 0 && i < STAIRS_COUNT;
             i += direction == D_BOT_TO_TOP ? 1 : -1)
        {
            int timeInStep = clamp(animation, 0, DURATION_PER_STEP);
            animation -= timeInStep;
            int brightness;
            int extra_brightness = (i == 0 || i == STAIRS_COUNT - 1) ? 1 : 0;
            if (state == S_ANIMATE)
            {
                brightness = (timeInStep * MAX_BRIGHTNESS) / DURATION_PER_STEP;
            }
            else if (state == S_REVERSE_ANIMATE)
            {
                brightness = MAX_BRIGHTNESS - (timeInStep * MAX_BRIGHTNESS) / DURATION_PER_STEP;
            }
            else
            {
                brightness = extra_brightness;
            }
            set_brightness(i, brightness + extra_brightness);
        }

        if (animation_time >= ANIMATION_DURATION_MS)
        {
            switch (presence)
            {
            case B_BOT:
            case B_TOP:
            case B_MID:
                state = S_HOLD;
                break;
            case B_MAX:
                state = S_OFF;
                direction = D_MAX;
                break;
            }
        }

        break;
    }
    case S_HOLD:
    {

        animation_time = ANIMATION_DURATION_MS;
        for (int i = 0; i < STAIRS_COUNT; i++)
        {
            set_brightness(i, MAX_BRIGHTNESS);
        }

        switch (presence)
        {
        case B_BOT:
        case B_TOP:
        case B_MID:
            break;
        case B_MAX:
            state = S_REVERSE_ANIMATE;
            animation_time = 0;
            break;
        }
        break;
    }
    }
    return (AnimDebugState){state, presence, direction};
}

static void kalman_update(struct Kalman1D *kf, float measurement, float process_noise, float measurement_noise)
{
    // Initialize on first valid measurement
    if (!kf->initialized && measurement != 0.0f)
    {
        kf->estimate = measurement;
        kf->previous_estimate = measurement;
        kf->error_cov = KALMAN_INITIAL_COVARIANCE;
        kf->initialized = true;
        return;
    }

    // Skip update if not initialized and measurement is zero
    if (!kf->initialized)
        return;

    // Store previous estimate for smoothing
    kf->previous_estimate = kf->estimate;

    // Simple outlier rejection - if measurement is too far from estimate, reduce its weight
    float innovation = measurement - kf->estimate;
    float adaptive_measurement_noise = measurement_noise;

    // Increase measurement noise for large innovations (outlier rejection)
    if ((innovation > KALMAN_OUTLIER_THRESHOLD) || (innovation < -KALMAN_OUTLIER_THRESHOLD))
    {
        adaptive_measurement_noise *= 3.0f;
    }

    // Prediction update
    kf->error_cov += process_noise;

    // Measurement update
    float K = kf->error_cov / (kf->error_cov + adaptive_measurement_noise);
    kf->estimate = kf->estimate + K * innovation;
    kf->error_cov = (1 - K) * kf->error_cov;

    // Apply light smoothing to reduce jitter
    kf->estimate = KALMAN_SMOOTHING_FACTOR * kf->estimate + (1.0f - KALMAN_SMOOTHING_FACTOR) * kf->previous_estimate;
}

static void run_stairs_logic()
{
    esp_log_level_set(ANIMATION_DEBUG_TAG, ESP_LOG_VERBOSE);

    struct Person
    {
        struct
        {
            int32_t x[MEM_BUFFER_SIZE];
            int32_t y[MEM_BUFFER_SIZE];
        } positions_memory;
        struct
        {
            int32_t x;
            int32_t y;
        } mean;
        struct Kalman1D kf_x;
        struct Kalman1D kf_y;
    } people[MAX_TRACKED_PEOPLE];

    struct sample
    {
        int32_t x;
        int32_t y;
    } samples[MAX_TRACKED_PEOPLE];

    // Zero out all people positions memory
    for (int person = 0; person < MAX_TRACKED_PEOPLE; person++)
    {
        memset(people[person].positions_memory.x, 0, MEM_BUFFER_SIZE * sizeof(int32_t));
        memset(people[person].positions_memory.y, 0, MEM_BUFFER_SIZE * sizeof(int32_t));

        // Initialize Kalman filter for X coordinate
        people[person].kf_x.estimate = 0.0f;
        people[person].kf_x.error_cov = KALMAN_INITIAL_COVARIANCE;
        people[person].kf_x.previous_estimate = 0.0f;
        people[person].kf_x.initialized = false;

        // Initialize Kalman filter for Y coordinate
        people[person].kf_y.estimate = 0.0f;
        people[person].kf_y.error_cov = KALMAN_INITIAL_COVARIANCE;
        people[person].kf_y.previous_estimate = 0.0f;
        people[person].kf_y.initialized = false;
    }

    uint32_t lastRecievedMillis = 0;

    uint32_t i = 0;
    while (true)
    {
        int person = 0;
        uint32_t currentMillis = 0;
        for (int person = 0; person < MAX_TRACKED_PEOPLE; person++)
        {
            // Get person position from LD2450 sensor
            // and store it in samples array
            currentMillis = get_person(ld2450_bottom, person, &samples[person].x, &samples[person].y);
        }

        if (currentMillis <= lastRecievedMillis)
        {
            continue;
        }

        // Fit samples to the best people, shitty way
        for (int p = 0; p < MAX_TRACKED_PEOPLE; p++)
        {
            people[p].positions_memory.x[i] = samples[p].x;
            people[p].positions_memory.y[i] = samples[p].y;
        }

        lastRecievedMillis = currentMillis;

        for (person = 0; person < MAX_TRACKED_PEOPLE; person++)
        {
            // Use the newest sample in the circular buffer
            int newest_idx = (i > 0) ? (i - 1) : (MEM_BUFFER_SIZE - 1);
            int32_t x_meas = people[person].positions_memory.x[newest_idx];
            int32_t y_meas = people[person].positions_memory.y[newest_idx];

            // Only update if we have a valid measurement
            if (x_meas != 0 || y_meas != 0)
            {
                kalman_update(&people[person].kf_x, (float)x_meas, KALMAN_PROCESS_NOISE, KALMAN_MEASUREMENT_NOISE);
                kalman_update(&people[person].kf_y, (float)y_meas, KALMAN_PROCESS_NOISE, KALMAN_MEASUREMENT_NOISE);
                people[person].mean.x = (int32_t)people[person].kf_x.estimate;
                people[person].mean.y = (int32_t)people[person].kf_y.estimate;
            }
        }

        AnimDebugState debug_state = led_animation(people[0].mean.x, people[0].mean.y);

        // DEBUG
        if (i % 10 == 0)
        {
            ESP_LOGV(ANIMATION_DEBUG_TAG, "1X=%d 1Y=%d", (int)people[0].mean.x, (int)people[0].mean.y);
            ESP_LOGV(ANIMATION_DEBUG_TAG,
                     "CURRENT_STATE=%s",
                     debug_state.state == S_OFF ? "S_OFF" : debug_state.state == S_ANIMATE       ? "S_ANIMATE"
                                                        : debug_state.state == S_HOLD            ? "S_HOLD"
                                                        : debug_state.state == S_REVERSE_ANIMATE ? "S_REVERSE_ANIMATE"
                                                                                                 : "S_MAX");
            ESP_LOGV(ANIMATION_DEBUG_TAG,
                     "CURRENT_PRESENCE=%s",
                     debug_state.presence == B_BOT ? "B_BOT" : debug_state.presence == B_TOP ? "B_TOP"
                                                           : debug_state.presence == B_MID   ? "B_MID"
                                                                                             : "B_MAX");

            ESP_LOGV(ANIMATION_DEBUG_TAG,
                     "CURRENT DIRECTION=%s",
                     debug_state.direction == D_BOT_TO_TOP ? "D_BOT_TO_TOP" : debug_state.direction == D_TOP_TO_BOT ? "D_TOP_TO_BOT"
                                                                                                                    : "D_MAX");
        }
        i++;
        if (i % MEM_BUFFER_SIZE == 0)
        {
            i = 0;
        }

        // DEBUG_END
    }
}

void thread_detect_person()
{
    xTaskCreate(
        run_stairs_logic, // Function to run in thread
        "GSC_LOGIC",      // Thread name
        5048,             // Rozmiar stosu (w słowach)
        NULL,             // Parametry przekazywane do wątku
        5,                // Priorytet zadania
        NULL              // Uchwyt do zadania (opcjonalny)
    );
}
