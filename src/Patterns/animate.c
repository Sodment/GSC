#include "animate.h"

const char *ANIMATION_DEBUG_TAG = "ANIM_DEBUG";
const uint32_t MEM_BUFFER_SIZE = 50; // Size of buffer that "remembers" last detected position

const int64_t ANIMATION_DURATION_MS = 3 * 1000;
const int64_t WAIT_DURATION_BEFORE_OFF_S = 2 * 1000;
const int64_t DURATION_PER_STEP = ANIMATION_DURATION_MS / STAIRS_COUNT;

const BoundingBox2D boundingBox[B_MAX] = {
    {{-60, 40}, {30, 130}},  // B_BOT
    {{-60, 40}, {130, 500}}, // B_MID
    {{-110, 0}, {500, 590}}  // B_TOP
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
    static int64_t wait_time = WAIT_DURATION_BEFORE_OFF_S;

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
        for (uint32_t i = 0; i < STAIRS_COUNT; i++)
        {
            set_brightness(i, 0);
        }
        // First and last step should have small light always on
        set_brightness(0, 3);
        set_brightness(STAIRS_COUNT - 1, 3);
        switch (presence)
        {
        case B_MAX:
            state = S_OFF; // Do not animate if person is in the middle
            break;
        case B_MID:
            state = S_HOLD;
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
    {
        // Progress animation

        animation_time += direction == D_BOT_TO_TOP ? delta_time : -delta_time;
        animation_time = clamp(animation_time, 0, ANIMATION_DURATION_MS);
        // ESP_LOGV(TAG, "A");

        int i;
        int64_t animation = animation_time;

        for (i = direction == D_BOT_TO_TOP ? 0 : STAIRS_COUNT - 1;
             i >= 0 && i < STAIRS_COUNT;
             i += direction == D_BOT_TO_TOP ? 1 : -1)
        {
            int timeInStep = clamp(animation, 0, DURATION_PER_STEP);
            animation -= timeInStep;
            int brightness = (timeInStep * MAX_BRIGHTNESS) / DURATION_PER_STEP;
            // ESP_LOGV(TAG, "stair=%d, brightness=%f", (int)i, brightness);
            set_brightness(i, brightness);
        }

        if (animation_time >= ANIMATION_DURATION_MS)
        {
            state = S_HOLD;
        }

        if (animation_time <= 0)
        {
            state = S_OFF;
        }

        break;
    }
    case S_HOLD:
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
            wait_time -= delta_time;
            if (wait_time <= 0)
            {
                wait_time = WAIT_DURATION_BEFORE_OFF_S;
                state = S_OFF;
            }
            break;
        }
        break;
    }
    return (AnimDebugState){state, presence};
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
        for (int x = 0; x < MAX_TRACKED_PEOPLE; x++)
        {
            for (int y = 0; y < MAX_TRACKED_PEOPLE; y++)
            {
                people[person].positions_memory.x[i] = samples[person].x;
                people[person].positions_memory.y[i] = samples[person].y;
            }
        }

        lastRecievedMillis = currentMillis;

        for (person = 0; person < MAX_TRACKED_PEOPLE; person++)
        {
            // Use weighted moving average with higher weight for recent positions
            int32_t weighted_sum_x = 0;
            int32_t weighted_sum_y = 0;
            int total_weight = 0;

            // In a circular buffer, the newest element is at position i-1
            // (or at the end of the buffer if i=0)
            int newest_idx = (i > 0) ? (i - 1) : (MEM_BUFFER_SIZE - 1);

            for (int j = 0; j < MEM_BUFFER_SIZE; j++)
            {
                // Calculate the actual position in the circular buffer
                // going from newest to oldest
                int buf_idx = (newest_idx - j + MEM_BUFFER_SIZE) % MEM_BUFFER_SIZE;

                // Skip zero entries (no data)
                if (people[person].positions_memory.x[buf_idx] == 0 &&
                    people[person].positions_memory.y[buf_idx] == 0)
                    continue;

                int weight = MEM_BUFFER_SIZE >> (j / 4);
                if (weight < 1)
                    weight = 1; // Ensure minimum weight of 1

                weighted_sum_x += people[person].positions_memory.x[buf_idx] * weight;
                weighted_sum_y += people[person].positions_memory.y[buf_idx] * weight;
                total_weight += weight;
            }

            // Update mean position if we have valid data
            if (total_weight > 0)
            {
                people[person].mean.x = weighted_sum_x / total_weight;
                people[person].mean.y = weighted_sum_y / total_weight;
            }

            // Special responsiveness hack: if the newest position is in BOT or TOP area,
            // immediately use it without averaging to make the response immediate
            if (people[person].positions_memory.x[newest_idx] != 0 ||
                people[person].positions_memory.y[newest_idx] != 0)
            {
                // Check if in BOT or TOP bounding box
                for (uint8_t box = 0; box < B_MAX; box++)
                {
                    if (in_bounding_box2D(boundingBox[box],
                                          people[person].positions_memory.x[newest_idx],
                                          people[person].positions_memory.y[newest_idx]))
                    {
                        if (box == B_BOT || box == B_TOP)
                        {
                            // Immediate response to stairs entry/exit points
                            people[person].mean.x = people[person].positions_memory.x[newest_idx];
                            people[person].mean.y = people[person].positions_memory.y[newest_idx];
                        }
                        break;
                    }
                }
            }
        }

        AnimDebugState debug_state = led_animation(people[0].mean.x, people[0].mean.y);

        // DEBUG
        if (i % 10 == 0)
        {
            ESP_LOGV(ANIMATION_DEBUG_TAG, "1X=%d 1Y=%d", (int)people[0].mean.x, (int)people[0].mean.y);
            ESP_LOGV(ANIMATION_DEBUG_TAG,
                     "CURRENT_STATE=%s",
                     debug_state.state == S_OFF ? "S_OFF" : debug_state.state == S_ANIMATE ? "S_ANIMATE"
                                                        : debug_state.state == S_HOLD      ? "S_HOLD"
                                                                                           : "S_MAX");
            ESP_LOGV(ANIMATION_DEBUG_TAG,
                     "CURRENT_PRESENCE=%s",
                     debug_state.presence == B_BOT ? "B_BOT" : debug_state.presence == B_TOP ? "B_TOP"
                                                           : debug_state.presence == B_MID   ? "B_MID"
                                                                                             : "B_MAX");
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
