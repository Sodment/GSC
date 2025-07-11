#include <stdbool.h>
#include "driver/gpio.h"

#define V_ON(x) gpio_set_level(x, 1)  // Turn LED Voltage ON
#define V_OFF(x) gpio_set_level(x, 0) // Turn LED Voltage OFF
#define ARRAY_SIZE(n) (sizeof(n) / sizeof(n[0]))

// Global
int64_t millis();
bool czy_minelo_ms(int64_t czas_poczatkowy_ms, int64_t ms);
void sleep_ms(uint32_t ms);

// PWM
void set_brightness(uint8_t index, uint8_t percent);

#define STAIRS_COUNT 13
#define MAX_BRIGHTNESS 10  // Max LED brightness in percentage points
#define S_OFF_BRIGHTNESS 1 // First and last step brightness when noone is on stairs
#define MAX_TRACKED_PEOPLE 1
