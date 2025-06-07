#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "string.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_task_wdt.h>
#include "esp_pm.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_private/esp_clk.h"
#include "common.h"
#include "Detector/HLK-LD2450.h"
#include "pwm/pwm.h"
#include "Patterns/animate.h"

// PIN MAPPING
#define LED_0 GPIO_NUM_25
#define LED_1 GPIO_NUM_33
#define LED_2 GPIO_NUM_32
#define LED_3 GPIO_NUM_26
#define LED_4 GPIO_NUM_27
#define LED_5 GPIO_NUM_14
#define LED_6 GPIO_NUM_12
#define LED_7 GPIO_NUM_13
#define LED_8 GPIO_NUM_15
#define LED_9 GPIO_NUM_2
#define LED_10 GPIO_NUM_4
#define LED_11 GPIO_NUM_5
#define LED_12 GPIO_NUM_18
#define LED_13 GPIO_NUM_19
#define LED_14 GPIO_NUM_22
#define LED_15 GPIO_NUM_23

// Detector 1
#define RX_PIN GPIO_NUM_36
#define TX_PIN GPIO_NUM_21

// Detector 2 if added
#define RX_PIN2 GPIO_NUM_17
#define TX_PIN2 GPIO_NUM_16

const int LEDS[] = {LED_0, LED_1, LED_2, LED_3, LED_4, LED_5, LED_6, LED_7, LED_8, LED_9, LED_10, LED_11, LED_12, LED_13, LED_14, LED_15};
#define LED_LENGTH (ARRAY_SIZE(LEDS))

static const char *TAG = "main";

void show_frequency()
{
    uint32_t cpu_freq = esp_clk_cpu_freq(); // Funkcja do uzyskania częstotliwości CPU
    printf("Current CPU frequency: %lu MHz\n", cpu_freq / (uint32_t)1000000);
}

ld2450_t *ld2450_bottom;
ld2450_t *ld2450_top;

/* Works only for the first time. Add all setups at the beginning.*/
void setup()
{
    esp_log_level_set("*", ESP_LOG_NONE);
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);

    for (int i = 0; i < LED_LENGTH; i++)
    {
        gpio_reset_pin(LEDS[i]);
        gpio_set_direction(LEDS[i], GPIO_MODE_OUTPUT);
    }

    ld2450_bottom = detector_init(1, TX_PIN, RX_PIN);
    ld2450_top = detector_init(2, TX_PIN2, RX_PIN2); // nie uzywany

    // ESP_LOGV(TAG, "Stop!");

    // gpio_reset_pin(RX_PIN);
    // gpio_set_direction(RX_PIN, GPIO_MODE_INPUT);
    // gpio_set_pull_mode(RX_PIN, GPIO_PULLUP_ONLY);
    // gpio_reset_pin(TX_PIN);
    // gpio_set_direction(RX_PIN, GPIO_MODE_OUTPUT);

    for (int i = 0; i < LED_LENGTH; i++)
    {
        pwm_init(i, LEDS[i], 400, 0);
    }
}

void app_main()
{
    setup();
    // esp_task_wdt_config.timeout_ms = 10000;
    // esp_task_wdt_config.idle_core_mask = 0;
    // esp_task_wdt_config.trigger_panic = true;
    // esp_task_wdt_init(&esp_task_wdt_config);
    // esp_task_wdt_add(NULL);
    esp_task_wdt_deinit(); // TODO: nie mozna nie uzywac watchdoga, a jak wlaczasz to zle sie resetuje

    // TODO: Detector works, reformat files
    thread_detect_person();

    while (true)
    {
        // Odśwież zegar watchdog
        // esp_task_wdt_reset();  // Resetuje zegar WDT dla tego zadania
        // test_PWM();

        sleep_ms(100);
    }
}
