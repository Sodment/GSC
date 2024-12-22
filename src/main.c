#include "driver/gpio.h"
#include "driver/ledc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>

// PIN MAPPING
#define LED_0 GPIO_NUM_15
#define LED_1 GPIO_NUM_2
#define LED_2 GPIO_NUM_4
#define LED_3 GPIO_NUM_32
#define LED_4 GPIO_NUM_33
#define LED_5 GPIO_NUM_25
#define LED_6 GPIO_NUM_26
#define LED_7 GPIO_NUM_27
#define LED_8 GPIO_NUM_14
#define LED_9 GPIO_NUM_12
#define LED_10 GPIO_NUM_13
#define LED_11 GPIO_NUM_23 // Transystor wisi
#define LED_12 GPIO_NUM_22
#define LED_13 GPIO_NUM_18
#define LED_14 GPIO_NUM_5
#define RX_PIN GPIO_NUM_19
#define TX_PIN GPIO_NUM_21

#define LED_LENGTH 15
const int LEDS[] = {LED_0, LED_1, LED_2, LED_3, LED_4, LED_5, LED_6, LED_7, LED_8, LED_9, LED_10, LED_11, LED_12, LED_13, LED_14};

#define V_ON(x) gpio_set_level(x, 1)  // Turn LED Voltage ON
#define V_OFF(x) gpio_set_level(x, 0) // Turn LED Voltage OFF

void setup()
{
    for (int i = 0; i < LED_LENGTH; i++)
    {
        gpio_reset_pin(LEDS[i]);
        gpio_set_direction(LEDS[i], GPIO_MODE_OUTPUT);
    }
    gpio_reset_pin(RX_PIN);
    gpio_set_direction(RX_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(RX_PIN, GPIO_PULLUP_ONLY);
    gpio_reset_pin(TX_PIN);
    gpio_set_direction(RX_PIN, GPIO_MODE_OUTPUT);
}

void app_main()
{
    setup();

    while (true)
    {
        for (int i = 0; i < LED_LENGTH; i++)
        {
            V_ON(LEDS[i]);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            V_OFF(LEDS[i]);
        }
    }
    // gpio_dump_io_configuration(stdout, (1ULL << 34) | (1ULL << 18) | (1ULL << 26));
}