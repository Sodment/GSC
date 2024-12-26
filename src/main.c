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
#include "Patterns/Test_PWM/test_PWM.h"

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
#define LED_11 GPIO_NUM_23 // Tranzystor wisi
#define LED_12 GPIO_NUM_22
#define LED_13 GPIO_NUM_18
#define LED_14 GPIO_NUM_5
#define LED_15 GPIO_NUM_19
#define RX_PIN GPIO_NUM_36
#define TX_PIN GPIO_NUM_21

const int LEDS[] = {LED_0, LED_1, LED_2, LED_3, LED_4, LED_5, LED_6, LED_7, LED_8, LED_9, LED_10, LED_11, LED_12, LED_13, LED_14, LED_15};
#define LED_LENGTH (ARRAY_SIZE(LEDS))

static const char *TAG = "main";
//ESP_LOGV(TAG, "Start!"); // bialy
//ESP_LOGW(TAG, "Start!"); // zolty
//ESP_LOGD(TAG, "Start!"); // bialy
//ESP_LOGI(TAG, "Start!"); // zielony
//ESP_LOGE(TAG, "Start!"); // czerwony


//static uint8_t test[]="test";

/*const esp_pm_config_esp32_t pm_config = {
    .max_freq_mhz = 240, // Maksymalna częstotliwość w MHz
    .min_freq_mhz = 240,  // Minimalna częstotliwość w MHz
    .light_sleep_enable = false // Opcjonalnie: włączenie trybu light sleep
};*/
/*
esp_err_t ret = esp_pm_configure(&pm_config);
if (ret != ESP_OK) {
    printf("Failed to configure power management: %s\n", esp_err_to_name(ret));
}
*/

void show_frequency() {
	uint32_t cpu_freq = esp_clk_cpu_freq();  // Funkcja do uzyskania częstotliwości CPU
    printf("Current CPU frequency: %lu MHz\n", cpu_freq / (uint32_t)1000000);
}

ld2450_t	*ld2450_bottom;
ld2450_t	*ld2450_top;

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

	ld2450_bottom 	= detector_init(1, 21, 36);
	ld2450_top 		= detector_init(2, 16, 17);	//nie uzywany


	//ESP_LOGV(TAG, "Stop!");
	

    //gpio_reset_pin(RX_PIN);
    //gpio_set_direction(RX_PIN, GPIO_MODE_INPUT);
    //gpio_set_pull_mode(RX_PIN, GPIO_PULLUP_ONLY);
    //gpio_reset_pin(TX_PIN);
    //gpio_set_direction(RX_PIN, GPIO_MODE_OUTPUT);
	
	//int i;
	// for (i = 0; i < LED_LENGTH/2; i++)
    //     {
    //         V_ON(LEDS[i]);
    //         // vTaskDelay(500 / portTICK_PERIOD_MS);
    //         // V_OFF(LEDS[i]);

	// 		//uart_dma_wyslij()
    //     }
		
	for (int i=0;i<LED_LENGTH;i++)
	{
		pwm_init(i, LEDS[i], 400, 50);
	}

}

void test_mrugania_ledow(void *pvParameters)
{
    // while (1) {
    //     printf("Wątek działa! \n");
    //     vTaskDelay(pdMS_TO_TICKS(1000)); // Opóźnienie 1 sekundy
    // }
	while (1)
	{
		for (int i = 0; i < LED_LENGTH; i++)
        {
            V_ON(LEDS[i]);
			sleep_ms(500);
            //vTaskDelay(500 / portTICK_PERIOD_MS);
            V_OFF(LEDS[i]);

			//uart_dma_wyslij()
        }
	}
}

// esp_task_wdt_config_t  esp_task_wdt_config;
void app_main()
{
    setup();
	// esp_task_wdt_config.timeout_ms = 10000;
	// esp_task_wdt_config.idle_core_mask = 0;
	// esp_task_wdt_config.trigger_panic = true;
	// esp_task_wdt_init(&esp_task_wdt_config);
	//esp_task_wdt_add(NULL);
	esp_task_wdt_deinit(); // TODO: nie mozna nie uzywac watchdoga, a jak wlaczasz to zle sie resetuje

	//uart_dma_wyslij()

	// Tworzenie wątku
	// 
    // xTaskCreate(
    //     test_mrugania_ledow,// Funkcja wątku
    //     "LED",          	// Nazwa zadania
    //     2048,              	// Rozmiar stosu (w słowach)
    //     NULL,              	// Parametry przekazywane do wątku
    //     5,                 	// Priorytet zadania
    //     NULL               	// Uchwyt do zadania (opcjonalny)
    // );

    // xTaskCreate(
    //     thread_uart,		// Funkcja wątku
    //     "UART",          	// Nazwa zadania
    //     2048,              	// Rozmiar stosu (w słowach)
    //     NULL,              	// Parametry przekazywane do wątku
    //     5,                 	// Priorytet zadania
    //     NULL               	// Uchwyt do zadania (opcjonalny)
    // );

	// TODO: Detector works, reformat files

	while(true)
	{
	// Odśwież zegar watchdog
        //esp_task_wdt_reset();  // Resetuje zegar WDT dla tego zadania
		test_PWM();
		sleep_ms(1);
	}

    // gpio_dump_io_configuration(stdout, (1ULL << 34) | (1ULL << 18) | (1ULL << 26));
}
