#include <stdbool.h>
#include <stdint.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pwm/pwm.h"
//--------------------------------------------------------------------------------------------------------------------------------------------------
int64_t millis()
{
	return esp_timer_get_time() / 1000;
}

bool czy_minelo_ms(int64_t czas_poczatkowy_ms, int64_t ms)
{
	return ((millis() - czas_poczatkowy_ms) > ms) ? true : false;
}

/* Wait x miliseconds. */
void sleep_ms(uint32_t ms)
{
	vTaskDelay(pdMS_TO_TICKS(ms));
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
/*
void print_matrix(int input_x, int input_y)
{
    int person_x = input_x / 20;
    int person_y = input_y / 20;

    int dimension = 2 * SIZE + 1; // 61
    char buffer[dimension + 1] ; // Buffer for the entire matrix


	//ESP_LOGV(TAG, "MATRIX");

    for (int i = SIZE-1; i >= 0; i--) {  // Bottom is the nearest 
		memset(buffer, 0, sizeof(buffer));
        for (int j = 0; j < dimension; j++) {
            if (i == person_y && j == person_x+SIZE) {
                buffer[j] = 'x';
				
            } else {
                buffer[j] = '-';
            }
        }
		//ESP_LOGV(TAG, "%s", buffer);
    }
	
}
*/

void set_brightness(uint8_t index, uint8_t percent)
{
	pwm_set_brightness(index, percent);
}