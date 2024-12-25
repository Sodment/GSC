#include <stdbool.h>
#include <stdint.h>
#include "esp_timer.h"
//--------------------------------------------------------------------------------------------------------------------------------------------------
int64_t millis()
{
	return esp_timer_get_time() / 1000;
}

bool czy_minelo_ms(int64_t czas_poczatkowy_ms, int64_t ms)
{
	return ((millis() - czas_poczatkowy_ms) > ms) ? true : false;
}