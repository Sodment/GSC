//-----------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "../../common.h"
//-----------------------------------------------------------------------
#define SIZE 16
extern const int LEDS[];
//-----------------------------------------------------------------------

void test_PWM()
{
	int step = 1;
	int WROOM = 100;
	int minBrightness = 0;
	int maxBrightness = 100;

	for(int brightness=minBrightness; brightness<=maxBrightness; brightness+=step)
	{
		for(int i=0;i<SIZE;i++)
		{
			set_brightness(i, brightness);
		}
		sleep_ms(WROOM);	
	}
	for(int brightness=maxBrightness; brightness>=minBrightness; brightness-=step)
	{
		for(int i=0;i<SIZE;i++)
		{
			set_brightness(i, brightness);
		}
		sleep_ms(WROOM);	
	}
}
