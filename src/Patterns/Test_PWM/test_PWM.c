//-----------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "../../common.h"
//-----------------------------------------------------------------------
#define SIZE 16
extern const int LEDS[];
//-----------------------------------------------------------------------
/*
void test_PWM()
{
	int step = 1;
	int WROOM = 10;

	for(int brightness=0;brightness<=100;brightness+=step)
	{
		for(int i=0;i<SIZE;i++)
		{
			set_brightness(i, brightness);
		}
		sleep_ms(WROOM);	
	}
	for(int brightness=100;brightness>=0;brightness-=step)
	{
		for(int i=0;i<SIZE;i++)
		{
			set_brightness(i, brightness);
		}
		sleep_ms(WROOM);	
	}
}
*/