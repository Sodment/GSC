#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "esp_log.h"
#include "../../common.h"
#include "../../Detector/HLK-LD2450.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//-----------------------------------------------------------------------
#define SIZE 16 // From -16 to 16
extern const int LEDS[];
static const char *TAG = "LD2450";
//-----------------------------------------------------------------------
/*
void test_show_LED(int input_x1, int input_y1) //, int input_x2, int input_y2
{
    uint32_t person_y1 = input_y1 / 30;
	//uint32_t person_y2 = input_y2 / 30;
	// 1
	if(person_y1>15) person_y1 = 15;

	//V_ON(LEDS[person_y1]);
	//set_brightness(person_y1, 100);

	// 2 TODO: dodaj wieloosobowosc w czujniku, wtedy mozesz odkomentowac
	//if(person_y2>15) person_y2 = 15;

	//V_ON(LEDS[person_y2]);

	for(int k=0;k<16;k++) // TODO: zmienic na ARRAY_SIZE zeby dzialalo
	{
		//if(person_y1 != k) V_OFF(LEDS[k]); // Dodaj potem to jak bedzie 2 osoba do ifa: && person_y2 != k
		if(person_y1 == k) 	set_brightness(k, 100);
		else 				set_brightness(k, 0);
	}
}
*/
uint8_t state;
enum
{
	STATE_WAIT_FOR_EVENT,
	STATE_WAIT_FOR_ANIMATE_FORWARD,
	STATE_WAIT_FOR_ANIMATE_BACK,
	STATE_WAIT_FOR_ANIMATE_BACKWARD,
	STATE_WAIT_FOR_ANIMATE_BACK2,
};
void show_LED2(int input_x1, int input_y1) //, int input_x2, int input_y2
{
	switch (state)
	{
		case STATE_WAIT_FOR_EVENT:
			if ((input_y1<=130) && ((input_y1>=50)))
			{
				state = STATE_WAIT_FOR_ANIMATE_FORWARD;
			}
			else
			if (input_y1>=400)
			{
				state = STATE_WAIT_FOR_ANIMATE_BACKWARD;
			}
			break;

		case STATE_WAIT_FOR_ANIMATE_FORWARD:
			for (int i=0;i<STAIRS_COUNT;i++)
			{
				set_brightness(i, 100);
				sleep_ms(200);
			}
			sleep_ms(3000);
			for (int i=0;i<STAIRS_COUNT;i++)
			{
				set_brightness(i, 0);
				sleep_ms(200);
			}
			state = STATE_WAIT_FOR_EVENT;
			break;
		case STATE_WAIT_FOR_ANIMATE_BACKWARD:
			for (int i=STAIRS_COUNT-1;i>=0;i--)
			{
				set_brightness(i, 100);
				sleep_ms(200);
			}
			sleep_ms(3000);
			for (int i=STAIRS_COUNT-1;i>=0;i--)
			{
				set_brightness(i, 0);
				sleep_ms(200);
			}
			state = STATE_WAIT_FOR_EVENT;
			break;
	}
}

void show_LED(int input_x1, int input_y1) //, int input_x2, int input_y2
{
	if(input_x1 > -100 && input_x1 < 100 && input_y1 > 0) 
	{
		uint8_t tab[] = {5, 50, 90, 100, 90, 50, 5}; // max number of stairs ON [ x x - - - - - - - - - - - ],   [ - - x x x x x x x - - - - ]
		int startLength = 130; // cm
		int endLength = 520; // cm
		int minLength = 0;
		int maxLength = 600;
		int32_t person_y1 = (input_y1 - startLength) / 30; // wartosc jest ujemna
		int width = (endLength - startLength) / 30; // 13
		int tabLength = sizeof(tab) / sizeof(tab[0]);

		int k = (int)person_y1;
		for(int j=k-5; j>=0 && j < k; j++)
		{
			set_brightness(j, 0);
		}
		if(k-4 >= 0) 	set_brightness(k-4, 5);
		if(k-3 >= 0) 	set_brightness(k-3, 10);
		if(k-2 >= 0) 	set_brightness(k-2, 50);
		if(k-1 >= 0) 	set_brightness(k-1, 90);
		set_brightness(k, 100);
		if(k+1 < 13) 	set_brightness(k+1, 90);
		if(k+2 < 13) 	set_brightness(k+2, 50);
		if(k+3 < 13) 	set_brightness(k+3, 10);
		if(k+4 < 13) 	set_brightness(k+4, 5);
		for(int j=k+5; j<13; j++)
		{
			set_brightness(j, 0);
		}
	}
}

//-----------------------------------------------------------------------
extern ld2450_t	*ld2450_bottom;

static void display_LED()
{
	esp_log_level_set(TAG, ESP_LOG_VERBOSE);
	//esp_log_level_set(TAG, ESP_LOG_VERBOSE);
	int32_t x, y;
	int32_t px=0, py=0;
	int32_t rx,ry;
	while(true) 
	{
		sleep_ms(10);
		//print_matrix((int)LD2450.target1X, (int)LD2450.target1Y);
		get_person(ld2450_bottom, 0, &x, &y);
		//get_person(ld2450_top, 0, &x2, &y2);
		rx = abs(x-px);
		ry = abs(y-py);
		if (rx > 15)
		{
			px = x;
		}
		if (ry > 15)
		{
			py = y;
		}
		show_LED((int)px, (int)py);
		//ESP_LOGV(TAG, "1X=%d 1Y=%d", (int)x, (int)y );
	}
}



void person_LED() 
{
	xTaskCreate(
		display_LED,		
		"Person LED",       // Nazwa zadania
		2048,              	// Rozmiar stosu (w słowach)
		NULL,              	// Parametry przekazywane do wątku
		5,                 	// Priorytet zadania
		NULL               	// Uchwyt do zadania (opcjonalny)
	);
}
