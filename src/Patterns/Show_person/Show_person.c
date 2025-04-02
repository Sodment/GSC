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

void show_LED3(int input_x1, int input_y1) //, int input_x2, int input_y2
{
	for(int i=0;i<13;i++)
	{
		set_brightness(i, 5);
	}
}

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
			if ((input_y1<=130) && (input_y1>=50))
			{
				state = STATE_WAIT_FOR_ANIMATE_FORWARD;
			}
			else
			if (input_y1>=500 && input_y1 <=540)
			{
				state = STATE_WAIT_FOR_ANIMATE_BACKWARD;
			}
			break;

		case STATE_WAIT_FOR_ANIMATE_FORWARD:
			for (int i=0;i<STAIRS_COUNT;i++)
			{
				set_brightness(i, 20);
				sleep_ms(300);
			}
			sleep_ms(3000);
			for (int i=0;i<STAIRS_COUNT;i++)
			{
				set_brightness(i, 0);
				sleep_ms(300);
			}
			state = STATE_WAIT_FOR_EVENT;
			break;
		case STATE_WAIT_FOR_ANIMATE_BACKWARD:
			for (int i=STAIRS_COUNT-1;i>=0;i--)
			{
				set_brightness(i, 20);
				sleep_ms(300);
			}
			sleep_ms(3000);
			for (int i=STAIRS_COUNT-1;i>=0;i--)
			{
				set_brightness(i, 0);
				sleep_ms(300);
			}
			state = STATE_WAIT_FOR_EVENT;
			break;
	}
}

void show_LED(int input_x1, int input_y1) //, int input_x2, int input_y2
{
	// Person is not detected, show default scene
	uint8_t offMode = 1;
	// Person is detected!
	uint8_t onMode[] = {1, 10, 30, 30, 30, 10, 1}; // max number of stairs ON [ x x - - - - - - - - - - - ],   [ - - x x x x x x x - - - - ]
	int onModeLen = sizeof(onMode) / sizeof(onMode[0]);
	
	//int minLength = 0; FYI
	//int maxLength = 600; FYI
	int gap = 32; // space between steps, cm
	int startLength = 90; // cm
	int endLength = startLength + 13 * gap; // 526 cm

	int32_t person_y1 = (input_y1 - startLength) / gap; // wartosc jest ujemna

	if(input_y1 > startLength && input_y1 < endLength && input_x1 > -200) 
	{
		/*int pos_y = (int)person_y1;
		for(int j=pos_y - (onModeLen / 2+1); j>=0 && j < pos_y; j++)
		{
			set_brightness(j, 0);
		}
		for(int j = pos_y - onModeLen / 2, tabCount = 0; tabCount<onModeLen; j++, tabCount++)
		{
			if(tabCount <= onModeLen/2)
			{
				if(j >= 0) set_brightness(j, onMode[tabCount]);
			}
			else 
			{
				if(j < 13) set_brightness(j, onMode[tabCount]);
			}
		}
		for(int j=pos_y+5; j<13; j++)
		{
			set_brightness(j, 0);
		}*/
		int k = (int)person_y1;
		for(int j=k-4; j>=0 && j < k; j++)
		{
			set_brightness(j, 0);
		}

		if(k-3 >= 0) 	set_brightness(k-3, onMode[0]);
		if(k-2 >= 0) 	set_brightness(k-2, onMode[1]);
		if(k-1 >= 0) 	set_brightness(k-1, onMode[2]);
		set_brightness(k, onMode[3]);
		if(k+1 < 13) 	set_brightness(k+1, onMode[4]);
		if(k+2 < 13) 	set_brightness(k+2, onMode[5]);
		if(k+3 < 13) 	set_brightness(k+3, onMode[6]);

		for(int j=k+4; j<13; j++)
		{
			set_brightness(j, 0);
		}

	}
	else
	{
		set_brightness(0, offMode);
		for(int j=1; j<12; j++)
		{
			set_brightness(j, 0);
		}
		set_brightness(12, offMode);
	}
}

//-----------------------------------------------------------------------
extern ld2450_t	*ld2450_bottom;

static void display_LED()
{
	esp_log_level_set(TAG, ESP_LOG_VERBOSE);
	//esp_log_level_set(TAG, ESP_LOG_VERBOSE);
	int32_t x, y;
	//int32_t x2, y2;
	//int32_t x3, y3;
	int32_t px=0, py=0;
	int32_t rx,ry;
	int i=0;
	while(true) 
	{
		sleep_ms(10);
		//print_matrix((int)LD2450.target1X, (int)LD2450.target1Y);
		get_person(ld2450_bottom, 0, &x, &y);
		//get_person(ld2450_bottom, 1, &x2, &y2);
		//get_person(ld2450_bottom, 1, &x3, &y3);
		//get_person(ld2450_top, 0, &x2, &y2);
		rx = abs(x-px);
		ry = abs(y-py);
		if (rx > 10)
		{
			px = x;
		}
		if (ry > 10)
		{
			py = y;
		}
		show_LED2((int)px, (int)py); // TODO: set to px and py back
		if(i % 10 == 0) ESP_LOGV(TAG, "1X=%d 1Y=%d", (int)px, (int)py );
		i++;
	}
}



void thread_detect_person() 
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
