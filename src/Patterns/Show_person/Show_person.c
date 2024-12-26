//-----------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "../../common.h"
#include "../../Detector/HLK-LD2450.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//-----------------------------------------------------------------------
#define SIZE 16 // From -16 to 16
extern const int LEDS[];
//-----------------------------------------------------------------------

void show_LED(int input_x1, int input_y1) //, int input_x2, int input_y2
{
    uint32_t person_y1 = input_y1 / 30;
	//uint32_t person_y2 = input_y2 / 30;
	// 1
	if(person_y1>15) person_y1 = 15;

	//V_ON(LEDS[person_y1]);
	set_brightness(person_y1, 100);

	// 2 TODO: dodaj wieloosobowosc w czujniku, wtedy mozesz odkomentowac
	//if(person_y2>15) person_y2 = 15;

	//V_ON(LEDS[person_y2]);

	for(int k=0;k<16;k++) // TODO: zmienic na ARRAY_SIZE zeby dzialalo
	{
		//if(person_y1 != k) V_OFF(LEDS[k]); // Dodaj potem to jak bedzie 2 osoba do ifa: && person_y2 != k
		if(person_y1 != k) set_brightness(person_y1, 0);
	}
}
//-----------------------------------------------------------------------
extern ld2450_t	*ld2450_bottom;
static void display_LED()
{
	int32_t x, y;
	while(true) 
	{
		sleep_ms(10);
		//print_matrix((int)LD2450.target1X, (int)LD2450.target1Y);
		get_person(ld2450_bottom, 0, &x, &y);
		show_LED((int)x, (int)y);
		//ESP_LOGV(TAG, "1X=%d 1Y=%d", (int)LD2450.target1X, (int)LD2450.target1Y);
	}
}



void test_person_LED() 
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
