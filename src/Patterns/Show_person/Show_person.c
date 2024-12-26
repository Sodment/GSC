//-----------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "../../common.h"
//-----------------------------------------------------------------------
#define SIZE 16 // From -16 to 16
extern const int LEDS[];
//-----------------------------------------------------------------------
/*
void show_LED(int input_x1, int input_y1, int input_x2, int input_y2)
{
    uint32_t person_y1 = input_y1 / 30;
	uint32_t person_y2 = input_y2 / 30;
	// 1
	if(person_y1>15) person_y1 = 15;

	V_ON(LEDS[person_y1]);

	// 2 TODO: dodaj wieloosobowosc w czujniku, wtedy mozesz odkomentowac
	//if(person_y2>15) person_y2 = 15;

	//V_ON(LEDS[person_y2]);

	for(int k=0;k<16;k++) // TODO: zmienic na ARRAY_SIZE zeby dzialalo
	{
		if(person_y1 != k && person_y2 != k) V_OFF(LEDS[k]);
	}
}
//-----------------------------------------------------------------------
static void display_LED(void *par)
{
	ld2450_int_t	*ld = par;
	while(true) 
	{
		sleep_ms(10);
		//print_matrix((int)LD2450.target1X, (int)LD2450.target1Y);
		show_LED((int)ld->target1X, (int)ld->target1Y, (int)ld->target2X, (int)ld->target2Y);
		//ESP_LOGV(TAG, "1X=%d 1Y=%d", (int)LD2450.target1X, (int)LD2450.target1Y);
	}
}




	xTaskCreate(
        display_LED,		// Funkcja wątku
        "LED",          	// Nazwa zadania
        2048,              	// Rozmiar stosu (w słowach)
        ld,              	// Parametry przekazywane do wątku
        5,                 	// Priorytet zadania
        NULL               	// Uchwyt do zadania (opcjonalny)
    );

 */
//-----------------------------------------------------------------------
