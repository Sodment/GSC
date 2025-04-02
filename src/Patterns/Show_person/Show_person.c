#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "../../common.h"
#include "../../Detector/HLK-LD2450.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//-----------------------------------------------------------------------
#define SIZE 16 // From -16 to 16
extern const int LEDS[];
static const char *TAG = "LD2450";
static const uint32_t MEM_BUFFER_SIZE = 50;

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

typedef struct range
{
	int min;
	int max;
} Range;

bool checkRange(Range r, int a)
{
	return r.min < a && a < r.max;
}

typedef struct BoundingBox2D
{
	Range X;
	Range Y;
} BoundingBox2D;

bool checkBoundingBox2D(BoundingBox2D bb, int x, int y)
{
	return checkRange(bb.X, x) && checkRange(bb.Y, y);
}

int clamp(int a, int min, int max)
{
	if (a < min)
		return min;
	if (a > max)
		return max;
	return a;
}

void show_LED4(int x, int y)
{
	enum BOXES
	{
		B_BOT,
		B_MID,
		B_TOP,
		B_MAX
	};

	enum STATE
	{
		S_FORWARD,
		S_BACKWARD,
		S_HOLD,
		S_OFF,
		S_MAX
	};

	static int state = S_OFF;
	static int direction = B_MAX;
	static int AnimationTime = 0;
	static int previousTime = 0;

	const int AnimationDurationMS = 3 * 1000;
	const int stairCount = 13;
	const int durationPerStep = AnimationDurationMS / stairCount;
	const int maxBrightness = 100 / 2;

	BoundingBox2D BB[B_MAX];

	BB[B_BOT] = (BoundingBox2D){{-200, 200}, {50, 130}};
	BB[B_MID] = (BoundingBox2D){{-200, 200}, {130, 450}};
	BB[B_TOP] = (BoundingBox2D){{-200, 200}, {450, 520}};

	int presence = B_MAX;
	for (int i = 0; i < B_MAX; i++)
	{
		if (checkBoundingBox2D(BB[i], x, y))
		{
			presence = i;
			break;
		}
	}

	int currentTime = millis();
	int dT = currentTime - previousTime;
	previousTime = currentTime;

	// DEBUG
	static int previousState = S_HOLD;
	if (previousState != state)
	{
		previousState = state;
		ESP_LOGV(TAG, "state = %d", (int)state);
	}

	static int previousPres = B_MAX;
	if (previousPres != presence)
	{
		previousPres = presence;
		ESP_LOGV(TAG, "X = %d, Y=%d", (int)x, (int)y);
		ESP_LOGV(TAG, "presence = %d", (int)presence);
	}
	// DEBUG_END

	switch (state)
	{
	case S_OFF:
	{
		AnimationTime = 0;
		int i;
		for (i = 0; i < stairCount; i++)
		{
			set_brightness(i, 0);
		}

		if (presence != B_MAX)
		{
			state = S_FORWARD;
			direction = presence != B_MID ? presence : B_BOT;
		}
		break;
	}
	case S_FORWARD:
	case S_BACKWARD:
	{
		// Progress animation

		AnimationTime += state == S_FORWARD ? dT : -dT;
		AnimationTime = clamp(AnimationTime, 0, AnimationDurationMS);
		// ESP_LOGV(TAG, "A");

		int i;
		int animation = AnimationTime;

		int botomToTop = (direction == B_BOT && state == S_FORWARD) || (direction == B_TOP && state == S_BACKWARD);

		for (i = botomToTop ? 0 : stairCount - 1; i >= 0 && i < stairCount; i += botomToTop ? 1 : -1)
		{
			int timeInStep = clamp(animation, 0, durationPerStep);
			animation -= timeInStep;

			uint8_t brightness = 0;
			if (timeInStep > 0)
			{
				brightness = 1;
			}
			// ESP_LOGV(TAG, "stair=%d, brightness=%f", (int)i, brightness);
			set_brightness(i, brightness);
		}

		if (AnimationTime == AnimationDurationMS)
		{
			state = S_HOLD;
		}

		if (AnimationTime == 0)
		{
			state = S_OFF;
		}

		break;
	}
	case S_HOLD:
		int i;
		AnimationTime = AnimationDurationMS;
		for (i = 0; i < stairCount; i++)
		{
			set_brightness(i, maxBrightness);
		}

		if (presence == B_MAX)
		{
			state = S_BACKWARD;
		}
	}
}

void show_LED3(int input_x1, int input_y1) //, int input_x2, int input_y2
{
	for (int i = 0; i < 13; i++)
	{
		set_brightness(i, 1);
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
		if ((input_y1 <= 130) && ((input_y1 >= 50)))
		{
			state = STATE_WAIT_FOR_ANIMATE_FORWARD;
		}
		else if (input_y1 >= 400)
		{
			state = STATE_WAIT_FOR_ANIMATE_BACKWARD;
		}
		break;

	case STATE_WAIT_FOR_ANIMATE_FORWARD:
		for (int i = 0; i < STAIRS_COUNT; i++)
		{
			set_brightness(i, 100);
			sleep_ms(200);
		}
		sleep_ms(3000);
		for (int i = 0; i < STAIRS_COUNT; i++)
		{
			set_brightness(i, 0);
			sleep_ms(200);
		}
		state = STATE_WAIT_FOR_EVENT;
		break;
	case STATE_WAIT_FOR_ANIMATE_BACKWARD:
		for (int i = STAIRS_COUNT - 1; i >= 0; i--)
		{
			set_brightness(i, 100);
			sleep_ms(200);
		}
		sleep_ms(3000);
		for (int i = STAIRS_COUNT - 1; i >= 0; i--)
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
	// Person is not detected, show default scene
	uint8_t offMode = 1;
	// Person is detected!
	uint8_t onMode[] = {1, 10, 40, 50, 40, 10, 1}; // max number of stairs ON [ x x - - - - - - - - - - - ],   [ - - x x x x x x x - - - - ]
	int onModeLen = sizeof(onMode) / sizeof(onMode[0]);

	// int minLength = 0; FYI
	// int maxLength = 600; FYI
	int gap = 32;							// space between steps, cm
	int startLength = 120;					// cm
	int endLength = startLength + 13 * gap; // 526 cm

	int32_t person_y1 = (input_y1 - startLength) / gap; // wartosc jest ujemna

	if (input_y1 > startLength && input_y1 < endLength)
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
		for (int j = k - 4; j >= 0 && j < k; j++)
		{
			set_brightness(j, 0);
		}

		if (k - 3 >= 0)
			set_brightness(k - 3, onMode[0]);
		if (k - 2 >= 0)
			set_brightness(k - 2, onMode[1]);
		if (k - 1 >= 0)
			set_brightness(k - 1, onMode[2]);
		set_brightness(k, onMode[3]);
		if (k + 1 < 13)
			set_brightness(k + 1, onMode[4]);
		if (k + 2 < 13)
			set_brightness(k + 2, onMode[5]);
		if (k + 3 < 13)
			set_brightness(k + 3, onMode[6]);

		for (int j = k + 4; j < 13; j++)
		{
			set_brightness(j, 0);
		}
	}
	else
	{
		set_brightness(0, offMode);
		for (int j = 1; j < 12; j++)
		{
			set_brightness(j, 0);
		}
		set_brightness(12, offMode);
	}
}

//-----------------------------------------------------------------------
extern ld2450_t *ld2450_bottom;

/*
static void display_LED()
{
	esp_log_level_set(TAG, ESP_LOG_VERBOSE);
	// esp_log_level_set(TAG, ESP_LOG_VERBOSE);
	int32_t x, y;
	// int32_t x2, y2;
	// int32_t x3, y3;
	int32_t px = 0, py = 0;
	int32_t rx, ry;
	int i = 0;
	while (true)
	{
		sleep_ms(10);
		// print_matrix((int)LD2450.target1X, (int)LD2450.target1Y);
		get_person(ld2450_bottom, 0, &x, &y);
		// get_person(ld2450_bottom, 1, &x2, &y2);
		// get_person(ld2450_bottom, 1, &x3, &y3);
		// get_person(ld2450_top, 0, &x2, &y2);
		rx = abs(x - px);
		ry = abs(y - py);
		if (rx > 10)
		{
			px = x;
		}
		if (ry > 10)
		{
			py = y;
		}
		show_LED((int)x, (int)y); // TODO: set to px and py back
		if (i % 10 == 0)
			ESP_LOGV(TAG, "1X=%d 1Y=%d", (int)x, (int)y);
		i++;
	}
}
*/

int comp(const void *elem1, const void *elem2)
{
	int32_t f = *((int32_t *)elem1);
	int32_t s = *((int32_t *)elem2);
	if (f > s)
		return 1;
	if (f < s)
		return -1;
	return 0;
}

const int MAX_PEOPLE = 3;
static void display_LED()
{
	esp_log_level_set(TAG, ESP_LOG_VERBOSE);

	struct Person
	{
		struct
		{
			int32_t x[MEM_BUFFER_SIZE];
			int32_t y[MEM_BUFFER_SIZE];
		} samples;
		struct
		{
			int32_t x;
			int32_t y;
		} mean;
	} people[MAX_PEOPLE];

	struct sample
	{
		int32_t x;
		int32_t y;
	} samples[MAX_PEOPLE];

	int32_t x_copy[MEM_BUFFER_SIZE];
	int32_t y_copy[MEM_BUFFER_SIZE];

	int person = 0;
	for (person = 0; person < MAX_PEOPLE; person++)
	{
		memset(people[person].samples.x, 0, MEM_BUFFER_SIZE * sizeof(int32_t));
		memset(people[person].samples.y, 0, MEM_BUFFER_SIZE * sizeof(int32_t));
	}

	uint32_t lastRecievedMillis = 0;

	int32_t x, y;
	uint32_t i = 0;
	while (true)
	{
		int person = 0;
		uint32_t currentMillis;
		for (person = 0; person < MAX_PEOPLE; person++)
		{
			currentMillis = get_person(ld2450_bottom, person, &samples[person].x, &samples[person].y);
		}

		if (currentMillis <= lastRecievedMillis)
		{
			continue;
		}

		// Fit samples to the best people, shitty way
		for (int x = 0; x < MAX_PEOPLE; x++)
		{
			for (int y = 0; y < MAX_PEOPLE; y++)
			{
				samples[person].x
			}
		}

		lastRecievedMillis = currentMillis;

		for (person = 0; person < MAX_PEOPLE; person++)
		{
			memcpy(x_copy, people[person].samples.x, MEM_BUFFER_SIZE * sizeof(int32_t));
			memcpy(y_copy, people[person].samples.y, MEM_BUFFER_SIZE * sizeof(int32_t));

			qsort(x_copy, MEM_BUFFER_SIZE, sizeof(int32_t), comp);
			qsort(y_copy, MEM_BUFFER_SIZE, sizeof(int32_t), comp);

			people[person].mean.x = x_copy[MEM_BUFFER_SIZE / 2];
			people[person].mean.y = y_copy[MEM_BUFFER_SIZE / 2];
		}

		show_LED(x, y);
		if (i % 10 == 0)
		{
			// ESP_LOGV(TAG, "1X=%d 1Y=%d", (int)x, (int)y);
		}
		i++;
		if (i % MEM_BUFFER_SIZE == 0)
		{
			i = 0;
		}
	}
}

void thread_detect_person()
{
	xTaskCreate(
		display_LED,
		"Person LED", // Nazwa zadania
		5048,		  // Rozmiar stosu (w słowach)
		NULL,		  // Parametry przekazywane do wątku
		5,			  // Priorytet zadania
		NULL		  // Uchwyt do zadania (opcjonalny)
	);
}
