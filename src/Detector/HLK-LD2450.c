//#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "HLK-LD2450.h"
#include "../common.h"
#include "uart_dma/uart_dma.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "driver/gpio.h"

#define CHECK_BIT(var, pos) (((var) >> (pos)) & 1)
#define STATE_SIZE 8
#define TARGETS 3

int64_t lastPeriodicMillis;// = millis();
extern const int LEDS[];
//int64_t millis();

// SENSOR //-----------------------------------------------------------------------
typedef struct {
    int32_t state;  // State to store the sensor's value
} Sensor;

typedef struct {
    Sensor lastCommandSuccess;

    Sensor target1Resolution;
    Sensor target1Speed;
    Sensor target1X;
    Sensor target1Y;

    Sensor target2Resolution;
    Sensor target2Speed;
    Sensor target2X;
    Sensor target2Y;

    Sensor target3Resolution;
    Sensor target3Speed;
    Sensor target3X;
    Sensor target3Y;

    Sensor targets;

    //uint32_t lastPeriodicMillis;

} LD2450_t;
LD2450_t LD2450;
static const char *TAG = "LD2450";

uint16_t twoByteToUint(char firstByte, char secondByte) {
	return (uint16_t)(secondByte << 8) + firstByte;
}

//-----------------------------------------------------------------------

void reportTargetInfo(int target, char *raw) 
{
    int16_t newX, newY, newSpeed, sum;
    uint16_t newResolution;

    // ESP_LOGV(TAG, "Will reporting taget %d", target);

    switch (target) {
	case 0:
        newX = twoByteToUint(raw[0], raw[1] & 0x7F);
        if (raw[1] >> 7 != 0x1)
          	newX = 0 - newX / 10;
        else
			newX = newX / 10;
        if (LD2450.target1X.state != newX)
			LD2450.target1X.state = newX;
          //target1X->publish_state(newX);
        newY = twoByteToUint(raw[2], raw[3] & 0x7F);
        if (raw[3] >> 7 != 0x1)
          newY = 0 - newY / 10;
        else
          newY = newY / 10;
        if (LD2450.target1Y.state != newY)
          LD2450.target1Y.state = newY;
		  //target1Y->publish_state(newY);
        newSpeed = twoByteToUint(raw[4], raw[5] & 0x7F);
        if (raw[5] >> 7 != 0x1)
          newSpeed = 0 - newSpeed;
        if (LD2450.target1Speed.state != newSpeed)
          LD2450.target1Speed.state = newSpeed;
        newResolution = twoByteToUint(raw[6], raw[7]);
        if (LD2450.target1Resolution.state != newResolution)
          LD2450.target1Resolution.state = newResolution;
        break;
      case 1:
        newX = twoByteToUint(raw[0], raw[1] & 0x7F);
        if (raw[1] >> 7 != 0x1)
          newX = 0 - newX / 10;
        else
          newX = newX / 10;
        if (LD2450.target2X.state != newX)
          LD2450.target2X.state = newX;
        newY = twoByteToUint(raw[2], raw[3] & 0x7F);
        if (raw[3] >> 7 != 0x1)
          newY = 0 - newY / 10;
        else
          newY = newY / 10;
        if (LD2450.target2Y.state != newY)
          LD2450.target2Y.state = newY;
        newSpeed = twoByteToUint(raw[4], raw[5] & 0x7F);
        if (raw[5] >> 7 != 0x1)
          newSpeed = 0 - newSpeed;
        if (LD2450.target2Speed.state != newSpeed)
          LD2450.target2Speed.state = newSpeed;
        newResolution = twoByteToUint(raw[6], raw[7]);
        if (LD2450.target2Resolution.state != newResolution)
          LD2450.target2Resolution.state = newResolution;
        break;
      case 2:
        newX = twoByteToUint(raw[0], raw[1] & 0x7F);
        if (raw[1] >> 7 != 0x1)
          newX = 0 - newX / 10;
        else
          newX = newX / 10;
        if (LD2450.target3X.state != newX)
          LD2450.target3X.state = newX;
        newY = twoByteToUint(raw[2], raw[3] & 0x7F);
        if (raw[3] >> 7 != 0x1)
          newY = 0 - newY / 10;
        else
          newY = newY / 10;
        if (LD2450.target3Y.state != newY)
          LD2450.target3Y.state = newY;
        newSpeed = twoByteToUint(raw[4], raw[5] & 0x7F);
        if (raw[5] >> 7 != 0x1)
          newSpeed = 0 - newSpeed;
        if (LD2450.target3Speed.state != newSpeed)
          LD2450.target3Speed.state = newSpeed;
        newResolution = twoByteToUint(raw[6], raw[7]);
        if (LD2450.target3Resolution.state != newResolution)
          LD2450.target3Resolution.state = newResolution;
        break;
    }
    sum = 0;
    if (LD2450.target1Resolution.state > 0){
      sum+=1;
    }
    if (LD2450.target2Resolution.state > 0){
      sum+=1;
    }
    if (LD2450.target3Resolution.state > 0){
      sum+=1;
    }
    if (LD2450.targets.state != sum){
      LD2450.targets.state = sum;
    }
  }

void handlePeriodicData(char *buffer, int len) 
{
	if (len < 29)
		return;  // 4 frame start bytes + 2 length bytes + 1 data end byte + 1 crc byte + 4 frame end bytes
	if (buffer[0] != 0xAA || buffer[1] != 0xFF || buffer[2] != 0x03 || buffer[3] != 0x00)
		return;  // check 4 frame start bytes
	if (buffer[len - 2] != 0x55 || buffer[len - 1] != 0xCC)
		return;  //  data end=0x55, 0xcc
	char stateBytes[STATE_SIZE];

	/*
		Reduce data update rate to prevent home assistant database size glow fast
	*/
	int64_t currentMillis = millis();
	if (currentMillis - lastPeriodicMillis < 1000)
		return;
	lastPeriodicMillis = currentMillis;
	for (int i = 0; i < TARGETS; i++) {
		memcpy(stateBytes, &buffer[4 + i * STATE_SIZE], STATE_SIZE);
		reportTargetInfo(i, stateBytes);
	}
}
uart_dma_t *uart_init()
{
	uart_dma_buf_cfg_t	cfg;

	//TODO: ponizsze parametry sa ustawione na sztywno, trzeba dorobic edycje ich za pomoca strony www oraz wykorzystac modul polaczenie uart (podobnie jak modul ethernet_uart)
	{
		cfg.numer_uarta				=	1;

		cfg.gpio_pin_tx				=	21;
		cfg.gpio_pin_rx				=	36;
		cfg.gpio_pin_rts			=	-1;
		cfg.rozmiar_kazdego_bufora_tx_i_rx	=	1024;
		cfg.baudrate				=	256000;
		cfg.uhci_dev				=	&UHCI0;
		cfg.inwersja_rx_tx			=	false;
		cfg.inwersja_dir			=	false;

		return uart_dma_init(&cfg);
	}
}

#define SIZE 16 // From -30 to 30

void show_LED(int input_x1, int input_y1, int input_x2, int input_y2) {
    uint32_t person_y1 = input_y1 / 30;
	uint32_t person_y2 = input_y2 / 30;
	// 1
	if(person_y1>15) person_y1 = 15;

	V_ON(LEDS[person_y1]);

	// 2
	if(person_y2>15) person_y2 = 15;

	V_ON(LEDS[person_y2]);

	for(int k=0;k<16;k++) // TODO: zmienic na ARRAY_SIZE zeby dzialalo
	{
		if(person_y1 != k && person_y2 != k) V_OFF(LEDS[k]);
	}
}

void print_matrix(int input_x, int input_y) {
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

void display_LED()
{
	while(true) 
	{
		sleep_ms(10);
		//print_matrix((int)LD2450.target1X.state, (int)LD2450.target1Y.state);
		//show_LED((int)LD2450.target1X.state, (int)LD2450.target1Y.state, (int)LD2450.target2X.state, (int)LD2450.target2Y.state);
		//ESP_LOGV(TAG, "1X=%d 1Y=%d", (int)LD2450.target1X.state, (int)LD2450.target1Y.state);

	}
}

void thread_detector() 
{
 	uart_dma_t	*uart;
	lastPeriodicMillis = millis();
	uart = uart_init();
	uart_dma_uruchom_odbior(uart);

	
	// ESP_LOGV(TAG, "Start watku detector!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
	//esp_task_wdt_add(NULL);
	while(true) 
	{
		// Odśwież zegar watchdog
        //esp_task_wdt_reset();  // Resetuje zegar WDT dla tego zadania

		sleep_ms(1);
		uart_dma_obsluga(uart);
		if(uart_dma_czy_odebrano_ramke(uart))
		{
			uint32_t dlugosc;
			dlugosc = uart->odebrano_bajtow;
			 handlePeriodicData((char*)uart->bufor_rx, dlugosc);
			//V_ON(LEDS[3]);
			// if (dlugosc==4)
			// {
				
			// 	if (memcmp(test, uart->bufor_rx, 4)==0)
			// 	{
			// 		V_ON(LEDS[4]);
			// 	}
			// }
			uart_dma_uruchom_odbior(uart);
			// ESP_LOGV(TAG, "1X=%d 1Y=%d!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!", (int)LD2450.target1X.state, (int)LD2450.target1Y.state);
			// uart_dma_wyslij(uart, test, 4);
			// uart_dma_czekaj_na_zakonczenie_nadawania(uart);
		}

	}
}

void detector_init()
{
	xTaskCreate(
        display_LED,		// Funkcja wątku
        "Debug console",          	// Nazwa zadania
        2048,              	// Rozmiar stosu (w słowach)
        NULL,              	// Parametry przekazywane do wątku
        5,                 	// Priorytet zadania
        NULL               	// Uchwyt do zadania (opcjonalny)
    );

		xTaskCreate(
        thread_detector,		// Funkcja wątku
        "Detector",          	// Nazwa zadania
        2048,              	// Rozmiar stosu (w słowach)
        NULL,              	// Parametry przekazywane do wątku
        5,                 	// Priorytet zadania
        NULL               	// Uchwyt do zadania (opcjonalny)
    );
}
