#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "HLK-LD2450.h"
#include "../common.h"
#include "../uart_dma/uart_dma.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "driver/gpio.h"
//-----------------------------------------------------------------------
#define CHECK_BIT(var, pos) 	(((var) >> (pos)) & 1)
#define STATE_SIZE 				8
#define TARGETS 				3

static const char *TAG = "LD2450";
//-----------------------------------------------------------------------
typedef struct
{
	uart_dma_t 	*uart;
    int32_t 	lastCommandSuccess;

    int32_t 	target1Resolution;
    int32_t 	target1Speed;
    int32_t 	target1X;
    int32_t 	target1Y;

    int32_t 	target2Resolution;
    int32_t 	target2Speed;
    int32_t 	target2X;
    int32_t 	target2Y;

    int32_t 	target3Resolution;
    int32_t 	target3Speed;
    int32_t 	target3X;
    int32_t 	target3Y;

    int32_t 	targets;

	int64_t 	lastPeriodicMillis;
} ld2450_int_t;
//-----------------------------------------------------------------------
static uint16_t twoByteToUint(char firstByte, char secondByte)
{
	return (uint16_t)(secondByte << 8) + firstByte;
}
//-----------------------------------------------------------------------
// Source: https://github.com/Chreece/LD2450-ESPHome/blob/master/custom_components/ld2450_uart.h
static void reportTargetInfo(ld2450_int_t *ld, int target, char *raw) 
{
    int16_t newX, newY, newSpeed, sum;
    uint16_t newResolution;

	//ESP_LOGV(TAG, "Will reporting taget %d", target);

    switch (target) {
	case 0:
        newX = twoByteToUint(raw[0], raw[1] & 0x7F);
        if (raw[1] >> 7 != 0x1)
          	newX = 0 - newX / 10;
        else
			newX = newX / 10;
        if (ld->target1X != newX)
			ld->target1X = newX;
          //target1X->publish_state(newX);
        newY = twoByteToUint(raw[2], raw[3] & 0x7F);
        if (raw[3] >> 7 != 0x1)
          newY = 0 - newY / 10;
        else
          newY = newY / 10;
        if (ld->target1Y != newY)
          ld->target1Y = newY;
		  //target1Y->publish_state(newY);
        newSpeed = twoByteToUint(raw[4], raw[5] & 0x7F);
        if (raw[5] >> 7 != 0x1)
          newSpeed = 0 - newSpeed;
        if (ld->target1Speed != newSpeed)
          ld->target1Speed = newSpeed;
        newResolution = twoByteToUint(raw[6], raw[7]);
        if (ld->target1Resolution != newResolution)
          ld->target1Resolution = newResolution;
        break;
      case 1:
        newX = twoByteToUint(raw[0], raw[1] & 0x7F);
        if (raw[1] >> 7 != 0x1)
          newX = 0 - newX / 10;
        else
          newX = newX / 10;
        if (ld->target2X != newX)
          ld->target2X = newX;
        newY = twoByteToUint(raw[2], raw[3] & 0x7F);
        if (raw[3] >> 7 != 0x1)
          newY = 0 - newY / 10;
        else
          newY = newY / 10;
        if (ld->target2Y != newY)
          ld->target2Y = newY;
        newSpeed = twoByteToUint(raw[4], raw[5] & 0x7F);
        if (raw[5] >> 7 != 0x1)
          newSpeed = 0 - newSpeed;
        if (ld->target2Speed != newSpeed)
          ld->target2Speed = newSpeed;
        newResolution = twoByteToUint(raw[6], raw[7]);
        if (ld->target2Resolution != newResolution)
          ld->target2Resolution = newResolution;
        break;
      case 2:
        newX = twoByteToUint(raw[0], raw[1] & 0x7F);
        if (raw[1] >> 7 != 0x1)
          newX = 0 - newX / 10;
        else
          newX = newX / 10;
        if (ld->target3X != newX)
          ld->target3X = newX;
        newY = twoByteToUint(raw[2], raw[3] & 0x7F);
        if (raw[3] >> 7 != 0x1)
          newY = 0 - newY / 10;
        else
          newY = newY / 10;
        if (ld->target3Y != newY)
          ld->target3Y = newY;
        newSpeed = twoByteToUint(raw[4], raw[5] & 0x7F);
        if (raw[5] >> 7 != 0x1)
          newSpeed = 0 - newSpeed;
        if (ld->target3Speed != newSpeed)
          ld->target3Speed = newSpeed;
        newResolution = twoByteToUint(raw[6], raw[7]);
        if (ld->target3Resolution != newResolution)
          ld->target3Resolution = newResolution;
        break;
    }
    sum = 0;
    if (ld->target1Resolution > 0){
      sum+=1;
    }
    if (ld->target2Resolution > 0){
      sum+=1;
    }
    if (ld->target3Resolution > 0){
      sum+=1;
    }
    if (ld->targets != sum){
      ld->targets = sum;
    }
  }
//-----------------------------------------------------------------------
// Source: https://github.com/Chreece/LD2450-ESPHome/blob/master/custom_components/ld2450_uart.h
static void handlePeriodicData(ld2450_int_t *ld, char *buffer, int len) 
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
	if (currentMillis - ld->lastPeriodicMillis < 1000)
		return;
	ld->lastPeriodicMillis = currentMillis;
	for (int i = 0; i < TARGETS; i++)
	{
		memcpy(stateBytes, &buffer[4 + i * STATE_SIZE], STATE_SIZE);
		reportTargetInfo(ld, i, stateBytes);
	}
}
//-----------------------------------------------------------------------
/*
	Inicjalizacja UARTa na potrzeby komunikacji z jednym czujnikiem LD2450
	uart_nr - 1 lub 2
*/
static uart_dma_t *uart_init(uint8_t uart_nr, int8_t pin_tx, int8_t pin_rx)
{
	uart_dma_buf_cfg_t	cfg;

	cfg.numer_uarta						=	uart_nr;

	cfg.gpio_pin_tx						=	pin_tx;
	cfg.gpio_pin_rx						=	pin_rx;
	cfg.gpio_pin_rts					=	-1;
	cfg.rozmiar_kazdego_bufora_tx_i_rx	=	1024;
	cfg.baudrate						=	256000;
	if (uart_nr==1)
	{
		cfg.uhci_dev					=	&UHCI0;
	}
	else
	{
		cfg.uhci_dev					=	&UHCI1;
	}
	cfg.inwersja_rx_tx					=	false;
	cfg.inwersja_dir					=	false;

	return uart_dma_init(&cfg);
}
//-----------------------------------------------------------------------
static void thread_sensor_poll(void *par)
{
	ld2450_int_t	*ld = par;
	uart_dma_t		*uart = ld->uart;

	ld->lastPeriodicMillis = millis();
	uart_dma_uruchom_odbior(uart);

	//esp_task_wdt_add(NULL);
	while(true) 
	{
		// Odśwież zegar watchdog
        //esp_task_wdt_reset();  // Resetuje zegar WDT dla tego zadania

		sleep_ms(1);
		uart_dma_obsluga(uart);
		if(uart_dma_czy_odebrano_ramke(uart))
		{
			uint32_t dlugosc = uart->odebrano_bajtow;
		 	handlePeriodicData(ld, (char*)uart->bufor_rx, dlugosc);
			uart_dma_uruchom_odbior(uart);
			// uart_dma_wyslij(uart, test, 4);
			// uart_dma_czekaj_na_zakonczenie_nadawania(uart);
		}
		ESP_LOGV(TAG, "Czujnik %d", uart->cfg.numer_uarta);
		//ESP_LOGV(TAG, "configMAX_PRIORITIES %d", configMAX_PRIORITIES);
	}
}
//-----------------------------------------------------------------------
ld2450_t *detector_init(uint8_t uart_nr, int8_t pin_tx, int8_t pin_rx)
{
	ld2450_int_t	*ld = calloc(1, sizeof(ld2450_int_t));
	if (!ld) return ld;

	esp_log_level_set(TAG, ESP_LOG_VERBOSE);

	ld->uart = uart_init(uart_nr, pin_tx, pin_rx);

		xTaskCreate(
        thread_sensor_poll,		// Funkcja wątku
        "Detector",          	// Nazwa zadania
        2048,              	// Rozmiar stosu (w słowach)
        ld,              	// Parametry przekazywane do wątku
        1,                 	// Priorytet zadania
        NULL               	// Uchwyt do zadania (opcjonalny)
    );
	return ld;
}
//-----------------------------------------------------------------------
