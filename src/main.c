#include "driver/gpio.h"
#include "driver/ledc.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>

#include "uart_dma/uart_dma.h"
#include "common.h"

// PIN MAPPING
#define LED_0 GPIO_NUM_15
#define LED_1 GPIO_NUM_2
#define LED_2 GPIO_NUM_4
#define LED_3 GPIO_NUM_32
#define LED_4 GPIO_NUM_33
#define LED_5 GPIO_NUM_25
#define LED_6 GPIO_NUM_26
#define LED_7 GPIO_NUM_27
#define LED_8 GPIO_NUM_14
#define LED_9 GPIO_NUM_12
#define LED_10 GPIO_NUM_13
#define LED_11 GPIO_NUM_23 // Tranzystor wisi
#define LED_12 GPIO_NUM_22
#define LED_13 GPIO_NUM_18
#define LED_14 GPIO_NUM_5
#define LED_15 GPIO_NUM_19
#define RX_PIN GPIO_NUM_36
#define TX_PIN GPIO_NUM_21

const int LEDS[] = {LED_0, LED_1, LED_2, LED_3, LED_4, LED_5, LED_6, LED_7, LED_8, LED_9, LED_10, LED_11, LED_12, LED_13, LED_14, LED_15};
#define LED_LENGTH (ARRAY_SIZE(LEDS))

static 	uart_dma_t	*uart;

void uart_init()
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

		uart = uart_dma_init(&cfg);
	}
}

void setup()
{
    for (int i = 0; i < LED_LENGTH; i++)
    {
        gpio_reset_pin(LEDS[i]);
        gpio_set_direction(LEDS[i], GPIO_MODE_OUTPUT);
    }
    gpio_reset_pin(RX_PIN);
    gpio_set_direction(RX_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(RX_PIN, GPIO_PULLUP_ONLY);
    gpio_reset_pin(TX_PIN);
    gpio_set_direction(RX_PIN, GPIO_MODE_OUTPUT);
	uart_init();
	uart_dma_uruchom_odbior(uart);
}

void test_mrugania_ledow(void *pvParameters)
{
    // while (1) {
    //     printf("Wątek działa! \n");
    //     vTaskDelay(pdMS_TO_TICKS(1000)); // Opóźnienie 1 sekundy
    // }
	while (1)
	{
		for (int i = 0; i < LED_LENGTH; i++)
        {
            V_ON(LEDS[i]);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            V_OFF(LEDS[i]);

			//uart_dma_wyslij()
        }
	}
}

static uint8_t test[]="test";



void app_main()
{
    setup();
	//uart_dma_wyslij()

	uart_dma_wyslij(uart, test, 4);
	uart_dma_czekaj_na_zakonczenie_nadawania(uart);
 
    V_ON(LEDS[2]);

	// Tworzenie wątku
    xTaskCreate(
        test_mrugania_ledow,// Funkcja wątku
        "LED",          	// Nazwa zadania
        2048,              	// Rozmiar stosu (w słowach)
        NULL,              	// Parametry przekazywane do wątku
        5,                 	// Priorytet zadania
        NULL               	// Uchwyt do zadania (opcjonalny)
    );

    while (true)
    {
        /*for (int i = 0; i < LED_LENGTH; i++)
        {
            V_ON(LEDS[i]);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            V_OFF(LEDS[i]);

			//uart_dma_wyslij()
        }
		*/
		
		uart_dma_obsluga(uart);
		if(uart_dma_czy_odebrano_ramke(uart))
		{
			uint32_t dlugosc;
			dlugosc = uart->odebrano_bajtow;
			V_ON(LEDS[3]);
			if (dlugosc==4)
			{
				
				if (memcmp(test, uart->bufor_rx, 4)==0)
				{
					V_ON(LEDS[4]);
				}
			}
			uart_dma_uruchom_odbior(uart);
			uart_dma_wyslij(uart, test, 4);
			uart_dma_czekaj_na_zakonczenie_nadawania(uart);
		}


    }
    // gpio_dump_io_configuration(stdout, (1ULL << 34) | (1ULL << 18) | (1ULL << 26));
}