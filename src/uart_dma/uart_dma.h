#ifndef __UART_DMA_H__
#define __UART_DMA_H__
//--------------------------------------------------------------------------------------------------------------------------------------------------
#include <rom/lldesc.h>
#include <soc/uart_struct.h>
#include <soc/uhci_struct.h>
#include <soc/uhci_reg.h>
//--------------------------------------------------------------------------------------------------------------------------------------------------
enum
{
	uart_dma_parzystosc_brak,
	uart_dma_parzystosc_parzysta,
	uart_dma_parzystosc_nieparzysta,
};
//--------------------------------------------------------------------------------------------------------------------------------------------------
enum
{
	uart_dma_bitow_stopu_1,
	uart_dma_bitow_stopu_1_5,
	uart_dma_bitow_stopu_2,
};
//--------------------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
	uint8_t				numer_uarta;
	int8_t				gpio_pin_tx;
	int8_t				gpio_pin_rx;
	int8_t				gpio_pin_rts;

	uint16_t			rozmiar_kazdego_bufora_tx_i_rx;
	uint32_t			baudrate;
	uint8_t 			parzystosc;			//0-brak, 1-parzysta, 2-nieparzysta
	uint8_t 			bitow_stopu;			//0-1bit stopu, 1-1,5bitow stopu, 2-2bity stopu
	uhci_dev_t			*uhci_dev;
	bool				inwersja_rx_tx;
	bool				inwersja_dir;
} uart_dma_buf_cfg_t;
//--------------------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
	uint8_t				*bufor;
	lldesc_t			*deskryptor;
	bool				uruchomiono_nadawanie_odbior;
} uart_dma_buf_desc_t;
//--------------------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
	uart_dev_t			*uart_dev;
	uart_dma_buf_cfg_t		cfg;
	uart_dma_buf_desc_t		tx;
	//uart_dma_buf_desc_t		rx;
	uint8_t				*bufor_rx;
	uint32_t			odebrano_bajtow;
	int64_t			    czas;
	uint32_t			timeout_ms;
	uint32_t			rozpoczeto_odbieranie_ramki	: 1;
	uint32_t			uruchomiono_odbior 		: 1;
	uint32_t			odebrano_ramke	 		: 1;
} uart_dma_t;
//--------------------------------------------------------------------------------------------------------------------------------------------------
	uart_dma_t	*uart_dma_init						(uart_dma_buf_cfg_t	*cfg);
	void		uart_dma_uruchom_odbior					(uart_dma_t		*uart_dma);
	void		uart_dma_ustaw_predkosc					(uart_dma_t		*uart_dma, uint32_t baudrate);
	void		uart_dma_wyslij						(uart_dma_t		*uart_dma, uint8_t *bufor, uint16_t dlugosc);
	int		uart_dma_czekaj_na_zakonczenie_nadawania		(uart_dma_t		*uart_dma);
	void		uart_dma_deinit						(uart_dma_t		**uart_dma);
	bool		uart_dma_obsluga					(uart_dma_t		*uart_dma);
	bool		uart_dma_czy_odebrano_ramke				(uart_dma_t		*uart_dma);
#else
	#define uart_dma_init(x...)						(NULL)
	#define uart_dma_uruchom_odbior(x...)					{}
	#define uart_dma_ustaw_predkosc(x...)					{}
	#define uart_dma_wyslij(x...)						{}
	#define uart_dma_czekaj_na_zakonczenie_nadawania(x...)			(0)
	#define uart_dma_deinit(x...)						{}
	#define uart_dma_obsluga(x...)						{}
	#define uart_dma_czy_odebrano_ramke(x...)				(0)
#endif
//--------------------------------------------------------------------------------------------------------------------------------------------------
