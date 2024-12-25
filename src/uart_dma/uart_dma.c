//--------------------------------------------------------------------------------------------------------------------------------------------------
//#include <Arduino.h>
//#include "crc.h"
//#include "comm.h"
//--------------------------------------------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------------------------------------------------------
#include <string.h>
#include <stdbool.h>
//#include <AsyncMqttClient.h>
#include <soc/i2s_struct.h>
#include <soc/i2s_reg.h>
#include <driver/periph_ctrl.h>
#include <soc/uart_struct.h>
//#include <esp32-hal-uart.h>
#include <soc/dport_reg.h>
//#include "_eeprom.h"
//#include "liczniki.h"
//#include "wm_mqtt.h"
#include "uart_dma.h"
#include <esp_heap_caps.h>
#include <esp_system.h>
#include "esp_private/esp_clk.h"
#include <soc/gpio_sig_map.h>
#include "driver/gpio.h"
#include "soc/gpio_sig_map.h"
#include "rom/gpio.h"
#include "../common.h"
//#include <cores/esp32/esp32-hal-uart.h>

//--------------------------------------------------------------------------------------------------------------------------------------------------
static void uart_dma_deinicjalizacja_bufora(uart_dma_buf_desc_t *bd)
{
	if (bd->bufor)
	{
		heap_caps_free(bd->bufor);
		bd->bufor = NULL;
	}

	if (bd->deskryptor)
	{
		heap_caps_free(bd->deskryptor);
		bd->deskryptor = NULL;
	}

}
//--------------------------------------------------------------------------------------------------------------------------------------------------
static bool uart_dma_inicjalizacja_bufora(uart_dma_buf_cfg_t *cfg, uart_dma_buf_desc_t *bd)
{
	bd->bufor			=	(uint8_t*)heap_caps_malloc(cfg->rozmiar_kazdego_bufora_tx_i_rx, MALLOC_CAP_DMA);
	if (!bd->bufor)	return false;

	lldesc_t *deskryptor;
	deskryptor			=	(lldesc_t *)heap_caps_malloc(sizeof(lldesc_t), MALLOC_CAP_DMA);
	if (!deskryptor) return false;

	bd->deskryptor = deskryptor;

	{
		//inicjalizacja deskryptora
		deskryptor->owner	= 1;
		deskryptor->sosf	= 0;
		deskryptor->buf		= bd->bufor;
		deskryptor->offset	= 0;
		deskryptor->empty	= 0;
		deskryptor->eof		= 1;
	}

	return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void uart_dma_deinit(uart_dma_t **uart_dma)
{
	uart_dma_t *ud = *uart_dma;
	if (!ud) return;
	if (ud->cfg.uhci_dev == &UHCI0)
	{
		periph_module_disable(PERIPH_UHCI0_MODULE);
	}
	if (ud->cfg.uhci_dev == &UHCI1)
	{
		periph_module_disable(PERIPH_UHCI1_MODULE);
	}
	uart_dma_deinicjalizacja_bufora(&ud->tx);
	//uart_dma_deinicjalizacja_bufora(&ud->rx);
	if (ud->bufor_rx)
	{
		free(ud->bufor_rx);
	}
	free(ud);
	*uart_dma = NULL;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void uart_dma_ustaw_predkosc(uart_dma_t *uart_dma, uint32_t baudrate)
{
	uart_dev_t		*uart_dev = uart_dma->uart_dev;
	uart_dma->cfg.baudrate = baudrate;
	//uint32_t clk_div = ((getApbFrequency()<<4)/baudrate);
	uint32_t clk_div = ((esp_clk_apb_freq()<<4)/baudrate);

	uart_dev->clk_div.div_int	= clk_div>>4 ;
	uart_dev->clk_div.div_frag	= clk_div & 0xf;
	uart_dma->timeout_ms = (35000/baudrate)+1;	//3,5 znaka
	//uart_dma->timeout_ms = 10;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
static void uart_dma_inicjalizacja_sprzetu(uart_dma_t *uart_dma)
{
	uart_dev_t		*uart_dev;
	uart_dma_buf_cfg_t	*cfg		=	&uart_dma->cfg;
	uint8_t			numer_uarta	=	cfg->numer_uarta;
	uint8_t			indeks_rx;
	uint8_t			indeks_tx;

	if (numer_uarta == 1)
	{
		//1
		//konsola_debug("Init sprzet UART 1\n");
		uart_dev = (volatile uart_dev_t *)(DR_REG_UART1_BASE);
		DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_UART1_CLK_EN);
		DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_UART1_RST);
		indeks_rx = U1RXD_IN_IDX;
		indeks_tx = U1TXD_OUT_IDX;
		cfg->uhci_dev->conf0.uart1_ce = 1;       //tu jest podlaczenie UARTa z okreslonym numerem do tego kanalu DMA
	}
	else
	if (numer_uarta == 2)
	{
		//2
		//konsola_debug("Init sprzet UART 2\n");
		uart_dev = (volatile uart_dev_t *)(DR_REG_UART2_BASE);
		DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_UART2_CLK_EN);
		DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_UART2_RST);
		indeks_rx = U2RXD_IN_IDX;
		indeks_tx = U2TXD_OUT_IDX;
		cfg->uhci_dev->conf0.uart2_ce = 1;       //tu jest podlaczenie UARTa z okreslonym numerem do tego kanalu DMA
	}
	else
	{
		//0
		//konsola_debug("Init sprzet UART 0\n");
		uart_dev = (volatile uart_dev_t *)(DR_REG_UART_BASE);
		DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_UART_CLK_EN);
		DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_UART_RST);
		indeks_rx = U0RXD_IN_IDX;
		indeks_tx = U0TXD_OUT_IDX;
		cfg->uhci_dev->conf0.uart0_ce = 1;       //tu jest podlaczenie UARTa z okreslonym numerem do tego kanalu DMA
	}

	uart_dma->uart_dev = uart_dev;

	uart_dma_ustaw_predkosc(uart_dma, cfg->baudrate);

	uart_dev->conf0.val		= 0x800001c; //SERIAL_8N1

	switch (cfg->parzystosc)
	{
		case uart_dma_parzystosc_brak:
			uart_dev->conf0.parity    = 0;
			uart_dev->conf0.parity_en = 0;
			break;
		case uart_dma_parzystosc_parzysta:
			uart_dev->conf0.parity    = 0;
			uart_dev->conf0.parity_en = 1;
			break;
		case uart_dma_parzystosc_nieparzysta:
			uart_dev->conf0.parity    = 1;
			uart_dev->conf0.parity_en = 1;
			break;
	}

	switch (cfg->bitow_stopu)
	{
		case uart_dma_bitow_stopu_1:
			uart_dev->conf0.stop_bit_num    = 1;
			break;
		case uart_dma_bitow_stopu_1_5:
			uart_dev->conf0.stop_bit_num    = 2;
			break;
		case uart_dma_bitow_stopu_2:
			uart_dev->conf0.stop_bit_num    = 3;
			break;
	}

	//pinMode(cfg->gpio_pin_rx, INPUT);
	gpio_set_direction(cfg->gpio_pin_rx, GPIO_MODE_INPUT);
	//pinMatrixInAttach(cfg->gpio_pin_rx, indeks_rx, cfg->inwersja_rx_tx);
	gpio_matrix_in( cfg->gpio_pin_rx, indeks_rx, cfg->inwersja_rx_tx);

	//pinMode(cfg->gpio_pin_tx, OUTPUT);
	gpio_set_direction(cfg->gpio_pin_tx, GPIO_MODE_OUTPUT);
	//pinMatrixOutAttach(cfg->gpio_pin_tx, indeks_tx, cfg->inwersja_rx_tx, false);
	gpio_matrix_out( cfg->gpio_pin_tx, indeks_tx, cfg->inwersja_rx_tx, false);

	if (cfg->gpio_pin_rts >= 0)
	{
		//pinMode(cfg->gpio_pin_rts, OUTPUT);
		gpio_set_direction(cfg->gpio_pin_rts, GPIO_MODE_OUTPUT);
		if (cfg->inwersja_dir)
		{
			//digitalWrite(cfg->gpio_pin_rts, HIGH);
			V_ON(cfg->gpio_pin_rts);
		}
		else
		{
			//digitalWrite(cfg->gpio_pin_rts, LOW);
			V_OFF(cfg->gpio_pin_rts);
		}
	}
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
uart_dma_t *uart_dma_init(uart_dma_buf_cfg_t *cfg)
{
	uart_dma_t	*uart_dma;
	uhci_dev_t	*uhci_dev	=	cfg->uhci_dev;

	uart_dma			=	(uart_dma_t*)calloc(1, sizeof(uart_dma_t));
	if (!uart_dma) goto out;

	uart_dma->cfg			=	*cfg;
	
	if (!uart_dma_inicjalizacja_bufora(cfg, &uart_dma->tx)) goto out;
	//if (!uart_dma_inicjalizacja_bufora(cfg, &uart_dma->rx)) goto out;

	uart_dma->bufor_rx = (uint8_t*) calloc(1, cfg->rozmiar_kazdego_bufora_tx_i_rx);
	if (!uart_dma->bufor_rx) goto out;

	if (cfg->uhci_dev == &UHCI0)
	{
		//konsola_debug("UHCI0 000000000\n");
		periph_module_enable(PERIPH_UHCI0_MODULE);
	}

	if (cfg->uhci_dev == &UHCI1)
	{
		//konsola_debug("UHCI1 11111111\n");
		periph_module_enable(PERIPH_UHCI1_MODULE);
	}

	uhci_dev->conf0.val = 0;		//w tym rejestrze sa bity, ktore domyslnie sa niezerowe ale wedlug dokumentacji trzeba je wyzerowac
	uhci_dev->conf1.val = 0;		//w tym rejestrze sa bity, ktore domyslnie sa niezerowe ale wedlug dokumentacji trzeba je wyzerowac

	uhci_dev->dma_out_link.addr	= (uint32_t)uart_dma->tx.deskryptor;
	//uhci_dev->dma_in_link.addr	= (uint32_t)uart_dma->rx.deskryptor;

	//rx-3, tx-1
	
	uart_dma_inicjalizacja_sprzetu(uart_dma);

	uhci_dev->int_clr.val		= 0xFFFFFFFF;

	uart_dma_uruchom_odbior(uart_dma);

	uhci_dev->escape_conf.val	= 0;	//domyslnie uart probuje analizowac znaki na liniach rx i tx i wstawia jakies inne swoje znaki !!!

	return uart_dma;

out:
	uart_dma_deinit(&uart_dma);
	return uart_dma;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void uart_dma_wyslij(uart_dma_t *uart_dma, uint8_t *bufor, uint16_t dlugosc)
{
	//mqtt_debug("wysylam: %p %s %d", uart_dma, bufor, dlugosc);
	if (!uart_dma)	return;
	if (!bufor)	return;
	if (!dlugosc)	return;

	uart_dma_buf_cfg_t	*cfg		=	&uart_dma->cfg;
	uhci_dev_t		*uhci_dev	=	cfg->uhci_dev;
	lldesc_t		*deskryptor	=	uart_dma->tx.deskryptor;
	uart_dev_t		*uart_dev	=	uart_dma->uart_dev;

	//periph_module_disable(PERIPH_UHCI0_MODULE);
	//periph_module_enable(PERIPH_UHCI0_MODULE);

	uart_dma_czekaj_na_zakonczenie_nadawania(uart_dma);

	if (dlugosc > cfg->rozmiar_kazdego_bufora_tx_i_rx)
	{
		return;
	}
	else
	{
		memcpy((void*)deskryptor->buf, bufor, dlugosc);
	}

	uhci_dev->dma_out_link.stop			= 1;

	deskryptor->length = dlugosc;   //liczba bajtow

	{
		//wyrownywanie do 4 bajtow
		if (dlugosc%4)
		{
			dlugosc =  dlugosc/4;
			dlugosc = (dlugosc*4) + 4;
		}
		deskryptor->size = dlugosc;	//wyrownane do slow 4 bajtowych
	}

	//uhci_dev->int_clr.val				= 0xFFFFFFFF;	//TODO: kasowac tylko potrzebne
	uhci_dev->int_clr.out_total_eof			= 1;
	uart_dev->int_clr.tx_done			= 1;
	uart_dev->int_ena.tx_done			= 1;
	
	if (cfg->gpio_pin_rts>=0)
	{
		if (cfg->inwersja_dir)
		{
			//digitalWrite(cfg->gpio_pin_rts, LOW);
			V_OFF(cfg->gpio_pin_rts);
		}
		else
		{
			//digitalWrite(cfg->gpio_pin_rts, HIGH);
			V_ON(cfg->gpio_pin_rts);
		}
	} 

	uhci_dev->dma_out_link.start			= 1;		//rozpoczecie wysylania
	uart_dma->tx.uruchomiono_nadawanie_odbior	= true;

	if (cfg->gpio_pin_rts>=0)	//tylko dla trybu RS485 jest oczekiwanie na zakonczenie nadawania w tym miejscu, bo trzeba wylaczyc sterowanie kierunkiem
	{
		uart_dma_czekaj_na_zakonczenie_nadawania(uart_dma);

		if (cfg->inwersja_dir)
		{
			//digitalWrite(cfg->gpio_pin_rts, HIGH);
			V_ON(cfg->gpio_pin_rts);
		}
		else
		{
			//digitalWrite(cfg->gpio_pin_rts, LOW);
			V_OFF(cfg->gpio_pin_rts);
		}

	}
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void uart_dma_uruchom_odbior(uart_dma_t *uart_dma)
{
	if (!uart_dma)	return;

	uart_dma_buf_cfg_t	*cfg		=	&uart_dma->cfg;
	uhci_dev_t		*uhci_dev	=	cfg->uhci_dev;
	uart_dev_t		*uart_dev	=	uart_dma->uart_dev;
	//lldesc_t		*deskryptor	=	uart_dma->rx.deskryptor;

	uhci_dev->dma_in_link.stop			= 1;
	
	/*
	uhci_dev->conf0.in_rst				= 1;
	uhci_dev->conf0.in_rst				= 0;
	uhci_dev->conf0.ahbm_fifo_rst			= 1;
	uhci_dev->conf0.ahbm_fifo_rst			= 0;
	uhci_dev->conf0.ahbm_rst			= 1;
	uhci_dev->conf0.ahbm_rst			= 0;
	*/

	 
	//uart_dev->conf1.rx_tout_en			= 1;
	uhci_dev->dma_in_link.addr			= 0;
	//uart_dev->conf0.rxfifo_rst			= 1;		//resetowanie FIFO, jezeli w tym miejscu resetowac fifo to pierwsza ramka jest odbierana poprawnie ale kolejne sa przeklamane dane (chcociaz ilosc sie zgadza), ale tylko wtedy gdy sa uzyte dwa EHCI (czyli dwa UARTy)
	//uart_dev->conf0.rxfifo_rst			= 0;		//resetowanie FIFO, jezeli w tym miejscu resetowac fifo to pierwsza ramka jest odbierana poprawnie ale kolejne sa przeklamane dane (chcociaz ilosc sie zgadza), ale tylko wtedy gdy sa uzyte dwa EHCI (czyli dwa UARTy)
	uart_dev->conf1.rx_tout_en			= 0;
	uart_dev->conf1.rx_tout_thrhd			= 0;
	uart_dev->idle_conf.rx_idle_thrhd		= 0;

	uhci_dev->pkt_thres.thrs			= 0;

	uhci_dev->hung_conf.rxfifo_timeout		= 0;
	uhci_dev->hung_conf.rxfifo_timeout_ena		= 0;

	uhci_dev->hung_conf.txfifo_timeout		= 0;
	uhci_dev->hung_conf.txfifo_timeout_ena		= 0;

	uhci_dev->conf0.indscr_burst_en			= 0;

	//uhci_dev->int_clr.val				= 0xFFFFFFFF;	//TODO: kasowac tylko potrzebne, bo mozna zepsuc nadawanie
	uhci_dev->int_ena.val				= 0;
	uhci_dev->dma_in_link.start			= 1;
	//uhci_dev->dma_in_link.restart			= 1;

	uart_dma->odebrano_bajtow			= 0;
	uart_dma->rozpoczeto_odbieranie_ramki		= 0;
	uart_dma->uruchomiono_odbior			= 1;
	uart_dma->odebrano_ramke			= 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void uart_dma_zakonczenie_odbioru(uart_dma_t *uart_dma)
{
	uart_dma->odebrano_ramke = 1;
	//mqtt_debug("odebrano: %d %d %d !", uart_dma->odebrano_bajtow, uart_dma->bufor_rx[0], uart_dma->bufor_rx[uart_dma->odebrano_bajtow-1]);
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
bool uart_dma_czy_odebrano_ramke(uart_dma_t *uart_dma)
{
	return uart_dma->odebrano_ramke;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
bool uart_dma_obsluga(uart_dma_t *uart_dma)
{
	bool			trwa_odbior=false;
	uhci_dev_t		*uhci_dev	=	uart_dma->cfg.uhci_dev;
	uart_dma_buf_cfg_t	*cfg		=	&uart_dma->cfg;

	if (uart_dma->odebrano_ramke)
	{
		return false;
	}

	if (uart_dma->rozpoczeto_odbieranie_ramki)
	{
		if (uhci_dev->dma_in_status.empty)
		{
			if (czy_minelo_ms(uart_dma->czas, uart_dma->timeout_ms))
			{
				uart_dma_zakonczenie_odbioru(uart_dma);
				trwa_odbior=false;
			}
		}
		else
		{
			int i;
			for (i=0;i<100;i++)	//odbieram maks 100 znakow
			{
				uart_dma->czas = millis();
				if (uart_dma->odebrano_bajtow < cfg->rozmiar_kazdego_bufora_tx_i_rx)
				{
					//uint8_t z=uhci_dev->dma_in_pop.fifo_rdata;
					//konsola_debug("Odebrano ASCII [dec]: "+String(z)+"\n");
					uart_dma->bufor_rx[uart_dma->odebrano_bajtow++] = uhci_dev->dma_in_pop.fifo_rdata;
					uhci_dev->dma_in_pop.fifo_pop = 1;
					//konsola_debug("S: " + String(uhci_dev->dma_in_status.val)+"\n");
					//konsola_debug("I: " + String(uhci_dev->int_st.val)+"\n");
					trwa_odbior=true;
				}
				else
				{
					uart_dma_zakonczenie_odbioru(uart_dma);
					trwa_odbior=false;
					break;
				}
				if (uhci_dev->dma_in_status.empty)
				{
					break;
				}
			}
		}
	}
	else
	{
		if (!uhci_dev->dma_in_status.empty)
		{
			uart_dma->rozpoczeto_odbieranie_ramki = 1;
		}
	}
	return trwa_odbior;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
/*uint16_t uart_dma_ile_odebrano(uart_dma_t *uart_dma)
{
	if (!uart_dma)	return 0;

	//lldesc_t		*deskryptor	=	uart_dma->rx.deskryptor;
	uhci_dev_t		*uhci_dev	=	uart_dma->cfg.uhci_dev;
	uart_dev_t		*uart_dev	=	uart_dma->uart_dev;

	mqtt_debug("uhci_dev->int_raw: 0x%X", uhci_dev->int_raw.val);
	mqtt_debug("uhci_dev->int_st: 0x%X", uhci_dev->int_st.val);
	mqtt_debug("uhci_dev->rx_head: 0x%X", uhci_dev->rx_head);
	mqtt_debug("uhci_dev->ack_num: 0x%X", uhci_dev->ack_num);

	mqtt_debug("uhci_dev->dma_in_dscr: 0x%X", uhci_dev->dma_in_dscr);
	mqtt_debug("uhci_dev->dma_in_dscr_bf0: 0x%X", uhci_dev->dma_in_dscr_bf0);
	mqtt_debug("uhci_dev->dma_in_dscr_bf1: 0x%X", uhci_dev->dma_in_dscr_bf1);

	mqtt_debug("uhci_dev->dma_in_suc_eof_des_addr: 0x%X", uhci_dev->dma_in_suc_eof_des_addr);
	mqtt_debug("uhci_dev->dma_in_dscr_bf1: 0x%X", uhci_dev->dma_in_dscr_bf1);

 

	mqtt_debug("uart_dev->int_raw: 0x%X", uart_dev->int_raw.val);
	mqtt_debug("uart_dev->int_st: 0x%X", uart_dev->int_st.val);

	mqtt_debug("uart_dev->status: 0x%X", uart_dev->status.val);
	mqtt_debug("uart_dev->mem_cnt_status.rx_cnt: %d", uart_dev->mem_cnt_status.rx_cnt);
	mqtt_debug("uart_dev->status.rxfifo_cnt: %d", uart_dev->status.rxfifo_cnt);
	mqtt_debug("uart_dev->status.txfifo_cnt: %d", uart_dev->status.txfifo_cnt);
	mqtt_debug("liczba: %d", (uart_dev->mem_cnt_status.rx_cnt<<8) | uart_dev->status.rxfifo_cnt);

	mqtt_debug("uart_dev->mem_rx_status.wr_addr: %d", uart_dev->mem_rx_status.wr_addr);
	mqtt_debug("uart_dev->mem_rx_status.rd_addr: %d", uart_dev->mem_rx_status.rd_addr);
	mqtt_debug("uhci_dev->dma_in_status.val: %d", uhci_dev->dma_in_status.val);
	mqtt_debug("uhci_dev->dma_in_status.empty: %d", uhci_dev->dma_in_status.empty);
	mqtt_debug("uhci_dev->dma_in_pop.val: %d", uhci_dev->dma_in_pop.val);
	mqtt_debug("uhci_dev->dma_in_link.val: 0x%X", uhci_dev->dma_in_link.val);
	
	//mqtt_debug("deskryptor->size: %d", deskryptor->size);
	int i;
	uint32_t *p;

	p = (uint32_t*)deskryptor;
	for (i=0;i<sizeof(lldesc_t)/4;i++)
	{
		mqtt_debug("i=%d: 0x%X", i, *p++);
	}

	p = (uint32_t*)uhci_dev;
	for (i=0;i<sizeof(uhci_dev_t)/4;i++)
	{
		mqtt_debug("i=%d: 0x%X", i, *p++);
	}

	p = (uint32_t*)uart_dev;
	for (i=0;i<sizeof(uart_dev_t)/4;i++)
	{
		mqtt_debug("i=%d: 0x%X", i, *p++);
	}

	//return deskryptor->length;
	return (uart_dev->mem_cnt_status.rx_cnt<<8) | uart_dev->status.rxfifo_cnt;
}*/
//--------------------------------------------------------------------------------------------------------------------------------------------------
int uart_dma_czekaj_na_zakonczenie_nadawania(uart_dma_t *uart_dma)	//zwraca 1 jezeli byl timeout
{
	if (!uart_dma)	return 0;
	if (!uart_dma->tx.uruchomiono_nadawanie_odbior) return 0;

	uhci_dev_t		*uhci_dev	=	uart_dma->cfg.uhci_dev;
	uart_dev_t		*uart_dev	=	uart_dma->uart_dev;

	uint32_t czas_start_ms = millis();
	while (!czy_minelo_ms(czas_start_ms, 5000))
	{
		//if (uhci_dev->int_raw.tx_brk_idle_done) return;

		if (uhci_dev->int_raw.out_total_eof && /*(uart_dev->status.txfifo_cnt==0) &&*/ (uart_dev->int_st./*txfifo_empty*/tx_done))	//samo out_total_eof nie uwzglednia glebokosci FIFO w UART, dlatego trzba sprawdzac czy nadajnik wszystko wyslal za pomoca tx_done ewntualnie txfifo_cnt
		{
			//mqtt_debug("uhci_dev->int_raw: 0x%X", uhci_dev->int_raw.val);
			//mqtt_debug("uhci_dev->int_st: 0x%X", uhci_dev->int_st.val);
			return 0;
		}
	}

	//mqtt_debug("Timeout uhci_dev->int_raw: 0x%X", uhci_dev->int_raw.val);
	//mqtt_debug("Timeout uhci_dev->int_st: 0x%X", uhci_dev->int_st.val);
	uart_dma->tx.uruchomiono_nadawanie_odbior = false;
	return 1;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------


/* 
//przyklad uarta z DMA 
https://www.esp32.com/viewtopic.php?f=13&t=3240&start=10 
 
lldesc_t widget_usb_Msg;

uint8_t dma_test_msg[8] __attribute__((aligned(32))) = {
		49,50,51,52,53,54,55,56
};
 
typedef struct lldesc_s {
    volatile uint32_t size  :12,
                        length:12,
                        offset: 5, // h/w reserved 5bit, s/w use it as offset in buffer 
                        sosf  : 1, // start of sub-frame 
                        eof   : 1, // end of frame 
                        owner : 1; // hw or sw 
    volatile uint8_t *buf;       // point to buffer data 
    union{
        volatile uint32_t empty;
        STAILQ_ENTRY(lldesc_s) qe;  // pointing to the next desc 
    };
} lldesc_t;  
 
 
void init_dmaDesc() {
	periph_module_enable(PERIPH_UHCI0_MODULE);
	widget_usb_Msg.length = 7; // number valid bytes
	widget_usb_Msg.size = 8; // word aligned buffer size
	widget_usb_Msg.owner = 1; // DMA can operate buffer
	widget_usb_Msg.sosf = 0; // ??
	widget_usb_Msg.buf = dma_test_msg; // location of buffer
	widget_usb_Msg.eof = 1; // last DMA desc
	UHCI0.conf0.val = 0;
	UHCI0.conf1.check_seq_en = 0;
	UHCI0.conf1.check_sum_en = 0;
	UHCI0.conf1.tx_ack_num_re = 0;
	UHCI0.conf1.tx_check_sum_re = 0;
	UHCI0.conf0.uart0_ce = 1;
	UHCI0.dma_out_link.start = 1;
	UHCI0.int_clr.val = 0xFFFFFFFF;
}

void send_USB_DMA () {

    UHCI0.dma_out_link.addr = (uint32_t)(&widget_usb_Msg);
	UHCI0.dma_out_link.start = 1;
    UHCI0.int_clr.val = 0xFFFFFFFF; 
 
*/
//--------------------------------------------------------------------------------------------------------------------------------------------------
/*
typedef volatile struct {
    union {
        struct {
            uint8_t rw_byte;                 //This register stores one byte data  read by rx fifo.
            uint8_t reserved[3];
        };
        uint32_t val;
    } fifo;
    union {
        struct {
 0           uint32_t rxfifo_full:      1;           //This interrupt raw bit turns to high level when receiver receives more data than (rx_flow_thrhd_h3 rx_flow_thrhd).
 1           uint32_t txfifo_empty:     1;           //This interrupt raw bit turns to high level when the amount of data in transmitter's fifo is less than ((tx_mem_cnttxfifo_cnt) .
 2           uint32_t parity_err:       1;           //This interrupt raw bit turns to high level when receiver detects the parity error of data.
 3           uint32_t frm_err:          1;           //This interrupt raw bit turns to high level when receiver detects data's frame error .
 4           uint32_t rxfifo_ovf:       1;           //This interrupt raw bit turns to high level when receiver receives more data than the fifo can store.
 5           uint32_t dsr_chg:          1;           //This interrupt raw bit turns to high level when receiver detects the edge change of dsrn signal.
 6           uint32_t cts_chg:          1;           //This interrupt raw bit turns to high level when receiver detects the edge change of ctsn signal.
  7          uint32_t brk_det:          1;           //This interrupt raw bit turns to high level when receiver detects the 0 after the stop bit.
  8          uint32_t rxfifo_tout:      1;           //This interrupt raw bit turns to high level when receiver takes more time than rx_tout_thrhd to receive a byte.
  9          uint32_t sw_xon:           1;           //This interrupt raw bit turns to high level when receiver receives xoff char with uart_sw_flow_con_en is set to 1.
  10          uint32_t sw_xoff:          1;           //This interrupt raw bit turns to high level when receiver receives xon char with uart_sw_flow_con_en is set to 1.
  11          uint32_t glitch_det:       1;           //This interrupt raw bit turns to high level when receiver detects the start bit.
  12          uint32_t tx_brk_done:      1;           //This interrupt raw bit turns to high level when transmitter completes  sending  0 after all the data in transmitter's fifo are send.
  13          uint32_t tx_brk_idle_done: 1;           //This interrupt raw bit turns to high level when transmitter has kept the shortest duration after the  last data has been send.
   14         uint32_t tx_done:          1;           //This interrupt raw bit turns to high level when transmitter has send all the data in fifo.
   15         uint32_t rs485_parity_err: 1;           //This interrupt raw bit turns to high level when rs485 detects the parity error.
   16         uint32_t rs485_frm_err:    1;           //This interrupt raw bit turns to high level when rs485 detects the data frame error.
   17         uint32_t rs485_clash:      1;           //This interrupt raw bit turns to high level when rs485 detects the clash between transmitter and receiver.
   18         uint32_t at_cmd_char_det:  1;           //This interrupt raw bit turns to high level when receiver detects the configured at_cmd chars.
   19         uint32_t reserved19:      13;
        };
        uint32_t val;
    } int_raw;
    union {
        struct {
            uint32_t rxfifo_full:      1;            //This is the status bit for rxfifo_full_int_raw when rxfifo_full_int_ena is set to 1.
            uint32_t txfifo_empty:     1;            //This is the status bit for  txfifo_empty_int_raw  when txfifo_empty_int_ena is set to 1.
            uint32_t parity_err:       1;            //This is the status bit for parity_err_int_raw when parity_err_int_ena is set to 1.
            uint32_t frm_err:          1;            //This is the status bit for frm_err_int_raw when fm_err_int_ena is set to 1.
            uint32_t rxfifo_ovf:       1;            //This is the status bit for rxfifo_ovf_int_raw when rxfifo_ovf_int_ena is set to 1.
            uint32_t dsr_chg:          1;            //This is the status bit for dsr_chg_int_raw when dsr_chg_int_ena is set to 1.
            uint32_t cts_chg:          1;            //This is the status bit for cts_chg_int_raw when cts_chg_int_ena is set to 1.
            uint32_t brk_det:          1;            //This is the status bit for brk_det_int_raw when brk_det_int_ena is set to 1.
            uint32_t rxfifo_tout:      1;            //This is the status bit for rxfifo_tout_int_raw when rxfifo_tout_int_ena is set to 1.
            uint32_t sw_xon:           1;            //This is the status bit for sw_xon_int_raw when sw_xon_int_ena is set to 1.
            uint32_t sw_xoff:          1;            //This is the status bit for sw_xoff_int_raw when sw_xoff_int_ena is set to 1.
            uint32_t glitch_det:       1;            //This is the status bit for glitch_det_int_raw when glitch_det_int_ena is set to 1.
            uint32_t tx_brk_done:      1;            //This is the status bit for tx_brk_done_int_raw when tx_brk_done_int_ena is set to 1.
            uint32_t tx_brk_idle_done: 1;            //This is the status bit for tx_brk_idle_done_int_raw when tx_brk_idle_done_int_ena is set to 1.
            uint32_t tx_done:          1;            //This is the status bit for tx_done_int_raw when tx_done_int_ena is set to 1.
            uint32_t rs485_parity_err: 1;            //This is the status bit for rs485_parity_err_int_raw when rs485_parity_int_ena is set to 1.
            uint32_t rs485_frm_err:    1;            //This is the status bit for rs485_fm_err_int_raw when rs485_fm_err_int_ena is set to 1.
            uint32_t rs485_clash:      1;            //This is the status bit for rs485_clash_int_raw when rs485_clash_int_ena is set to 1.
            uint32_t at_cmd_char_det:  1;            //This is the status bit for at_cmd_det_int_raw when at_cmd_char_det_int_ena is set to 1.
            uint32_t reserved19:      13;
        };
        uint32_t val;
    } int_st;
    union {
        struct {
            uint32_t rxfifo_full:      1;           //This is the enable bit for rxfifo_full_int_st register.
            uint32_t txfifo_empty:     1;           //This is the enable bit for rxfifo_full_int_st register.
            uint32_t parity_err:       1;           //This is the enable bit for parity_err_int_st register.
            uint32_t frm_err:          1;           //This is the enable bit for frm_err_int_st register.
            uint32_t rxfifo_ovf:       1;           //This is the enable bit for rxfifo_ovf_int_st register.
            uint32_t dsr_chg:          1;           //This is the enable bit for dsr_chg_int_st register.
            uint32_t cts_chg:          1;           //This is the enable bit for cts_chg_int_st register.
            uint32_t brk_det:          1;           //This is the enable bit for brk_det_int_st register.
            uint32_t rxfifo_tout:      1;           //This is the enable bit for rxfifo_tout_int_st register.
            uint32_t sw_xon:           1;           //This is the enable bit for sw_xon_int_st register.
            uint32_t sw_xoff:          1;           //This is the enable bit for sw_xoff_int_st register.
            uint32_t glitch_det:       1;           //This is the enable bit for glitch_det_int_st register.
            uint32_t tx_brk_done:      1;           //This is the enable bit for tx_brk_done_int_st register.
            uint32_t tx_brk_idle_done: 1;           //This is the enable bit for tx_brk_idle_done_int_st register.
            uint32_t tx_done:          1;           //This is the enable bit for tx_done_int_st register.
            uint32_t rs485_parity_err: 1;           //This is the enable bit for rs485_parity_err_int_st register.
            uint32_t rs485_frm_err:    1;           //This is the enable bit for rs485_parity_err_int_st register.
            uint32_t rs485_clash:      1;           //This is the enable bit for rs485_clash_int_st register.
            uint32_t at_cmd_char_det:  1;           //This is the enable bit for at_cmd_char_det_int_st register.
            uint32_t reserved19:      13;
        };
        uint32_t val;
    } int_ena;
    union {
        struct {
            uint32_t rxfifo_full:      1;           //Set this bit to clear the rxfifo_full_int_raw interrupt.
            uint32_t txfifo_empty:     1;           //Set this bit to clear txfifo_empty_int_raw interrupt.
            uint32_t parity_err:       1;           //Set this bit to clear parity_err_int_raw interrupt.
            uint32_t frm_err:          1;           //Set this bit to clear frm_err_int_raw interrupt.
            uint32_t rxfifo_ovf:       1;           //Set this bit to clear rxfifo_ovf_int_raw interrupt.
            uint32_t dsr_chg:          1;           //Set this bit to clear the dsr_chg_int_raw interrupt.
            uint32_t cts_chg:          1;           //Set this bit to clear the cts_chg_int_raw interrupt.
            uint32_t brk_det:          1;           //Set this bit to clear the brk_det_int_raw interrupt.
            uint32_t rxfifo_tout:      1;           //Set this bit to clear the rxfifo_tout_int_raw interrupt.
            uint32_t sw_xon:           1;           //Set this bit to clear the sw_xon_int_raw interrupt.
            uint32_t sw_xoff:          1;           //Set this bit to clear the sw_xon_int_raw interrupt.
            uint32_t glitch_det:       1;           //Set this bit to clear the glitch_det_int_raw interrupt.
            uint32_t tx_brk_done:      1;           //Set this bit to clear the tx_brk_done_int_raw interrupt..
            uint32_t tx_brk_idle_done: 1;           //Set this bit to clear the tx_brk_idle_done_int_raw interrupt.
            uint32_t tx_done:          1;           //Set this bit to clear the tx_done_int_raw interrupt.
            uint32_t rs485_parity_err: 1;           //Set this bit to clear the rs485_parity_err_int_raw interrupt.
            uint32_t rs485_frm_err:    1;           //Set this bit to clear the rs485_frm_err_int_raw interrupt.
            uint32_t rs485_clash:      1;           //Set this bit to clear the rs485_clash_int_raw interrupt.
            uint32_t at_cmd_char_det:  1;           //Set this bit to clear the at_cmd_char_det_int_raw interrupt.
            uint32_t reserved19:      13;
        };
        uint32_t val;
    } int_clr;
    union {
        struct {
            uint32_t div_int:    20;                //The register value is  the  integer part of the frequency divider's factor.
            uint32_t div_frag:    4;                //The register  value is the decimal part of the frequency divider's factor.
            uint32_t reserved24:  8;
        };
        uint32_t val;
    } clk_div;
    union {
        struct {
            uint32_t en: 1;                         //This is the enable bit for detecting baudrate.
            uint32_t reserved1:   7;
            uint32_t glitch_filt: 8;                //when input pulse width is lower then this value ignore this pulse.this register is used in auto-baud detect process.
            uint32_t reserved16: 16;
        };
        uint32_t val;
    } auto_baud;
    union {
        struct {
            uint32_t rxfifo_cnt: 8;                 //(rx_mem_cnt rxfifo_cnt) stores the byte number of valid data in receiver's fifo. rx_mem_cnt register stores the 3 most significant bits  rxfifo_cnt stores the 8 least significant bits.
            uint32_t st_urx_out: 4;                 //This register stores the value of receiver's finite state machine. 0:RX_IDLE  1:RX_STRT  2:RX_DAT0  3:RX_DAT1  4:RX_DAT2  5:RX_DAT3  6:RX_DAT4  7:RX_DAT5  8:RX_DAT6  9:RX_DAT7   10:RX_PRTY   11:RX_STP1  12:RX_STP2 13:RX_DL1
            uint32_t reserved12: 1;
            uint32_t dsrn:       1;                 //This register stores the level value of the internal uart dsr signal.
            uint32_t ctsn:       1;                 //This register stores the level value of the internal uart cts signal.
            uint32_t rxd:        1;                 //This register stores the level value of the internal uart rxd signal.
            uint32_t txfifo_cnt: 8;                 //(tx_mem_cnt txfifo_cnt) stores the byte number of valid data in transmitter's fifo.tx_mem_cnt stores the 3 most significant bits  txfifo_cnt stores the 8 least significant bits.
            uint32_t st_utx_out: 4;                 //This register stores the value of transmitter's finite state machine. 0:TX_IDLE  1:TX_STRT  2:TX_DAT0  3:TX_DAT1  4:TX_DAT2   5:TX_DAT3 6:TX_DAT4  7:TX_DAT5  8:TX_DAT6 9:TX_DAT7  10:TX_PRTY   11:TX_STP1  12:TX_STP2  13:TX_DL0   14:TX_DL1
            uint32_t reserved28: 1;
            uint32_t dtrn:       1;                 //The register represent the level value of the internal uart dsr signal.
            uint32_t rtsn:       1;                 //This register represent the level value of the internal uart cts signal.
            uint32_t txd:        1;                 //This register represent the  level value of the internal uart rxd signal.
        };
        uint32_t val;
    } status;
    union {
        struct {
            uint32_t parity:             1;         //This register is used to configure the parity check mode.  0:even 1:odd
            uint32_t parity_en:          1;         //Set this bit to enable uart parity check.
            uint32_t bit_num:            2;         //This register is used to set the length of data:  0:5bits 1:6bits 2:7bits 3:8bits
            uint32_t stop_bit_num:       2;         //This register is used to set the length of  stop bit. 1:1bit  2:1.5bits  3:2bits
            uint32_t sw_rts:             1;         //This register is used to configure the software rts signal which is used in software flow control.
            uint32_t sw_dtr:             1;         //This register is used to configure the software dtr signal which is used in software flow control..
            uint32_t txd_brk:            1;         //Set this bit to enable transmitter to  send 0 when the process of sending data is done.
            uint32_t irda_dplx:          1;         //Set this bit to enable irda loop-back mode.
            uint32_t irda_tx_en:         1;         //This is the start enable bit for irda transmitter.
            uint32_t irda_wctl:          1;         //1?the irda transmitter's 11th bit is the same to the 10th bit. 0?set irda transmitter's 11th bit to 0.
            uint32_t irda_tx_inv:        1;         //Set this bit to inverse the level value of  irda transmitter's level.
            uint32_t irda_rx_inv:        1;         //Set this bit to inverse the level value of irda receiver's level.
            uint32_t loopback:           1;         //Set this bit to enable uart loop-back test mode.
            uint32_t tx_flow_en:         1;         //Set this bit to enable transmitter's flow control function.
            uint32_t irda_en:            1;         //Set this bit to enable irda protocol.
            uint32_t rxfifo_rst:         1;         //Set this bit to reset uart receiver's fifo.
            uint32_t txfifo_rst:         1;         //Set this bit to reset uart transmitter's fifo.
            uint32_t rxd_inv:            1;         //Set this bit to inverse the level value of uart rxd signal.
            uint32_t cts_inv:            1;         //Set this bit to inverse the level value of uart cts signal.
            uint32_t dsr_inv:            1;         //Set this bit to inverse the level value of uart dsr signal.
            uint32_t txd_inv:            1;         //Set this bit to inverse the level value of uart txd signal.
            uint32_t rts_inv:            1;         //Set this bit to inverse the level value of uart rts signal.
            uint32_t dtr_inv:            1;         //Set this bit to inverse the level value of uart dtr signal.
            uint32_t clk_en:             1;         //1?force clock on for registers?support clock only when write registers
            uint32_t err_wr_mask:        1;         //1?receiver stops storing data int fifo when data is wrong. 0?receiver stores the data even if the  received data is wrong.
            uint32_t tick_ref_always_on: 1;         //This register is used to select the clock.1?apb clock?ref_tick
            uint32_t reserved28:         4;
        };
        uint32_t val;
    } conf0;
    union {
        struct {
            uint32_t rxfifo_full_thrhd:  7;         //When receiver receives more data than its threshold value?receiver will produce rxfifo_full_int_raw interrupt.the threshold value is (rx_flow_thrhd_h3 rxfifo_full_thrhd).
            uint32_t reserved7:          1;
            uint32_t txfifo_empty_thrhd: 7;         //when the data amount in transmitter fifo is less than its threshold value? it will produce txfifo_empty_int_raw interrupt. the threshold value is (tx_mem_empty_thrhd txfifo_empty_thrhd)
            uint32_t reserved15:         1;
            uint32_t rx_flow_thrhd:      7;         //when receiver receives more data than its threshold value? receiver produce signal to tell the transmitter stop transferring data. the threshold value is (rx_flow_thrhd_h3 rx_flow_thrhd).
            uint32_t rx_flow_en:         1;         //This is the flow enable bit for uart receiver. 1:choose software flow control with configuring sw_rts signal
            uint32_t rx_tout_thrhd:      7;         //This register is used to configure the timeout value for uart receiver receiving a byte.
            uint32_t rx_tout_en:         1;         //This is the enable bit for uart receiver's timeout function.
        };
        uint32_t val;
    } conf1;
    union {
        struct {
            uint32_t min_cnt:     20;               //This register stores the value of the minimum duration time for the low level pulse? it is used in baudrate-detect process.
            uint32_t reserved20:  12;
        };
        uint32_t val;
    } lowpulse;
    union {
        struct {
            uint32_t min_cnt:     20;               //This register stores  the value of the maximum duration time for the high level pulse? it is used in baudrate-detect process.
            uint32_t reserved20:  12;
        };
        uint32_t val;
    } highpulse;
    union {
        struct {
            uint32_t edge_cnt:    10;               //This register stores the count of rxd edge change? it is used in baudrate-detect process.
            uint32_t reserved10:  22;
        };
        uint32_t val;
    } rxd_cnt;
    union {
        struct {
            uint32_t sw_flow_con_en: 1;             //Set this bit to enable software  flow control. it is used with register sw_xon or sw_xoff .
            uint32_t xonoff_del:     1;             //Set this bit to remove flow control char from the received data.
            uint32_t force_xon:      1;             //Set this bit to clear ctsn to stop the  transmitter from sending data.
            uint32_t force_xoff:     1;             //Set this bit to set ctsn to enable the transmitter to go on sending data.
            uint32_t send_xon:       1;             //Set this bit to send xon char? it is cleared by hardware automatically.
            uint32_t send_xoff:      1;             //Set this bit to send xoff char? it is cleared by hardware automatically.
            uint32_t reserved6:     26;
        };
        uint32_t val;
    } flow_conf;
    union {
        struct {
            uint32_t active_threshold:10;           //When the input rxd edge changes more than this register value? the uart is active from light sleeping mode.
            uint32_t reserved10:      22;
        };
        uint32_t val;
    } sleep_conf;
    union {
        struct {
            uint32_t xon_threshold:  8;             //when the data amount in receiver's fifo is more than this register value? it will send a xoff char with uart_sw_flow_con_en set to 1.
            uint32_t xoff_threshold: 8;             //When the data amount in receiver's fifo is less than this register value? it will send a xon char with uart_sw_flow_con_en set to 1.
            uint32_t xon_char:       8;             //This register stores the xon flow control char.
            uint32_t xoff_char:      8;             //This register stores the xoff flow control char.
        };
        uint32_t val;
    } swfc_conf;
    union {
        struct {
            uint32_t rx_idle_thrhd:10;              //when receiver takes more time than this register value to receive a byte data? it will produce frame end signal for uhci to stop receiving data.
            uint32_t tx_idle_num:  10;              //This register is used to configure the duration time between transfers.
            uint32_t tx_brk_num:    8;              //This register is used to configure the number of 0 send after the process of sending data is done. it is active when txd_brk is set to 1.
            uint32_t reserved28:    4;
        };
        uint32_t val;
    } idle_conf;
    union {
        struct {
            uint32_t en:               1;           //Set this bit to choose rs485 mode.
            uint32_t dl0_en:           1;           //Set this bit to delay the stop bit by 1 bit.
            uint32_t dl1_en:           1;           //Set this bit to delay the stop bit by 1 bit.
            uint32_t tx_rx_en:         1;           //Set this bit to enable loop-back transmitter's output data signal to receiver's input data signal.
            uint32_t rx_busy_tx_en:    1;           //1: enable rs485's transmitter to send data when rs485's receiver is busy. 0:rs485's transmitter should not send data when its receiver is busy.
            uint32_t rx_dly_num:       1;           //This register is used to delay the receiver's internal data signal.
            uint32_t tx_dly_num:       4;           //This register is used to delay the transmitter's internal data signal.
            uint32_t reserved10:      22;
        };
        uint32_t val;
    } rs485_conf;
    union {
        struct {
            uint32_t pre_idle_num:24;               //This register is used to configure the idle duration time before the first at_cmd is received by receiver? when the the duration is less than this register value it will not take the next data received as at_cmd char.
            uint32_t reserved24:   8;
        };
        uint32_t val;
    } at_cmd_precnt;
    union {
        struct {
            uint32_t post_idle_num:24;              //This register is used to configure the duration time between the last at_cmd and the next data? when the duration is less than this register value  it will not take the previous data as at_cmd char.
            uint32_t reserved24:    8;
        };
        uint32_t val;
    } at_cmd_postcnt;
    union {
        struct {
            uint32_t rx_gap_tout:24;                //This register is used to configure the duration time between the at_cmd chars? when the duration time is less than this register value it will not take the data as continous at_cmd chars.
            uint32_t reserved24:  8;
        };
        uint32_t val;
    } at_cmd_gaptout;
    union {
        struct {
            uint32_t data:        8;                //This register is used to configure the content of at_cmd char.
            uint32_t char_num:    8;                //This register is used to configure the number of continuous at_cmd chars received by receiver.
            uint32_t reserved16: 16;
        };
        uint32_t val;
    } at_cmd_char;
    union {
        struct {
            uint32_t mem_pd:             1;         //Set this bit to power down memory?when reg_mem_pd registers in the 3 uarts are all set to 1  memory will enter low power mode.
            uint32_t reserved1:          1;
            uint32_t reserved2:          1;
            uint32_t rx_size:            4;         //This register is used to configure the amount of mem allocated to receiver's fifo. the default byte num is 128.
            uint32_t tx_size:            4;         //This register is used to configure the amount of mem allocated to transmitter's fifo.the default byte num is 128.
            uint32_t reserved11:         4;
            uint32_t rx_flow_thrhd_h3:   3;         //refer to the rx_flow_thrhd's description.
            uint32_t rx_tout_thrhd_h3:   3;         //refer to the rx_tout_thrhd's description.
            uint32_t xon_threshold_h2:   2;         //refer to the uart_xon_threshold's description.
            uint32_t xoff_threshold_h2:  2;         //refer to the uart_xoff_threshold's description.
            uint32_t rx_mem_full_thrhd:  3;         //refer to the rxfifo_full_thrhd's description.
            uint32_t tx_mem_empty_thrhd: 3;         //refer to txfifo_empty_thrhd 's description.
            uint32_t reserved31:         1;
        };
        uint32_t val;
    } mem_conf;
    union {
        struct {
            uint32_t status:24;
            uint32_t reserved24:    8;
        };
        uint32_t val;
    } mem_tx_status;
    union {
        struct {
            uint32_t status:      24;
            uint32_t reserved24:   8;
        };
        struct {
            uint32_t reserved0:     2;
            uint32_t rd_addr:      11;              //This register stores the rx mem read address.
            uint32_t wr_addr:      11;              //This register stores the rx mem write address.
            uint32_t reserved:      8;
        };
        uint32_t val;
    } mem_rx_status;
    union {
        struct {
            uint32_t rx_cnt: 3;                      //refer to the rxfifo_cnt's description.
            uint32_t tx_cnt: 3;                      //refer to the txfifo_cnt's description.
            uint32_t reserved6: 26;
        };
        uint32_t val;
    } mem_cnt_status;
    union {
        struct {
            uint32_t min_cnt:     20;                 //This register stores the count of rxd pos-edge edge? it is used in baudrate-detect process.
            uint32_t reserved20:  12;
        };
        uint32_t val;
    } pospulse;
    union {
        struct {
            uint32_t min_cnt:     20;                 //This register stores the count of rxd neg-edge edge? it is used in baudrate-detect process.
            uint32_t reserved20:  12;
        };
        uint32_t val;
    } negpulse;
    uint32_t reserved_70;
    uint32_t reserved_74;
    uint32_t date;                                    //
    uint32_t id;                                      //
} uart_dev_t; 
 
*/













/*
typedef volatile struct {
    union {
        struct {
            uint32_t in_rst:             1;                //Set this bit to reset in link operations.
            uint32_t out_rst:            1;                //Set this bit to reset out link operations.
            uint32_t ahbm_fifo_rst:      1;                //Set this bit to reset dma ahb fifo.
            uint32_t ahbm_rst:           1;                //Set this bit to reset dma  ahb interface.
            uint32_t in_loop_test:       1;                //Set this bit to enable loop test for in links.
            uint32_t out_loop_test:      1;                //Set this bit to enable loop test for out links.
            uint32_t out_auto_wrback:    1;                //when in link's length is 0  go on to use the next in link automatically.
            uint32_t out_no_restart_clr: 1;                //don't use
            uint32_t out_eof_mode:       1;                //Set this bit to produce eof after DMA pops all data  clear this bit to produce eof after DMA pushes all data
            uint32_t uart0_ce:           1;                //Set this bit to use UART to transmit or receive data.
            uint32_t uart1_ce:           1;                //Set this bit to use UART1 to transmit or receive data.
            uint32_t uart2_ce:           1;                //Set this bit to use UART2 to transmit or receive data.
            uint32_t outdscr_burst_en:   1;                //Set this bit to enable DMA in links to use burst mode.
            uint32_t indscr_burst_en:    1;                //Set this bit to enable DMA out links to use burst mode.
            uint32_t out_data_burst_en:  1;                //Set this bit to enable DMA burst MODE
            uint32_t mem_trans_en:       1;
            uint32_t seper_en:           1;                //Set this bit to use special char to separate the data frame.
            uint32_t head_en:            1;                //Set this bit to enable to use head packet  before the data frame.
            uint32_t crc_rec_en:         1;                //Set this bit to enable receiver''s ability of crc calculation  when crc_en bit in head packet is 1 then there will be crc bytes after data_frame
            uint32_t uart_idle_eof_en:   1;                //Set this bit to enable to use idle time  when the idle time after data frame is satisfied  this means the end of a data frame.
            uint32_t len_eof_en:         1;                //Set this bit to enable to use packet_len in packet head  when the received data is equal to packet_len  this means the end of a data frame.
            uint32_t encode_crc_en:      1;                //Set this bit to enable crc calculation for data frame when bit6 in the head packet is 1.
            uint32_t clk_en:             1;                //Set this bit to enable clock-gating for read or write registers.
            uint32_t uart_rx_brk_eof_en: 1;                //Set this bit to enable to use brk char as the end of a data frame.
            uint32_t reserved24:         8;
        };
        uint32_t val;
    } conf0;
    union {
        struct {
            uint32_t rx_start:            1;               //when a separator char has been send  it will produce uhci_rx_start_int interrupt.
            uint32_t tx_start:            1;               //when DMA detects a separator char it will produce uhci_tx_start_int interrupt.
            uint32_t rx_hung:             1;               //when DMA takes a lot of time to receive a data   it will produce uhci_rx_hung_int interrupt.
            uint32_t tx_hung:             1;               //when DMA takes a lot of time to read a data from RAM  it will produce uhci_tx_hung_int interrupt.
            uint32_t in_done:             1;               //when a in link descriptor has been completed it will produce uhci_in_done_int interrupt.
            uint32_t in_suc_eof:          1;               //when a data packet has been received  it will produce uhci_in_suc_eof_int interrupt.
            uint32_t in_err_eof:          1;               //when there are some errors about eof in in link descriptor  it will produce uhci_in_err_eof_int interrupt.
            uint32_t out_done:            1;               //when a out link descriptor is completed  it will produce uhci_out_done_int interrupt.
            uint32_t out_eof:             1;               //when the current descriptor's eof bit is 1  it will produce uhci_out_eof_int interrupt.
            uint32_t in_dscr_err:         1;               //when there are some errors about the out link descriptor  it will produce uhci_in_dscr_err_int interrupt.
            uint32_t out_dscr_err:        1;               //when there are some errors about the in link descriptor  it will produce uhci_out_dscr_err_int interrupt.
            uint32_t in_dscr_empty:       1;               //when there are not enough in links for DMA it will produce uhci_in_dscr_err_int interrupt.
            uint32_t outlink_eof_err:     1;               //when there are some errors about eof in outlink descriptor  it will produce uhci_outlink_eof_err_int interrupt.
            uint32_t out_total_eof:       1;               //When all data have been send  it will produce uhci_out_total_eof_int interrupt.
            uint32_t send_s_q:            1;               //When use single send registers to send a short packets it will produce this interrupt when dma has send the short packet.
            uint32_t send_a_q:            1;               //When use always_send registers to send a series of short packets it will produce this interrupt when dma has send the short packet.
            uint32_t dma_in_fifo_full_wm: 1;
            uint32_t reserved17:         15;
        };
        uint32_t val;
    } int_raw;
    union {
        struct {
            uint32_t rx_start:            1;
            uint32_t tx_start:            1;
            uint32_t rx_hung:             1;
            uint32_t tx_hung:             1;
            uint32_t in_done:             1;
            uint32_t in_suc_eof:          1;
            uint32_t in_err_eof:          1;
            uint32_t out_done:            1;
            uint32_t out_eof:             1;
            uint32_t in_dscr_err:         1;
            uint32_t out_dscr_err:        1;
            uint32_t in_dscr_empty:       1;
            uint32_t outlink_eof_err:     1;
            uint32_t out_total_eof:       1;
            uint32_t send_s_q:            1;
            uint32_t send_a_q:            1;
            uint32_t dma_in_fifo_full_wm: 1;
            uint32_t reserved17:         15;
        };
        uint32_t val;
    } int_st;
    union {
        struct {
            uint32_t rx_start:            1;
            uint32_t tx_start:            1;
            uint32_t rx_hung:             1;
            uint32_t tx_hung:             1;
            uint32_t in_done:             1;
            uint32_t in_suc_eof:          1;
            uint32_t in_err_eof:          1;
            uint32_t out_done:            1;
            uint32_t out_eof:             1;
            uint32_t in_dscr_err:         1;
            uint32_t out_dscr_err:        1;
            uint32_t in_dscr_empty:       1;
            uint32_t outlink_eof_err:     1;
            uint32_t out_total_eof:       1;
            uint32_t send_s_q:            1;
            uint32_t send_a_q:            1;
            uint32_t dma_in_fifo_full_wm: 1;
            uint32_t reserved17:         15;
        };
        uint32_t val;
    } int_ena;
    union {
        struct {
            uint32_t rx_start:            1;
            uint32_t tx_start:            1;
            uint32_t rx_hung:             1;
            uint32_t tx_hung:             1;
            uint32_t in_done:             1;
            uint32_t in_suc_eof:          1;
            uint32_t in_err_eof:          1;
            uint32_t out_done:            1;
            uint32_t out_eof:             1;
            uint32_t in_dscr_err:         1;
            uint32_t out_dscr_err:        1;
            uint32_t in_dscr_empty:       1;
            uint32_t outlink_eof_err:     1;
            uint32_t out_total_eof:       1;
            uint32_t send_s_q:            1;
            uint32_t send_a_q:            1;
            uint32_t dma_in_fifo_full_wm: 1;
            uint32_t reserved17:         15;
        };
        uint32_t val;
    } int_clr;
    union {
        struct {
            uint32_t full:       1;                      //1:DMA out link descriptor's fifo is full.
            uint32_t empty:      1;                      //1:DMA in link descriptor's fifo is empty.
            uint32_t reserved2: 30;
        };
        uint32_t val;
    } dma_out_status;
    union {
        struct {
            uint32_t fifo_wdata: 9;                      //This is the data need to be pushed into out link descriptor's fifo.
            uint32_t reserved9:  7;
            uint32_t fifo_push:  1;                      //Set this bit to push data in out link descriptor's fifo.
            uint32_t reserved17:15;
        };
        uint32_t val;
    } dma_out_push;
    union {
        struct {
            uint32_t full:         1;
            uint32_t empty:        1;
            uint32_t reserved2:    2;
            uint32_t rx_err_cause: 3;                    //This register stores the errors caused in out link descriptor's data packet.
            uint32_t reserved7:   25;
        };
        uint32_t val;
    } dma_in_status;
    union {
        struct {
            uint32_t fifo_rdata:  12;                    //This register stores the data pop from in link descriptor's fifo.
            uint32_t reserved12:   4;
            uint32_t fifo_pop:     1;                    //Set this bit to pop data in in link descriptor's fifo.
            uint32_t reserved17:  15;
        };
        uint32_t val;
    } dma_in_pop;
    union {
        struct {
            uint32_t addr:         20;                   //This register stores the least 20 bits of the first out link descriptor's address.
            uint32_t reserved20:    8;
            uint32_t stop:          1;                   //Set this bit to stop dealing with the out link descriptors.
            uint32_t start:         1;                   //Set this bit to start dealing with the out link descriptors.
            uint32_t restart:       1;                   //Set this bit to mount on new out link descriptors
            uint32_t park:          1;                   //1? the out link descriptor's fsm is in idle state. 0:the out link descriptor's fsm is working.
        };
        uint32_t val;
    } dma_out_link;
    union {
        struct {
            uint32_t addr:       20;                     //This register stores the least 20 bits of the first in link descriptor's address.
            uint32_t auto_ret:    1;                     //1:when a packet is wrong in link descriptor returns to the descriptor which is lately used.
            uint32_t reserved21:  7;
            uint32_t stop:        1;                     //Set this bit to stop dealing with the in link descriptors.
            uint32_t start:       1;                     //Set this bit to start dealing with the in link descriptors.
            uint32_t restart:     1;                     //Set this bit to mount on new in link descriptors
            uint32_t park:        1;                     //1:the in link descriptor's fsm is in idle state.   0:the in link descriptor's fsm is working
        };
        uint32_t val;
    } dma_in_link;
    union {
        struct {
            uint32_t check_sum_en:          1;            //Set this bit to enable decoder to check  check_sum in packet header.
            uint32_t check_seq_en:          1;            //Set this bit to enable decoder to check seq num in packet header.
            uint32_t crc_disable:           1;            //Set this bit to disable crc calculation.
            uint32_t save_head:             1;            //Set this bit to save packet header .
            uint32_t tx_check_sum_re:       1;            //Set this bit to enable hardware replace check_sum in packet header automatically.
            uint32_t tx_ack_num_re:         1;            //Set this bit to enable hardware replace ack num in packet header automatically.
            uint32_t check_owner:           1;            //Set this bit to check the owner bit in link descriptor.
            uint32_t wait_sw_start:         1;            //Set this bit to enable software way to add packet header.
            uint32_t sw_start:              1;            //Set this bit to start inserting the packet header.
            uint32_t dma_in_fifo_full_thrs:12;            //when data amount in link descriptor's fifo is more than this register value  it will produce uhci_dma_in_fifo_full_wm_int interrupt.
            uint32_t reserved21:           11;
        };
        uint32_t val;
    } conf1;
    uint32_t state0;                                       //
    uint32_t state1;                                       //
    uint32_t dma_out_eof_des_addr;                         //This register stores the address of out link description when eof bit in this descriptor is 1.
    uint32_t dma_in_suc_eof_des_addr;                      //This register stores the address of in link descriptor when eof bit in this descriptor is 1.
    uint32_t dma_in_err_eof_des_addr;                      //This register stores the address of in link descriptor when there are some errors in this descriptor.
    uint32_t dma_out_eof_bfr_des_addr;                     //This register stores the address of out link descriptor when there are some errors in this descriptor.
    union {
        struct {
            uint32_t test_mode:   3;                       //bit2 is ahb bus test enable ?bit1 is used to choose write(1) or read(0) mode. bit0 is used to choose test only once(1) or continue(0)
            uint32_t reserved3:   1;
            uint32_t test_addr:   2;                       //The two bits represent ahb bus address bit[20:19]
            uint32_t reserved6:  26;
        };
        uint32_t val;
    } ahb_test;
    uint32_t dma_in_dscr;                                  //The content of current in link descriptor's third dword
    uint32_t dma_in_dscr_bf0;                              //The content of current in link descriptor's first dword
    uint32_t dma_in_dscr_bf1;                              //The content of current in link descriptor's second dword
    uint32_t dma_out_dscr;                                 //The content of current out link descriptor's third dword
    uint32_t dma_out_dscr_bf0;                             //The content of current out link descriptor's first dword
    uint32_t dma_out_dscr_bf1;                             //The content of current out link descriptor's second dword
    union {
        struct {
            uint32_t tx_c0_esc_en: 1;                      //Set this bit to enable  0xc0 char decode when DMA receives data.
            uint32_t tx_db_esc_en: 1;                      //Set this bit to enable  0xdb char decode when DMA receives data.
            uint32_t tx_11_esc_en: 1;                      //Set this bit to enable  flow control char 0x11 decode when DMA receives data.
            uint32_t tx_13_esc_en: 1;                      //Set this bit to enable flow control char 0x13 decode when DMA receives data.
            uint32_t rx_c0_esc_en: 1;                      //Set this bit to enable  0xc0 char replace when DMA sends data.
            uint32_t rx_db_esc_en: 1;                      //Set this bit to enable  0xdb char replace when DMA sends data.
            uint32_t rx_11_esc_en: 1;                      //Set this bit to enable  flow control char 0x11 replace when DMA sends data.
            uint32_t rx_13_esc_en: 1;                      //Set this bit to enable  flow control char 0x13 replace when DMA sends data.
            uint32_t reserved8:   24;
        };
        uint32_t val;
    } escape_conf;
    union {
        struct {
            uint32_t txfifo_timeout:       8;              //This register stores the timeout value.when DMA takes more time than this register value to receive a data  it will produce  uhci_tx_hung_int interrupt.
            uint32_t txfifo_timeout_shift: 3;              //The tick count is cleared when its value >=(17'd8000>>reg_txfifo_timeout_shift)
            uint32_t txfifo_timeout_ena:   1;              //The enable bit for tx fifo receive data  timeout
            uint32_t rxfifo_timeout:       8;              //This register stores the timeout value.when DMA takes more time than this register value to read a data from RAM  it will produce  uhci_rx_hung_int interrupt.
            uint32_t rxfifo_timeout_shift: 3;              //The tick count is cleared when its value >=(17'd8000>>reg_rxfifo_timeout_shift)
            uint32_t rxfifo_timeout_ena:   1;              //This is the enable bit for DMA  send data timeout
            uint32_t reserved24:           8;
        };
        uint32_t val;
    } hung_conf;
    uint32_t ack_num;                                      //
    uint32_t rx_head;                                      //This register stores the packet header received by DMA
    union {
        struct {
            uint32_t single_send_num: 3;                   //The bits are used to choose which short packet
            uint32_t single_send_en:  1;                   //Set this bit to enable  send a short packet
            uint32_t always_send_num: 3;                   //The bits are used to choose which short packet
            uint32_t always_send_en:  1;                   //Set this bit to enable continuously send the same short packet
            uint32_t reserved8:      24;
        };
        uint32_t val;
    } quick_sent;
    struct{
        uint32_t w_data[2];                                //This register stores the content of short packet's dword
    } q_data[7];
    union {
        struct {
            uint32_t seper_char:      8;                   //This register stores the separator char  separator char is used to separate the data frame.
            uint32_t seper_esc_char0: 8;                   //This register stores the first char used to replace separator char in data.
            uint32_t seper_esc_char1: 8;                   //This register stores the second char used to replace separator char in data . 0xdc 0xdb replace 0xc0 by default.
            uint32_t reserved24:      8;
        };
        uint32_t val;
    } esc_conf0;
    union {
        struct {
            uint32_t seq0:       8;                        //This register stores the first substitute char used to replace the separate char.
            uint32_t seq0_char0: 8;                        //This register stores the first char used to replace reg_esc_seq0 in data.
            uint32_t seq0_char1: 8;                        //This register stores the second char used to replace the reg_esc_seq0 in data
            uint32_t reserved24: 8;
        };
        uint32_t val;
    } esc_conf1;
    union {
        struct {
            uint32_t seq1:       8;                        //This register stores the flow control char to turn on the flow_control
            uint32_t seq1_char0: 8;                        //This register stores the first char used to replace the reg_esc_seq1 in data.
            uint32_t seq1_char1: 8;                        //This register stores the second char used to replace the reg_esc_seq1 in data.
            uint32_t reserved24: 8;
        };
        uint32_t val;
    } esc_conf2;
    union {
        struct {
            uint32_t seq2:       8;                        //This register stores the flow_control char to turn off the flow_control
            uint32_t seq2_char0: 8;                        //This register stores the first char used to replace the reg_esc_seq2 in data.
            uint32_t seq2_char1: 8;                        //This register stores  the second char used to replace the reg_esc_seq2 in data.
            uint32_t reserved24: 8;
        };
        uint32_t val;
    } esc_conf3;
    union {
        struct {
            uint32_t thrs:      13;                        //when the amount of packet payload is larger than this value the process of receiving data is done.
            uint32_t reserved13:19;
        };
        uint32_t val;
    } pkt_thres;
    uint32_t reserved_c4;
    uint32_t reserved_c8;
    uint32_t reserved_cc;
    uint32_t reserved_d0;
    uint32_t reserved_d4;
    uint32_t reserved_d8;
    uint32_t reserved_dc;
    uint32_t reserved_e0;
    uint32_t reserved_e4;
    uint32_t reserved_e8;
    uint32_t reserved_ec;
    uint32_t reserved_f0;
    uint32_t reserved_f4;
    uint32_t reserved_f8;
    uint32_t date;                                         //version information
} uhci_dev_t; 
 
*/
