typedef void ld2450_t;

ld2450_t *detector_init(uint8_t uart_nr, int8_t pin_tx, int8_t pin_rx);
uint32_t get_person(ld2450_t *ld2450, uint8_t person_number, int32_t *x, int32_t *y);
