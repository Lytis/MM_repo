#ifndef MM_APP_TRANSMIT_H_
#define MM_APP_TRANSMIT_H_

#include "main.h"
#include "stm32f4xx_hal.h"


#define AUXILIARY_SPI   1

//SPI_1 is the main
//SPI_2 is the aux

void transmit_init(void);
void transmit_packet(int16_t*);


#endif

