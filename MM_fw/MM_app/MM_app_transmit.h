#ifndef MM_APP_TRANSMIT_H_
#define MM_APP_TRANSMIT_H_

#include <stdlib.h>

#include "main.h"
#include "stm32f4xx_hal.h"

#include "MM_app_control.h"
#include "MM_app_sampling.h"
#include "MM_app_storage.h"

#define AUXILIARY_SPI   1
#define TEST_BUFFER     1

//SPI_1 is the main
//SPI_2 is the aux

void transmit_init(void);
void transmit_packet(int16_t *);


#endif

