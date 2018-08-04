#ifndef MM_APP_STORAGE_H_
#define MM_APP_STORAGE_H_

#include "main.h"
#include "stm32f4xx_hal.h"

#include "MM_app_sampling.h"

#define SUB_PACKETS             8
#define TRANSMIT_PACKET_SIZE    SAI_BUFFER_SIZE*SUB_PACKETS + 1

/* 
every packet has duration of subpacets*packet duration
8*4ms = 32msec
 */

void storage_init(void);
void push_samples(int32_t *);


#endif
