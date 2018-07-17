#ifndef MM_APP_SAMPLING_H_
#define MM_APP_SAMPLING_H_

#include "main.h"
#include "stm32f4xx_hal.h"

#define PACKET_SIZE     480*8


void sampling_init(void);
void packet_recieve_event(void);


#endif

