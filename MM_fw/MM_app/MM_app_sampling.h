#ifndef MM_APP_SAMPLING_H_
#define MM_APP_SAMPLING_H_

#include "main.h"
#include "stm32f4xx_hal.h"


#define SAMPLES_PER_MIC         32
#define SAI_BUFFER_SIZE         SAMPLES_PER_MIC*8


void sampling_init(void);

void half_transfer_event(void);
void full_transfer_event(void);


#endif

