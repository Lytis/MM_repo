#ifndef MM_APP_SAMPLING_H_
#define MM_APP_SAMPLING_H_

#include "main.h"
#include "stm32f4xx_hal.h"


#define SAMPLES_PER_MIC         32
#define SAMPLE_PACKET_SIZE      SAMPLES_PER_MIC*8



void sampling_init(void);
void packet_recieve_event(void);


#endif

