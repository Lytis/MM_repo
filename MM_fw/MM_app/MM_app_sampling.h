#ifndef MM_APP_SAMPLING_H_
#define MM_APP_SAMPLING_H_

#include "main.h"
#include "stm32f4xx_hal.h"

#include "MM_app_control.h"
#include "MM_app_storage.h"
#include "MM_app_transmit.h"

#define SAMPLES_PER_MIC         32
#define SAI_BUFFER_SIZE         SAMPLES_PER_MIC*8

/*
sample window time = samples/sampliung frequency
 (now it is 8k)
 32/8k = 4msec

 */


void sampling_init(void);
void sampling_pause(void);
void sampling_resume(void);

void half_transfer_event(void);
void full_transfer_event(void);


#endif

