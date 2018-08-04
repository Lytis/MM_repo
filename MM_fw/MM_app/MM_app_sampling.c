#include "MM_app_sampling.h"

extern SAI_HandleTypeDef hsai_BlockA1;

int32_t SAI_buffer[SAI_BUFFER_SIZE];             //8*32 = 256 samples // 32 samples: 32/48 mili seconds

int32_t *p_firstHalf = &SAI_buffer[0];
int32_t *p_secondHalf = &SAI_buffer[SAI_BUFFER_SIZE / 2];


void sampling_init()
{

    while (HAL_OK != HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t*)SAI_buffer, SAI_BUFFER_SIZE));
    
}

void half_transfer_event()
{

    push_samples(p_firstHalf, SAI_BUFFER_SIZE/2);

}

void full_transfer_event()
{

    push_samples(p_secondHalf);
    
}
