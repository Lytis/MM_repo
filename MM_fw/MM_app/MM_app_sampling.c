#include "MM_app_sampling.h"

extern SAI_HandleTypeDef hsai_BlockA1;

int32_t raw_packet[SAMPLE_PACKET_SIZE];

void sampling_init()
{

    while (HAL_OK != HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t*)raw_packet, SAMPLE_PACKET_SIZE));
    
}

void half_transfer_event()
{

    int i;

    for (i=1; i<SAMPLE_PACKET_SIZE; i++)
    {
        
    }
    

}

void full_transfer_event()
{

}
