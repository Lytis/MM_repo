#include "MM_app_sampling.h"

extern SAI_HandleTypeDef hsai_BlockA1;

int32_t raw_packet[PACKET_SIZE+1];

void sampling_init()
{

    while (HAL_OK != HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t*)&raw_packet[1], PACKET_SIZE));
    
}

void packet_recieve_event()
{

    

}

