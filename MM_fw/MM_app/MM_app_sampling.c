#include "MM_app_sampling.h"

extern SAI_HandleTypeDef hsai_BlockA1;

int32_t SAI_buffer[SAI_BUFFER_SIZE];             //8*32 = 256 samples // 32 samples: 32/48 mili seconds

int32_t *p_firstHalf = NULL;
int32_t *p_secondHalf = NULL;


void sampling_init()
{

    while (HAL_OK != HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t*)SAI_buffer, SAI_BUFFER_SIZE));

}

void sampling_pause()
{
    while (HAL_OK != HAL_SAI_DMAPause(&hsai_BlockA1));
}

void sampling_resume()
{
    while (HAL_OK != HAL_SAI_DMAResume(&hsai_BlockA1));
}

void half_transfer_event()
{
    p_firstHalf = &SAI_buffer[0];
    push_samples((int32_t*)p_firstHalf);
    HAL_GPIO_TogglePin(SPI_1_EN_GPIO_Port, SPI_1_EN_Pin);

}

void full_transfer_event()
{
    p_secondHalf = &SAI_buffer[SAI_BUFFER_SIZE/2];
    push_samples((int32_t*)p_secondHalf);
    HAL_GPIO_TogglePin(SPI_1_EN_GPIO_Port, SPI_1_EN_Pin);
    
}


