#include "MM_app_transmit.h"


extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

static SPI_HandleTypeDef spiPort;




void transmit_init(void)
{
    if (AUXILIARY_SPI == 1)
    {
        spiPort = hspi2;
    }else
    {
        spiPort = hspi1;
    }

}

void transmit_packet(int16_t* buffer)
{

    while (HAL_OK != HAL_SPI_Transmit_DMA(&spiPort, (uint8_t)buffer, SAMPLES_PER_MIC * PACKETS_PER_TRANSMITION));

}

