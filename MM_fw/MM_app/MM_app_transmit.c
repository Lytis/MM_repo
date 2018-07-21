#include "MM_app_transmit.h"


extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

static SPI_HandleTypeDef spiPort;

int16_t tranmition_packet[SAMPLES_PER_MIC * PACKETS_PER_TRANSMITION]



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

void transmit_packet(void)
{

}

