#include "MM_app_transmit.h"

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

static SPI_HandleTypeDef spiPort;

int16_t *testing_buffer = NULL;
int16_t testing_buffer_1[TRANSMIT_PACKET_SIZE];


void transmit_init(void)
{
    if (AUXILIARY_SPI == 1)
    {
        spiPort = hspi2;
    }else
    {
        spiPort = hspi1;
    }

    if (TEST_BUFFER == 1)
    {
		int i;

        /* testing_buffer = (int16_t *)malloc((int)TRANSMIT_PACKET_SIZE*sizeof(int16_t)); */

        testing_buffer = testing_buffer_1;
        for (i=0; i<TRANSMIT_PACKET_SIZE; i++)
        {
            *testing_buffer = (int16_t)i;
            testing_buffer ++;
        }
    }

}

void transmit_packet(int16_t* buffer)
{
    if (TEST_BUFFER == 1)
    {
        buffer = testing_buffer;
    }
    while (HAL_OK != HAL_SPI_Transmit_DMA(&spiPort, (uint8_t*)buffer, TRANSMIT_PACKET_SIZE));

}

