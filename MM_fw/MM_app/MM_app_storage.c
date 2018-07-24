#include "MM_app_storage.h"

#include "MM_app_transmit.h"
#include "main.h"
#include "stm32f4xx_hal.h"

static int16_t *p_store_buffer = NULL, *p_transmit_buffer = NULL;
static int16_t *temp_buffer_1 = NULL, *temp_buffer_2 = NULL, *transmit_buffer = NULL;

int16_t *p_buffer_1, *p_buffer_2; 

static int sub_packets_counter = 0;
static int bank_flag = 1;

void storage_init(void)
{
    
    p_buffer_1 = (int16_t*)malloc((TRANSMIT_PACKET_SIZE)*sizeof(int16_t));
    p_buffer_2 = (int16_t*)malloc((TRANSMIT_PACKET_SIZE)*sizeof(int16_t));

    temp_buffer_1 = p_buffer_1 ++;
    transmit_buffer = p_buffer_2;
    bank_flag = 1;

}

void push_samples(int32_t *p_SAI_buffer, int samples)
{

    int i;

    for (i=0; i<samples; i++)
    {
        *temp_buffer_1 = *p_SAI_buffer;
        p_SAI_buffer++;
        temp_buffer_1++;
    }

    sub_packets_counter++;

    if (sub_packets_counter >= SUB_PACKETS)
    {
        sub_packets_counter = 0;
        //change buffers
        if (bank_flag == 1)
        {
            bank_flag = 2;
            temp_buffer_1 = p_buffer_2;
            transmit_buffer = p_buffer_1;
        }
        if (bank_flag == 2)
        {
            bank_flag = 1;
            temp_buffer_1 = p_buffer_1;
            transmit_buffer = p_buffer_2;
        }

    transmit_packet(transmit_buffer);
    }

}

