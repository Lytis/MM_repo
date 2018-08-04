#include "MM_app_storage.h"


static int16_t *temp_buffer_1 = NULL, *transmit_buffer = NULL;

int16_t *p_buffer_1, *p_buffer_2; 

int16_t buffer_1[TRANSMIT_PACKET_SIZE], buffer_2[TRANSMIT_PACKET_SIZE];

static int sub_packets_counter = 0;
static int bank_flag = 1;

extern UART_HandleTypeDef huart1;

void storage_init(void)
{
/*     p_buffer_1 = (int16_t *) malloc(TRANSMIT_PACKET_SIZE*sizeof(int16_t));
    p_buffer_2 = (int16_t *) malloc(TRANSMIT_PACKET_SIZE*sizeof(int16_t)); */

    p_buffer_1 = buffer_1;
    p_buffer_2 = buffer_2;

    if ((p_buffer_1 == NULL) && (p_buffer_2 == NULL))
    {
        HAL_UART_Transmit(&huart1, (uint8_t*)"buffers_fail\n\r", sizeof("buffers_fail\n\r"), 0xFFFF);
    }else
    {
        HAL_UART_Transmit(&huart1, (uint8_t*)"buffers_ok\n\r", sizeof("buffers_ok\n\r"), 0xFFFF);
    }

    temp_buffer_1 = p_buffer_1 ++;
    transmit_buffer = p_buffer_2;
    bank_flag = 1;
}

void push_samples(int32_t *p_SAI_buffer)
{

    int i;

    for (i=0; i<SAI_BUFFER_SIZE; i++)
    {
        *temp_buffer_1 = (int16_t) *p_SAI_buffer;
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

