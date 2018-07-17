#include "MM_app_control.h"


uint8_t command;

extern USART_HandleTypeDef husart1;
extern UART_HandleTypeDef huart6;

void app_control_init(void)
{

    if (AUXILIARY_UART == 1)
    {
        while (HAL_OK != HAL_UART_Receive_DMA(&huart6, (uint8_t*)&command, 1));
    }
    else
    {
        while (HAL_OK != HAL_USART_Receive_DMA(&husart1, (uint8_t*)&command, 1));
        HAL_GPIO_WritePin(UART_DIR_GPIO_Port, UART_DIR_Pin, GPIO_PIN_SET);
    }

}

void app_control_function(void)
{

    switch (command)
    {
        case START:
        {

            break;
        }
        case STOP:
        {

            break;
        }
        case RESET:
        {

            break;
        }
    }

}


