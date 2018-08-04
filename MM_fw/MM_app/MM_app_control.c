#include "MM_app_control.h"

static uint8_t command;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

uint8_t msg_START[] = "START\n\r",
        msg_RUNNING[] = "APP_RUNNING\n\r",
        msg_STOP[] = "STOP\n\r",
        msg_RESET[] = "RESET\n\r",
        msg_ERROR[] = "ERROR\n\r";


bool app_started = false;

void app_control_init(void)
{

    if (AUXILIARY_UART == 1)
    {
        while (HAL_OK != HAL_UART_Receive_DMA(&huart1, (uint8_t*)&command, 1));
    }
    else
    {
        while (HAL_OK != HAL_UART_Receive_DMA(&huart6, (uint8_t*)&command, 1));
        HAL_GPIO_WritePin(UART_DIR_GPIO_Port, UART_DIR_Pin, GPIO_PIN_SET);
    }

}

void app_control_function(void)
{

    switch (command)
    {
        case START:
        {
            if (app_started == false)
            {
                if (AUXILIARY_UART==1)
                    HAL_UART_Transmit(&huart1, (uint8_t*)msg_START, sizeof(msg_START), 0xFFFF);
                storage_init();
                transmit_init();
                sampling_init();
                app_started = true;
            }
            else
            {
                if (AUXILIARY_UART==1)
                    HAL_UART_Transmit(&huart1, (uint8_t*)msg_RUNNING, sizeof(msg_RUNNING), 0xFFFF);
            }

            break;
        }
        case STOP:
        {
            if (AUXILIARY_UART==1)
                HAL_UART_Transmit(&huart1, (uint8_t*)msg_STOP, sizeof(msg_STOP), 0xFFFF);
            break;
        }
        case RESET:
        {
            if (AUXILIARY_UART==1)
                HAL_UART_Transmit(&huart1, (uint8_t*)msg_RESET, sizeof(msg_RESET), 0xFFFF);
            break;
        }
        default:
        {
            if (AUXILIARY_UART==1)
                HAL_UART_Transmit(&huart1, (uint8_t*)msg_ERROR, sizeof(msg_ERROR), 0xFFFF);
        }

    }

    app_control_init();

}


