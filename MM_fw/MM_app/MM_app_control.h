#ifndef MM_APP_CONTROL_H_
#define MM_APP_CONTROL_H_

#include "main.h"
#include "stm32f4xx_hal.h"


#define START   0x01
#define STOP    0x02
#define RESET   0x03

#define AUXILIARY_UART  1

// UART6 is the main
// USART1 is the aux

void app_control_init(void);
void app_control_function(void);


#endif

