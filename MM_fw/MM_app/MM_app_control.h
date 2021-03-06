#ifndef MM_APP_CONTROL_H_
#define MM_APP_CONTROL_H_

#include <stdbool.h>
#include "main.h"
#include "stm32f4xx_hal.h"


#include "MM_app_sampling.h"
#include "MM_app_storage.h"
#include "MM_app_transmit.h"


#define START   '1'
#define STOP    '2'
#define RESUME  '3'
#define RESET   '4'

#define AUXILIARY_UART  1

// UART6 is the main
// USART1 is the aux

void app_control_init(void);
void app_control_function(void);


#endif

