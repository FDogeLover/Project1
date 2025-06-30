#ifndef _USARTTOT265_H
#define _USARTTOT265_H

#include "stm32f4xx.h"
#include "DronePara.h"
#include <string.h>
#include <os.h>
#include "Task.h"
void Uart5_tx(uint8_t *data,uint16_t size);
void Usart5toOnboardPC_Init(u32 Bound);

extern _Data_Rx t265_Rx;

#endif 
