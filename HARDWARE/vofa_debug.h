#ifndef VOFA_DEBUG_H
#define VOFA_DEBUG_H
#include "stm32f4xx_hal.h"
void Vofa_Init(void);
void Vofa_Task(void);
void Vofa_RxComplete(void);
void Vofa_RxError(void);
#endif
