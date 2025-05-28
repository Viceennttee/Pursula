#ifndef TIMER_H
#define TIMER_H
#include "main.h"
#define THIRTY_MIN 18000 //18000 cuentas de 10hz
#define TEN_SECS 100 //100 cuentas de 10hz
void turnOffMCU();
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void TIM6_IRQHandler(void);
#endif
